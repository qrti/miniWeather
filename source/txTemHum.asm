;txTemHum.asm V0.5 QRT211225, qrt@qland.de
;
;fuse bits          cksel3...0     1101    ceramic/crystal 3..8 MHz                     <-
;                   cksel3...0     1111    ceramic/crystal 8..  MHz
;                   sut1..0          00    ceramic resonator, slow rising power
;                                    11    crystal oscillator, slow rising power        <-
;
;                   cksel3...0     0010    for calibrated internal RC oscillator 4 MHz
;                   cksel3...0     0100    for calibrated internal RC oscillator 8 MHz
;                   sut1..0          10    RC oscillator, slow rising power
;
;                   bodlevel2..0    111    off   brownout detection                     <-
;                                   110    1.8 V
;                                   101    2.7 V
;                                   100    4.3 V
;
;                   low     high    extended
;                   0xfd    0xdf    0xff
;
;V0.5 initial version

;-------------------------------------------------------------------------------

.include "tn2313def.inc"

.cseg
.org $0000
rjmp main                                   ;Reset Handler
;.org $0001
;rjmp INT0_IR                               ;External Interrupt0 Handler
;.org $0002
;rjmp INT1_IR                               ;External Interrupt1 Handler
;.org $0003
;rjmp TIM1_CAPT_IR                          ;Timer1 Capture Handler
;.org $0004
;rjmp TIM1_COMPA_IR                         ;Timer1 CompareA Handler
;.org $0005
;rjmp TIM1_OVF_IR                           ;Timer1 Overflow Handler
;.org $0006
;rjmp TIM0_OVF_IR                           ;Timer0 Overflow Handler
;.org $0007
;rjmp USART0_RXC_IR                         ;USART0 RX Complete Handler
;.org $0008
;rjmp USART0_DRE_IR                         ;USART0,UDR Empty Handler
;.org $0009
;rjmp USART0_TXC_IR                         ;USART0 TX Complete Handler
;.org $000a
;rjmp ANA_COMP_IR                           ;Analog Comparator Handler
;.org $000b
;rjmp PCINT_IR                              ;Pin Change Interrupt
;.org $000c
;rjmp TIMER1_COMPB_IR                       ;Timer1 CompareB Handler
;.org $000d
;rjmp TIMER0_COMPA_IR                       ;Timer0 Compare A Handler
;.org $000e
;rjmp TIMER0_COMPB_IR                       ;Timer0 Compare B Handler
;.org $000f
;rjmp USI_START_IR                          ;USI Start Handler
;.org $0010
;rjmp USI_OVERFLOW_IR                       ;USI Overflow Handler
;.org $0011
;rjmp EE_READY_IR                           ;EEPROM Ready Handler
.org $0012
rjmp WDT_OVERFLOW_IR                       ;Watchdog Overflow Handler

;-------------------------------------------------------------------------------

.def    a0          =   r0              ;main registers set a
.def    a1          =   r1
.def    a2          =   r2
.def    a3          =   r3
.def    a4          =   r24             ;main registers set a immediate
.def    a5          =   r25
.def    a6          =   r16
.def    a7          =   r17

.def    c4          =   r18             ;main registers set c immediate
.def    c5          =   r19
.def    c6          =   r20
.def    c7          =   r21

.def    srsave      =   r14             ;status register save
.def    NULR        =   r15             ;NULL value register

;-------------------------------------------------------------------------------

.equ    CTRLBP      =   PORTB           ;control B port
.equ    CTRLBPP     =   PINB            ;          pinport
.equ    I2CCLK      =   PINB7           ;I2C SCL                            SCL
.equ    UNUSEDB6    =   PINB6           ;                                   MISO
.equ    I2CDAT      =   PINB5           ;I2C SDA                    SDA     MOSI

.equ    CTRLDP      =   PORTD           ;control D port
.equ    CTRLDD      =   DDRD            ;          DDR
.equ    CTRLDPP     =   PIND            ;          pinport
.equ    LED         =   PIND6           ;LED                    out
.equ    TXPOW       =   PIND5           ;TX power               out
.equ    SEPOW       =   PIND4           ;sensor power           out
.equ    TXD         =   PIND1           ;TX serial data         out         TXD

;port B                   76543210
;                         c-d-----      c i2c SCL, d i2c SDA
;wake                     O-O-----      I input, O output, . not present, - unused
.equ    DDRBM_W     =   0b10100000
;                         P-P-----      L low, H high, P pullup, N no pullup
.equ    PORTBM_W    =   0b10100000
;
;sleep                    O-O-----
.equ    DDRBM_S     =   0b10100000
;                         L-L-----
.equ    PORTBM_S    =   0b10100000

;port D                   76543210
;                         .lts--x-      l led, t tx power, s sensor power, x tx
;wake                     .OOO--O-      I input, O output, . not present, - unused
.equ    DDRDM_W     =   0b01110010
;                         .LLL--L-      L low, H high, P pullup, N no pullup
.equ    PORTDM_W    =   0b00000000
;
;sleep                    .OII-II-
.equ    DDRDM_S     =   0b01000000
;                         .LNN-NN-
.equ    PORTDM_S    =   0b00000000

;-------------------------------------------------------------------------------
;send protocol
;DEVID STATUS TT HH      every 60 s 

.equ    FOSC        =   4000000                         ;oscillator frequency
.equ    BAUDRATE    =   4000                            ;baudrate, DIV = 16 for U2X=0, = 8 for U2X=1, (8064)
.equ    UBRRVAL     =   FOSC / (8 * BAUDRATE) - 1       ;UBRR = fosc / (DIV * BAUDRATE) - 1 -> real baud = fosc / (DIV * (UBRR + 1))

.equ    SENIV       =   60                  ;nominal send interval in s
.equ    CMDCODE     =   $f0                 ;manchester command code
.equ    DATALEN     =   6                   ;len txDat                    
.equ    CHKLEN      =   2                   ;    checksum
.equ    TXREPEAT    =   1                   ;TX repetitions
.equ    DEVID       =   14                  ;device ID

;-------------------------------------------------------------------------------
;flags

;-------------------------------------------------------------------------------

.equ    RNDN        =   5                       ;rnd width

.equ    txDat       =   SRAM_START              ;serial txDat               DATALEN + CHKLEN    byte
.equ    rtmp        =   (txDat+DATALEN+CHKLEN)  ;first rtmp register        RNDN + 1

;-------------------------------------------------------------------------------

.define SENSE                   1           ;sensor in                      sonsor routine function
.define CONVERT                 2           ;sensor or binary in -> converted out
.define FUNC                    (SENSE)     ;sensor in -> binary out

.include "i2cMaster.inc"
.include "sensor.inc"
.include "manTx.inc"
.include "rnd.inc"

;-------------------------------------------------------------------------------

main:
        ldi     a4,low(RAMEND)              ;set stack pointer
        out     SPL,a4                      ;to top of RAM

;- - - - - - - - - - - - - - - - - - - -

        ldi     a4,(1<<CLKPCE)              ;CLKPCE=1                       init
        out     CLKPR,a4
        ldi     a4,0                        ;CLKPCE=0, div 1
        out     CLKPR,a4                    ;system clock 4 MHz

;       ldi     a4,(1<<AIN1D|1<<AIN0D)      ;digital input buffers for ACIN1 + 0 off
;       out     DIDR,a4

;       sbi     ACSR,ACD                    ;comparator off

;- - - - - - - - - - - - - - - - - - - -

        clr     NULR                        ;init NULR (ZH)
        ldi     ZL,29                       ;reset registers
        st      Z,NULR                      ;store indirect
        dec     ZL                          ;decrement address
        brpl    PC-2                        ;r0..29 = 0, ZL = $ff, ZH = 0 (NULR)

        ldi     ZL,low(SRAM_START)          ;clear SRAM
        st      Z+,NULR                     ;
        cpi     ZL,low(RAMEND-1)            ;
        brne    PC-2                        ;

;- - - - - - - - - - - - - - - - - - - -

        ldi     a4,low(UBRRVAL)             ;set paramamters for serial transmission
        ldi     a5,high(UBRRVAL)            ;
        out     UBRRH,a5                    ;
        out     UBRRL,a4                    ;

        ldi     a4,(1<<U2X)                 ;double speed
        out     UCSRA,a4

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        ldi     a4,DEVID                    ;store fixed device ID
        sts     txDat,a4                    ;

;- - - - - - - - - - - - - - - - - - - -

m00:    rcall   wakeUp                      ;wake up
        rcall   prepData                    ;prepare txDat
        rcall   sendData                    ;send txDat
        rcall   shutDown                    ;shutdown
        rjmp    m00                         ;main loop

;-------------------------------------------------------------------------------

wakeUp:
        ldi     a4,PORTBM_W                 ;port B
        out     PORTB,a4
        ldi     a4,DDRBM_W                  ;ddr B
        out     DDRB,a4

        ldi     a4,PORTDM_W                 ;port D
        out     PORTD,a4
        ldi     a4,DDRDM_W                  ;ddr D
        out     DDRD,a4

        rcall   disWdt                      ;disable WDT

;       cbi     ACSR,ACD                    ;comparator on
;       sbi     ACSR,ACBG                   ;bandgap on

        sbi     CTRLDP,SEPOW                ;sensor power on

        ldi     a4,10                       ;wait 50 ms
        rcall   wait5xms                    ;

        rjmp    i2cInit                     ;init I2C

;-------------------------------------------------------------------------------

;WDT IR enable, WDTON fuse = 1 (unprogrammed)                               interval
.equ    WDTEN025    =  (1<<WDIE|1<<WDP2)                    ;-> WDTCSR      0.25 s
.equ    WDTEN8      =  (1<<WDIE|1<<WDP3|1<<WDP0)            ;               8 s

;WDT disable sequence, delete IR flag, next interval 8 s
.equ    WDTDI80     =  (1<<WDCE|1<<WDE|1<<WDP3|1<<WDP0)     ;-> WDTCSR      8 s
.equ    WDTDI81     =  (1<<WDIF|0<<WDE|1<<WDP3|1<<WDP0)     ;

shutDown:
;       sbi     ACSR,ACD                    ;comparator off
;       cbi     ACSR,ACBG                   ;bandgap off

        ldi     a4,PORTBM_S                 ;port B
        out     PORTB,a4
        ldi     a4,DDRBM_S                  ;ddr B
        out     DDRB,a4

        ldi     a4,PORTDM_S                 ;port D
        out     PORTD,a4
        ldi     a4,DDRDM_S                  ;ddr D
        out     DDRD,a4

        ldi     a4,WDTEN8                   ;start WDT
        out     WDTCSR,a4                   ;

        ldi     a4,(1<<SE|0<<SM1|1<<SM0)    ;sleep enable, power-down mode
        out     MCUCR,a4

        rcall   rnd                         ;send interval
        ldi     XH,SENIV                    ;(60 + 0..15) / 8 -> 56..72 s
        add     XH,a5
        lsr     XH
        lsr     XH
        lsr     XH

        sei                                 ;enable IRs

cSleep: sleep                               ;sleep

;- - - - - - - - - - - - - - - - - - - -

        sbi     CTRLDP,LED                  ;LED on
        rcall   wait5ms                     ;wait 5 ms
        cbi     CTRLDP,LED                  ;LED off

        dec     XH                          ;count down WDT sleep intervals
        brne    cSleep                      ;intervals down? no, continue sleep

        cli                                 ;disable IRs
;       out     MCUCR,NULR                  ;sleep enable off
        ret

;-------------------------------------------------------------------------------

prepData:
        ldi     ZL,txDat+1                  ;txDat pointer (+1 after DEVID)
        st      Z+,NULR                     ;status, always 0, unused

        sbi     CTRLDP,LED                  ;LED on

        rcall   readTemp                    ;~ 50 ms
        rcall   chSaRe                      ;check and save read

        cbi     CTRLDP,LED                  ;LED off
        sbi     CTRLDP,TXPOW                ;TX power on

        rcall   readHumi                    ;~ 50 ms
        rcall   chSaRe                      ;check and save read
        
        rjmp    genCheckSum                 ;append checksum

;---------------------------------------

chSaRe: breq    PC+3                        ;read ok? yes, store read
        ldi     a4,0xff                     ;error value
        ldi     a5,0xff                     ;
        st      Z+,a4                       ;store LB
        st      Z+,a5                       ;      HB
        ret

;-------------------------------------------------------------------------------
;3 + (a6 * ((a5 * 3) + 3)) + 4, 20000 cycles @ 4 MHz = 5 ms
;
.equ    DLX     =    28
.equ    DLY     =    237

wait10ms:
       ldi     a4,2
       rjmp    PC+2

wait5ms:
       ldi     a4,1

wait5xms:
       ldi     a6,DLX
       ldi     a5,DLY
       dec     a5
       brne    PC-1
       dec     a6
       brne    PC-4
       dec     a4
       brne    wait5xms

       ret

;-------------------------------------------------------------------------------

disWdt:
        wdr                                 ;WDT disable sequence
        out     MCUSR,NULR                  ;reset WDRF-bit
        ldi     a4,WDTDI80
        ldi     a5,WDTDI81
        out     WDTCSR,a4
        out     WDTCSR,a5
        ret

;-------------------------------------------------------------------------------

WDT_OVERFLOW_IR:
        reti
