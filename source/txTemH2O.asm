;txTemH2O.asm V0.6 QRT211225, qrt@qland.de
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
;V0.5 initial version
;V0.6 DS18B20 one wire temperature

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

.def    tucnt       =   r4              ;temperature update counter

.def    srsave      =   r14             ;status register save
.def    NULR        =   r15             ;NULL value register

;-------------------------------------------------------------------------------

.equ    CTRLBP      =   PORTB           ;control B port
.equ    CTRLBPP     =   PINB            ;          pinport
.equ    UNUSEDB7    =   PINB7           ;unused                         UCSK
.equ    UNUSEDB6    =   PINB6           ;                               MISO
.equ    UNUSEDB5    =   PINB5           ;                               MOSI
.equ    AC1M        =   PINB1           ;                       in      AIN1
.equ    AC0P        =   PINB0           ;                       in      AIN0

.equ    CTRLDP      =   PORTD           ;control D port
.equ    CTRLDD      =   DDRD            ;          DDR
.equ    CTRLDPP     =   PIND            ;          pinport
.equ    LED         =   PIND6           ;LED                    out
.equ    TXPOW       =   PIND5           ;TX power               out            
.equ    SEPOW       =   PIND4           ;sensor power           out
.equ    SEDAT       =   PIND2           ;       txDat            in/out
.equ    TXD         =   PIND1           ;TX serial txDat         out        

;port B                   76543210      
;                         ------mp      m ac minus, p ac plus
;wake                     ------II      I input, O output, . not present, - unused   
.equ    DDRBM_W     =   0b00000000
;                         ------NN      L low, H high, P pullup, N no pullup
.equ    PORTBM_W    =   0b00000000
;
;sleep                    ------II                                                      
.equ    DDRBM_S     =   0b00000000
;                         ------NN      
.equ    PORTBM_S    =   0b00000000

;port D                   76543210
;                         .lps-dt-      l led, p tx power, s sensor power, d sensor data, t tx
;wake                     .OOO-OO-      I input, O output, . not present, - unused     
.equ    DDRDM_W     =   0b01110110
;                         .LHL-LL-      L low, H high, P pullup, N no pullup
.equ    PORTDM_W    =   0b00000000
;
;sleep                    .OII-II-                                                      
.equ    DDRDM_S     =   0b01000000
;                         .LNN-NN-      
.equ    PORTDM_S    =   0b00000000

;-------------------------------------------------------------------------------
;send protocol
;DEVID STATUS T T 0xff 0xff

.equ    FOSC        =   4000000                         ;oscillator frequency
.equ    BAUDRATE    =   4000                            ;baudrate, DIV = 16 for U2X=0, = 8 for U2X=1, (8064)
.equ    UBRRVAL     =   FOSC / (8 * BAUDRATE) - 1       ;UBRR = fosc / (DIV * BAUDRATE) - 1 -> real baud = fosc / (DIV * (UBRR + 1))

.equ    SENIV       =   60                  ;nominal send interval in s
.equ    CMDCODE     =   $f0                 ;CMD code
.equ    DATALEN     =   6                   ;len txDat                   
.equ    CHKLEN      =   2                   ;    checksum               
.equ    TXREPEAT    =   1                   ;TX repetitions
.equ    DEVID       =   15                  ;device ID

.equ    TTXINV      =   60                  ;temperature TX every 60 wakeup cycles 

.equ    TEMDS18B20  =   0                   ;temperature DS18B20                        device status
.equ    TEMUPDATE   =   1                   ;            update bit
.equ    H2OSENSOR   =   4                   ;H2O detection sensor
.equ    H2ODETECT   =   5                   ;              ACO bit 5 in ACSR

;-------------------------------------------------------------------------------
;flags

;-------------------------------------------------------------------------------

.equ    RNDN        =   5                       ;rnd width
.equ    SCSIZE      =   9                       ;DS18B20 scratch size

.equ    txDat       =   SRAM_START              ;serial txDat           DATALEN + CHKLEN    byte        
.equ    rtmp        =   (txDat+DATALEN+CHKLEN)  ;first rtmp register    RNDN + 1
.equ    scra        =   (rtmp+RNDN+1)           ;DS18B20 scratch        SCSIZE

;-------------------------------------------------------------------------------

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

        ldi     a4,(1<<AIN1D|1<<AIN0D)      ;digital input buffers for ACIN1 + 0 off
        out     DIDR,a4

;       sbi     ACSR,ACD                    ;comparator off

;- - - - - - - - - - - - - - - - - - - -

        clr     NULR                        ;init NULR (ZH)
        ldi     ZL,29                       ;reset registers
        st      Z,NULR                      ;store indirect
        dec     ZL                          ;decrement address
        brpl    PC-2                        ;r0..29 = 0, ZL = $ff, ZH = 0 (NULR)

        ldi     ZL,low(SRAM_START)          ;clear SRAM
        st      Z+,NULR
        cpi     ZL,low(RAMEND-1)
        brne    PC-2

;- - - - - - - - - - - - - - - - - - - -

        ldi     a4,low(UBRRVAL)             ;set paramamters for serial transmission
        ldi     a5,high(UBRRVAL)
        out     UBRRH,a5
        out     UBRRL,a4

        ldi     a4,(1<<U2X)                 ;double speed
        out     UCSRA,a4

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        ldi     a4,DEVID                    ;store fixed device ID
        sts     txDat,a4

;- - - - - - - - - - - - - - - - - - - -

m00:    rcall   wakeUp                      ;wake up
        rcall   prepData                    ;prepare txDat
        rcall   sendData                    ;send txDat
        rcall   shutDown                    ;shutdown
        rjmp    m00                         ;mainloop

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
                
        cbi     ACSR,ACD                    ;comparator on
        sbi     ACSR,ACBG                   ;bandgap on

        tst     tucnt                       ;temperature update counter down?
        brne    PC+2                        ;no, jump
        sbi     CTRLDP,SEPOW                ;sensor power on
        
        breq    PC+2                        ;counter down?
        sbi     CTRLDP,TXPOW                ;no, TX power on

        ldi     a4,10                       ;wait 50 ms
        rcall   wait5xms                    ;
        rcall   blinkLed                    ;blink LED

        tst     tucnt                       ;counter down?
        brne    cs09                        ;no, exit
        rjmp    getTemp                     ;get temperature, TX power on shortly before end of conversion

;-------------------------------------------------------------------------------

;WDT IR enable, WDTON fuse = 1 (unprogrammed)                               interval
.equ    WDTEN025    =  (1<<WDIE|1<<WDP2)                    ;-> WDTCSR      0.25 s
.equ    WDTEN8      =  (1<<WDIE|1<<WDP3|1<<WDP0)            ;               8 s

;WDT disable sequence, delete IR flag, next interval 8 s
.equ    WDTDI80     =  (1<<WDCE|1<<WDE|1<<WDP3|1<<WDP0)     ;-> WDTCSR      8 s
.equ    WDTDI81     =  (1<<WDIF|0<<WDE|1<<WDP3|1<<WDP0)     ;

shutDown:
        sbi     ACSR,ACD                    ;comparator off
        cbi     ACSR,ACBG                   ;bandgap off

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

        rcall   blinkLed                    ;blink LED
        dec     XH                          ;count down WDT sleep intervals
        brne    cSleep                      ;intervals down? no, continue sleep

        cli                                 ;disable IRs
;       out     MCUCR,NULR                  ;sleep enable off
cs09:   ret

;-------------------------------------------------------------------------------

prepData:
        ldi     ZL,txDat+1                   ;+1 (after DEVID)

        in      a4,ACSR                         ;AC output ACO bit 5
        andi    a4,(1<<H2ODETECT)               ;status H2O detection
        ori     a4,(1<<H2OSENSOR|1<<TEMDS18B20) ;       has H2O sensor and DS18B20 sensor

        tst     tucnt                       ;temperature update counter down?
        brne    pd01                        ;no, jump
        ori     a4,(1<<TEMUPDATE)           ;temperature update
        ldi     a5,(TTXINV+1)               ;reinit counter
        mov     tucnt,a5                    ;

pd01:   st      Z+,a4                       ;store status
        dec     tucnt                       ;counter --

;- - - - - - - - - - - - - - - - - - - -

        ldi     YL,scra                     ;temperature scratch pointer
        ld      a4,Y+                       ;            LB
        ld      a5,Y+                       ;            HB
        st      Z+,a4                       ;
        st      Z+,a5                       ;
        ldi     a4,0xff                     ;humi not valid
        st      Z+,a4                       ;
        st      Z+,a4                       ;

        rjmp    genCheckSum                 ;checksum

;-------------------------------------------------------------------------------

blinkLed:
        sbi     CTRLDP,LED                  ;blink LED
        rcall   wait5ms                     ;
        cbi     CTRLDP,LED                  ;
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

;---------------------------------------

waitWdt750:
        ldi     a4,WDTEN025

;- - - - - - - - - - - - - - - - - - - -

waitWdt:
        wdr                                 ;WDT reset
        out     WDTCSR,a4                   ;    start
        ldi     a4,(1<<SE|0<<SM1|1<<SM0)    ;sleep enable, power-down mode
        out     MCUCR,a4
        sei                                 ;enable IRs

        sleep                               ;sleep 3 * 250 ms (+ 65 ms wakeup time)
        sleep                               ;
        sbi     CTRLDP,TXPOW                ;TX power on
        sleep                               ;

        cli                                 ;disable IRs

;- - - - - - - - - - - - - - - - - - - -
       
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

;-------------------------------------------------------------------------------
;one wire commands
.equ    MATCH_ROM           =       $55     ;rom commands
.equ    SKIP_ROM            =       $cc
.equ    SEARCH_ROM          =       $f0
.equ    READ_ROM            =       $33
.equ    START_CONVERT       =       $44     ;function commands
.equ    READ_SCRATCH        =       $be
.equ    WRITE_SCRATCH       =       $4e
.equ    STORE_EE            =       $48
.equ    RECALL_EE           =       $b8
.equ    ERRCODE             =       $8000   ;temperature error code 

getTemp:
;       sbi     CTRLDP,SEPOW                ;DS18B20 power on
;       ldi     a4,10
;       rcall   wait5xms                    ;power on time 50 ms

;- - - - - - - - - - - - - - - - - - - -
;7 6 5 4 3 2 1 0 7 6 5 4 3 2 1 0
;s s s s s b b b b b b b a a a a            s sign, b before point, a after point 1/16 °C

        rcall   owiReset                    ;one-wire reset
        brts    error                       ;device not present

        ldi     a4,SKIP_ROM                 ;write SKIP_ROM
        rcall   owiWriteByte

        ldi     a4,START_CONVERT            ;write START_CONVERT
        rcall   owiWriteByte

;       ldi     a4,75*2                     ;750 ms conversion time
;       rcall   wait5xms

        rcall   waitWdt750                  ;wait 750 ms via WDT for conversion

        rcall   owiReset                    ;one-wire reset
        brts    error                       ;device not present

        ldi     a4,SKIP_ROM                 ;write SKIP_ROM
        rcall   owiWriteByte

        ldi     a4,READ_SCRATCH             ;write READ_SCRATCH
        rcall   owiWriteByte

        clr     a0                          ;reset CRC
        ldi     YL,scra                     ;scratch pointer
        ldi     a7,0x18                     ;eor mask for CRC
        ldi     a5,SCSIZE                   ;9 bytes to read
gt01:   rcall   owiReadByte                 ;read next byte
        st      Y+,a4                       ;store next byte
        rcall   crc8Update                  ;update CRC
        dec     a5                          ;loop counter --
        brne    gt01

        tst     a0                          ;compare CRC with zero
        brne    error                       ;bad CRC

gt09:   cbi     CTRLDP,SEPOW                ;DS18B20 power off
        ret                                 ;exit

;---------------------------------------

error:  ldi     a4,low(ERRCODE)             ;store error code
        ldi     a5,high(ERRCODE)
        sts     scra,a4                     
        sts     scra+1,a5

        rjmp    gt09                        ;DS18B20 power of and exit

;---------------------------------------

owiReset:
        cbi     CTRLDP,SEDAT                ;L
        sbi     CTRLDD,SEDAT                ;output

        ldi     c5,(240-3)                  ;wait 480 us
        rcall   waitUs
        ldi     c5,(240-3)
        rcall   waitUs

        cbi     CTRLDD,SEDAT                ;input, nopullup

        ldi     c5,(70-3)                   ;wait 60..240 us
        rcall   waitUs

        set                                 ;assume error
        sbis    CTRLDPP,SEDAT               ;input H?
        clt                                 ;no, ok

        ldi     c5,(240-3)                  ;wait 240 us
        rjmp    waitUs

;---------------------------------------

owiWriteBit:
        brcc    owiWriteZero

owiWriteOne:
        sbi     CTRLDD,SEDAT
        lpm     c5,Z                        ;1 us   3
        nop                                 ;       1
        cbi     CTRLDD,SEDAT

        ldi     c5,(60-3)
        rjmp    waitUs

owiWriteZero:
        sbi     CTRLDD,SEDAT
        ldi     c5,(90-3)                   ;60..120 us
        rcall   waitUs
        cbi     CTRLDD,SEDAT

        ldi     c5,(60-3)
        rjmp    waitUs

;---------------------------------------

owiWriteByte:
        ldi     c4,8

owiWriteLoop:
        ror     a4
        rcall   owiWriteBit
        dec     c4
        brne    owiWriteLoop

        ret

;---------------------------------------

owiReadBit:
        sbi     CTRLDD,SEDAT
        lpm     c5,Z                        ;1 us   3
        nop                                 ;       1
        cbi     CTRLDD,SEDAT

        ldi     c5,(5-3)
        rcall   waitUs

        clt
        sbic    CTRLDPP,SEDAT
        set

        ldi     c5,(50-3)
        rcall   waitUs

        sec
        brts    PC+2
        clc

        ret

;---------------------------------------

owiReadByte:
        ldi     c4,8

owiReadLoop:
        rcall   owiReadBit
        ror     a4
        dec     c4
        brne    owiReadLoop

        ret

;---------------------------------------
;1      + 3     + c5 * 4 + 1    + 3   + 4   = 12 + c5 * 4 = 3 us + c5 * 1 us
;ldi c5   rcall            brne   lpm   ret

waitUs:
        dec     c5              ;1
        nop                     ;1
        brne    PC-2            ;1/2

        lpm     c5,Z            ;3 (dummy)
        ret                     ;4

;---------------------------------------

crc8Update:
        mov     a1,a4           ;save b
        ldi     a6,8            ;8 bits

crc8l:  eor     a4,a0           ;b ^= crc
        ror     a4              ;C -> b -> C
        mov     a4,a0           ;b = crc
        brcc    crc8z           ;C == 0?
        eor     a4,a7           ;no, b ^= 0x18

crc8z:  ror     a4              ;C -> b -> C
        mov     a0,a4           ;crc = b

        mov     a4,a1           ;restore b
        lsr     a4              ;0 -> b -> C
        mov     a1,a4           ;save b

        dec     a6              ;next bit
        brne    crc8l           ;all bits?

        ret
