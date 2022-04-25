;rxOled.asm V0.6 QRT211226, qrt@qland.de
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
;V0.6 device number display

;-------------------------------------------------------------------------------

.include "tn2313def.inc"

.define     WARNMODE       0                ;0 as received, 1 endless after triggerd

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
;.org $0012
;rjmp WDT_OVERFLOW_IR                       ;Watchdog Overflow Handler

;-------------------------------------------------------------------------------

.def    a0          =   r0              ;main registers set a
.def    a1          =   r1
.def    a2          =   r2
.def    a3          =   r3
.def    a4          =   r24             ;main registers set a immediate
.def    a5          =   r25
.def    a6          =   r16
.def    a7          =   r17

.def    c0          =   r4              ;main registers set c
.def    c1          =   r5
.def    c2          =   r6
.def    c3          =   r7

.def    c4          =   r18             ;main registers set c immediate
.def    c5          =   r19
.def    c6          =   r20
.def    c7          =   r21

.def    NULR        =   r15             ;NULL value register

.def    phase       =   r22             ;RX phase
.def    rxdp        =   r26             ;rxDat pointer      (XL)

.def    tic         =   r8              ;50 ms ticker
.def    tbinL       =   r9              ;Tbin LB
.def    tbinH       =   r10             ;     HB
.def    oledX       =   r11             ;oled position  x 0..127
.def    oledY       =   r12             ;               y 0..7
.def    keypc       =   r13             ;key press counter
.def    selDev      =   r14             ;selected device
.def    dishti      =   r23             ;device ID show timer

;-------------------------------------------------------------------------------

.equ    CTRLBP      =   PORTB           ;control B port
.equ    CTRLBPP     =   PINB            ;          pinport
.equ    UNUSEDB7    =   PINB7           ;unused                         UCSK
.equ    UNUSEDB6    =   PINB6           ;                               MISO
.equ    UNUSEDB5    =   PINB5           ;                               MOSI

.equ    CTRLDP      =   PORTD           ;control D port
.equ    CTRLDPP     =   PIND            ;          pinport
.equ    LED         =   PIND6           ;LED                    out
.equ    BUZ         =   PIND5           ;buzzer                 out     OC0B
.equ    KEY         =   PIND4           ;key                    in
.equ    TXD         =   PIND1           ;send    serial rxDat   out     TX
.equ    RXD         =   PIND0           ;receive                in      RX

;port B                   76543210
;                         --------
;                         --------      I input, O output, . not present, - unused
.equ    DDRBM       =   0b00000000
;                         --------      L low, H high, P pullup, N no pullup
.equ    PORTBM      =   0b00000000
;
;port D                   76543210
;                         .lbk--tr      l led, b buzzer, k key, t tx, r rx
;                         .OOI--OI      I input, O output, . not present, - unused
.equ    DDRDM       =   0b01100010
;                         .LLP--LN      L low, H high, P pullup, N no pullup
.equ    PORTDM      =   0b00010000

;-------------------------------------------------------------------------------

.equ    FOSC        =   4000000                         ;oscillator frequency
.equ    BAUDRATE    =   4000                            ;baudrate, DIV = 16 for U2X=0, = 8 for U2X=1, (8064)
.equ    UBRRVAL     =   FOSC / (8 * BAUDRATE) - 1       ;UBRR = fosc / (DIV * BAUDRATE) - 1 -> real baud = fosc / (DIV * (UBRR + 1))
.equ    UBRRVAL96   =   FOSC / (8 * 9600) - 1

.equ    CMDCODE     =   $f0                 ;manchester command code
.equ    RXD_LEN     =   6                   ;len rxDat
.equ    CHK_LEN     =   2                   ;    checksum
.equ    BUZCYC      =   150                 ;4E6 / DIV / 2 / 1000 Hz, DIV = 8
.equ    TIMERCYC    =   6250                ;0.1 ms / (4E6^-1 * DIV), DIV = 64
.equ    RX_TIMO     =   12                  ;RX timeout ~ 5 m * 60 s / (100 ms * 256)
.equ    SI_TIRX     =   5                   ;signal time RX in 100 ms
.equ    DISHTIME    =   50                  ;device ID show time 5 s / 100 ms

.equ    KEYLONGT    =   40                  ;2 s / 50 ms, key long press time

;-------------------------------------------------------------------------------
;flags GPIOR0
.equ    DATRDY      =   1                   ;rxDat ready                                global flags
.equ    H2ODET      =   2                   ;H2O detection

;-------------------------------------------------------------------------------
;dddddd         device  (6 byte)
;ssssss ss      struct  (8 byte)
;isvvvv tf      id status value timer flags                     

.equ    TEMDS18B20  =   0                   ;temperature DS18B20                        device status
.equ    TEMUPDATE   =   1                   ;            update bit
.equ    H2OSENSOR   =   4                   ;H2O detection sensor
.equ    H2ODETECT   =   5                   ;              ACO bit 5 in ACSR

.equ    SYM0        =   0                   ;symbol bit 0                               struct flags
.equ    SYM1        =   1                   ;           1
.equ    FIR         =   2                   ;first receive                              
.equ    DRX         =   3                   ;data received

;-------------------------------------------------------------------------------

rxDevIdTab:
.DB     1, 14, 15, 0                            ;rx device IDs

.equ    RXS_NUM     =   3                       ;rx struct num
.equ    RXS_LEN     =   8                       ;          len

.equ    rxDat       =   SRAM_START              ;serial receive data        RXD_LEN + CHK_LEN    byte
.equ    rxStr       =   (rxDat+RXD_LEN+CHK_LEN) ;receive device structs     RXS_NUM * RXS_LEN
.equ    ascDig      =   (rxStr+RXS_NUM*RXS_LEN) ;text buffer

;-------------------------------------------------------------------------------

.define SENSE                   1           ;sensor in                      sonsor.inc function
.define CONVERT                 2           ;sensor or binary in -> converted out
.define FUNC                    (CONVERT)   ;binary in -> converted out

.include "i2cMaster.inc"
.include "sensor.inc"
.include "oled.inc"
.include "fonts.inc"
.include "show.inc"

;-------------------------------------------------------------------------------

main:
        ldi     a4,low(RAMEND)              ;set stack pointer
        out     SPL,a4                      ;to top of RAM

;- - - - - - - - - - - - - - - - - - - -

        ldi     a4,(1<<CLKPCE)              ;CLKPCE=1                       init
        out     CLKPR,a4
        ldi     a4,0                        ;CLKPCE=0, div 1
        out     CLKPR,a4                    ;system clock 4 MHz

        sbi     ACSR,ACD                    ;comparator off

;- - - - - - - - - - - - - - - - - - - -

        ldi     a4,PORTBM                   ;port B
        out     PORTB,a4
        ldi     a4,DDRBM                    ;ddr B
        out     DDRB,a4

        ldi     a4,PORTDM                   ;port D
        out     PORTD,a4
        ldi     a4,DDRDM                    ;ddr D
        out     DDRD,a4

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

        ldi     XL,rxDat                    ;reset rxDat pointer
        mov     rxdp,XL                     ;

;- - - - - - - - - - - - - - - - - - - -

        ldi     a4,low(UBRRVAL)             ;serial transmission paramamters
        ldi     a5,high(UBRRVAL)
        out     UBRRH,a5
        out     UBRRL,a4

        sbi     UCSRA,U2X                   ;double speed
        sbi     UCSRB,RXEN                  ;enable RX
        sbi     UCSRB,TXEN                  ;       TX

;- - - - - - - - - - - - - - - - - - - -

        ldi     a4,(BUZCYC-1)               ;T0 CMP A value for buzzer
        out     OCR0A,a4

;- - - - - - - - - - - - - - - - - - - -

        ldi     a4,low(TIMERCYC)            ;T1 100 ms cycle
        ldi     a5,high(TIMERCYC)
        out     OCR1AH,a5
        out     OCR1AL,a4

        ldi     a4,(1<<WGM12|1<<CS11|1<<CS10)   ;CTC, DIV 64
        out     TCCR1B,a4                       ;start

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        ldi     ZL,low(rxDevIdTab<<1)       ;store device IDs in struct
        ldi     ZH,high(rxDevIdTab<<1)      ;
        ldi     YL,rxStr                    ;
in01:   lpm     a4,Z+                       ;
        st      Y,a4                        ;
        subi    YL,-RXS_LEN                 ;
        cpi     YL,(rxStr+RXS_NUM*RXS_LEN)  ;
        brne    in01                        ;

        rcall   buzz                        ;buzz

        rcall   i2cInit                     ;init I2C
        rcall   oledInit                    ;     oled
        ldi     dishti,DISHTIME             ;start ID show time to trigger schedule
        rcall   scShowDevice                ;schedule show current device

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

m00:    rcall   rxData                      ;receive data

        sbic    GPIOR0,DATRDY               ;data ready?
        rcall   evalData                    ;yes, evaluate

        rcall   timer                       ;timer
        rjmp    m00                         ;mainloop

;-------------------------------------------------------------------------------

rxData:
        sbis     UCSRA,RXC                  ;data received?
        ret                                 ;no, exit

        rcall   rxByte                      ;rx byte
        brtc    rd01                        ;command byte? no, jump

        tst     phase                       ;start command?
        breq    rd08                        ;yes, phase++, exit
        sbrs    phase,0                     ;phase ok for stop command?
        rjmp    rd06                        ;no, error
        cpi     XL,(rxDat+RXD_LEN+CHK_LEN)  ;stop command at end of data?
        brne    rd06                        ;no, error
        sbi     GPIOR0,DATRDY               ;flag data ready
rd06:   ldi     XL,rxDat                    ;reset rxDat pointer
        clr     phase                       ;reset phase
        ret                                 ;exit

rd01:   brne    rd06                        ;data valid? no, error
        tst     phase                       ;     in phase 0?
        breq    rd09                        ;yes, exit
        sbrc    phase,0                     ;every second phase
        rjmp    rd07                        ;hb0 <- hb, phase++
        cpi     XL,(rxDat+RXD_LEN+CHK_LEN)  ;data overflow?
        brsh    rd06                        ;yes, error
        mov     a4,c7                       ;rxDat[X++] =  hb << 4 | hb0
        swap    a4                          ;
        or      a4,c6                       ;
        st      X+,a4                       ;

rd07:   mov     c6,c7                       ;hb0 <- hb
rd08:   inc     phase                       ;phase++
rd09:   ret                                 ;exit

;---------------------------------------

rxByte:
        in      a4,UDR                      ;get data (r)

        clt                                 ;assume data, T=0
        cpi     a4,CMDCODE                  ;command?
        breq    rb08                        ;yes, T=1, exit

;- - - - - - - - - - - - - - - - - - - -

        clr     a5                          ;check val (v)
        ldi     c7,0b00010000               ;hb, 4 iterations (h)
rb01:   lsl     a4                          ;C <- r<<=1
        rol     a5                          ;     v<<=1 <- C
        lsl     a4                          ;C <- r<<=1
        rol     c7                          ;     h<<=1 <- C
        brcc    rb01                        ;loop

        eor     a5,c7                       ;v^=h
        cpi     a5,0x0f                     ;v!=0x0f, v==0x0f
rb09:   ret                                 ;Z=0 err, Z=1 ok, a4=0, c7 hb=0000xxxx, T=0

rb08:   set                                 ;command, T=1
        ret                                 ;exit

;-------------------------------------------------------------------------------

evalData:
        rcall   genCheckSum                 ;generate checksum
        tst     a0                          ;received data checksum + received checksum == 0?
        brne    ed08                        ;no, data error, exit
        tst     a1                          ;
        brne    ed08                        ;no, data error, exit

;- - - - - - - - - - - - - - - - - - - -

        ldi     YL,rxDat                    ;rxDat pointer
        ld      a5,Y+                       ;load device ID

        ldi     ZL,rxStr                    ;find ID in structs
ed10:   ld      a4,Z+                       ;struct pointer on status
        cp      a4,a5                       ;
        breq    ed11                        ;found, jump
        subi    ZL,-(RXS_LEN-1)             ;next struct
        cpi     ZL,(rxStr+RXS_NUM*RXS_LEN)  ;all structs?
        brne    ed10                        ;no, jump

        rjmp    ed08                        ;not found, exit

;- - - - - - - - - - - - - - - - - - - -

ed11:   ld      a6,Y+                       ;load device status
        st      Z+,a6                       ;store in struct

        ldi     a5,RXD_LEN-2                ;store data in struct (-2, fixed device ID + status)
ed12:   ld      a4,Y+                       ;
        st      Z+,a4                       ;
        dec     a5                          ;
        brne    ed12                        ;

        ldi     a4,(RX_TIMO<<4|SI_TIRX)     ;set timer
        st      Z+,a4                       ;

        ld      a4,Z                        ;struct flags
        ori     a4,(1<<DRX|1<<FIR)          ;data received, timer running, first receive
        st      Z+,a4                       ;

;- - - - - - - - - - - - - - - - - - - -

        sbrs    a6,H2OSENSOR                ;device with H2O sensor?
        rjmp    ed08                        ;no, jump
        
.if WARNMODE == 0                           ;as received
        cbi     GPIOR0,H2ODET               ;reset global flag
.endif
        sbrc    a6,H2ODETECT                ;detection?
        sbi     GPIOR0,H2ODET               ;yes, set global flag

;- - - - - - - - - - - - - - - - - - - -

ed08:   cbi     GPIOR0,DATRDY               ;reset flag
        ret

;-------------------------------------------------------------------------------

timer:
        in      a4,TIFR                     ;T1 OCF1A event?
        sbrs    a4,OCF1A                    ;yes, jump
        ret

        ldi     a4,(1<<OCF1A)               ;reset flag                 every 100 ms
        out     TIFR,a4                     ;

;- - - - - - - - - - - - - - - - - - - -

        dec     tic                         ;ticker--

        ldi     YL,rxStr                    ;struct pointer
        clr     a7                          ;       num
tm10:   ldd     a6,Y+7                      ;       flags

        ldd     c4,Y+6                      ;load timers
        mov     c5,c4                       ;copy
        andi    c5,0xf0                     ;RX timer
        andi    c4,0x0f                     ;signal timer
        breq    PC+2                        ;==0?
        dec     c4                          ;no, dec

        tst     tic                         ;ticker==0?
        brne    tm11                        ;no, jump
        
        tst     c5                          ;RX timer
        breq    PC+2                        ;==0?
        subi    c5,0x10                     ;no, dec

tm11:   or      c4,c5                       ;RX | signal timer
        std     Y+6,c4                      ;store timers

        rcall   showStatus                  ;yes, show status

        sbrs    a6,DRX                      ;data received
        rjmp    tm12                        ;no, jump

        andi    a6,~(1<<DRX)                ;reset flag

        cp      a7,selDev                   ;selected device?
        brne    tm12                        ;no, jump
        rcall   showSelTemp                 ;show selected temp
        rcall   showSelHumi                 ;              humi

tm12:   std     Y+7,a6                      ;store flags

        inc     a7                          ;struct num ++
        subi    YL,-RXS_LEN                 ;next struct
        cpi     YL,rxStr+RXS_NUM*RXS_LEN    ;
        brne    tm10                        ;

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        sbis    GPIOR0,H2ODET               ;H2O detection?
        rjmp    tm13                        ;no, jump

        mov     a5,tic                      ;sound buzzer   -.-.-.
        andi    a5,(1<<3)-1                 ;and blink LED
        brne    PC+3                        ;
        rcall   buzOn                       ;
        cbi     CTRLDP,LED                  ;
        cpi     a5,(1<<2)                   ;
        brne    diShowTimer                 ;

tm13:   rcall   buzOff                      ;buzzer off
        sbi     CTRLDP,LED                  ;LED on

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

diShowTimer:
        tst     dishti
        breq    checkKey

        dec     dishti
        brne    PC+2
        rcall   clearDevId

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

checkKey:
        sbic    CTRLDPP,KEY                 ;key pressed?
        rjmp    keyRel                      ;no, key released

        inc     keypc                       ;limit to 255
        brne    PC+2                        ;
        dec     keypc                       ;

        ldi     a4,KEYLONGT                 ;long press?
        cp      keypc,a4                    ;
        brne    ky09                        ;no, exit

;- - - - - - - - - - - - - - - - - - - -

keyLong:
        ret

;- - - - - - - - - - - - - - - - - - - -

keyRel: tst     keypc                       ;>= 50 ms
        breq    ky09

        ldi     a4,KEYLONGT                 ;long press already executed?
        cp      keypc,a4                    ;
        breq    ky08                        ;yes, jump

;- - - - - - - - - - - - - - - - - - - -

keyShort:
        tst     dishti                      ;device ID show time down?
        breq    scShowDevice                ;yes, show it

        ldi     a4,RXS_NUM                  ;switch devices
        inc     selDev                      ;
        cp      selDev,a4                   ;
        brlo    PC+2                        ;
        clr     selDev                      ;

scShowDevice:                                  
        mov     YL,selDev                   ;calc pointer to selected device struct 
        lsl     YL                          ;* RXS_LEN
        lsl     YL                          ;
        lsl     YL                          ;
        subi    YL,-(rxStr)                 ;+ struct pointer
        
        tst     dishti                      ;device ID show time down? 
        breq    showDevId                   ;yes, show it

        ldd     a4,Y+7                      ;load flags
        ori     a4,(1<<DRX)                 ;fake receive to schedule show
        std     Y+7,a4                      ;store flags

;- - - - - - - - - - - - - - - - - - - -

showDevId:
        push    XL                          ;save                   show device ID
        ldi     XL,ascDig                   ;text buffer pointer

        ld      a4,Y                        ;device ID
        rcall   bin1Dec2LZ                  ;

        st      X+,NULR                     ;termination

        ldi     a4,128-2*8                  ;x pos
        ldi     a5,7                        ;y
        mov     oledX,a4                    ;
        mov     oledY,a5                    ;
        ldi     ZL,low(oled_88<<1)          ;font
        ldi     ZH,high(oled_88<<1)         ;        
        rcall   oledTextSR                  ;

        pop     XL                          ;restore

        ldi     dishti,DISHTIME             ;start ID show time

;- - - - - - - - - - - - - - - - - - - -

ky08:   clr     keypc                       ;reset key press counter
ky09:   ret                                 ;

;-------------------------------------------------------------------------------

clearDevId:
        ldi     a4,128-2*8                  ;x pos
        ldi     a5,7                        ;y
        rcall   oledXYs                     ;set xy  

        rcall   oledData                    ;send data
        ldi     a4,2*8                      ;num bytes
cd01:   ldi     c4,0                        ;zero
        rcall   i2cWrite                    ;write
        dec     a4                          ;all bytes?
        brne    cd01                        ;no, jump

        rjmp    i2cStop                     ;i2c stop

;-------------------------------------------------------------------------------

showStatus:
        mov     c5,c4                       ;copy timers
        mov     c6,a6                       ;     flags
        andi    a6,~(1<<SYM1|1<<SYM0)       ;reset symbol flags

        ldi     a5,0x00                     ;assume signal timer running    space
        andi    c4,0x0f                     ;running?
        brne    so01                        ;yes, jump

        ldi     a5,0x02                     ;assume device selected         device selected
        cpse    a7,selDev                   ;selected device?
        ldi     a5,0x01                     ;no                             device unselected

        andi    c5,0xf0                     ;RX timer down?
        brne    so01                        ;no, jump                     

        sbrc    tic,3                       ;toggle symbol
        ldi     a5,0x00                     ;                               space

;- - - - - - - - - - - - - - - - - - - -
        
so01:   or      a6,a5                       ;update symbol flags
        cp      a6,c6                       ;flags changed?
        breq    ky09                        ;no, exit

        push    YL                          ;save struct pointer
        push    a6                          ;            flags            
        push    a7                          ;device num 

        ldi     a4,' '                      ;space
        cpi     a5,0x00                     ;
        breq    so02                        ;

        ldi     a4,CHRDEV                   ;device unselected
        cpi     a5,0x01                     ;
        breq    so02                        ;

        ldi     a4,CHRDEVSEL                ;device selected

so02:   ldi     a5,120                      ;show symbol
        mov     oledX,a5                    ;y pos
        lsl     a7                          ;
        mov     oledY,a7                    ;x
        rcall   oledTextR                   ;

        rjmp    sv09                        ;restore and exit

;---------------------------------------

showSelTemp:
        push    YL                          ;save
        push    a6                          ;
        push    a7                          ;

        ldd     a4,Y+2                      ;Tb
        ldd     a5,Y+3                      ;
        rcall   checkVal                    ;
        brne    sv02                        ;

        ldd     a0,Y+1                      ;deviceStatus
        sbrs    a0,TEMDS18B20               ;DS18B20 temperature sensor?
        rjmp    sv01
        rcall   readTempDS                  ;°C in a4.a7, sign in YH SREG_C
        rjmp    sv02

sv01:   rcall   readTemp                    ;°C in a4.a7, sign in YH SREG_C
sv02:   rcall   showTemp                    ;show
        rjmp    sv09                        ;restore and exit

;---------------------------------------

showSelHumi:
        push    YL                          ;save
        push    a6                          ;
        push    a7                          ;

        ldd     a4,Y+4                      ;Hb
        ldd     a5,Y+5                      ;
        rcall   checkVal                    ;
        brne    sv03                        ;

        rcall   readHumi                    ;RH in a4.a7
sv03:   rcall   showHumi                    ;show

sv09:   pop     a7                          ;restore
        pop     a6                          ;
        pop     YL                          ;
        ret

;---------------------------------------

checkVal:
        clz                                 ;assume error
        sbrs    a6,FIR                      ;first receive?
        ret                                 ;no, error

        ldi     a7,0xff                     ;prepare cpse
        cpse    a4,a7                       ;LB != 0xff
        sez                                 ;yes, ok
        cpse    a5,a7                       ;HB != 0xff
        sez                                 ;yes, ok

        ret

;---------------------------------------
;a5              a4
;7 6 5 4 3 2 1 0 7 6 5 4 3 2 1 0
;s s s s s b b b b b b b a a a a            s sign, b before point, a after point 1/16 °C

readTempDS:
        mov     a7,a4                       ;copy a
        andi    a7,0x0f                     ;0000 aaaa

        lsr     a5                          ;0sss ssbb
        ror     a4                          ;bbbb baaa
        lsr     a5                          ;00ss sssb
        ror     a4                          ;bbbb bbaa
        lsr     a5                          ;000s ssss
        ror     a4                          ;bbbb bbba
        lsr     a4                          ;0bbb bbbb

        mov     YH,a5                       ;000s ssss, sign -> SREG_C

        mov     a5,a7                       ;a * 5 / 8
        lsl     a7                          ;a * 4
        lsl     a7                          ;
        add     a7,a5                       ;+ a
        lsr     a7                          ;/ 8
        lsr     a7                          ;
        lsr     a7                          ;
        brcc    PC+2                        ;round
        inc     a7                          ;

        sez                                 ;ok
        ret

;-------------------------------------------------------------------------------

genCheckSum:
        clr     a0                          ;reset checksum
        clr     a1

        ldi     ZL,rxDat                    ;build checksum from received data
gc01:   ld      a4,Z+
        rcall   checkSum
        cpi     ZL,(rxDat+RXD_LEN)
        brne    gc01

        rcall   gc02                        ;add received checksum

gc02:   ld      a4,Z+                       ;load checksum byte
        rjmp    checkSumNA                  ;no add address

;- - - - - - - - - - - - - - - - - - - -

checkSum:
        add     a4,ZL                       ;add address (must be same address on TX side)

checkSumNA:
        add     a4,a0                       ;a0 = (a0 + rxDat) % 0xff
        rcall   mod255
        mov     a0,a4

        add     a4,a1                       ;a1 = (a1 + a0) % 0xff
        rcall   mod255
        mov     a1,a4

        ret

;---------------------------------------

mod255: adc     a4,NULR                     ;modulo 255 (limited)
        cpi     a4,255
        brne    PC+2
        inc     a4
        ret

;-------------------------------------------------------------------------------

buzz:   rcall   buzOn                       ;sound buzzer on start

        in      a4,TIFR                     ;T1 OCF1A event?    100 ms delay
        sbrs    a4,OCF1A                    ;yes, jump
        rjmp    PC-2                        ;

        ldi     a4,(1<<OCF1A)               ;reset flag 
        out     TIFR,a4                     ;

buzOff: out     TCCR0B,NULR                 ;stop T0
        out     TCCR0A,NULR                 ;normal mode, OC0B L
        ret

buzOn:  ldi     a4,(1<<COM0B0|1<<WGM01)     ;OC0B toggle, CTC
        out     TCCR0A,a4
        ldi     a4,(1<<CS01)                ;div 8
        out     TCCR0B,a4                   ;start T0
        ret

;-------------------------------------------------------------------------------
;3 + (a6 * ((a5 * 3) + 3)) + 4, 20000 cycles @ 4 MHz = 5 ms
;
;.equ    DLX     =    28
;.equ    DLY     =    237
;
;wait5xms:
;       ldi     a6,DLX
;       ldi     a5,DLY
;       dec     a5
;       brne    PC-1
;       dec     a6
;       brne    PC-4
;       dec     a4
;       brne    wait5xms
;
;       ret

;-------------------------------------------------------------------------------

;sendData:
;        ldi     a4,low(UBRRVAL96)           ;serial transmission paramamters
;        ldi     a5,high(UBRRVAL96)
;        out     UBRRH,a5
;        out     UBRRL,a4
;
;        ldi     ZL,rxDat                    ;send
;        ld      a4,Z+
;        rcall   txByte
;        cpi     ZL,rxDat+7
;        brne    PC-3
;
;        sbis    UCSRA,TXC                   ;all bits shifted out?
;        rjmp    PC-1                        ;no, wait
;
;        sbi     UCSRA,TXC                   ;manually reset TXC
;
;        ldi     a4,low(UBRRVAL)             ;serial transmission paramamters
;        ldi     a5,high(UBRRVAL)
;        out     UBRRH,a5
;        out     UBRRL,a4
;
;        ret
;
;;---------------------------------------
;
;txByte:
;        sbis    UCSRA,UDRE                  ;ready for next data?
;        rjmp    PC-1                        ;no, wait
;
;        out     UDR,a4                      ;transmit
;        ret

;-------------------------------------------------------------------------------

;        push    ZL                          ;test store to eeprom
;
;        ldi     a5,0
;        ldi     ZL,rxDat+2
;        ld      a4,Z+
;        rcall   eewrite
;        cpi     ZL,rxDat+5
;        brne    PC-3
;
;        pop     ZL

;eewrite:
;        sbic    EECR,EEWE                   ;wait for last write
;        rjmp    eewrite
;
;        out     EEAR,a5                     ;set up address
;        out     EEDR,a4                     ;write rxDat
;;       cli
;        sbi     EECR,EEMWE                  ;write logical one to EEMWE         |
;        sbi     EECR,EEWE                   ;start eeprom write by setting EEWE |
;;       sei
;        inc     a5
;        ret
