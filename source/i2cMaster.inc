;i2cMaster.inc V0.5 QRT211226, qrt@qland.de
;
;c4 data, c5 help, c6 read counter

.define MAX             0
.define FAST            1
.define SLOW            2
.define TWI_MODE        MAX

.equ    I2CPORT         =   PORTB
.equ    I2CPIN          =   PINB
.equ    I2CDDR          =   DDRB

.equ    SCL             =   PINB7
.equ    SDA             =   PINB5

.equ    USISR_8bit      =   (1<<USISIF|1<<USIOIF|1<<USIPF|1<<USIDC|0x0<<USICNT0)
.equ    USISR_1bit      =   (1<<USISIF|1<<USIOIF|1<<USIPF|1<<USIDC|0xE<<USICNT0)
.equ    TWI_NACK_BIT    =   0

;-------------------------------------------------------------------------------
;timing @ 4 MHz

.if TWI_MODE == MAX                     ;SCL >400 kHz  - - - - - - - - - - - - -
.macro  DELAY_T2                        ;
.endm                                   ;

.macro  DELAY_T4                        ;
.endm                                   ;

.elif TWI_MODE == FAST                  ;SCL 100-400 kHz - - - - - - - - - - - -
.macro DELAY_T2                         ;>1.3 us -> 1.5 us
        lpm     c5,Z                    ;2 * 3 = 6 / 4 = 1.5
        lpm     c5,Z                    ;
.endm                                   ;

.macro DELAY_T4                         ;>0.6 us -> 0.75 us
        lpm     c5,Z                    ;1 * 3 = 3 / 4 = 0.75
.endm                                   ;

.elif TWI_MODE == SLOW                  ;SCL <= 100 kHz  - - - - - - - - - - - -
.macro DELAY_T2                         ;>4.7 us -> 5.25 us
        ldi     c5,7                    ;1 + 6*3 - 1 = 21 / 4 = 5.25
        dec     c5                      ;
        brne    PC-1                    ;
.endm

.macro DELAY_T4                         ;>4.0 us -> 4.5 us
        ldi     c5,6                    ;1 + 6*3 - 1 = 18 / 4 = 4.5
        dec     c5                      ;
        brne    PC-1                    ;
.endm
.endif

;-------------------------------------------------------------------------------

i2cInit:
        sbi     I2CPORT,SDA             ;SDA pullup
        sbi     I2CPORT,SCL             ;SCL
        sbi     I2CDDR,SCL              ;SCL output
        sbi     I2CDDR,SDA              ;SDA

        ldi     c4,0xff
        out     USIDR,c4

        ldi     c4,(0<<USISIE|0<<USIOIE|1<<USIWM1|0<<USIWM0|1<<USICS1|0<<USICS0|1<<USICLK|0<<USITC)
        out     USICR,c4

        ldi     c4,(1<<USISIF|1<<USIOIF|1<<USIPF|1<<USIDC|0x0<<USICNT0)
        out     USISR,c4

        ret
        
;-------------------------------------------------------------------------------
;-> c4 address, uses c4 c5, Z=0 error Z=1 ok ->
;
i2cStart:
        sbi     I2CPORT,SCL

        sbis    I2CPIN,SCL
        rjmp    PC-1

.if  TWI_MODE == FAST
        DELAY_T4
.elif TWI_MODE == SLOW
        DELAY_T2
.endif

        cbi     I2CPORT,SDA
        DELAY_T4
        cbi     I2CPORT,SCL
        sbi     I2CPORT,SDA

        clz                             ;assume error
        sbis    USISR,USISIF            ;
        ret                             ;Z=0 error, Z=1 ok

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;-> c4 data, uses c4 c5, Z=0 error Z=1 ok ->
;
i2cWrite:
        cbi     I2CPORT,SCL             ;SCL=L
        out     USIDR,c4                ;data (address)

        ldi     c4,USISR_8bit           ;transfer 8 bit
        rcall   i2cTransfer             ;

        cbi     I2CDDR,SDA              ;SDA input
        ldi     c4,USISR_1bit           ;receive ACK/NACK
        rcall   i2cTransfer             ;
        andi    c4,(1<<TWI_NACK_BIT)    ;Z=0 NACK error, Z=1 ACK ok
        ret

;-------------------------------------------------------------------------------
;-> c6 num bytes, uses c4 c5 c6, c4 data ->
;
i2cRead:
       dec     c6                      ;num bytes --

       cbi     I2CDDR,SDA              ;SDA input
       ldi     c4,USISR_8bit           ;
       rcall   i2cTransfer             ;
       push    c4                      ;save data

       ldi     c4,0xff                 ;NACK, end transmission
       cpse    c6,NULR                 ;
       ldi     c4,0x00                 ;ACK, continue transmission
       out     USIDR,c4                ;

       ldi     c4,USISR_1bit           ;send ACK/NACK
       rcall   i2cTransfer             ;

       pop     c4                      ;restore data
       ret

;-------------------------------------------------------------------------------

i2cStop:
        cbi     I2CPORT,SDA
        sbi     I2CPORT,SCL

        sbis    I2CPIN,SCL
        rjmp    PC-1

        DELAY_T4
        sbi     I2CPORT,SDA
        DELAY_T2
        ret

;-------------------------------------------------------------------------------
;-> c4 data, uses c4 c5, c4 data ->
;
i2cTransfer:
        out     USISR,c4

        ldi     c4,(0<<USISIE|0<<USIOIE|1<<USIWM1|0<<USIWM0|1<<USICS1|0<<USICS0|1<<USICLK|1<<USITC)
it01:
        DELAY_T2
        out     USICR,c4

        sbis    I2CPIN,SCL              ;SCL==H?
        rjmp    PC-1                    ;no, wait

        DELAY_T4
        out     USICR,c4

        sbis    USISR,USIOIF            ;USIOIF==H?
        rjmp    it01                    ;no, jump

        DELAY_T2
        in      c4,USIDR                ;get data

        ldi     c5,0xff
        out     USIDR,c5

        sbi     I2CDDR,SDA              ;SDA output
        ret
