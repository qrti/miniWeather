;sensor.inc V0.5 QRT211216, qrt@qland.de
;GY-21 temperature and humidity sensor

.define GY21_I2C_ADDR           0x40

.define GY21_READ_TEMP          0xf3
.define GY21_READ_HUMI          0xf5
.define GY21_READ_TEMP_HOLD     0xe3
.define GY21_READ_HUMI_HOLD     0xe5
.define GY21_WRITE_USER_REG     0xe6
.define GY21_READ_USER_REG      0xe7
.define GY21_SOFT_RESET         0xfe

;.define SENSE                   1
;.define CONVERT                 2
;.define FUNC                    (SENSE)

;-------------------------------------------------------------------------------

.if FUNC & SENSE
readVal:
        ldi     c4,(GY21_I2C_ADDR<<1)   ;I2C start
        rcall   i2cStart                ;ok?
        brne    exit                    ;no, error exit

        mov     c4,a4                   ;read temp or humi
        rcall   i2cWrite

        ldi     c4,(GY21_I2C_ADDR<<1 | 1)   ;wait for result
        rcall   i2cStart                    ;about 50 ms @ 14-bit accuracy
        brne    PC-2                        ;

        ldi     c6,3                ;3 byte
        rcall   i2cRead             ;HB
        mov     a5,c4               ;save
        rcall   i2cRead             ;LB
        mov     a4,c4               ;save
        rcall   i2cRead             ;checksum in c4
        rcall   i2cStop             ;I2C stop

        rcall   checkCRC8           ;CRC ok?
        brne    exit                ;no, error exit

;- - - - - - - - - - - - - - - - - - - -

        andi    a4,0xfc             ;value in a5:a4, vvvv vvvv vvvv vvss, v value s status
        sez                         ;Z=1 ok
exit:   ret                         ;Z=0 error, Z=1 ok

;-------------------------------------------------------------------------------

.define CRC8_POLY   0x13100         ;crc8 polynomial for 16-bit value, CRC8 -> x^8 + x^5 + x^4 + 1

checkCRC8:
        ldi     c6,low(CRC8_POLY)
        ldi     c7,high(CRC8_POLY)

        mov     a6,a4
        mov     a7,a5

        ldi     XH,16
cr01:   lsl     a6
        rol     a7
        brcc    PC+3
        eor     a6,c6
        eor     a7,c7
        dec     XH
        brne    cr01

        cp      c4,a7               ;error Z=0, ok Z=1
        ret
.endif

;-------------------------------------------------------------------------------
; T = 175.72 * T / 2^16 - 46.85
;
readTemp:
.if FUNC == SENSE
        ldi     a4,GY21_READ_TEMP   ;temperature
        rjmp    readVal             ;read

.elif FUNC == (SENSE | CONVERT)
        ldi     a4,GY21_READ_TEMP   ;temperature
        rcall   readVal             ;read
        brne    exit                ;ok? no, error exit
.endif

.if FUNC & CONVERT
        mov     tbinL,a4            ;store Tb
        mov     tbinH,a5            ;

        ldi     a6,low(17572)       ;Tr = Tb * 17572
        ldi     a7,high(17572)      ;
        rcall   mul16u              ;a5:a4 * a7:a6 -> a3:a2:a1:a0

        ldi     a4,0                ;4685 << 16, Tc
        ldi     a5,0                ;
        ldi     a6,low(4685)        ;
        ldi	    a7,high(4685)       ;

        rcall   vrmvc               ;T = Tr - Tc
        in      YH,SREG             ;save sign
        brcc    corVal              ;T < 0? no, jump    C=0 sign +

        sbc     a0,NULR             ;T < 0              C=1 sign -
        sbc     a1,NULR             ;T = -T, neg32
        sbc     a2,NULR             ;
        sbc     a3,NULR             ;
        com     a0                  ;
        com     a1                  ;
        com     a2                  ;
        com     a3                  ;

;- - - - - - - - - - - - - - - - - - - -

corVal: mov     a4,a2               ;V >>= 16, correct value
        mov     a5,a3               ;
        ldi     a6,10               ;
        rcall   div16_8u            ;V /= 10

        cpi     a7,5                ;reminder >= 5?
        brlo    PC+2                ;no, jump
        adiw    a5:a4,1             ;V = round(V / 10)
        rjmp    div16_8u            ;V /= 10, V in a4.a7, sign in YH, Z=1 ok
.endif

;-------------------------------------------------------------------------------
;humi = H - 0.15 * (25 - T)         compensated
;     = (125 * Hb + 26.358 * Tb - 16.7775 * 2^16) / 2^16
;     = (12500 * Hb + 2636 * Tb - 0x068dc000) / 2^16 / 100
;Tb   = (T + 46.85) / 175.72 * 2^16
;Hb   = (H + 6) / 125 * 2^16
;
.define TBIN0       0x4441          ;Tbin  0 °C
.define TBIN80      0xb8cd          ;     80 °C
.define HUMIC       0x068dc000      ;16.7775 * 100 * 2^16
.define HBIN100     0xd916          ;Hbin 100

readHumi:
.if FUNC == SENSE
        ldi     a4,GY21_READ_HUMI   ;humidity
        rjmp    readVal             ;read

.elif FUNC == (SENSE | CONVERT)
        ldi     a4,GY21_READ_HUMI   ;humidity
        rcall   readVal             ;read
        brne    exit                ;ok? no, error exit
.endif

.if FUNC & CONVERT
        ldi     a6,low(12500)       ;Hr = Hb * 12500
        ldi     a7,high(12500)      ;
        rcall   mul16u              ;a5:a4 * a7:a6 -> a3:a2:a1:a0

        mov     a4,tbinL            ;load Tb
        mov     a5,tbinH            ;

        ldi     c5,high(TBIN0)      ;T > 0?
        cpi     a4,low(TBIN0)       ;
        cpc     a5,c5               ;
        brlo    rh01                ;no, regular calculation

        ldi     c5,high(TBIN80)     ;T < 80?
        cpi     a4,low(TBIN80)      ;
        cpc     a5,c5               ;
        brsh    rh01                ;no, regular calculation

        mov     c0,a0               ;save Hr
        mov     c1,a1               ;
        mov     c2,a2               ;
        mov     c3,a3               ;

        ldi     a6,low(2636)        ;Tf = Tb * 2636
        ldi     a7,high(2636)       ;
        rcall   mul16u              ;a5:a4 * a7:a6 -> a3:a2:a1:a0

        add     a0,c0               ;Hr + Tf
        adc     a1,c1               ;
        adc     a2,c2               ;
        adc     a3,c3               ;

        ldi     a4,low(HUMIC)       ;Hc
        ldi     a5,high(HUMIC)      ;
        ldi     a6,low(HUMIC>>16)   ;
        ldi     a7,high(HUMIC>>16)  ;
        rjmp    rh02

;-------------------------------------------------------------------------------
;humi = 125 * H / 2^16 - 6          regular
;
rh01:
        ldi     a4,0                ;600 << 16
        ldi     a5,0                ;
        ldi     a6,low(600)         ;
        ldi     a7,high(600)        ;

rh02:   rcall   vrmvc               ;H = Hr - Hc
        rcall   corVal              ;correct value, V in a4.a7, Z=1 ok

        tst     a4                  ;H<0? restrict
        brpl    PC+3                ;
        ldi     a4,0                ;yes, H=0.0
        clr     a7                  ;

        cpi     a4,100              ;H>=100?
        brlo    PC+3                ;                            
        ldi     a4,99               ;yes, H=99.9
        ldi     a7,9                ;

        sez                         ;Z=1, ok
        ret

;---------------------------------------

vrmvc:  sub     a0,a4               ;V = Vr - Vc
        sbc     a1,a5               ;
        sbc     a2,a6               ;
        sbc     a3,a7               ;
        ret

;-------------------------------------------------------------------------------

;c4 * c5 -> c5:c4
;
.def    muld    =   r20             ;multiplicand    c6 <- c5
.def    mulr    =   r18             ;multiplier         <- c4
.def    resL    =   r18             ;result low byte    -> c4
.def    resH    =   r19             ;result high byte   -> c5
.def    loopc   =   r21             ;loop counter          c7

mul8u:
        mov     muld,resH           ;copy multiplicand

        clr     resH                ;clear 2 highest bytes of result
        ldi     loopc,8             ;init loop counter
        ror     mulr                ;rotate right multiplier
mu1:    brcc    mu2                 ;if bit 0 of multiplier set
        add     resH,muld           ;add multiplicand high to byte 3 of res
mu2:    ror     resH                ;shift right result high byte
        ror     resL                ;rotate right result low byte
        dec     loopc               ;decrement loop counter
        brne    mu1                 ;all done?

        ret                         ;return

;-------------------------------------------------------------------------------
;a7:a6 * a5:a4 -> a3:a2:a1:a0       B1:A1 * B2:A2 = D0:C0:B0:A0

mul16u:
        mov     c4,a5               ;B1 * B2
        mov     c5,a7
        rcall   mul8u
        mov     a2,c4               ;-> DO:C0
        mov     a3,c5

        mov     c4,a4               ;A1 * A2
        mov     c5,a6
        rcall   mul8u
        mov     a0,c4               ;-> B0:A0
        mov     a1,c5

        mov     c4,a4               ;B1 *  A2
        mov     c5,a7
        rcall   mul8u
        rcall   mu3

        mov     c4,a5               ;A1 * B2
        mov     c5,a6
        rcall   mul8u
mu3:    add     a1,c4               ;+ D0:C0:B0
        adc     a2,c5
        adc     a3,NULR

mu9:    ret

;-------------------------------------------------------------------------------

.def    dvqL    =   r24             ;dividend + quotient LB     (a4)
.def    dvqH    =   r25             ;                    HB     (a5)
.def    dvs     =   r16             ;divisor                    (a6)
.def    rem     =   r17             ;remainder                  (a7)
.def    cnt     =   r31             ;counter                    (ZH)

div16_8u:
        clr     rem                 ;clear remainder
        ldi     cnt,16              ;16 bits

div01:  lsl     dvqL                ;rotate dividend
        rol     dvqH                ;
        rol     rem                 ;to remainder
;       brcs    div02               ;this should never, ever, happen
        cp      rem,dvs             ;trial subtraction
        brcs    div03               ;not big enough

div02:  sub     rem,dvs             ;ok, subtract it
        inc     dvqL                ;add this power of 2 to quotient

div03:  dec     cnt                 ;count bit
        brne    div01               ;do until done

        ret                         ;Z=1
.endif

;-------------------------------------------------------------------------------

;div16u - 16/16 bit unsigned division, dvq = dvq / dvs
;dividend + quotient    dvqH  : dvqL
;divisor                dvsH  : dvsL
;remainder              remH : remL
;number of words        19
;number of cycles       235/251 (min/max)
;
;.def    remL        =       r0          ;reminder LB                (a0)
;.def    remH        =       r1          ;         HB                (a1)
;.def    dvqL        =       r24         ;dividend & quotient LB     (a4)
;.def    dvqH        =       r25         ;                    HB     (a5)
;.def    dvsL        =       r16         ;divisor  LB                (a6)
;.def    dvsH        =       r17         ;         HB                (a7)
;.def    cnt         =       r31         ;counter                    (ZH)
;
;div16u:
;        clr     remL                    ;clear remainder
;        sub     remH,remH               ;                and carry
;        ldi     cnt,17                  ;init loop counter
;
;div01:  rol     dvqL                    ;shift left dividend
;        rol     dvqH
;        dec     cnt                     ;decrement counter
;        brne    div02                   ;if done
;        ret                             ;return
;
;div02:  rol     remL                    ;shift dividend into remainder
;        rol     remH
;        sub     remL,dvsL               ;remainder = remainder - divisor
;        sbc     remH,dvsH
;        brcc    div03                   ;if result negative
;
;        add     remL,dvsL               ;restore remainder
;        adc     remH,dvsH
;        clc                             ;clear carry to be shifted into result
;        rjmp    div01                   ;else
;
;div03:  sec                             ;set carry to be shifted into result
;        rjmp    div01

;-------------------------------------------------------------------------------

;neg16:  com     a5
;        neg     a4
;        sbci    a5,-1

;neg32:  subi    a4,1
;        sbci    a5,0
;        sbci    a6,0
;        sbci    a7,0
;        com     a4
;        com     a5
;        com     a6
;        com     a7

;-------------------------------------------------------------------------------

;https://github.com/enjoyneering/HTU21D
;https://github.com/adafruit/Adafruit_HTU21DF_Library
;https://github.com/JonasGMorsch/GY-21
