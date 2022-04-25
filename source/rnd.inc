;rnd.inc V0.9 QRT211217, qrt@qland.de
;

;.equ   RNDN        =   5           ;rnd width
;.equ   rtmp        =   SRAMx       ;first rtmp register        RNDN + 1 byte
;

rndSeed:                           ;5 byte + padding
.DB     137, 48, 231, 13, 160, 0

;-------------------------------------------------------------------------------
;output         a5  0..255 % 16 -> 0..15
;modifies       a5, a6, YL, ZL, ZH
;
;with div8u     input      a4      number of random elements 0..255, 0=256
;               output     a0      random number             0..a4-1, a4=0 -> a5=$ff a0=a5
;               modifies   a0, a5, a6, YL, ZL, ZH, a4 unchanged, needs 0 in register NULR

rnd:
        ldi     ZL,rtmp             ;check rtmp to be non zero
rn01:   ld      a5,Z+
        tst     a5
        brne    rnd8
        cpi     ZL,rtmp+RNDN
        brne    rn01

        ldi     ZL,low(rndSeed<<1)  ;init rtmp with rndSeed
        ldi     ZH,high(rndSeed<<1)
        ldi     YL,rtmp
rn02:   lpm     a5,Z+
        st      Y+,a5
        cpi     YL,rtmp+RNDN
        brne    rn02

;- - - - - - - - - - - - - - - - - - - -

rnd8:   ldi     ZL,rtmp             ;buffer base address
        ld      a5,Z+               ;(x[n-1] + .. + x[n-RNDN]) & $ff

        ld      a6,Z+
        add     a5,a6
        cpi     ZL,rtmp+RNDN
        brne    PC-3

        lsl     a5                  ;<<3
        adc     a5,NULR
        lsl     a5
        adc     a5,NULR
        lsl     a5
        adc     a5,NULR             ;x[n]

        ld      a6,Z                ;load cnt
        inc     a6                  ;cnt++
        cpi     a6,RNDN             ;c<RNDN?
        brlo    PC+2                ;yes, jump
        clr     a6                  ;no, cnt=0
        st      Z,a6                ;store cnt

        sub     ZL,a6               ;Z -= cnt
        st      -Z,a5               ;*(Z-cnt-1) = x[n]

;- - - - - - - - - - - - - - - - - - - -

        andi    a5,0x0f             ;a5 % 16
        ret                         ;

;- - - - - - - - - - - - - - - - - - - -
;remainder = dividend % divisor     a0
;result    = dividend / divisor     a5
;dividend                           a5
;divisor                            a4
;loop counter                       a6
;
;div8u:
;        sub     a0,a0                   ;clear remainder and carry
;        ldi     a6,9                    ;init loop counter
;d8u1:   rol     a5                      ;left shift carry to dividend/result
;        dec     a6                      ;decrement counter
;        brne    d8u2                    ;all done?
;rn09:   ret                             ;return
;
;d8u2:   rol     a0                      ;shift dividend into remainder
;        sub     a0,a4                   ;remainder = remainder - divisor
;        brcc    d8u3                    ;result negative?
;        add     a0,a4                   ;yes, remainder + divisor
;        clc                             ;clear carry to be shifted into result
;        rjmp    d8u1                    ;else
;
;d8u3:   sec                             ;set carry to be shifted into result
;        rjmp    d8u1
