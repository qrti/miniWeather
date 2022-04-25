;show.inc V0.5 QRT211226, qrt@qland.de
;

showTemp:
        mov     oledX,NULR                  ;x, clr would change Z flag
        mov     oledY,NULR                  ;y
        ldi     c4,CHRDEGCEL                ;symbol °C

        push    XL                          ;save
        ldi     XL,ascDig                   ;init text buffer pointer
        brne    showNV                      ;error? yes, show no value

        mov     a0,a7                       ;°C a4.a7 -> a4.a0, sign in YH
        ldi     a6,'+'                      ;sign
        sbrc    YH,SREG_C                   ;
        ldi     a6,'-'                      ;

        cp      a4,NULR                     ;a4.a0 == 0?
        cpc     a0,NULR                     ;
        brne    PC+2                        ;no, - +

;- - - - - - - - - - - - - - - - - - - -

showV:  ldi     a6,' '                      ;space
        st      X+,a6                       ;space - +

        rcall   bin1Dec2LS                  ;before point in a4
        ldi     a4,'.'                      ;point
        st      X+,a4                       ;
        mov     a4,a0                       ;after point
        rcall   bin1Dec1                    ;

;- - - - - - - - - - - - - - - - - - - -

showS:  st      X+,c4                       ;symbol
        st      X+,NULR                     ;termination

        ldi     ZL,low(oled_1424<<1)        ;font table start
        ldi     ZH,high(oled_1424<<1)       ;
        rcall   oledTextSR                  ;show
        pop     XL                          ;restore
        ret

;-------------------------------------------------------------------------------

showHumi:
        mov     oledX,NULR                  ;x, clr would change Z flag
        ldi     a6,4                        ;y
        mov     oledY,a6                    ;
        ldi     c4,CHRPERCRH                ;symbol RH

        mov     a0,a7                       ;copy after point value

        push    XL                          ;save
        ldi     XL,ascDig                   ;init text buffer pointer
        brne    showNV                      ;error? yes, show no value

        tst     a4                          ;H >= 0?
        brpl    sh01                        ;yes, jump
        clr     a4                          ;a4:a0 = 0:0
        clr     a0                          ;

sh01:   cpi     a4,100                      ;H < 100?
        brlo    sh02                        ;yes, jump
        ldi     a4,100                      ;a4:a0 = 100:0
        clr     a0                          ;

sh02:   rjmp    showV                       ;show value

;---------------------------------------

showNV: ldi     a4,' '                      ;
        st      X+,a4                       ;space
        ldi     a5,'-'                      ;
        st      X+,a5                       ;-
        st      X+,a5                       ;-
        ldi     a4,'.'                      ;
        st      X+,a4                       ;.
        st      X+,a5                       ;-
        rjmp    showS                       ;show symbol

;-------------------------------------------------------------------------------

bin1Dec2LZ:                                 ;00..99
        rcall   b1d2                        ;dig 1
        rjmp    b1d2a                       ;store

bin1Dec2LS:                                 ; 0..99
        rcall   b1d2                        ;dig 1

        cpi     a7,'0'                      ;leading space
        brne    PC+2                        ;
        ldi     a7,' '                      ;

b1d2a:  st      X+,a7                       ;store

;- - - - - - - - - - - - - - - - - - - -

b1d1:   ldi     a7,'0'+10                   ;dig 0
        add     a7,a4                       ;
        st      X+,a7                       ;
        ret

;---------------------------------------

bin1Dec1:                                   ;0..9
        subi    a4,10                       ;
        rjmp    b1d1                        ;dig 0

;---------------------------------------

 b1d2:  ldi     a7,'0'-1                    ;dig 1
        inc     a7                          ;
        subi    a4,low(10)                  ;
        brcc    PC-2                        ;

        ret
