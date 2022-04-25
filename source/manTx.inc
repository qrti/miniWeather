;manTx.asm, manchester coded transmission, V0.6 QRT211226, qrt@qland.de
;

sendData:   
        ldi     c7,TXREPEAT                 ;transmit repeat
ws01:   rcall   txStart                     ;command start

        ldi     ZL,txDat                    ;send txDat + checksum
        ld      a4,Z+
        rcall   sendByteE
        cpi     ZL,(txDat+DATALEN+CHKLEN)
        brne    PC-3

        rcall   txStop                      ;command stop

        dec     c7                          ;all repetitions?
        breq    ws09                        ;yes, exit

        ldi     a4,50                       ;wait between repetitions
        rcall   wait5xms                    ;250 ms

        rjmp    ws01                        ;repeat send

ws09:   ret

;-------------------------------------------------------------------------------

txStart:
;       sbi     CTRLDP,TXD                  ;TX-module wakeup                
        sbi     UCSRB,TXEN                  ;enable TX, 8 bits, 1 stop, no parity, default in UCSRC
        rcall   wait10ms                    ;wait 10 ms for powerup

;- - - - - - - - - - - - - - - - - - - -

txCmd:
        ldi     a5,CMDCODE                  ;command code

;- - - - - - - - - - - - - - - - - - - -
;send order START 0 1 2 3 4 5 6 7 STOP

txByte:
        sbis    UCSRA,UDRE                  ;ready for next txDat?
        rjmp    PC-1                        ;no, wait

        out     UDR,a5                      ;transmit
        ret

;---------------------------------------

txStop:
        rcall   txCmd                       ;send command

        sbis    UCSRA,TXC                   ;all bits shifted out?
        rjmp    PC-1                        ;no, wait

        sbi     UCSRA,TXC                   ;manually reset TXC

        cbi     UCSRB,TXEN                  ;disable TX
;       cbi     CTRLDP,TXD                  ;TX-module sleep
        ret

;-------------------------------------------------------------------------------

sendWordE:
        mov     a4,a1                       ;send HB
        rcall   sendByteE

        mov     a4,a0                       ;send LB

;- - - - - - - - - - - - - - - - - - - -

sendByteE:
        ldi     a7,2                        ;2 half bytes
enc4:   ldi     a6,4                        ;4 bits

en01:   lsr     a5                          ;next 2 bits, manchester coding
        lsr     a5                          ;

        sbrs    a4,0                        ;txDat bit0==0
        ori     a5,0b10000000               ;0->1 from right to left
        sbrc    a4,0                        ;txDat bit0==1
        ori     a5,0b01000000               ;1->0 from right to left

        lsr     a4                          ;txDat bit 0..7
        dec     a6                          ;next 2 bits
        brne    en01                        ;

        rcall   txByte                      ;3322 1100, 7766 5544

        dec     a7                          ;next half byte
        brne    enc4                        ;

        ret

;-------------------------------------------------------------------------------

genCheckSum:
        clr     a0                          ;reset checksum
        clr     a1

        ldi     ZL,txDat                  
gc01:   ld      a4,Z+
        rcall   checkSum
        cpi     ZL,(txDat+DATALEN)
        brne    gc01

        rcall   gc02                        ;(a1 in a4), c0 = 0xff - ((a0 + a1) % 0xff)

gc02:   add     a4,a0                       ;(c0 in a4), c1 = 0xff - ((a0 + c0) % 0xff)
        rcall   mod255
        com     a4
        st      Z+,a4
        ret

;- - - - - - - - - - - - - - - - - - - -

checkSum:
        add     a4,ZL                       ;add address (must be same address on RX side)

checkSumNA:
        add     a4,a0                       ;a0 = (a0 + txDat) % 0xff
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
