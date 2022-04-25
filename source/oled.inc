;oled.inc V0.5 QRT211226, qrt@qland.de
;ssd1306 oled controller

.define OLED_I2C_ADDR               0x78

.define OLCOMA                      0x00
.define OLDATA                      0x40

.define SET_CONTRAST_CONTROL        0x81
.define SET_DISPLAY_OFF             0xae
.define SET_DISPLAY_ON              0xaf
.define SET_DISPLAY_NORMAL          0xa6
.define SET_DISPLAY_INVERSE         0xa7

.define ENTIRE_DISPLAY_FORAM        0xa4    ;display follow RAM
.define ENTIRE_DISPLAY_IGRAM        0xa5    ;        ignore

.define SET_MEMORY_ADDR_MODE        0x20
.define HORIZONTAL_ADDRESSING_MODE  0x00
.define VERTICAL_ADDRESSING_MODE    0x01
.define PAGE_ADDRESSING_MODE        0x02

.define SET_PAGE_START_ADDRESS      0xb0    ;no default, init page start 0
.define SET_COLUMN_ADDRESS          0x21    
.define SET_PAGE_ADDRESS            0x22    ;default start 0, end 7, no init

.define SET_COM_OUTPUT_NORMAL       0xc0
.define SET_COM_OUTPUT_REMAPPED     0xc8

.define SET_DISPLAY_OFFSET          0xd3
.define SET_DISPLAY_CLOCK           0xd5
.define SET_PRECHARGE_PERIOD        0xd9
.define SET_COM_PINS                0xda

.define SET_VCOMH_DESELECT          0xdb    ;0x00, 0x20, 0x30

.define SET_LOWER_COLUMN            0x00
.define SET_HIGHER_COLUMN           0x10
.define SET_DISPLAY_START_LINE      0x40    ;default 0, init 0

.define SET_SEGMENT_REMAP_O1        0xa0    
.define SET_SEGMENT_REMAP_O2        0xa1

.define SET_MULTIPLEX_RATIO         0xa8
.define COMMAND_NO_OPERATION        0xe3

.define CHARGE_BUMB_SETTING         0x8d

;-------------------------------------------------------------------------------

.define INITLEN     29

ssd1306Init:
.DB     SET_DISPLAY_OFF, COMMAND_NO_OPERATION, SET_MEMORY_ADDR_MODE, HORIZONTAL_ADDRESSING_MODE
.DB     SET_PAGE_START_ADDRESS, SET_COM_OUTPUT_REMAPPED, SET_LOWER_COLUMN, SET_HIGHER_COLUMN
;.DB     SET_COLUMN_ADDRESS, 0, 127, SET_PAGE_ADDRESS, 0, 7
.DB     SET_DISPLAY_START_LINE, ENTIRE_DISPLAY_FORAM, SET_SEGMENT_REMAP_O2, SET_DISPLAY_NORMAL
.DB     SET_CONTRAST_CONTROL,   0xff
.DB     SET_MULTIPLEX_RATIO,    0x3f
.DB     SET_DISPLAY_OFFSET, 	0x00 
.DB     SET_DISPLAY_CLOCK,      0xf0
.DB     SET_PRECHARGE_PERIOD,   0x22 
.DB     SET_COM_PINS,           0x12
.DB     SET_VCOMH_DESELECT,	    0x20 
.DB     CHARGE_BUMB_SETTING,    0x14
.DB     SET_DISPLAY_ON, 0

;-------------------------------------------------------------------------------

oledInit:
        rcall   oledCmd       

        ldi     ZL,low(ssd1306Init<<1)
        ldi     ZH,high(ssd1306Init<<1)
        ldi     a4,INITLEN
oi01:   lpm     c4,Z+
        rcall   i2cWrite
        dec     a4
        brne    oi01

        rcall   i2cStop

;- - - - - - - - - - - - - - - - - - - -

oledClear:
        rcall   oledXY0
        rcall   oledData

        ldi     a4,low(128<<3)
        ldi     a5,high(128<<3)
oc01:   ldi     c4,0                 
        rcall   i2cWrite
        sbiw    a5:a4,1
        brne    oc01        

        rjmp    i2cStop

;-------------------------------------------------------------------------------

oledXY0:
        ldi     a4,0
        ldi     a5,0

oledXYs:
        mov     oledX,a4
        mov     oledY,a5

oledXY:
        rcall   oledCmd     

        ldi     c4,SET_PAGE_START_ADDRESS       ;y 0..7
        or      c4,a5                           ;page start y
        rcall   i2cWrite                        ;

        ldi     c4,SET_COLUMN_ADDRESS           ;x 0..127
        rcall   i2cWrite

        mov     c4,a4                           ;column start  
        rcall   i2cWrite

        ldi     c4,0x7f                         ;       end
        rcall   i2cWrite

        rjmp    i2cStop

;-------------------------------------------------------------------------------

oledData:
        ldi     c4,OLED_I2C_ADDR
        rcall   i2cStart
        ldi     c4,OLDATA
        rjmp    i2cWrite

oledCmd:
        ldi     c4,OLED_I2C_ADDR
        rcall   i2cStart
        ldi     c4,OLCOMA
        rjmp    i2cWrite

;-------------------------------------------------------------------------------

oledTextR:
        ldi     ZL,low(oled_88<<1)      ;font
        ldi     ZH,high(oled_88<<1)     ;

        mov     c0,ZL                   ;save font table start
        mov     c1,ZH                   ;
        rjmp    oledChar                ;show char

;-------------------------------------------------------------------------------

oledTextSR:
        ldi     XL,ascDig               ;text buffer pointer
        mov     c0,ZL                   ;save font table start
        mov     c1,ZH                   ;

ot01:   ld      a4,X+                   
        tst     a4
        breq    oh09
        mov     ZL,c0                   ;restore font table start
        mov     ZH,c1                   ;
        rcall   oledChar
        rjmp    ot01

;-------------------------------------------------------------------------------

oledChar:
        lpm     a6,Z+                   ;font hight     (+0)  
        lpm     a0,Z+                   ;     spacing   (+1) 

        adiw    ZH:ZL,15                ;char start     (+17)
oh01:   lpm     a5,Z+                   ;     asc       (+18)
        tst     a5                      ;end of chars?
        breq    oh09                    ;yes, exit
        cp      a4,a5                   ;char found?
        breq    oh02                    ;yes, jump
        adiw    ZH:ZL,1                 ;overread char width
        lpm     a5,Z+                   ;char data len
        add     ZL,a5                   ;pointer to next char
        adc     ZH,NULR                 ;
        rjmp    oh01                    ;next char

;- - - - - - - - - - - - - - - - - - - -

oh02:   lpm     a1,Z+                   ;char width
        adiw    ZH:ZL,1                 ;overread char data len
        clr     a7                      ;repeat counter
        dec     a6                      ;font height -= 1

        mov     a4,oledX                ;get x
        mov     a5,oledY                ;    y
oh03:   rcall   oledXY                  ;set xy  
        rcall   oledData                ;start data
        
        mov     a2,a1                   ;char width copy
oh04:   rcall   deComp                  ;get next byte
        rcall   i2cWrite                ;write
        dec     a2                      ;char width copy --
        brne    oh04                    ;==0?, no jump

        rcall   i2cStop                 ;stop

        inc     a5                      ;y++             
        subi    a6,8                    ;font height -= 8
        brcc    oh03                    ;>=0? yes, jump       

        add     oledX,a1                ;x += char width
        add     oledX,a0                ;x += spacing

oh09:   ret

;-------------------------------------------------------------------------------

deComp:
        tst     a7                      ;repeat counter down?
        breq    dc02                    ;yes, jump
        brmi    dc05                    ;raw byte? yes, jump

        dec     a7                      ;repeat counter --
dc01:   mov     c4,a3                   ;repeat byte to i2c register
        ret        

dc02:   lpm     a7,Z+                   ;get byte
        cpi     a7,0xf0                 ;frequent byte?
        brsh    dc03                    ;no, jump

        push    ZL                      ;save data pointer
        push    ZH                      ;
        mov     ZL,c0                   ;font table start
        mov     ZH,c1                   ;
        adiw    ZH:ZL,2                 ;freq bytes pointer (+2)        
        mov     c4,a7                   ;copy byte
        andi    c4,0xf0                 ;byte number only
        swap    c4                      ;HB->LB
        add     ZL,c4                   ;add
        adc     ZH,NULR                 ;
        lpm     a3,Z                    ;get freq byte
        pop     ZH                      ;restore data pointer
        pop     ZL                      ;
        andi    a7,0x0f                 ;repeat counter
        rjmp    dc01                    ;repeat byte to i2c register
        
dc03:   sbrc    a7,3                    ;byte repeat?
        rjmp    dc04                    ;no, jump
        
        lpm     a3,Z+                   ;store repeat byte
        andi    a7,0x07                 ;repeat counter + 1
        inc     a7                      ;(first repeat already done)
        rjmp    dc01                    ;repeat byte to i2c register

dc04:   andi    a7,0x07                 ;raw byte
        inc     a7                      ;
        neg     a7                      ;

dc05:   inc     a7                      ;repeat counter ++
        lpm     c4,Z+                   ;byte to i2c register
        ret
