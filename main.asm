;***************************************************************************
;*
;* Title: stroboscope
;* Author: Riwaz Awasthi
;* Version: 1.0
;* Last updated:11/28/2017
;* Target: ATmega324A
;*
;* DESCRIPTION
;* This system inputs a 3 bit key code code from a 74HC148 each time a
;* key is pressed. The program uses interrupt to record the positive edge at PD2(INT0), 
;* and once it finds a positive edge at PD2(where  EO is connected), it calls 
;* the interrupt service subroutine(keypress_isr). The isr keypress_isr reads the input from 
;* PORTA and shifts the position of the bits 6, 5 and 4 to 2, 1 and 0.Checks if key 7,6,5,4,3
;* 2,1 or 0 was pressed.If key 7 is pressed, the frequency is doubled whereas key 6 halves the 
;* frequency. Key 5 course increments the frequency and  key 4 course decrements the frequency.
;* Key 3 fine increments the frequency and key 2 fine decrements the frequency.
;* If key 1 is pressed, the frequency is increased by 1Hz. If key 0 is pressed, the frequency is 
;* decreased by 1 Hz. The corresponding value of OCR1A is calculated and loaded after each valid 
;* keypress.The program also uses a Timer/Counter1 interrupt.The interrupt service routine 
;* "tmr1_comp_match " is called every time the counter reaches the value stored at 
;* OCR1A(output compare register 1A). The counter counts the clock cycles, prescalar value of 8 
;* is used for the clock; evrytime the counter reaches the specified value in OCR1A it clears.
;* The isr "tmr1_comp_match " produces a pulse at PD0 with a width of 1ms. 
;* The delay of 1ms is provided by the use of the sybroutine "var_delay".
;* Ports:
;* PortB: Configured as outputs. Used to make connection to the 10 pin header to the Advanced DC 
;* motor interface. 
;*
;* PortC:
;* PC5 - PC2 JTAG interface, do not modify (IMPORTANT)
;*
;* PortD:
;* PD2 as input, EO from the priority encoder connected. PD0 as output. 
;* PortA:
;* PA6 - PA4 input, /A2, /A1, and /A0 respectively
;* VERSION HISTORY
;* 1.0 Original version
;***************************************************************************
.nolist 
.include "m324adef.inc"

.list

.dseg 
freq_L: .byte 1 ;low byte of frequency setting in binary 
freq_H: .byte 1 ;high byte of frequency setting in binary 
rpm_L: .byte 1 ;rpm low byte in binary 
rpm_H: .byte 1 ;rpm high byte in binary

.cseg
reset:
.org RESET
	rjmp start
.org INT0addr           ;INT0 interrupt vector
    rjmp keypress_isr
.org OC1Aaddr           ;addr for TC1 compare match OCR1A interrupt vector
    jmp tmr1_comp_match 
.include "lcd_dog_asm_driver_m324a.inc"  ; LCD DOG init/update procedures.
start:
;Configure port B as an output port
    ldi r16, $FF        ;load r16 with all 1s
    out DDRB, r16       ;port B - all bits configured as outputs

    ;Configure port A pin
    ldi r16, $00        ;load r16 with all $03
    out DDRA, r16       ;port A - all bits configured as inputs

	;Configure port D pin
	cbi DDRD,2 
	sbi DDRD, 0          ;PD2 configured as input, rest configured as outputs
	        
    
	ldi r16, low(RAMEND)  ; init stack/pointer
    out SPL, r16          ;
    ldi r16, high(RAMEND) ;
    out SPH, r16

	;Default frequency value
	ldi r16, HIGH(1000)
	sts freq_H, r16
	ldi r16, LOW(1000)
	sts freq_L, r16

	;Default rpm value
	ldi r16, HIGH(60000)
	sts rpm_H, r16
	ldi r16, LOW(60000)
	sts rpm_L, r16

	;Load default messagess for the LCD DOG module
	ldi ZH, HIGH(default_freq_msg<<1)
	ldi ZL, LOW(default_freq_msg<<1)
	rcall load_msg

	ldi ZH, HIGH(default_rpm_msg<<1)
	ldi ZL, LOW(default_rpm_msg<<1)
	rcall load_msg
	
	rcall init_lcd_dog    ; init display, using SPI serial interface
	rcall update_lcd_dog
	rcall update_lcd_dog
	;OCR1A starting value
	ldi r16, HIGH(126)  ;load compare value into OCR1A
	sts OCR1AH, r16       ;high byte must be written first
	ldi r16, LOW(126)
	sts OCR1AL, r16
	
	;Configure INT0 at rising edge
	ldi r16, (1 << ISC01) | (1 << ISC00)    ;interrupt sense control bits
    sts EICRA, r16      ;rising edge requests interrupt (EICRA in ext. I/O)
    ldi r16, 1<<INT0    ;enable interrupt request at INT0    
    out EIMSK, r16

	;Configure Timer/Counter1 for fast PWM, OCR1A defines top
	ldi r16, 1<<WGM11 | 1<<WGM10   ;fast PWM mode 15
	sts TCCR1A, r16
	ldi r16, 1<<WGM13 | 1<<WGM12 | 1<<CS11 ;clock prescalar 8
	sts TCCR1B, r16

	ldi r16, 1<<OCIE1A     ;enable compare A match interrupt
	sts TIMSK1, r16

	sei                 ;set global interrupt enable


	

	
	
main_loop:
    nop
    rjmp main_loop

;***************************************************************************
;* 
;* Title: keypress_isr
;*
;* Description:Reads the input from PORTA and shifts the position of the bits 6, 5 and 4 to 2, 1 and 0.	
;* Checks if key 7,6,5,4,3,2 was pressed. If pressed, doubles, reduces to half, increases by 50, decreases by 50,
;* increases by 10 or decreases by 10 the value of OCR1A based on the key value. 7 for half, 6 for double, 5 for 
;* increasing by 50, 4 for decreasing by 50, 3 for increasing by 10 and 2 for decreasing by 10. 

;* Author: Riwaz Awasthi
;* Version:1.0
;* Last updated:11/7/2017
;* Target: ATmega324a
;* Number of words:
;* Number of cycles:
;* Low registers modified:
;* High registers modified:r24,r25
;***************************************************************************
keypress_isr:
    push r16
	in r16, SREG
	push r16
	in r17, PINA           ;input the complemented values for A0, A1 and A2
	lsr r17             
	lsr r17
	lsr r17
	lsr r17                ;bits in the position 6, 5 and 4 are in the position 3, 2 and 1
	com r17                ;complements the contents of r18
	andi r17, $07          ;force all bits to 0 except the bits at position 3, 2 and 1
	rcall delay_50ms

	
pressed:
    sbis PIND, 2
	rjmp done 
    cpi r17, 1
	brne not_1
	rcall F_plus
	rjmp repeat
not_1:
    cpi r17, 0
	brne not_0
	rcall F_minus 
	rjmp repeat
not_0:
    cpi r17,7
	brne not_7
	rcall F_double
	rjmp repeat
not_7:
    cpi r17,6
	brne not_6
	rcall F_half
	rjmp repeat
not_6: 
    cpi r17,5
	brne not_5
	rcall F_course_plus
	rjmp repeat
not_5:
    cpi r17, 4
	brne not_4
	rcall F_course_minus
	rjmp repeat
not_4:
    cpi r17, 3
	brne not_3
	rcall F_fine_plus
	rjmp repeat
not_3:
    cpi r17,2
	brne done
	rcall F_fine_minus
	rjmp repeat
repeat:
	;sei
	push r17
	rcall dsp_update_freq
	rcall dsp_update_rpm
	rcall update_lcd_dog
	pop r17
	
	rcall var_delay2
	
    rjmp pressed
done:
    pop r16
	out SREG, r16
	pop r16
	reti

F_plus:
    push r17
    lds r24, OCR1AL
	lds r25, OCR1AH
	;checks if the value at OCR1A is 125, if yes, the value is not changed
	mov r19, r25
	or r19, r24
	cpi r19, 126
	brlo F_plus_done 
	;*******************************************************************
	clc
	lds r24, freq_L
	lds r25, freq_H
	adiw r25:r24, 1
	sts freq_H, r25
	sts freq_L, r24
	rcall freq_ocr1a_ldval
	rcall rpm_ldval
	
F_plus_done:
    pop r17
	ret


F_minus:
    push r17
    lds r24, OCR1AL
	lds r25, OCR1AH
	;checks if the value at OCR1A is $ffff, if yes, the value is not changed
	cpi r25, $ff
	breq F_minus_done
	;***********************************************************************
	          
	clc
	lds r24, freq_L
	lds r25, freq_H
	sbiw r25:r24, 1
	sts freq_H, r25
	sts freq_L, r24
	rcall freq_ocr1a_ldval
	rcall rpm_ldval
	
F_minus_done:
    pop r17
	ret
F_double:
    push r17
	
    lds r24, OCR1AL
	lds r25, OCR1AH
	;checks if the value at OCR1A is 250, if yes, the value is not changed
	mov r19, r25
	cpi r19, 0
	brne continue
	or r19, r24
	cpi r19, 250
	brlo F_double_done 
	;*******************************************************************
continue:
	clc
	ror r25
	sts OCR1AH, r25
	ror r24
	sts OCR1AL, r24
	clc
	lds r24, freq_L
	lds r25, freq_H
	rol r24
	rol r25
	sts freq_H, r25
	sts freq_L, r24
	rcall rpm_ldval
	
F_double_done:
    pop r17
	ret

F_half:
    push r17
	
    lds r24, OCR1AL
	lds r25, OCR1AH
	;checks if the value at OCR1A is 250, if yes, the value is not changed
	cpi r25, $80
	brsh F_half_done 
	;*******************************************************************
	clc
	rol r24
	rol r25
	sts OCR1AH, r25
	sts OCR1AL, r24
	clc
	lds r24, freq_L
	lds r25, freq_H
	ror r25
	ror r24
	sts freq_H, r25
	sts freq_L, r24
	rcall rpm_ldval
	
F_half_done:
    pop r17
	ret
F_course_plus:
    push r17
    cpi r17,5
	brne F_course_plus_done
    lds r24, OCR1AL
	lds r25, OCR1AH
	;checks if the value at OCR1A is less than 175, if yes, the value is not changed
	mov r19, r25
	or r19, r24
	cpi r19, 175
	brlo F_course_plus_done 
	;*******************************************************************
	clc
	sbiw r25:r24, 50
	sts OCR1AH, r25
	sts OCR1AL, r24
	rcall ocr1a_freq_ldval
	rcall rpm_ldval
	
F_course_plus_done:
    pop r17
	ret
F_course_minus:
    push r17
    cpi r17,4
	brne F_course_minus_done
    lds r24, OCR1AL
	lds r25, OCR1AH
	;checks if the value at OCR1A is more than 65485, if yes, the value is not changed
	mov r19, r25
	and r19, r24
	cpi r19, 206
	brsh F_course_minus_done 
	;*******************************************************************
	clc
	adiw r25:r24, 50
	sts OCR1AH, r25
	sts OCR1AL, r24
	rcall ocr1a_freq_ldval
	rcall rpm_ldval
	
F_course_minus_done:
    pop r17
	ret
F_fine_plus:
    push r17
    cpi r17,3
	brne F_fine_plus_done
    lds r24, OCR1AL
	lds r25, OCR1AH
	;checks if the value at OCR1A is less than 135, if yes, the value is not changed
	mov r19, r25
	or r19, r24
	cpi r19, 135
	brlo F_fine_plus_done 
	;*******************************************************************
	clc
	sbiw r25:r24, 10
	sts OCR1AH, r25
	sts OCR1AL, r24
	rcall ocr1a_freq_ldval
	rcall rpm_ldval
	
F_fine_plus_done:
    pop r17
	ret
F_fine_minus:
    push r17
    cpi r17,2
	brne F_fine_minus_done
    lds r24, OCR1AL
	lds r25, OCR1AH
	;checks if the value at OCR1A is more than 65525, if yes, the value is not changed
	mov r19, r25
	and r19, r24
	cpi r19, 246
	brsh F_fine_minus_done 
	;*******************************************************************
	clc
	adiw r25:r24, 10
	sts OCR1AH, r25
	sts OCR1AL, r24
	rcall ocr1a_freq_ldval
	rcall rpm_ldval
	
F_fine_minus_done:
    pop r17
	ret
;***************************************************************************
;* 
;* Title: ocr1a_freq_ldval
;*
;* Description: Computes the required value of frequency corresponding
;*              to the OCR1A value
;* Author: Riwaz Awasthi
;* Version:1.0
;* Last updated:11/28/2017
;* Target: ATmega324a
;* Number of words:
;* Number of cycles:
;* Low registers modified:none
;* High registers modified:r22,r23,r24,r25
;***************************************************************************
ocr1a_freq_ldval:
	lds r22, OCR1AL
	lds r23, OCR1AH
	ldi r24, 0
	ldi r25, 0
	rcall div32u
	sts freq_H, dres32u1
	sts freq_L, dres32u0
	ret
;***************************************************************************
;* 
;* Title: freq_ocr1a_ldval
;*
;* Description: Computes the required value of OCR1A to produce the frequency
;* Author: Riwaz Awasthi
;* Version:1.0
;* Last updated:11/28/2017
;* Target: ATmega324a
;* Number of words:
;* Number of cycles:
;* Low registers modified:none
;* High registers modified:r22,r23,r24,r25
;***************************************************************************
freq_ocr1a_ldval:
	lds r22, freq_L
	lds r23, freq_H
	ldi r24, 0
	ldi r25, 0
	rcall div32u
	sts OCR1AH, r19
	sts OCR1AL, r18
	ret
;***************************************************************************
;* 
;* Title: rpm_ldval
;*
;* Description: Computes the required value of OCR1A to produce the frequency
;* Author: Riwaz Awasthi
;* Version:1.0
;* Last updated:11/28/2017
;* Target: ATmega324a
;* Number of words:
;* Number of cycles:
;* Low registers modified:none
;* High registers modified:r22,r23,r24,r25
;***************************************************************************
rpm_ldval:
    push r17
	lds mp16uH, freq_H
	lds mp16uL, freq_L
	ldi mc16uH, HIGH(60)
	ldi mc16uL, LOW(60)
	rcall mpy16u
	sts rpm_H, m16u1
	sts rpm_L, m16u0
	pop r17
	ret


;***************************************************************************
;*
;* "bin2BCD16" - 16-bit Binary to BCD conversion
;*
;* This subroutine converts a 16-bit number (fbinH:fbinL) to a 5-digit 
;* packed BCD number represented by 3 bytes (tBCD2:tBCD1:tBCD0).
;* MSD of the 5-digit number is placed in the lowermost nibble of tBCD2.
;*  
;* Number of words	:25
;* Number of cycles	:751/768 (Min/Max)
;* Low registers used	:3 (tBCD0,tBCD1,tBCD2) 
;* High registers used  :4(fbinL,fbinH,cnt16a,tmp16a)	
;* Pointers used	:Z
;*
;***************************************************************************

;***** Subroutine Register Variables

.equ	AtBCD0	=13		;address of tBCD0
.equ	AtBCD2	=15		;address of tBCD1

.def	tBCD0	=r13		;BCD value digits 1 and 0
.def	tBCD1	=r14		;BCD value digits 3 and 2
.def	tBCD2	=r15		;BCD value digit 4
.def	fbinL	=r16		;binary value Low byte
.def	fbinH	=r17		;binary value High byte
.def	cnt16a	=r18		;loop counter
.def	tmp16a	=r19		;temporary value

;***** Code

bin2BCD16:
    push r17
	ldi	cnt16a,16	;Init loop counter	
	clr	tBCD2		;clear result (3 bytes)
	clr	tBCD1		
	clr	tBCD0		
	clr	ZH		;clear ZH (not needed for AT90Sxx0x)
bBCDx_1:lsl	fbinL		;shift input value
	rol	fbinH		;through all bytes
	rol	tBCD0		;
	rol	tBCD1
	rol	tBCD2
	dec	cnt16a		;decrement loop counter
	brne	bBCDx_2		;if counter not zero
	pop r17
	ret			;   return

bBCDx_2:ldi	r30,AtBCD2+1	;Z points to result MSB + 1
bBCDx_3:
	ld	tmp16a,-Z	;get (Z) with pre-decrement
;----------------------------------------------------------------
;For AT90Sxx0x, substitute the above line with:
;
;	dec	ZL
;	ld	tmp16a,Z
;
;----------------------------------------------------------------
	subi	tmp16a,-$03	;add 0x03
	sbrc	tmp16a,3	;if bit 3 not clear
	st	Z,tmp16a	;	store back
	ld	tmp16a,Z	;get (Z)
	subi	tmp16a,-$30	;add 0x30
	sbrc	tmp16a,7	;if bit 7 not clear
	st	Z,tmp16a	;	store back
	cpi	ZL,AtBCD0	;done all three?
	brne	bBCDx_3		;loop again if not
	rjmp	bBCDx_1		

;***************************************************************************
;*
;* "mpy16u" - 16x16 Bit Unsigned Multiplication
;*
;* This subroutine multiplies the two 16-bit register variables 
;* mp16uH:mp16uL and mc16uH:mc16uL.
;* The result is placed in m16u3:m16u2:m16u1:m16u0.
;*  
;* Number of words	:14 + return
;* Number of cycles	:153 + return
;* Low registers used	:None
;* High registers used  :7 (mp16uL,mp16uH,mc16uL/m16u0,mc16uH/m16u1,m16u2,
;*                          m16u3,mcnt16u)	
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	mc16uL	=r16		;multiplicand low byte
.def	mc16uH	=r17		;multiplicand high byte
.def	mp16uL	=r18		;multiplier low byte
.def	mp16uH	=r19		;multiplier high byte
.def	m16u0	=r18		;result byte 0 (LSB)
.def	m16u1	=r19		;result byte 1
.def	m16u2	=r20		;result byte 2
.def	m16u3	=r21		;result byte 3 (MSB)
.def	mcnt16u	=r22		;loop counter

;***** Code

mpy16u:	clr	m16u3		;clear 2 highest bytes of result
	clr	m16u2
	ldi	mcnt16u,16	;init loop counter
	lsr	mp16uH
	ror	mp16uL

m16u_1:	brcc	noad8		;if bit 0 of multiplier set
	add	m16u2,mc16uL	;add multiplicand Low to byte 2 of res
	adc	m16u3,mc16uH	;add multiplicand high to byte 3 of res
noad8:	ror	m16u3		;shift right result byte 3
	ror	m16u2		;rotate right result byte 2
	ror	m16u1		;rotate result byte 1 and multiplier High
	ror	m16u0		;rotate result byte 0 and multiplier Low
	dec	mcnt16u		;decrement loop counter
	brne	m16u_1		;if not done, loop more
	ret


;***************************************************************************
;*
;* "div32u" - 32/32 Bit Unsigned Division
;*
;* Ken Short
;*
;* This subroutine divides the two 32-bit numbers 
;* "dd32u3:dd32u2:dd32u1:dd32u0" (dividend) and "dv32u3:dv32u2:dv32u3:dv32u2"
;* (divisor). 
;* The result is placed in "dres32u3:dres32u2:dres32u1:dres32u0" and the
;* remainder in "drem32u3:drem32u2:drem32u3:drem32u2".
;*  
;* Number of words	:
;* Number of cycles	:655/751 (Min/Max) ATmega16
;* #Low registers used	:2 (drem16uL,drem16uH)
;* #High registers used  :5 (dres16uL/dd16uL,dres16uH/dd16uH,dv16uL,dv16uH,
;*			    dcnt16u)
;* A $0000 divisor returns $FFFF
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	drem32u0=r12    ;remainder
.def	drem32u1=r13
.def	drem32u2=r14
.def	drem32u3=r15

.def	dres32u0=r18    ;result (quotient)
.def	dres32u1=r19
.def	dres32u2=r20
.def	dres32u3=r21

.def	dd32u0	=r18    ;dividend
.def	dd32u1	=r19
.def	dd32u2	=r20
.def	dd32u3	=r21

.def	dv32u0	=r22    ;divisor
.def	dv32u1	=r23
.def	dv32u2	=r24
.def	dv32u3	=r25

.def	dcnt32u	=r17

;***** Code

div32u:
;Loading the value of the constant C_f as a 32 bit number in the registers r21, r20, r19, r18
	ldi r18, LOW(125000)
	ldi r19, BYTE2(125000)
	ldi r20, BYTE3(125000)
	ldi r21, BYTE4(125000)
	
	clr	drem32u0	;clear remainder Low byte
    clr drem32u1
    clr drem32u2
	sub	drem32u3,drem32u3;clear remainder High byte and carry
	ldi	dcnt32u,33	;init loop counter
d32u_1:
	rol	dd32u0		;shift left dividend
	rol	dd32u1
	rol	dd32u2    
	rol	dd32u3
	dec	dcnt32u		;decrement counter
	brne	d32u_2		;if done
	ret			;    return
d32u_2:
	rol	drem32u0	;shift dividend into remainder
    rol	drem32u1
    rol	drem32u2
	rol	drem32u3

	sub	drem32u0,dv32u0	;remainder = remainder - divisor
    sbc	drem32u1,dv32u1
    sbc	drem32u2,dv32u2
	sbc	drem32u3,dv32u3	;
	brcc	d32u_3		;   branch if reult is pos or zero

	add	drem32u0,dv32u0	;    if result negative restore remainder
	adc	drem32u1,dv32u1
	adc	drem32u2,dv32u2
	adc	drem32u3,dv32u3
	clc			;    clear carry to be shifted into result
	rjmp	d32u_1		;else
d32u_3:	sec			;    set carry to be shifted into result
	rjmp	d32u_1

;***************************************************************************
;* 
;* Title: dsp_update_freq
;*
;* Description: Updates the frequency value in the LCD display
;* Author: Riwaz Awasthi
;* Version:1.0
;* Last updated:11/28/2017
;* Target: ATmega324a
;* Number of words:
;* Number of cycles:
;* Low registers modified:none
;* High registers modified:r22,r23,r24,r25
;***************************************************************************

dsp_update_freq:
    push r17
    lds fbinL, freq_L
	lds fbinH, freq_H
	rcall bin2BCD16
	ldi YH, HIGH(dsp_buff_1)
	ldi YL, LOW(dsp_buff_1)
    ldi r16, 48
    mov r18,tBCD1
	swap r18
	andi r18, $0f
	add r18,r16
	std Y+12, r18
	mov r18,tBCD1
	andi r18, $0f
	add r18, r16
	std Y+13, r18
	mov r18,tBCD0
	swap r18
	andi r18, $0f
	add r18, r16
	std Y+14, r18
	mov r18,tBCD0
	andi r18,$0f
	add r18, r16
	std Y+15, r18
	pop r17
	ret

;***************************************************************************
;* 
;* Title: dsp_update_rpm
;*
;* Description: Updates the value of rpm im the LCD display
;* Author: Riwaz Awasthi
;* Version:1.0
;* Last updated:11/28/2017
;* Target: ATmega324a
;* Number of words:
;* Number of cycles:
;* Low registers modified:none
;* High registers modified:r22,r23,r24,r25
;***************************************************************************

dsp_update_rpm:
    push r17
    lds fbinL, rpm_L
	lds fbinH, rpm_H
	rcall bin2BCD16
	ldi YH, HIGH(dsp_buff_2)
	ldi YL, LOW(dsp_buff_2)
    ldi r16, 48
	mov r18,tBCD2
	andi r18, $0f
	add r18,r16
	std Y+11, r18
    mov r18,tBCD1
	swap r18
	andi r18, $0f
	add r18,r16
	std Y+12, r18
	mov r18,tBCD1
	andi r18, $0f
	add r18, r16
	std Y+13, r18
	mov r18,tBCD0
	swap r18
	andi r18, $0f
	add r18, r16
	std Y+14, r18
	mov r18,tBCD0
	andi r18,$0f
	add r18, r16
	std Y+15, r18
	
	pop r17
	ret
;***************************************************************************
;* 
;* Title: tmr1_comp_match
;*
;* Description: Produces a 1ms wide pulse at PD0
;* Author: Riwaz Awasthi
;* Version:1.0
;* Last updated:11/7/2017
;* Target: ATmega324a
;* Number of words:
;* Number of cycles:
;* Low registers modified:none
;* High registers modified:none
;***************************************************************************
tmr1_comp_match:
    push r16
	in r16, SREG
	push r16
	sbi PORTD, 0
	rcall var_delay
	cbi PORTD, 0
	pop r16
	out SREG, r16
	pop r16 
	reti
	
;***************************************************************************
;*
;* "var_delay" - Variable Delay - 0.1 ms increments
;*
;* Description:
;* Delays for a time equal to r16 * 0.1 ms when ATmega324 clocked at 1 MHz
;*
;* Author: Riwaz Awasthi
;* Version: 1.0
;* Last updated:
;* Target: ATmega324 @ 1 MHz
;* Number of words:
;* Number of cycles: ~100 * r16
;* Low registers modified: none
;* High registers modified: none
;*
;* Parameters:
;* r16 - outer loop control variable
;*
;* Returns:
;* delay of 0.1ms * r16
;*
;* Notes:
;* Delay is designed for ATmega324 with 1 MHz clock
;*
;***************************************************************************
var_delay:             ;delay for ATmega324 @ 1MHz = r16 * 0.1 ms 
push r17
push r16
ldi r16, 10
outer_loop:    
	ldi r17, 32       
inner_loop:    
	dec r17    
	brne inner_loop    
	dec r16    
	brne outer_loop
	pop r16
	pop r17
	ret 

	;***************************************************************************
;*
;* "delay_50ms" - Produces a delay of 50ms
;*
;* Description:
;* Delays for a time equal to r16 * 0.1 ms when ATmega324 clocked at 1 MHz
;*
;* Author: Riwaz Awasthi
;* Version: 1.0
;* Last updated:
;* Target: ATmega324 @ 1 MHz
;* Number of words:
;* Number of cycles: ~100 * r16
;* Low registers modified: none
;* High registers modified: none
;*
;* Parameters:
;* r16 - outer loop control variable
;*
;* Returns:
;* delay of 0.1ms * r16
;*
;* Notes:
;* Delay is designed for ATmega324 with 1 MHz clock
;*
;***************************************************************************
delay_50ms:             ;delay for ATmega324 @ 1MHz = 50ms 
push r17
push r16
ldi r16, 84
outer_loop2:    
	ldi r17, 199      
inner_loop2:    
	dec r17    
	brne inner_loop2    
	dec r16    
	brne outer_loop2
	pop r16
	pop r17
	ret 
	;***************************************************************************
;*
;* "var_delay2" - Variable Delay - 0.1 ms increments
;*
;* Description:
;* Delays for a time equal to r16 * 0.1 ms when ATmega324 clocked at 1 MHz
;*
;* Author: Riwaz Awasthi
;* Version: 1.0
;* Last updated:
;* Target: ATmega324 @ 1 MHz
;* Number of words:
;* Number of cycles: ~100 * r16
;* Low registers modified: none
;* High registers modified: none
;*
;* Parameters:
;* r16 - outer loop control variable
;*
;* Returns:
;* delay of 0.1ms * r16
;*
;* Notes:
;* Delay is designed for ATmega324 with 1 MHz clock
;*
;***************************************************************************
var_delay2:             ;delay for ATmega324 @ 1MHz = r16 * 0.1 ms 
push r17
push r16
ldi r16, 255
outer_loop1:    
	ldi r17, 255      
inner_loop1:    
	dec r17    
	brne inner_loop1    
	dec r16    
	brne outer_loop1
	pop r16
	pop r17
	ret 


;*******************************************************************
;NAME:      load_msg
;FUNCTION:  Loads a predefined string msg into a specified diplay
;           buffer.
;ASSUMES:   Z = offset of message to be loaded. Msg format is 
;           defined below.
;RETURNS:   nothing.
;MODIFIES:  r16, Y, Z
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
; Message structure:
;   label:  .db <buff num>, <text string/message>, <end of string>
;
; Message examples (also see Messages at the end of this file/module):
;   msg_1: .db 1,"First Message ", 0   ; loads msg into buff 1, eom=0
;   msg_2: .db 1,"Another message ", 0 ; loads msg into buff 1, eom=0
;
; Notes: 
;   a) The 1st number indicates which buffer to load (either 1, 2, or 3).
;   b) The last number (zero) is an 'end of string' indicator.
;   c) Y = ptr to disp_buffer
;      Z = ptr to message (passed to subroutine)
;********************************************************************
load_msg:
     ldi YH, high (dsp_buff_1) ; Load YH and YL as a pointer to 1st
     ldi YL, low (dsp_buff_1)  ; byte of dsp_buff_1 (Note - assuming 
                               ; (dsp_buff_1 for now).
     lpm R16, Z+               ; get dsply buff number (1st byte of msg).
     cpi r16, 1                ; if equal to '1', ptr already setup.
     breq get_msg_byte         ; jump and start message load.
     adiw YH:YL, 16            ; else set ptr to dsp buff 2.
     cpi r16, 2                ; if equal to '2', ptr now setup.
     breq get_msg_byte         ; jump and start message load.
     adiw YH:YL, 16            ; else set ptr to dsp buff 2.
        
get_msg_byte:
     lpm R16, Z+               ; get next byte of msg and see if '0'.        
     cpi R16, 0                ; if equal to '0', end of message reached.
     breq msg_loaded           ; jump and stop message loading operation.
     st Y+, R16                ; else, store next byte of msg in buffer.
     rjmp get_msg_byte         ; jump back and continue...
msg_loaded:
     ret

default_freq_msg: .db 1,"frequency =01000",0
default_rpm_msg : .db 2,"rpm       =60000",0