;***********************************************************
;*
;*	This is the skeleton file for Lab 7 of ECE 375
;*
;*	 Author: Christian Ovchinikov
;*	   Date: 02/23/2022
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multipurpose register
.def	waitcnt = r23			; Wait Loop Counter
.def	ilcnt = r24				; Inner Loop Counter
.def	olcnt = r25				; Outer Loop Counter

.equ	WTime = 50				; Time to wait in wait loop
.equ	EngEnR = 4				; right Engine Enable Bit
.equ	EngEnL = 7				; left Engine Enable Bit
.equ	EngDirR = 5				; right Engine Direction Bit
.equ	EngDirL = 6				; left Engine Direction Bit

;/////////////////////////////////////////////////////////////
; These macros are the values to make the TekBot Move.
;
; Referenced from skeleton code for Lab 1 for BasicBumpBot.asm
;/////////////////////////////////////////////////////////////

.equ MovFwd = (1<<EngDirR|1<<EngDirL) ; Move Forward Command

;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; beginning of code segment

;***********************************************************
;*	Interrupt Vectors
;***********************************************************
.org	$0000
		rjmp	INIT		; reset interrupt

.org	$0002
		rcall	SPEED_DOWN	; Call function to decrease speed level
		reti				; Return from interrupt

.org	$0004
		rcall	SPEED_UP	; Call function to increase speed level
		reti				; Return from interrupt

.org	$0006
		rcall	SPEED_MIN	; Call function to set speed level to min level (0)
		reti				; Return from interrupt

.org	$0008
		rcall	SPEED_MAX	; Call function to set speed level to max level (15)
		reti				; Return from interrupt

.org	$0046				; end of interrupt vectors

;***********************************************************
;*	Program Initialization
;***********************************************************
INIT:
		; Initialize the Stack Pointer
		ldi		mpr, high(RAMEND)	; Loads higher byte of RAMEND into mpr = R16
		out		SPH, mpr			; Stores R16 into higher byte of Stack Pointer
		ldi		mpr, low(RAMEND)	; Loads lower byte of RAMEND into mpr
		out		SPL, mpr			; Stores R16 into lower byte of Stack Pointer
		; Configure I/O ports
			; Initialize Port B for output
		ldi		mpr, 0b11111111		; Sets mpr to full of 1's
		out		DDRB, mpr			; Loads mpr into Data Direction Register B (DDRB) with 1's configuring data for output
		ldi		mpr, 0b00000000		; Sets mpr to full of 0's
		out		PORTB, mpr			; Loads mpr into Port B Data Register with 0's which switches off the pull-up resistor configuring it as an output
			; Initialize Port D for input
		ldi		mpr, 0b00000000		; Sets mpr to full of 0's
		out		DDRD, mpr			; Loads mpr into Data Direction Register D (DDRD) with 0's configuring data for input
		ldi		mpr, 0b11111111		; Sets mpr to full of 1's
		out		PORTD, mpr			; Loads mpr into Port B Data Register with 1's which switches on the pull-up resistor configuring it as an input
		
		; Configure External Interrupts, if needed
		ldi		mpr, 0b10101010		; Sets INT0, INT1, INT2, and INT3 to falling edge which generates asynchronouysly an interrupt request
		sts		EICRA, mpr			; Stores the falling edge values to data space in External Interrupt Control Register A which is for INT0-INT3
		
		; Configure the External Interrupt Mask
		ldi		mpr, 0b00001111		; loads into mpr the 1 value for the corresponding INTx interrupts that will enable them
		out		EIMSK, mpr			; sends out mpr into EIMSK setting the Interrupt Enable bit
		; Configure 8-bit Timer/Counters
		ldi		mpr, 0b01111001		; Sets WGM01:0 bits for Fast PWM, Sets COM01:0 bits controls output compare pin (Sets OC0, clears bottom - inverting mode / TCNT0 = OCR0), CS02:0 sets no prescaling
		out		TCCR0, mpr			; Sends that out to Timer/Counter Control Register 0
		ldi		mpr, 0b01111001		; Same values, just reload mpr
		out		TCCR2, mpr			; Sends out to Timer/Counter Control Register 2
								; no prescaling = 0bXXXXX001

		; Set TekBot to Move Forward (1<<EngDirR|1<<EngDirL)
		ldi		mpr, MovFwd			; loads value defined by MovFwd for Move Forward command
		out		PORTB, mpr			; sends Move Forward command to Port B (output)

		; Set initial speed, display on Port B pins 3:0
		ldi		mpr, 0b00000000		; load all 0's initializing the speed to be at level 0
		out		OCR0, mpr			; sends out 0x00 telling the Output Compare Register 0 to compare 0x00 = BOTTOM
		ldi		mpr, 0b00000000		; Same values, just reload mpr
		out		OCR2, mpr			; sends out 0x00 telling the Output Compare Register 2 to compare 0x00 = BOTTOM
		; Enable global interrupts (if any are used)
		sei

;***********************************************************
;*	Main Program
;***********************************************************
MAIN:
		; poll Port D pushbuttons (if needed)

								; if pressed, adjust speed
								; also, adjust speed indication

		rjmp	MAIN			; return to top of MAIN

;***********************************************************
;*	Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
; Func:	SPEED_DOWN
; Desc:	This function decreases the speed level by 1,
;		if level is min, do not decrease, remain at min level.
;-----------------------------------------------------------
SPEED_DOWN:

		; If needed, save variables by pushing to the stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register

		; Execute the function here
		ldi		waitcnt, WTime		; Wait for 1/2 second
		rcall	WaitSub				; Calls wait function

		in		mpr, OCR0			; Load Output Compare Register 0 into olcnt register
		ldi		olcnt, 0b00000000	; Load 0/255 into olcnt register for checking lowest level
		ldi		ilcnt, 0b00010001	; Load 255/15 = 17 into ilcnt register for decreasing level
		cp		mpr, olcnt			; Compares mpr to OCR0 to see if we are at the minimum level (0)
		breq	DOWN_END			; If at the minimum level, do not decrease, skip to end of function

		sub		mpr, ilcnt			; Subtracts 17 from mpr decreasing the level by 1
		out		OCR0, mpr			; Sends updated value to OCR0
		out		OCR2, mpr			; Sends updated value to OCR2

		in		mpr, PORTB			; Loads output values of Port B
		dec		mpr					; Decrements the value indicating a decrease in speed
		out		PORTB, mpr			; Sends back out to Port B output of LEDs

DOWN_END:
		; Clear queued interrupts
		ldi		mpr, 0b00001111		; Writing a logical 1 to 0-3 bits will which will clear the corresponding interrupt flag in INT0-3
		out		EIFR, mpr			; Clears queued interrupts

		; Restore any saved variables by popping from stack
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr

		ret							; Return from subroutine

;-----------------------------------------------------------
; Func:	SPEED_UP
; Desc:	This function increases the speed level by 1,
;		if level is max, do not increase, remain at max level.
;-----------------------------------------------------------
SPEED_UP:

		; If needed, save variables by pushing to the stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		; Execute the function here
		ldi		waitcnt, WTime		; Wait for 1/2 second
		rcall	WaitSub				; Calls wait function

		in		mpr, OCR0			; Load Output Compare Register 0 into olcnt register
		ldi		olcnt, 0b11111111	; Load 0/255 into olcnt register for checking lowest level
		ldi		ilcnt, 0b00010001	; Load 255/15 = 17 into ilcnt register for decreasing level
		cp		mpr, olcnt			; Compares mpr to OCR0 to see if we are at the minimum level (0)
		breq	UP_END				; If at the minimum level, do not decrease, skip to end of function

		add		mpr, ilcnt			; Subtracts 17 from mpr decreasing the level by 1
		out		OCR0, mpr			; Sends updated value to OCR0
		out		OCR2, mpr			; Sends updated value to OCR2

		in		mpr, PORTB			; Loads output values of Port B
		inc		mpr					; Increments the value indicating an increase in speed
		out		PORTB, mpr			; Sends back out to Port B output of LEDs

UP_END:
		; Clear queued interrupts
		ldi		mpr, 0b00001111		; Writing a logical 1 to 0-3 bits will which will clear the corresponding interrupt flag in INT0-3
		out		EIFR, mpr			; Clears queued interrupts

		; Restore any saved variables by popping from stack
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr

		ret							; Return from subroutine

;-----------------------------------------------------------
; Func:	SPEED_MIN
; Desc:	This function sets the speed level to 0
;-----------------------------------------------------------
SPEED_MIN:

		; If needed, save variables by pushing to the stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register

		; Execute the function here
		ldi		waitcnt, WTime		; Wait for 1/2 second
		rcall	WaitSub				; Calls the wait function

		ldi		mpr, 0b00000000		; Loads 0's into mpr reinitializing the speed to level 0
		out		OCR0, mpr			; Sends out to OCR0
		out		OCR2, mpr			; Sends out to OCR2

		ldi		mpr, 0b11110000		; Loads LED configuration for halt 100% cycle
		out		PORTB, mpr			; Sends out to Port B

		; Clear queued interrupts
		ldi		mpr, 0b00001111		; Writing a logical 1 to 0-3 bits will which will clear the corresponding interrupt flag in INT0-3
		out		EIFR, mpr			; Clears queued interrupts

		; Restore any saved variables by popping from stack
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr

		ret							; End a function with RET

;-----------------------------------------------------------
; Func:	SPEED_MAX
; Desc:	This function sets the speed level to 15
;-----------------------------------------------------------
SPEED_MAX:

		; If needed, save variables by pushing to the stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register

		; Execute the function here
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	WaitSub				; Calls the wait function

		ldi		mpr, 0b11111111		; Loads 1's into mpr reinitializing the speed to level max 15
		out		OCR0, mpr			; Sends out to OCR0
		out		OCR2, mpr			; Sends out to OCR2

		ldi		mpr, 0b01101111		; Loads LED configuration for full speed 0% duty cycle
		out		PORTB, mpr			; Sends out to Port B

		; Clear queued interrupts
		ldi		mpr, 0b00001111		; Writing a logical 1 to 0-3 bits will which will clear the corresponding interrupt flag in INT0-3
		out		EIFR, mpr			; Clears queued interrupts

		; Restore any saved variables by popping from stack
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr

		ret							; End a function with RET

;----------------------------------------------------------------
; Sub: Wait
; Desc: A wait loop that is 16 + 159975*waitcnt cycles or roughly 
;		waitcnt*10ms. Just initialize wait for the specific amount 
;		of time in 10ms intervals. Here is the general eqaution
;		for the number of clock cycles in the wait loop:
;		((3 * ilcnt + 3) * olcnt + 3) * waitcnt + 13 + call
;
; Referenced from skeleton code for Lab 1 for BasicBumpBot.asm
;----------------------------------------------------------------
WaitSub:
		; If needed, save variables by pushing to the stack
		push	waitcnt			; Save wait register
		push	ilcnt			; Save ilcnt register
		push	olcnt			; Save olcnt register

		; Execute the function here
Loop:	ldi		olcnt, 224		; load olcnt register
OLoop:	ldi		ilcnt, 237		; load ilcnt register
ILoop:	dec		ilcnt			; decrement ilcnt
		brne	ILoop			; Continue Inner Loop
		dec		olcnt			; decrement olcnt
		brne	OLoop			; Continue Outer Loop
		dec		waitcnt			; Decrement wait 
		brne	Loop			; Continue Wait loop

		; Restore any saved variables by popping from stack
		pop		olcnt			; Restore olcnt register
		pop		ilcnt			; Restore ilcnt register
		pop		waitcnt			; Restore wait register

		ret						; Return from subroutine

;***********************************************************
;*	Stored Program Data
;***********************************************************
		; Enter any stored data you might need here

;***********************************************************
;*	Additional Program Includes
;***********************************************************
		; There are no additional file includes for this program