;***********************************************************
;*	This is the skeleton file for Lab 6 of ECE 375
;*
;*	 Author: Christian Ovchinikov
;*	   Date: 02/13/2022
;*
;***********************************************************

.include "m128def.inc"						; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16							; Multipurpose register
.def	waitcnt = r23						; Wait Loop Counter
.def	ilcnt = r24							; Inner Loop Counter
.def	olcnt = r25							; Outer Loop Counter

.equ	WTime = 100							; Time to wait in wait loop

.equ	WskrR = 0							; Right Whisker Input Bit
.equ	WskrL = 1							; Left Whisker Input Bit
.equ	EngEnR = 4							; Right Engine Enable Bit
.equ	EngEnL = 7							; Left Engine Enable Bit
.equ	EngDirR = 5							; Right Engine Direction Bit
.equ	EngDirL = 6							; Left Engine Direction Bit

;/////////////////////////////////////////////////////////////
; These macros are the values to make the TekBot Move.
;
; Referenced from skeleton code for Lab 1 for BasicBumpBot.asm
;/////////////////////////////////////////////////////////////

.equ	MovFwd = (1<<EngDirR|1<<EngDirL)	; Move Forward Command
.equ	MovBck = $00						; Move Backward Command
.equ	TurnR = (1<<EngDirL)				; Turn Right Command
.equ	TurnL = (1<<EngDirR)				; Turn Left Command
.equ	Halt = (1<<EngEnR|1<<EngEnL)		; Halt Command

;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;***********************************************************
;*	Interrupt Vectors
;***********************************************************
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt

		; Set up interrupt vectors for any interrupts being used
.org	$0002
		rcall	HitRight		; Hit Right interrupt
		reti

.org	$0004
		rcall	HitLeft			; Hit Left interrupt
		reti

.org	$0006
		rcall	ClearRight		; Clear Right interrupt
		reti

.org	$0008
		rcall	ClearLeft		; Clear Left interrupt
		reti

.org	$0046					; End of Interrupt Vectors

;***********************************************************
;*	Program Initialization
;***********************************************************
INIT:							; The initialization routine
		; Initialize Stack Pointer
		ldi		mpr, high(RAMEND)	; Loads higher byte of RAMEND into mpr = R16
		out		SPH, mpr			; Stores R16 into higher byte of Stack Pointer
		ldi		mpr, low(RAMEND)	; Loads lower byte of RAMEND into mpr
		out		SPL, mpr			; Stores R16 into lower byte of Stack Pointer

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

		; Initialize external interrupts and set the Interrupt Sense Control to falling edge
		ldi		mpr, 0b10101010		; Sets INT0, INT1, INT2, and INT3 to falling edge which generates asynchronouysly an interrupt request
		sts		EICRA, mpr			; Stores the falling edge values to data space in External INterrupt Control Register A which is for INT0-INT3

		; Configure the External Interrupt Mask
		ldi		mpr, (1<<INT0) | (1<<INT1) | (1<<INT2) | (1<<INT3)	; loads into mpr, left-shift of either of the 4 external interrupts
		out		EIMSK, mpr											; loads mpr into EIMSK setting the Interrupt Enable bit
		
		; Initialize LCD Display
		rcall	LCDInit				; Calls LCDInit Function

		; Turn on interrupts
		sei							; NOTE: This must be the last thing to do in the INIT function

;***********************************************************
;*	Main Program
;***********************************************************
MAIN:							; The Main program

		ldi		mpr,  MovFwd	; Loads Move Forward command into mpr
		out		PORTB, mpr		; Sends Move Forward command to PORTB (output for motors)

		rjmp	MAIN			; Create an infinite while loop to signify the
								; end of the program.

;***********************************************************
;*	Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
;	You will probably want several functions, one to handle the
;	left whisker interrupt, one to handle the right whisker
;	interrupt, and maybe a wait function
;------------------------------------------------------------

;-----------------------------------------------------------
; Func: Template function header
; Desc: Cut and paste this and fill in the info at the
;		beginning of your functions
;-----------------------------------------------------------
FUNC:							; Begin a function with a label

		; Save variable by pushing them to the stack

		; Execute the function here

		; Restore variable by popping them from the stack in reverse order

		ret						; End a function with RET

;----------------------------------------------------------------
; Sub:	HitRight
; Desc:	Handles functionality of the TekBot when the right whisker
;		is triggered.
;
; Referenced from skeleton code for Lab 1 for BasicBumpBot.asm
;----------------------------------------------------------------
HitRight:
		; Save register values by pushing to stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		in		mpr, SREG			; Save program state
		push	mpr					;

		; Move Backwards for a second
		ldi		mpr, MovBck			; Load Move Backward command
		out		PORTB, mpr			; Send command to port
		ldi		waitcnt, WTime		; Wait for 1 second
		rcall	WaitFunc			; Call wait function

		; Turn left for a second
		ldi		mpr, TurnL			; Load Turn Left command
		out		PORTB, mpr			; Send command to port
		ldi		waitcnt, WTime		; Wait for 1 second
		rcall	WaitFunc			; Call wait function

Continue_HR:
		; Move Forward again	
		ldi		mpr, MovFwd			; Load Move Forward command
		out		PORTB, mpr			; Send command to port

		; Counter for LCD Display
		ldi		XH, $01				; Loads high bit of first character on first line
		ldi		XL, $00				; Loads low bit of first character on first line
		ld		mpr, X				; Loads the value that X is pointing to into mpr
		ldi		YL, 0b00111010		; Sets YL to the next ASCII symbol after 9 (:)
		inc		mpr					; increments mpr
		cp		mpr, YL				; compares mpr and YL to see if the value on the display has surpassed 9
		breq	NextChar_HR			; if the output value surpasses 9, then branch to next character position
		
		; If not over 9, store the value of mpr into the first character slot and skip to end of function
		st		X, mpr				; Stores the value from mpr into the address pointed to by X
		rjmp	End_HR				; Jumps to end of this function

NextChar_HR:
		; If the output is over 9, move the output position to allow double digit outputs
		ldi		mpr, 0b00110000		; Loads the 0 ASCII symbol into mpr
		ld		YL, -X				; Decrements X then loads the value that X is pointing to into YL
		inc		YL					; Increments YL
		st		X+, YL				; Stores the value in YL into X and incremenets the position of X
		st		X, mpr				; Stores the value in mpr into X

End_HR:
		; Write to LCD Display
		rcall	LCDWrite			; Utilizes LCD function to write to display

		; Clear queued interrupts
		ldi		mpr, 0b00001111		; Writing a logical 1 to 0-3 bits will which will clear the corresponding interrupt flag in INT0-3
		out		EIFR, mpr			; Clears queued interrupts

		; Clear register values by popping out of stack
		pop		mpr					; Restore program state
		out		SREG, mpr			;
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr
		ret							; Return from subroutine

;----------------------------------------------------------------
; Sub:	HitLeft
; Desc:	Handles functionality of the TekBot when the left whisker
;		is triggered.
;
; Referenced from skeleton code for Lab 1 for BasicBumpBot.asm
;----------------------------------------------------------------
HitLeft:
		push	mpr			; Save mpr register
		push	waitcnt			; Save wait register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Move Backwards for a second
		ldi		mpr, MovBck	; Load Move Backward command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	WaitFunc			; Call wait function

		; Turn right for a second
		ldi		mpr, TurnR	; Load Turn Left Command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	WaitFunc			; Call wait function

		; Move Forward again	
		ldi		mpr, MovFwd	; Load Move Forward command
		out		PORTB, mpr	; Send command to port

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		waitcnt		; Restore wait register
		pop		mpr		; Restore mpr
		ret				; Return from subroutine

;----------------------------------------------------------------
; Sub:	HitLeft
; Desc:	Handles functionality of the TekBot when the left whisker
;		is triggered.
;
; Referenced from skeleton code for Lab 1 for BasicBumpBot.asm
;----------------------------------------------------------------
ClearRight:
		push	mpr			; Save mpr register
		push	waitcnt			; Save wait register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Move Backwards for a second
		ldi		mpr, MovBck	; Load Move Backward command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	WaitFunc			; Call wait function

		; Turn right for a second
		ldi		mpr, TurnR	; Load Turn Left Command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	WaitFunc			; Call wait function

		; Move Forward again	
		ldi		mpr, MovFwd	; Load Move Forward command
		out		PORTB, mpr	; Send command to port

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		waitcnt		; Restore wait register
		pop		mpr		; Restore mpr
		ret				; Return from subroutine


;----------------------------------------------------------------
; Sub:	HitLeft
; Desc:	Handles functionality of the TekBot when the left whisker
;		is triggered.
;
; Referenced from skeleton code for Lab 1 for BasicBumpBot.asm
;----------------------------------------------------------------
ClearLeft:
		push	mpr			; Save mpr register
		push	waitcnt			; Save wait register
		in		mpr, SREG	; Save program state
		push	mpr			;

		; Move Backwards for a second
		ldi		mpr, MovBck	; Load Move Backward command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	WaitFunc			; Call wait function

		; Turn right for a second
		ldi		mpr, TurnR	; Load Turn Left Command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime	; Wait for 1 second
		rcall	WaitFunc			; Call wait function

		; Move Forward again	
		ldi		mpr, MovFwd	; Load Move Forward command
		out		PORTB, mpr	; Send command to port

		pop		mpr		; Restore program state
		out		SREG, mpr	;
		pop		waitcnt		; Restore wait register
		pop		mpr		; Restore mpr
		ret				; Return from subroutine

;----------------------------------------------------------------
; Sub:	Wait
; Desc:	A wait loop that is 16 + 159975*waitcnt cycles or roughly 
;		waitcnt*10ms.  Just initialize wait for the specific amount 
;		of time in 10ms intervals. Here is the general eqaution
;		for the number of clock cycles in the wait loop:
;			((3 * ilcnt + 3) * olcnt + 3) * waitcnt + 13 + call
;
; Referenced from skeleton code for Lab 1 for BasicBumpBot.asm
;----------------------------------------------------------------
WaitFunc:
		push	waitcnt			; Save wait register
		push	ilcnt			; Save ilcnt register
		push	olcnt			; Save olcnt register

Loop:	ldi		olcnt, 224		; load olcnt register
OLoop:	ldi		ilcnt, 237		; load ilcnt register
ILoop:	dec		ilcnt			; decrement ilcnt
		brne	ILoop			; Continue Inner Loop
		dec		olcnt		; decrement olcnt
		brne	OLoop			; Continue Outer Loop
		dec		waitcnt		; Decrement wait 
		brne	Loop			; Continue Wait loop	

		pop		olcnt		; Restore olcnt register
		pop		ilcnt		; Restore ilcnt register
		pop		waitcnt		; Restore wait register
		ret				; Return from subroutine

;***********************************************************
;*	Stored Program Data
;***********************************************************

; Enter any stored data you might need here

;***********************************************************
;*	Additional Program Includes
;***********************************************************
.include "LCDDriver.asm"		; Include the LCD Driver