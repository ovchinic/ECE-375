;***********************************************************
;*
;*	This is the TRANSMIT skeleton file for Lab 8 of ECE 375
;*
;*	 Author: Christian Ovchinikov
;*	   Date: 3/4/2022
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multi-Purpose Register
.def	waitcnt = r23			; Wait Loop Counter
.def	ilcnt = r24				; Inner Loop Counter
.def	olcnt = r25				; Outer Loop Counter

.equ	WTime = 100				; Time to wait in wait loop
.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit

; Use these action codes between the remote and robot
; MSB = 1 thus:
.equ	BotAddress = $08		; TX Bot Address = 0b00001000

; control signals are shifted right by one and ORed with 0b10000000 = $80
.equ	MovFwd = ($80|1<<(EngDirR-1)|1<<(EngDirL-1))	;0b10110000 Move Forward Action Code
.equ	MovBck = ($80|$00)								;0b10000000 Move Backward Action Code
.equ	TurnR = ($80|1<<(EngDirL-1))					;0b10100000 Turn Right Action Code
.equ	TurnL = ($80|1<<(EngDirR-1))					;0b10010000 Turn Left Action Code
.equ	Halt = ($80|1<<(EngEnR-1)|1<<(EngEnL-1))		;0b11001000 Halt Action Code
.equ	Freeze = 0b11111000								;0b11111000 Freeze Action Code

;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;***********************************************************
;*	Interrupt Vectors
;***********************************************************
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt

.org	$0046					; End of Interrupt Vectors

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

	; USART1 
		; Initialize USART1
		ldi mpr, (1<<U2x1)			; Set double data rate (Bit 1)
		sts UCSR1A, mpr				; Send out to UCSR1A

		; Set baudrate at 2400bps
		ldi mpr, high(832)			; Load high byte of 832 because UBBR = 832 due to divder being changed by U2X1 from 16 to 8
		sts UBRR1H, mpr				; UBRR1H in extended I/O space
		ldi mpr, low(832)			; Load low byte of 832
		sts UBRR1L, mpr				; 832 was used for UBRR because U2x1 Bit 1 = 1, from page 196 ATmega128, Table 85
		
		;Enable transmitter
		ldi mpr, (1<<TXEN1)			; Sets mpr to 0b00010000 enabling TX Transmitter
		sts UCSR1B, mpr		

		; Set frame format: 8 data, 2 stop bits; Asynchronous/Disabled Parity is already implied with bits set to 0
		ldi mpr, (1<<USBS1 | 1<<UCSZ11 | 1<<UCSZ10)			; Sets mpr to 0bX0XX111X enabling 2-bits, and 8-bit data.
		sts UCSR1C, mpr										; UCSR1C in extended I/O space

;***********************************************************
;*	Main Program
;***********************************************************
MAIN:
	;TODO:
		in		mpr, PIND		; Reads in PIN D inputs from the buttons

		; Check for Forward Command Button
		cpi		mpr, 0b11111110
		brne	MAIN_NEXT
		rcall	ForwardAction
		rjmp	MAIN_END

MAIN_NEXT:
		; Check for Backward Command Button
		cpi		mpr, 0b11111101
		brne	MAIN_NEXT1
		rcall	BackwardAction
		rjmp	MAIN_END

MAIN_NEXT1:
		; Check for Right Command Button
		cpi		mpr, 0b11101111
		brne	MAIN_NEXT2
		rcall	RightAction
		rjmp	MAIN_END

MAIN_NEXT2:
		; Check for Left Command Button
		cpi		mpr, 0b11011111
		brne	MAIN_NEXT3
		rcall	LeftAction
		rjmp	MAIN_END

MAIN_NEXT3:
		; Check for Halt Command Button
		cpi		mpr, 0b10111111
		brne	MAIN_NEXT4
		rcall	HaltAction
		rjmp	MAIN_END

MAIN_NEXT4:
		; Check for Freeze Command Button
		cpi		mpr, 0b01111111
		brne	MAIN_END
		rcall	FreezeAction

MAIN_END:
		rjmp	MAIN

;***********************************************************
;*	Functions and Subroutines
;***********************************************************

;----------------------------------------------------------------
; Sub:	Forward Action
; Desc:	Handles functionality of the Remote when Button 0
;		is triggered.
;----------------------------------------------------------------
ForwardAction:
		; Save register values by pushing to stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		in		mpr, SREG			; Save program state
		push	mpr					;
			
		; Checks to see if the data buffer is empty and ready for transmission of first 8-bit 
USART_Forward:
		lds		mpr, UCSR1A			; Loop until UDR1 is empty
		andi	mpr, (1<<UDRE1)		; Checks UDRE1 flag which will be set when it is empty
		breq	USART_Forward

		; Loads Bot Address into UDR1, first 8-bit signal to indicate address
		ldi		mpr, BotAddress
		sts		UDR1, mpr

		; Checks to see if the data buffer is empty and ready for transmission of second 8-bit 
USART_Forward2:
		lds		mpr, UCSR1A			; Loop until UDR1 is empty
		andi	mpr, (1<<UDRE1)		; Checks UDRE1 flag which will be set when it is empty
		breq	USART_Forward2

		; Loads Action Code into UDR1, second 8-bit signal to indicate address
		ldi		mpr, MovFwd
		sts		UDR1, mpr

		; Updates LEDs to indicate the corresponding button pressed
		ldi		mpr, 0b00000001
		out		PORTB, mpr

		; Clear register values by popping out of stack
		pop		mpr					; Restore program state
		out		SREG, mpr			;
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr
		ret							; Return from subroutine

;----------------------------------------------------------------
; Sub:	Backward Action
; Desc:	Handles functionality of the Remote when Button 1
;		is triggered.
;----------------------------------------------------------------
BackwardAction:
		; Save register values by pushing to stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		in		mpr, SREG			; Save program state
		push	mpr					;

		; Checks to see if the data buffer is empty and ready for transmission of first 8-bit 
USART_Backward:
		lds		mpr, UCSR1A			; Loop until UDR1 is empty
		andi	mpr, (1<<UDRE1)		; Checks UDRE1 flag which will be set when it is empty
		breq	USART_Backward

		; Loads Bot Address into UDR1, first 8-bit signal to indicate address
		ldi		mpr, BotAddress
		sts		UDR1, mpr
		
		; Checks to see if the data buffer is empty and ready for transmission of second 8-bit 
USART_Backward2:
		lds		mpr, UCSR1A			; Loop until UDR1 is empty
		andi	mpr, (1<<UDRE1)		; Checks UDRE1 flag which will be set when it is empty
		breq	USART_Backward2

		; Loads Action Code into UDR1, second 8-bit signal to indicate address
		ldi		mpr, MovBck
		sts		UDR1, mpr

		; Updates LEDs to indicate the corresponding button pressed
		ldi		mpr, 0b00000010
		out		PORTB, mpr

		; Clear register values by popping out of stack
		pop		mpr					; Restore program state
		out		SREG, mpr			;
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr
		ret							; Return from subroutine

;----------------------------------------------------------------
; Sub:	Turn Right Action
; Desc:	Handles functionality of the Remote when Button 4
;		is triggered.
;----------------------------------------------------------------
RightAction:
		; Save register values by pushing to stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		in		mpr, SREG			; Save program state
		push	mpr					;


		; Checks to see if the data buffer is empty and ready for transmission of first 8-bit 
USART_Right:
		lds		mpr, UCSR1A			; Loop until UDR1 is empty
		andi	mpr, (1<<UDRE1)		; Checks UDRE1 flag which will be set when it is empty
		breq	USART_Right
		
		; Loads Bot Address into UDR1, first 8-bit signal to indicate address
		ldi		mpr, BotAddress
		sts		UDR1, mpr

		; Checks to see if the data buffer is empty and ready for transmission of second 8-bit 
USART_Right2:
		lds		mpr, UCSR1A			; Loop until UDR1 is empty
		andi	mpr, (1<<UDRE1)		; Checks UDRE1 flag which will be set when it is empty
		breq	USART_Right2

		; Loads Action Code into UDR1, second 8-bit signal to indicate address
		ldi		mpr, TurnR
		sts		UDR1, mpr

		; Updates LEDs to indicate the corresponding button pressed
		ldi		mpr, 0b00010000
		out		PORTB, mpr

		; Clear register values by popping out of stack
		pop		mpr					; Restore program state
		out		SREG, mpr			;
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr
		ret							; Return from subroutine

;----------------------------------------------------------------
; Sub:	Turn Left Action
; Desc:	Handles functionality of the Remote when Button 5
;		is triggered.
;----------------------------------------------------------------
LeftAction:
		; Save register values by pushing to stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		in		mpr, SREG			; Save program state
		push	mpr					;

		; Checks to see if the data buffer is empty and ready for transmission of first 8-bit 
USART_Left:
		lds		mpr, UCSR1A			; Loop until UDR1 is empty
		andi	mpr, (1<<UDRE1)		; Checks UDRE1 flag which will be set when it is empty
		breq	USART_Left

		; Loads Bot Address into UDR1, first 8-bit signal to indicate address	
		ldi		mpr, BotAddress
		sts		UDR1, mpr

		; Checks to see if the data buffer is empty and ready for transmission of second 8-bit 
USART_Left2:
		lds		mpr, UCSR1A			; Loop until UDR1 is empty
		andi	mpr, (1<<UDRE1)		; Checks UDRE1 flag which will be set when it is empty
		breq	USART_Left2

		; Loads Action Code into UDR1, second 8-bit signal to indicate address
		ldi		mpr, TurnL
		sts		UDR1, mpr

		; Updates LEDs to indicate the corresponding button pressed
		ldi		mpr, 0b00100000
		out		PORTB, mpr

		; Clear register values by popping out of stack
		pop		mpr					; Restore program state
		out		SREG, mpr			;
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr
		ret							; Return from subroutine

;----------------------------------------------------------------
; Sub:	Halt Action
; Desc:	Handles functionality of the Remote when Button 6
;		is triggered.
;----------------------------------------------------------------
HaltAction:
		; Save register values by pushing to stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		in		mpr, SREG			; Save program state
		push	mpr					;

		; Checks to see if the data buffer is empty and ready for transmission of first 8-bit 
USART_Halt:
		lds		mpr, UCSR1A			; Loop until UDR1 is empty
		andi	mpr, (1<<UDRE1)		; Checks UDRE1 flag which will be set when it is empty
		breq	USART_Halt
		
		; Loads Bot Address into UDR1, first 8-bit signal to indicate address
		ldi		mpr, BotAddress
		sts		UDR1, mpr

		; Checks to see if the data buffer is empty and ready for transmission of second 8-bit 
USART_Halt2:
		lds		mpr, UCSR1A			; Loop until UDR1 is empty
		andi	mpr, (1<<UDRE1)		; Checks UDRE1 flag which will be set when it is empty
		breq	USART_Halt2

		; Loads Action Code into UDR1, second 8-bit signal to indicate address
		ldi		mpr, Halt
		sts		UDR1, mpr

		; Updates LEDs to indicate the corresponding button pressed
		ldi		mpr, 0b01000000
		out		PORTB, mpr

		; Clear register values by popping out of stack
		pop		mpr					; Restore program state
		out		SREG, mpr			;
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr
		ret							; Return from subroutine

;----------------------------------------------------------------
; Sub:	Freeze Action
; Desc:	Handles functionality of the Remote when Button 7
;		is triggered sending out a Freeze Action command
;----------------------------------------------------------------
FreezeAction:
		; Save register values by pushing to stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		in		mpr, SREG			; Save program state
		push	mpr					;

		; Checks to see if the data buffer is empty and ready for transmission of first 8-bit 
USART_Freeze:
		lds		mpr, UCSR1A			; Loop until UDR1 is empty
		andi	mpr, (1<<UDRE1)		; Checks UDRE1 flag which will be set when it is empty
		breq	USART_Freeze

		; Loads Bot Address into UDR1, first 8-bit signal to indicate address
		ldi		mpr, BotAddress
		sts		UDR1, mpr

		; Checks to see if the data buffer is empty and ready for transmission of second 8-bit 
USART_Freeze2:
		lds		mpr, UCSR1A			; Loop until UDR1 is empty
		andi	mpr, (1<<UDRE1)		; Checks UDRE1 flag which will be set when it is empty
		breq	USART_Freeze2

		; Loads Action Code into UDR1, second 8-bit signal to indicate address
		ldi		mpr, Freeze
		sts		UDR1, mpr

		; Updates LEDs to indicate the corresponding button pressed
		ldi		mpr, 0b10000000
		out		PORTB, mpr

		; Clear register values by popping out of stack
		pop		mpr					; Restore program state
		out		SREG, mpr			;
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr
		ret							; Return from subroutine

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
WaitFunc:
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

;***********************************************************
;*	Additional Program Includes
;***********************************************************