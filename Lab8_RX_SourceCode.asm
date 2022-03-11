;***********************************************************
;*
;*	This is the RECEIVE skeleton file for Lab 8 of ECE 375
;*
;*	 Author: Christian Ovchinikov
;*	   Date: 3/7/2022
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multi-Purpose Register
.def	freezecnt = r17			; Freeze Counter
.def	curAction = r18			; Current Action Register
.def	waitcnt = r23			; Wait Loop Counter
.def	ilcnt = r24				; Inner Loop Counter
.def	olcnt = r25				; Outer Loop Counter

.equ	WTime = 100				; Time to wait in wait loop
.equ	WskrR = 0				; Right Whisker Input Bit
.equ	WskrL = 1				; Left Whisker Input Bit
.equ	EngEnR = 4				; Right Engine Enable Bit
.equ	EngEnL = 7				; Left Engine Enable Bit
.equ	EngDirR = 5				; Right Engine Direction Bit
.equ	EngDirL = 6				; Left Engine Direction Bit

.equ	BotAddress = $08		; RX Bot Address = 0b00001000

;/////////////////////////////////////////////////////////////
;These macros are the values to make the TekBot Move.
;/////////////////////////////////////////////////////////////
.equ	MovFwd =  (1<<EngDirR|1<<EngDirL)	;0b01100000 Move Forward Action Code
.equ	MovBck =  $00						;0b00000000 Move Backward Action Code
.equ	TurnR =   (1<<EngDirL)				;0b01000000 Turn Right Action Code
.equ	TurnL =   (1<<EngDirR)				;0b00100000 Turn Left Action Code
.equ	Halt =    (1<<EngEnR|1<<EngEnL)		;0b10010000 Halt Action Code
.equ	Freeze = 0b11111000					;0b11111000 Freeze Action Code

;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;***********************************************************
;*	Interrupt Vectors
;***********************************************************
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt

;Should have Interrupt vectors for:

;- Left whisker
.org	$0002
		rcall	HitRight		; Hit Right interrupt
		reti

;- Right whisker
.org	$0004
		rcall	HitLeft			; Hit Left interrupt
		reti

;- USART receive
.org	$003C
		rcall	USART_Receive	; USART Receive interrupt
		reti

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
		ldi		mpr, 0b00000011		; Sets mpr to 1 for input buttons in Port D
		out		PORTD, mpr			; Loads mpr into Port B Data Register with 1's which switches on the pull-up resistor configuring it as an input

	; USART1 
		; Initialize USART1
		ldi mpr, (1<<U2x1)			; Set double data rate (Bit 1)
		sts UCSR1A, mpr				; Send out to UCSR1A

		; Set baudrate at 2400bps
		ldi		mpr, high(832)			; Load high byte of 0x0340
		sts		UBRR1H, mpr				; UBRR1H in extended I/O space
		ldi		mpr, low(832)			; Load low byte of 0x0340
		sts		UBRR1L, mpr				; 832 was used for UBRR because U2x1 Bit 1 = 1, from page 196 ATmega128, Table 85
		
		; Enable receiver and enable receive interrupts
		ldi		mpr, (1<<RXEN1 | 1<<TXEN1 | 1<<RXCIE1)			; Sets mpr to 0b10010000 enabling RX Interrupt, Receiver and TX Transmitter for Freeze
		sts		UCSR1B, mpr										;

		; Set frame format: 8 data, 2 stop bits, asynchronous
		ldi		mpr, (1<<USBS1 | 1<<UCSZ11 | 1<<UCSZ10)			; Sets mpr to 0bX0XX111X enabling 2-bits, and 8-bit data.
		sts		UCSR1C, mpr										; UCSR1C in extended I/O space

	; External Interrupts
		; Set the Interrupt Sense Control to falling edge detection
		ldi		mpr, 0b00001010		; Sets INT0 and INT1 to falling edge which generates asynchronously an interrupt request
		sts		EICRA, mpr			; Stores the falling edge values to data space in External Interrupt Control Register A which is for INT0-INT3
		
		; Set the External Interrupt Mask
		ldi		mpr, 0b00000011		; Loads into mpr the 1 value for the corresponding INTx interrupts that will enable them
		out		EIMSK, mpr			; Sends out mpr into EIMSK setting the Interrupt Enable bit

	; Set TekBot to Move Forward (1<<EngDirR|1<<EngDirL) and intialize other variables
		ldi		mpr, MovFwd				; Loads value defined by MovFwd for Move Forward command
		mov		curAction, mpr			; Copy current action from mpr to curAction register for initial state
		out		PORTB, mpr				; Sends Move Forward command to Port B (output)
		ldi		freezecnt, 0b00000000	; Initialize freezecnt to be 0 at the beginning of the board starting
		sei
		

;***********************************************************
;*	Main Program
;***********************************************************
MAIN:
	;TODO: ???
		rjmp	MAIN

;***********************************************************
;*	Functions and Subroutines
;***********************************************************

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

		; Disable Receive Interrupt in Bit 4 of UCSR1B
		lds		mpr, UCSR1B			; Loads current value set in USART1 Control/Status Register B
		andi	mpr, 0b11101111		; Sets the Bit 4 of UCSR1B to 0 disabling
		sts		UCSR1B, mpr			; Sends out to UCSR1B

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

		; Move Forward again	
		ldi		mpr, MovFwd	; Load Move Forward command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime		; Wait for 1 second
		rcall	WaitFunc			; Call wait function

		; Return to Prior Action
		out		PORTB, curAction	; Returns LEDs to previous action

		; Clear queued interrupts
		ldi		mpr, 0b00000011		; Writing a logical 1 to 0-1 bits will which will clear the corresponding interrupt flag in INT0-1
		out		EIFR, mpr			; Clears queued interrupts

		; Re-enable receiver and enable receive interrupts
		ldi		mpr, (1<<RXEN1 | 1<<TXEN1 | 1<<RXCIE1)			; Sets mpr to 0b10010000 enabling RX Interrupt, Receiver and TX Transmitter for Freeze
		sts		UCSR1B, mpr

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
		; Save register values by pushing to stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		in		mpr, SREG			; Save program state
		push	mpr					;

		; Disable Receive Interrupt in Bit 4 of UCSR1B
		lds		mpr, UCSR1B			; Loads current value set in USART1 Control/Status Register B
		andi	mpr, 0b11101111		; Sets the Bit 4 of UCSR1B to 0 disabling
		sts		UCSR1B, mpr			; Sends out to UCSR1B

		; Move Backwards for a second
		ldi		mpr, MovBck			; Load Move Backward command
		out		PORTB, mpr			; Send command to port
		ldi		waitcnt, WTime		; Wait for 1 second
		rcall	WaitFunc			; Call wait function

		; Turn right for a second
		ldi		mpr, TurnR			; Load Turn Left Command
		out		PORTB, mpr			; Send command to port
		ldi		waitcnt, WTime		; Wait for 1 second
		rcall	WaitFunc			; Call wait function

		; Move Forward again	
		ldi		mpr, MovFwd	; Load Move Forward command
		out		PORTB, mpr	; Send command to port
		ldi		waitcnt, WTime		; Wait for 1 second
		rcall	WaitFunc			; Call wait function

		; Return to Prior Action
		out		PORTB, curAction	; Returns LEDs to previous action

		; Clear queued interrupts
		ldi		mpr, 0b00000011		; Writing a logical 1 to 0-1 bits will which will clear the corresponding interrupt flag in INT0-1
		out		EIFR, mpr			; Clears queued interrupts

		; Re-enable receiver and enable receive interrupts
		ldi		mpr, (1<<RXEN1 | 1<<TXEN1 | 1<<RXCIE1)			; Sets mpr to 0b10010000 enabling RX Interrupt, Receiver and TX Transmitter for Freeze
		sts		UCSR1B, mpr

		; Clear register values by popping out of stack
		pop		mpr					; Restore program state
		out		SREG, mpr			;
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr
		ret							; Return from subroutine

;----------------------------------------------------------------
; Sub:	USART_Receive
; Desc:	Handles functionality of the TekBot receiving incoming
;		USART data.
;----------------------------------------------------------------
USART_Receive:
		; Save register values by pushing to stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		in		mpr, SREG			; Save program state
		push	mpr					;

		; Checks if incoming byte is for the right bot address
		lds		mpr, UDR1			; Get data from Receive Data Buffer

		; Check for incoming Freeze Attack
		cpi		mpr, 0b01010101		; Checks if the incoming data is a Freeze Attack from another robot
		brne	Receive_BOTCHECK	; If not, skip to Bot Address identification
		inc		freezecnt			; Increments Freeze Counter
		rcall	FreezeTag			; If so, call FreezeTag and end receiving transmission
		rjmp	Receive_END

Receive_BOTCHECK:
		cpi		mpr, BotAddress		; Checks first bytes including the Bot Address
		brne	Receive_END			; Branches if the Bot Address is not equal to the incoming transmission

		; Reads and executes action code
		lds		mpr, UDR1			; Gets data from Receive Data Buffer again (2nd pair of bytes including the Action code)

		; Checks if Freeze Action Command is present
		cpi		mpr, Freeze
		brne	Receive_ACTION
		rcall	USART_TXFreeze		; If so, call USART_TXFreeze to send Freeze signal to another robot
		rjmp	Receive_END

Receive_ACTION:
		lsl		mpr					; Action codes are shifted 1 bit between RX and TX
		mov		curAction, mpr		; Copies the current data (Action Code) into the saved Current Action Register
		out		PORTB, mpr			; Send command to port
		ldi		waitcnt, WTime		; Wait for 1 second
		rcall	WaitFunc			; Call wait function

Receive_END:
		; Clear queued interrupts
		ldi		mpr, 0b00000011		; Writing a logical 1 to 0-1 bits will which will clear the corresponding interrupt flag in INT0-1
		out		EIFR, mpr			; Clears queued interrupts

		; Clear register values by popping out of stack
		pop		mpr					; Restore program state
		out		SREG, mpr			;
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr
		ret							; Return from subroutine

;----------------------------------------------------------------
; Sub:	FreezeTag
; Desc:	Handles functionality of the TekBot when a freeze signal
;		is sent directly to the robot.
;----------------------------------------------------------------
FreezeTag:
		; Save register values by pushing to stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		in		mpr, SREG			; Save program state
		push	mpr					;

		; Disable Receive Interrupt in Bit 4 of UCSR1B
		lds		mpr, UCSR1B			; Loads current value set in USART1 Control/Status Register B
		andi	mpr, 0b11101111		; Sets the Bit 4 of UCSR1B to 0 disabling
		sts		UCSR1B, mpr			; Sends out to UCSR1B

		; Display to LED output that the robot is frozen
		ldi		mpr, 0b11110000		; Loads value into mpr to be outputted to Port B
		out		PORTB, mpr			; Sets LEDs to display Freeze

		; Increments Freeze Counter and checks if we reached 3 freezes
		cpi		freezecnt, 3		; Checks to see if we reached 3 freezes
		brne	FreezeTag_SKIP	; If 3 freezes occur, end function without enabling interrupts

FreezeTag_STUCK:
		rjmp	FreezeTag_STUCK

FreezeTag_SKIP:		
		; Wait for 5 seconds
		ldi		waitcnt, 250		; Wait for 2.5 second
		rcall	WaitFunc			; Call wait function
		rcall	WaitFunc			; Call wait function

		; Return to Prior Action
		out		PORTB, curAction	; Returns LEDs to previous action

		; Clear queued interrupts
		ldi		mpr, 0b00000011		; Writing a logical 1 to 0-1 bits will which will clear the corresponding interrupt flag in INT0-1
		out		EIFR, mpr			; Clears queued interrupts

		; Re-enable receiver and enable receive interrupts
		ldi		mpr, (1<<RXEN1 | 1<<TXEN1 | 1<<RXCIE1)		; Sets mpr to 0b10011000 enabling RX Interrupt, Receiver and TX Transmitter for Freeze
		sts		UCSR1B, mpr

		; Clear register values by popping out of stack
		pop		mpr					; Restore program state
		out		SREG, mpr			;
		pop		waitcnt				; Restore wait register
		pop		mpr					; Restore mpr
		ret							; Return from subroutine

;----------------------------------------------------------------
; Sub:	USART_TXFreeze
; Desc:	Handles functionality of the TekBot when a Freeze Action
;		is sent to the robot to be sent back out.
;----------------------------------------------------------------
USART_TXFreeze:
		; Save register values by pushing to stack
		push	mpr					; Save mpr register
		push	waitcnt				; Save wait register
		in		mpr, SREG			; Save program state
		push	mpr					;

		; Disable Receive Interrupt in Bit 4 of UCSR1B
		lds		mpr, UCSR1B			; Loads current value set in USART1 Control/Status Register B
		andi	mpr, 0b11101111		; Sets the Bit 4 of UCSR1B to 0 disabling
		sts		UCSR1B, mpr			; Sends out to UCSR1B

		; Checks to see if the data buffer is empty and ready for transmission of first 8-bit 
TX_USART:
		lds		mpr, UCSR1A			; Loop until UDR1 is empty
		andi	mpr, (1<<UDRE1)		; Checks UDRE1 flag which will be set when it is empty
		breq	TX_USART

		ldi		mpr, 0b01010101		; Send out Freeze Signal to UDR1 (Transmit Data Buffer Register)
		sts		UDR1, mpr

		; Clear queued interrupts
		ldi		mpr, 0b00000011		; Writing a logical 1 to 0-1 bits will which will clear the corresponding interrupt flag in INT0-1
		out		EIFR, mpr			; Clears queued interrupts

		; Re-enable receiver and enable receive interrupts
		ldi		mpr, (1<<RXEN1 | 1<<TXEN1 | 1<<RXCIE1)		; Sets mpr to 0b10010000 enabling RX Interrupt, Receiver and TX Transmitter for Freeze
		sts		UCSR1B, mpr

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
		push	freezecnt

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
		pop		freezecnt
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