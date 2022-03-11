;***********************************************************
;*	This is the skeleton file for Lab 5 of ECE 375
;*
;*	 Author: Christian Ovchinikov
;*	   Date: 2/7/2022
;*
;***********************************************************

.include "m128def.inc"			; Include definition file

;***********************************************************
;*	Internal Register Definitions and Constants
;***********************************************************
.def	mpr = r16				; Multipurpose register
.def	rlo = r0				; Low byte of MUL result
.def	rhi = r1				; High byte of MUL result
.def	zero = r2				; Zero register, set to zero in INIT, useful for calculations
.def	A = r3					; A variable
.def	B = r4					; Another variable

.def	oloop = r17				; Outer Loop Counter
.def	iloop = r18				; Inner Loop Counter


;***********************************************************
;*	Start of Code Segment
;***********************************************************
.cseg							; Beginning of code segment

;-----------------------------------------------------------
; Interrupt Vectors
;-----------------------------------------------------------
.org	$0000					; Beginning of IVs
		rjmp 	INIT			; Reset interrupt

.org	$0046					; End of Interrupt Vectors

;-----------------------------------------------------------
; Program Initialization
;-----------------------------------------------------------
INIT:							; The initialization routine

		; Initialize Stack Pointer
		ldi		mpr, high(RAMEND)	; Loads higher byte of RAMEND into R16
		out		SPH, mpr			; Stores mpr(R16) into higher byte of Stack Pointer
		ldi		mpr, low(RAMEND)	; Loads lower byte of RAMEND into R16
		out		SPL, mpr			; Stores mpr into higher byte of Stack Pointer


		; TODO

		clr		zero			; Set the zero register to zero, maintain
										; these semantics, meaning, don't
										; load anything else into it.

;-----------------------------------------------------------
; Main Program
;-----------------------------------------------------------

;*
;* For this lab, you must follow these requirements.
;* - operands for each function need to be initially stored in program data.
;*	 (see "Stored Program Data" and "Data Memory Allocation" sections below)
;* - comments wrapped in stars(*) must be kept.
;* - In break points, data needs to be observed in memory window.
;*

MAIN:							; The Main program


;-----------------------------------------------------------
; Setup the ADD16 function direct test
;-----------------------------------------------------------
;*
;* Move values 0xFCBA and 0xFFFF in program memory to data memory (memory locations where ADD16 will get its inputs from)
;*

		; Set Z to point at low and high operands of ADD16_OperandA (0xFCBA) from Program Memory
		ldi		ZH, high(ADD16_OperandA<<1)	; Load high byte address from program memory ($__##)
		ldi		ZL, low(ADD16_OperandA<<1)	; Load low byte address from program memory ($##__)

		; Set Y to low and high byte of ADD16_OP1 from Data Memory ($0110)
		ldi		YH, high(ADD16_OP1)			; Load high byte address ($01)
		ldi		YL, low(ADD16_OP1)			; Load low byte address ($10)

		lpm		mpr, Z+			; Loads Z (low operand of ADD16_OperandA = 0xBA) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xBA) into Y then increments Y to $0111

		lpm		mpr, Z+			; Loads Z (high operand of ADD16_OperandA = 0xFC) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFC) into Y then increments Y to $0112

		lpm		mpr, Z+			; Loads Z (low operand of ADD16_OperandB = 0xFF) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y then increments Y to $0113

		lpm		mpr, Z+			; Loads Z (high operand of ADD16_OperandB = 0xFF) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y then increments Y to $0114

			nop ; * Let your TA check load ADD16 operands (Set Break point here #1) *

				; Call ADD16 function to test its correctness (calculate FCBA + FFFF)
		rcall	ADD16
        
			nop ; * Let your TA check ADD16 result (Set Break point here #2) *


;-----------------------------------------------------------
; Setup the SUB16 function direct test
;-----------------------------------------------------------
;*
;* Move values 0xFCB9 and 0xE420 in program memory to data memory (memory locations where ADD16 will get its inputs from)
;*

		; Set Z to point at low and high operands of SUB16_OP1 (0xFCB9) from Program Memory
		ldi		ZH, high(SUB16_OperandA<<1)	; Load high byte address from program memory ($__##)
		ldi		ZL, low(SUB16_OperandA<<1)	; Load low byte address from program memory ($##__)

		; Set Y to low and high byte of SUB16_OP1 from Data Memory ($0114)
		ldi		YH, high(SUB16_OP1)			; Load high byte address ($01)
		ldi		YL, low(SUB16_OP1)			; Load low byte address ($14)

		lpm		mpr, Z+			; Loads Z (low operand of SUB16_OperandA = 0xB9) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xB9) into Y then increments Y to $0115

		lpm		mpr, Z+			; Loads Z (high operand of SUB16_OperandA = 0xFC) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFC) into Y then increments Y to $0116

		lpm		mpr, Z+			; Loads Z (low operand of SUB16_OperandB = 0x20) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y then increments Y to $0117

		lpm		mpr, Z+			; Loads Z (high operand of SUB16_OperandB = 0xe4) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y then increments Y to $0118

			nop ; * Let your TA check load SUB16 operands (Set Break point here #3) *

				; Call SUB16 function to test its correctness (calculate FCB9 - E420)
		rcall	SUB16
			nop ; * Let your TA check SUB16 result (Set Break point here #4) *


;-----------------------------------------------------------
; Setup the MUL24 function direct test
;-----------------------------------------------------------
;*
;* Move values 0xFFFFFF and 0xFFFFFF in program memory to data memory (memory locations where MUL24 will get its inputs from)
;*

		; Set Z to point at low and high operands of MUL24_OperandA (0xFFFF) from Program Memory
		ldi		ZH, high(MUL24_OperandA<<1)	; Load high byte address from program memory ($__##)
		ldi		ZL, low(MUL24_OperandA<<1)	; Load low byte address from program memory ($##__)

		; Set Y to low and high byte of MUL24_OP1 from Data Memory ($0118)
		ldi		YH, high(MUL24_OP1)			; Load high byte address ($01)
		ldi		YL, low(MUL24_OP1)			; Load low byte address ($18)

		lpm		mpr, Z+			; Loads Z (low operand of MUL24_OperandA = 0xFF) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y then increments Y to $0119

		lpm		mpr, Z+			; Loads Z (high operand of MUL24_OperandA = 0xFF) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y then increments Y to $011A

		lpm		mpr, Z+			; Loads Z (low operand of MUL24_OperandA = 0xFF) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y then increments Y to $011B

		lpm		mpr, Z+			; Loads Z (high operand of MUL24_OperandA = 0x00) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0x00) into Y then increments Y to $011C

		lpm		mpr, Z+			; Loads Z (low operand of MUL24_OperandB = 0xFF) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y then increments Y to $011D
		
		lpm		mpr, Z+			; Loads Z (high operand of MUL24_OperandB = 0xFF) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y then increments Y to $011E

		lpm		mpr, Z+			; Loads Z (low operand of MUL24_OperandB = 0xFF) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y then increments Y to $011F

		lpm		mpr, Z+			; Loads Z (high operand of MUL24_OperandB = 0x00) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0x00) into Y then increments Y to $011G

			nop ; * Let your TA check load SUB16 operands (Set Break point here #5) *

				; Call MUL24 function to test its correctness (calculate FFFFFF * FFFFFF)
		rcall	MUL24
			nop ; * Let your TA check SUB16 result (Set Break point here #6) *


;-----------------------------------------------------------
; Setup the COMPOUND function direct test
;-----------------------------------------------------------
;*
;* Move values 0xFCBA, 0x2019, 0x21BB in program memory to data memory (memory locations = OperandD, OperandE, OperandF)
;*

		; Set Z to point at low and high operands of OperandD (0xFCBA) from Program Memory
		ldi		ZH, high(OperandD<<1)	; Load high byte address from program memory ($__##)
		ldi		ZL, low(OperandD<<1)	; Load low byte address from program memory ($##__)

		; Set Y to low and high byte of SUB16_OP1 from Data Memory ($0114)
		ldi		YH, high(SUB16_OP1)			; Load high byte address ($01)
		ldi		YL, low(SUB16_OP1)			; Load low byte address ($14)

		lpm		mpr, Z+			; Loads Z (low operand of OperandD = 0xBA) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y at SUB16_OP1 then increments Y to $0115

		lpm		mpr, Z+			; Loads Z (high operand of OperandD = 0xFC) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y at SUB16_OP1 then increments Y to $0116

		lpm		mpr, Z+			; Loads Z (low operand of OperandE = 0x19) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y at SUB16_OP2 then increments Y to $0117

		lpm		mpr, Z+			; Loads Z (low operand of OperandE = 0x20) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y at SUB16_OP2 then increments Y to $0118

		; Set Y to low and high byte of ADD16_OP1 from Data Memory ($0110)
		ldi		YH, high(ADD16_OP1)			; Load high byte address ($01)
		ldi		YL, low(ADD16_OP1)			; Load low byte address ($10)

		lpm		mpr, Z+			; Loads Z (low operand of OperandF = 0xBB) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y then increments Y to $0110
		
		lpm		mpr, Z+			; Loads Z (high operand of OperandF = 0x21) from program memory to mpr, then increments Z
		st		Y+, mpr			; Stores mpr (0xFF) into Y then increments Y to $0111

			nop ; * Let your TA check load COMPOUND operands (Set Break point here #7) * (STORED AT $0111:$0110 and $0117:$0114)

				; Call the COMPOUND function
		rcall	COMPOUND
			nop ; * Let your TA check COMPUND result (Set Break point here #8) * (STORED AT $0137:$0132)



DONE:	rjmp	DONE			; Create an infinite while loop to signify the
								; end of the program.

;***********************************************************
;*	Functions and Subroutines
;***********************************************************

;-----------------------------------------------------------
; Func: ADD16
; Desc: Adds two 16-bit numbers and generates a 24-bit number
;		where the high byte of the result contains the carry
;		out bit.
;-----------------------------------------------------------
ADD16:
		push 	A				; Save A register
		push 	mpr				; Save mpr register
		push	XH				; Save X-ptr
		push	XL
		push	YH				; Save Y-ptr
		push	YL
		push	ZH				; Save Z-ptr
		push	ZL

		; Load beginning address of first operand into X (First 8-bits at $0110)
		ldi		XL, low(ADD16_OP1)	; Load low byte of address ($10)
		ldi		XH, high(ADD16_OP1)	; Load high byte of address ($01)

		; Load beginning address of second operand into Y (First 8-bits at $0112)
		ldi		YL, low(ADD16_OP2)	; Load low byte of address ($12)
		ldi		YH, high(ADD16_OP2)	; Load high byte of address ($01)

		; Load beginning address of result into Z (First 8-bits at $0120)
		ldi		ZL, low(ADD16_Result)	; Load low byte of address ($20)
		ldi		ZH, high(ADD16_Result)	; Load high byte of address ($01)

		; Perform addition on first 8-bits of each operand
		ld		mpr, X+		; Load beginning address of first operand from X into mpr, then increments X to $0111
		ld		A, Y+		; Load beginning address of second operand from Y into A (r3), then increments Y to $0113
		add		A, mpr		; Adds beginning of first and second operand in A and mpr and place into A
		st		Z+, A		; Stores the sum of beginning first and second operand into Z and increments Z to $0121
		
		; Perform addition on second 8-bits of each operand
		ld		mpr, X		; Load ending address of first operand from X into mpr
		ld		A, Y		; Load ending address of second operand from Y into A
		adc		A, mpr		; Adds ending of first and second operand in A and mpr and place into A
		st		Z+, A		; Stores the sum of ending first and second operand into Z and increments Z to $0122
		brcc	EXIT_ADD16	; If carry is clear, skip to EXIT; if carry is set, continue to store it
		st		Z, XH		; Stores carry into Z at $0122
EXIT_ADD16:

		pop		ZL				; Restore all registers in reverves order
		pop		ZH
		pop		YL
		pop		YH
		pop		XL
		pop		XH
		pop		mpr
		pop		A

		ret						; End a function with RET

;-----------------------------------------------------------
; Func: SUB16
; Desc: Subtracts two 16-bit numbers and generates a 16-bit
;		result.
;-----------------------------------------------------------
SUB16:
		push 	A				; Save A register
		push 	mpr				; Save mpr register
		push	XH				; Save X-ptr
		push	XL
		push	YH				; Save Y-ptr
		push	YL
		push	ZH				; Save Z-ptr
		push	ZL

		; Load beginning address of first operand into X (First 8-bits at $0114)
		ldi		XL, low(SUB16_OP1)	; Load low byte of address ($14)
		ldi		XH, high(SUB16_OP1)	; Load high byte of address ($01)

		; Load beginning address of second operand into Y (First 8-bits at $0116)
		ldi		YL, low(SUB16_OP2)	; Load low byte of address ($16)
		ldi		YH, high(SUB16_OP2)	; Load high byte of address ($01)

		; Load beginning address of result into Z (First 8-bits at $0123)
		ldi		ZL, low(SUB16_Result)	; Load low byte of address ($23)
		ldi		ZH, high(SUB16_Result)	; Load high byte of address ($01)

		; Perform subtraction on first 8-bits of each operand
		ld		mpr, X+		; Load beginning address of first operand from X into mpr, then increments X to $0114
		ld		A, Y+		; Load beginning address of second operand from Y into A (r3), then increments Y to $0117
		sub		mpr, A		; Subtracts beginning of first and second operand in mpr and A and place into mpr
		st		Z+, mpr		; Stores the difference of beginning first and second operand into Z and increments Z to $0124
		
		; Perform addition on second 8-bits of each operand
		ld		mpr, X		; Load ending address of first operand from X into mpr
		ld		A, Y		; Load ending address of second operand from Y into A
		sbc		mpr, A		; Subtracts ending of first and second operand in mpr and A and place into mpr
		st		Z+, mpr		; Stores the difference of ending first and second operand into Z and increments Z to $0125
		brcc	EXIT_SUB16	; If carry is clear, skip to EXIT; if carry is set, continue to store it
		st		Z, XH		; Stores carry into Z at $0125
EXIT_SUB16:

		pop		ZL				; Restore all registers in reverves order
		pop		ZH
		pop		YL
		pop		YH
		pop		XL
		pop		XH
		pop		mpr
		pop		A

		ret						; End a function with RET

;-----------------------------------------------------------
; Func: MUL24
; Desc: Multiplies two 24-bit numbers and generates a 48-bit
;		result.
;-----------------------------------------------------------
MUL24:
		push 	A				; Save A register
		push	B				; Save B register
		push	rhi				; Save rhi register
		push	rlo				; Save rlo register
		push	zero			; Save zero register
		push	mpr				; Save mpr register
		push	XH				; Save X-ptr
		push	XL
		push	YH				; Save Y-ptr
		push	YL
		push	ZH				; Save Z-ptr
		push	ZL
		push	oloop			; Save counters
		push	iloop

		clr		zero			; Maintain zero semantics

		; Set Y to beginning address of MUL24_OP1 (First 8-bits at $0118)
		ldi		YH, high(MUL24_OP1)	; Load high byte ($01)
		ldi		YL, low(MUL24_OP1)	; Load low byte ($18)

		; Set Z to begginning address of resulting Product at MUL24_Result ($0126)
		ldi		ZH, high(MUL24_Result)	; Load high byte ($01)
		ldi		ZL, low(MUL24_Result)	; Load low byte ($26)

		; Begin outer for loop
		ldi		oloop, 3		; Load counter

MUL24_OLOOP:
		; Set X to beginning address of MUL24_OP2 (First 8-bits at $011C)
		ldi		XH, high(MUL24_OP2)	; Load high byte ($01)
		ldi		XL, low(MUL24_OP2)	; Load low byte ($1C)

		; Begin inner for loop
		ldi		iloop, 3		; Load counter
		cpi		oloop, 3		; Check loop status
		breq	MUL24_ILOOP		; Each outer loop will be shifted over one byte
		sbiw	ZH:ZL, 1		; Z <= Z - 1

MUL24_ILOOP:
		ld		A, X+			; Get byte of A operand (0xFF) and increment X
		ld		B, Y			; Get byte of B operand (0xFF)
		mul		A,B				; Multiply A and B (0xFF * 0xFF = 0xFE01) and store in r1:r0
		ld		A, Z+			; Get a result byte from memory and increments Z
		ld		B, Z+			; Get the next result byte from memory and increments Z
		ld		mpr, Z			; Gets the furthest byte for carry
		add		rlo, A			; rlo <= rlo + A
		adc		rhi, B			; rhi <= rhi + B + carry /////////
		ld		A, Z			; Get a third byte from the result
		adc		A, zero			; Add carry to A
		brcc	SKIP_MUL24		; Skips if carry is not set
		adc		mpr, zero		; Adds carry to mpr
		brcc	SKIP_MUL24		; Skips if carry is not set
		adc		mpr, zero		; Adds additional carry to mpr
		adiw	ZH:ZL, 1		; Z <= Z + 1
		st		Z, mpr			; Stores carry from mpr into Z
		sbiw	ZH:ZL, 1		; Z <= Z - 1

SKIP_MUL24:
		st		Z, A			; Store third byte to memory
		st		-Z, rhi			; Store second byte to memory
		st		-Z, rlo			; Store first byte to memory
		adiw	ZH:ZL, 1		; Z <= Z + 1
		dec		iloop			; Decrement counter
		brne	MUL24_ILOOP		; Loop if iLoop != 0
		; End inner for loop

		sbiw	ZH:ZL, 1		; Z <= Z - 1
		adiw	YH:YL, 1		; Y <= Y + 1
		dec		oloop			; Decrement counter
		brne	MUL24_OLOOP		; Loop if oLoop != 0
		; End outer for loop

		pop		iloop			; Restore all registers in reverves order
		pop		oloop
		pop		ZL
		pop		ZH
		pop		YL
		pop		YH
		pop		XL
		pop		XH
		pop		mpr
		pop		zero
		pop		rlo
		pop		rhi
		pop		B
		pop		A
		ret						; End a function with RET

;-----------------------------------------------------------
; Func: COMPOUND
; Desc: Computes the compound expression ((D - E) + F)^2
;		by making use of SUB16, ADD16, and MUL24.
;
;		D, E, and F are declared in program memory, and must
;		be moved into data memory for use as input operands.
;
;		All result bytes should be cleared before beginning.
;-----------------------------------------------------------
COMPOUND:

		; Setup SUB16 with operands D and E
		; Perform subtraction to calculate D - E
		call SUB16

		; Setup the ADD16 function with SUB16 result and operand F
		; Perform addition next to calculate (D - E) + F
		ldi		ZH, high(SUB16_Result)	; Load high byte ($01)
		ldi		ZL, low(SUB16_Result)	; Load low byte ($23)

		ldi		YH, high(ADD16_OP2)		; Load high byte of address ($01)
		ldi		YL, low(ADD16_OP2)		; Load low byte of address ($12)

		; This will move the results from SUB16 to the second operand of ADD16 at ADD16_OP2
		ld		A, Z+					; Load the lower subtraction result byte into A
		st		Y+, A					; Store the lower result byte into ADD16_OP2
		ld		A, Z					; Load the higher subtraction result byte into A
		st		Y, A					; Store the higher result byte into ADD16_OP2

		call ADD16

		ldi		ZH, high(ADD16_Result)	; Load high byte ($01)
		ldi		ZL, low(ADD16_Result)	; Load low byte ($20)

		ldi		YH, high(addrA)		; Load high byte of address ($01)
		ldi		YL, low(addrA)		; Load low byte of address ($00)

		; This will move the result from ADD16 to the first and second operand of MUL16
		ld		A, Z+					; Load the lower addition result byte into A, then increments Z to $0121
		st		Y+, A					; Store the lower addition result byte into addrA (first operand of MUL16), then increments Y to $0101
		ld		A, Z					; Load the higher addition result byte into A
		st		Y+, A					; Store the higher addition result byte into addrA (first operand of MUL16), then increments Y to $0102
		
		; Reloop through the same addition result values since we are multiplying the same values together
		sbiw	ZH:ZL, 1			; Z <= Z - 1 = $0120
		ld		A, Z+					; Load the lower addition result byte into A, then increments Z to $0121
		st		Y+, A					; Store the lower addition result byte into addrB (second operand of MUL16), then increments Y to $0103
		ld		A, Z					; Load the higher addition result byte into A
		st		Y+, A					; Store the higher addition result byte into addrB (second operand of MUL16), then increments Y to $0104

		; Setup the MUL24 function with ADD16 result as both operands
		; Perform multiplication to calculate ((D - E) + F)^2
		call MUL16

		ldi		ZH, high(LaddrP)				; Load high byte ($01)
		ldi		ZL, low(LaddrP)					; Load low byte ($04)

		ldi		YH, high(COMPOUND_Result)		; Load high byte of address ($01)
		ldi		YL, low(COMPOUND_Result)		; Load low byte of address ($32)
		
		ld		A, Z+							; Load the first MUL16 result byte into A, then increments Z to $0105
		st		Y+, A							; Store the first MUL16 result byte into COMPOUND_Result ($0132), then increments Y to $0133
		ld		A, Z+							; Load the second MUL16 result byte into A, then increments Z to $0106
		st		Y+, A							; Store the second MUL16 result byte into COMPOUND_Result ($0133), then increments Y to $0134
		ld		A, Z+							; Load the third MUL16 result byte into A, then increments Z to $0107
		st		Y+, A							; Store the third MUL16 result byte into COMPOUND_Result ($0134), then increments Y to $0135
		ld		A, Z							; Load the fourth MUL16 result byte into A
		st		Y, A							; Store the fourth MUL16 result byte into COMPOUND_Result ($0135)

		ret						; End a function with RET

;-----------------------------------------------------------
; Func: MUL16
; Desc: An example function that multiplies two 16-bit numbers
;			A - Operand A is gathered from address $0101:$0100
;			B - Operand B is gathered from address $0103:$0102
;			Res - Result is stored in address
;					$0107:$0106:$0105:$0104
;		You will need to make sure that Res is cleared before
;		calling this function.
;-----------------------------------------------------------
MUL16:
		push 	A				; Save A register
		push	B				; Save B register
		push	rhi				; Save rhi register
		push	rlo				; Save rlo register
		push	zero			; Save zero register
		push	XH				; Save X-ptr
		push	XL
		push	YH				; Save Y-ptr
		push	YL
		push	ZH				; Save Z-ptr
		push	ZL
		push	oloop			; Save counters
		push	iloop

		clr		zero			; Maintain zero semantics

		; Set Y to beginning address of B
		ldi		YL, low(addrB)	; Load low byte
		ldi		YH, high(addrB)	; Load high byte

		; Set Z to begginning address of resulting Product
		ldi		ZL, low(LAddrP)	; Load low byte
		ldi		ZH, high(LAddrP); Load high byte

		; Begin outer for loop
		ldi		oloop, 2		; Load counter
MUL16_OLOOP:
		; Set X to beginning address of A
		ldi		XL, low(addrA)	; Load low byte
		ldi		XH, high(addrA)	; Load high byte

		; Begin inner for loop
		ldi		iloop, 2		; Load counter
MUL16_ILOOP:
		ld		A, X+			; Get byte of A operand
		ld		B, Y			; Get byte of B operand
		mul		A,B				; Multiply A and B
		ld		A, Z+			; Get a result byte from memory
		ld		B, Z+			; Get the next result byte from memory
		add		rlo, A			; rlo <= rlo + A
		adc		rhi, B			; rhi <= rhi + B + carry
		ld		A, Z			; Get a third byte from the result
		adc		A, zero			; Add carry to A
		st		Z, A			; Store third byte to memory
		st		-Z, rhi			; Store second byte to memory
		st		-Z, rlo			; Store first byte to memory
		adiw	ZH:ZL, 1		; Z <= Z + 1
		dec		iloop			; Decrement counter
		brne	MUL16_ILOOP		; Loop if iLoop != 0
		; End inner for loop

		sbiw	ZH:ZL, 1		; Z <= Z - 1
		adiw	YH:YL, 1		; Y <= Y + 1
		dec		oloop			; Decrement counter
		brne	MUL16_OLOOP		; Loop if oLoop != 0
		; End outer for loop

		pop		iloop			; Restore all registers in reverves order
		pop		oloop
		pop		ZL
		pop		ZH
		pop		YL
		pop		YH
		pop		XL
		pop		XH
		pop		zero
		pop		rlo
		pop		rhi
		pop		B
		pop		A
		ret						; End a function with RET

;***********************************************************
;*	Stored Program Data
;*  Enter any stored data you need here
;***********************************************************
; ADD16 operands
ADD16_OperandA:
	.DW 0xFCBA				; test value A for ADD16 = 64698
ADD16_OperandB:
	.DW 0xFFFF				; test value B for ADD16 = 65535

; SUB16 operands
SUB16_OperandA:
	.DW 0xFCB9				; test value A for SUB16 = 64697
SUB16_OperandB:
	.DW 0xE420				; test value B for SUB16 = 58400
; MUL24 operands
MUL24_OperandA:
	.DW 0xFFFF, 0x00FF		; test value A for MUL24 = 16777215, cannot be > 0xFFFF (65535)
MUL24_OperandB:
	.DW 0xFFFF, 0x00FF		; test value B for MUL24 = 16777215

; Compound operands
OperandD:
	.DW	0xFCBA				; test value for operand D
OperandE:
	.DW	0x2019				; test value for operand E
OperandF:
	.DW	0x21BB				; test value for operand F

;***********************************************************
;*	Data Memory Allocation
;***********************************************************
.dseg
.org	$0100				; data memory allocation for MUL16 example
addrA:	.byte 2																;	($0101:$0100)
addrB:	.byte 2																;	($0103:$0102)
LAddrP:	.byte 4																;	($0107:$0104)

; Below is an example of data memory allocation for ADD16.
; Consider using something similar for SUB16 and MUL24.
.org	$0110				; data memory allocation for operands

; Addition Operand
ADD16_OP1:
		.byte 2				; allocate two bytes for first operand of ADD16		($0111:$0110)
ADD16_OP2:
		.byte 2				; allocate two bytes for second operand of ADD16	($0113:$0112)

; Subtraction Operand
SUB16_OP1:
		.byte 2				; allocate two bytes for first operand of SUB16		($0115:$0114)
SUB16_OP2:
		.byte 2				; allocate two bytes for second operand of SUB16	($0117:$0116)
		
; Multiplication Operand
MUL24_OP1:
		.byte 4				; allocate four bytes for first operand of MUL24	($011B:$0118)
MUL24_OP2:
		.byte 4				; allocate four bytes for second operand of MUL24	($011F:$011C)

.org	$0120				; data memory allocation for results
ADD16_Result:
		.byte 3				; allocate three bytes for ADD16 result				($0122:$0120)
SUB16_Result:
		.byte 3				; allocate three bytes for SUB16 result				($0125:$0123)
MUL24_Result:
		.byte 6				; allocate six bytes for MUL24 result				($012B:$0126)

; Compound Operand and Result
COMPOUND_OPD:
		.byte 2				; allocate two bytes for COMPOUND OperandD			($012D:$012C)
COMPOUND_OPE:
		.byte 2				; allocate two bytes for COMPOUND OperandD			($012F:$012E)
COMPOUND_OPF:
		.byte 2				; allocate two bytes for COMPOUND OperandD			($0131:$0130)

COMPOUND_Result:
		.byte 6				; allocate six bytes for COMPOUND result			($0137:$0132)
;***********************************************************
;*	Additional Program Includes
;***********************************************************
; There are no additional file includes for this program