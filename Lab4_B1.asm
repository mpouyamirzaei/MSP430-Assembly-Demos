;-------------------------------------------------------------------------------
; Laboratory 4 (Servo and Stepper Motor) Part 1
; M. Pouya Mirzaei
;-------------------------------------------------------------------------------
            .cdecls C,LIST,"msp430FR2433.h"       ; Include device header file
            
;-------------------------------------------------------------------------------
            .def    RESET                   ; Export program entry-point to
                                            ; make it known to linker
;-------------------------------------------------------------------------------
            .text                           ; Assemble into program memory
            .retain                         ; Override ELF conditional linking
                                            ; And retain current section
            .retainrefs                     ; And retain any sections that have
                                            ; References to current section

; Parameters
MAX_COUNT	.equ	655						; PWM Frequency: 50 Hz or 20 ms

; Variables
SIGNAL1		.equ	R4						; Pulse width for Signal 1
SIGNAL2 	.equ	R5						; Pulse width for Signal 2

; States
S1State		.equ	R7						; Signal 1 State Index #
S2State		.equ	R8						; Signal 2 State Index #

;-------------------------------------------------------------------------------
; Global Initialization
;-------------------------------------------------------------------------------
RESET       mov.w   #__STACK_END,SP         ; Initialize stackpointer
StopWDT     mov.w   #WDTPW|WDTHOLD,&WDTCTL  ; Stop watchdog timer
			bic.w	#LOCKLPM5, &PM5CTL0		; Unlocks GPIO pins

; Setup GPIO
SetupP1     ; Outputs
			bis.b   #BIT1, &P1DIR        	; Signal 1 pin set to output
            bis.b   #BIT1, &P1SEL1       	; Signal 1 pin set to secondary function
            bis.b	#BIT2, &P1DIR			; Signal 2 pin set to output
            bis.b	#BIT2, &P1SEL1      	; Signal 2 pin set to secondary function

SetupP2		; Buttons
			bic.b	#BIT3,	&P2DIR			; P2.3 as Input (G)
			bis.b	#BIT3,	&P2REN			; P2.3 Activate Pullup/Pulldown Resistor (G)
			bis.b	#BIT3,	&P2OUT			; P2.3 Pullup R Enable (G)
			bic.b	#BIT3,	&P2IES			; P2.3 hi/low edge (button press) (G)
			bic.b	#BIT3,	&P2IFG			; P2.3 IFG cleared (G)
			bis.b	#BIT3,	&P2IE			; P2.3 Interrupt Enable (G)

			bic.b	#BIT7,	&P2DIR			; P2.7 as Input (G)
			bis.b	#BIT7,	&P2REN			; P2.7 Activate Pullup/Pulldown Resistor (G)
			bis.b	#BIT7,	&P2OUT			; P2.7 Pullup R Enable (G)
			bic.b	#BIT7,	&P2IES			; P2.7 hi/low edge (button press) (G)
			bic.b	#BIT7,	&P2IFG			; P2.7 IFG cleared (G)
			bis.b	#BIT7,	&P2IE			; P2.7 Interrupt Enable (G)

; Setup Timer (Tick = 1 MHz)
ClearStates ; Setup State Machine
			clr 	S1State					; Set S1State to 0
			clr		S2State					; Set S2State to 0
			mov.w	S1Table(S1State), SIGNAL1	; Load in default PWM value (assume index = 0)
			mov.w	S2Table(S2State), SIGNAL2	; Load in default PWM value (assume index = 0)

SetupC0		; Setup CCR0
			clr		&TA0CCTL0				; Set CCR0's control register (CTL) to default
			mov.w   #MAX_COUNT, &TA0CCR0  	; Set CCR0's value to HALF_PERIOD (controls period of timer)

SetupC1     ; Setup CCR1
			mov.w   #OUTMOD_7, &TA0CCTL1    ; Set CCR1's control register to reset/set pin mode
            mov.w   SIGNAL1, &TA0CCR1  		; Set CCR1's value

SetupC2		; Setup CCR2
			mov.w   #OUTMOD_7, &TA0CCTL2 	; Set CCR2's control register to reset/set pin mode
            mov.w   SIGNAL2, &TA0CCR2  		; Set CCR2's value

SetupTA     ; Setup Timer A
			mov.w   #TASSEL__ACLK|ID_0|MC__UP|TACLR, &TA0CTL ; Configure the timer to SMCLK and Up Mode

; Enable GIE
EnableGIE	; Enable General Interrupts (DISABLED)
			nop
			bis.w	#GIE,	SR				; Enable interrupts in SR
			nop

;-------------------------------------------------------------------------------
; Main Loop
;-------------------------------------------------------------------------------

Loop		; Loop (Wait for button press)
			nop
			bit.b	#BIT3,	&P2IFG	; Check to see if P2.3 was pressed
			jne		P2_3_db					; Jump if true
			bit.b	#BIT7,	&P2IFG	; Check to see if P2.7 was pressed
			jne		P2_7_db					; Jump if true
			jmp		Loop					; Loop

; Button Press (BttnFlag = 1)

P2_3_db		; Button 2.3 debounce
			bit.b	#BIT3, &P2IN 			; Check to see if the button P2.3 is pressed
			jnz		P2_3					; Jump to P2_3 if false
			jz		P2_3_db					; Loop

P2_3		; Button 2.3 has been pressed
			incd	S1State 				; Increment S1 State (double)
			cmp		#4, S1State				; Have we reached our maximum value?
			jl		P2_3_Skip				; Skip state reset if we haven't.
			clr 	S1State					; Reset S1State

P2_3_Skip	; Skip state reset for P2.3
			mov.w	S1Table(S1State), SIGNAL1	; Move table value to SIGNAL1
			mov.w   SIGNAL1, &TA0CCR1  		; Set CCR1's value
			bic.b	#BIT3,	&P2IFG			; P2.3 IFG cleared
			bis.b	#BIT3,	&P2IE			; P2.3 Interrupt Enabled
			bis.b	#BIT7,	&P2IE			; P2.7 Interrupt Enabled
			jmp 	Loop

P2_7_db		; Button 2.7 debounce
			bit.b	#BIT7, &P2IN 			; Check to see if the button P2.3 was pressed
			jnz		P2_7					; Jump to P2_3 if false
			jz		P2_7_db					; Loop

P2_7		; Button 2.7 has been pressed
			incd	S2State 				; Increment S1 State (double)
			cmp		#4, S2State				; Have we reached our maximum value?
			jl		P2_7_Skip				; Skip state reset if we haven't.
			clr 	S2State					; Reset S1State

P2_7_Skip	; Skip state reset for P2.7
			mov.w	S2Table(S2State), SIGNAL2	; Move table value to SIGNAL2
			mov.w   SIGNAL2, &TA0CCR2  		; Set CCR2's value
			bic.b	#BIT7,	&P2IFG			; P2.7 IFG cleared
			bis.b	#BIT3,	&P2IE			; P2.3 Interrupt Enabled
			bis.b	#BIT7,	&P2IE			; P2.7 Interrupt Enabled
			jmp 	Loop
			nop

;-------------------------------------------------------------------------------
; Tables
;-------------------------------------------------------------------------------
S1Table:	; Signal 1 Range
 				.word	13				; Min
 				.word	78				; Max

S2Table:	; Signal 2 Range
 				.word	13				; Min
 				.word	78				; Max

;-------------------------------------------------------------------------------
; Interrupt Service Routines (ISR)
;-------------------------------------------------------------------------------
P2ISR 		; Button pressed
			bic.b	#BIT7,	&P2IE			; P2.7 Interrupt Disable
			bic.b	#BIT3,	&P2IE			; P2.3 Interrupt Disable
			reti
			nop

;-------------------------------------------------------------------------------
; Stack Pointer definition
;-------------------------------------------------------------------------------
            .global __STACK_END
            .sect   .stack

;-------------------------------------------------------------------------------
; Interrupt Vectors
;-------------------------------------------------------------------------------
            .sect   ".reset"                ; MSP430 RESET Vector
            .short  RESET
            .sect	".int41"				; P2.3 interrupt
            .short	P2ISR
			.end

