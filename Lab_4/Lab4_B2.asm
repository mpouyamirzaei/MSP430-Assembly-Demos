;-------------------------------------------------------------------------------
; Laboratory 4 (Servo and Stepper Motor) Part 2
; M. Pouya Mirzaei
;-------------------------------------------------------------------------------
				.cdecls C,LIST,  "msp430FR2433.h"

				.def    RESET                   ; Export program entry-point to
												; make it known to linker.

				.global _initialize            ; export initialize as a global symbol (subroutine)

;-------------------------------------------------------------------------------
; Parameters
;-------------------------------------------------------------------------------
maxPhaseStates	.equ	8				; Maximum # of motor states
maxSpeedStates  .equ    8				; Maximum # of speed states (/2)

; Port 1
A1				.equ	BIT0		; P1.0
B1				.equ	BIT1
A2				.equ	BIT2
B2				.equ	BIT3		; P1.4
ALL				.equ	A1+B1+A2+B2 ; All pins

; Port 2
Butt1			.equ	BIT3
Butt2			.equ	BIT7

;-------------------------------------------------------------------------------
; Variables
;-------------------------------------------------------------------------------
phase			.equ	R4		; Motor Phase State value
phaseState		.equ	R5		; Motor Phase State variable
speed			.equ	R6
speedState		.equ	R7
direction		.equ	R8		; Direction Variable (0 = forward, 1 = backward)

;-------------------------------------------------------------------------------
; Program Start
;-------------------------------------------------------------------------------
                .text							; Executable code goes below

; Code entry point
RESET
				mov.w   #__STACK_END, SP        ; Initialize stackpointer
				call	#_initialize			; Execute subroutine to label "_initialize"
				jmp		mainLoop                    ; Jump to loop

;-------------------------------------------------------------------------------
; Initialization Subroutine
;-------------------------------------------------------------------------------
_initialize
				mov.w   #WDTPW+WDTHOLD, &WDTCTL ; Stop WDT
				bic.w	#LOCKLPM5, &PM5CTL0		; CLEAR LOCK-LowPowerMode5 bit which unlocks GPIO pins

; Initialize register values
SetupVar
				clr		phaseState					; Clear State counter
				clr		phase						; Clear Phase value
				clr		direction
				clr		speedState
				clr		speed

setupPort		; Setup Port 1 (Output)
				bis.b   #ALL, &P1DIR      	; Configure Port 1 as output
				bic.b	#ALL, &P1OUT		; Clear the output

; Set up Port 2 (Button)
SetupButts
				bic.b	#Butt1, &P2DIR 		; Configure Port 2 as input
				bis.b	#Butt1, &P2REN		; Enables pullup/pulldown resistor that locks in the voltage
				bis.b	#Butt1, &P2OUT		; Sets inactive state as 1 for button pin
				bic.b	#Butt1,	&P2IFG		; P2.3 IFG cleared
				bis.b	#Butt1,	&P2IE		; P2.3 Interrupt Enable

				bic.b	#Butt2, &P2DIR 		; Configure Port 2 as input
				bis.b	#Butt2, &P2REN		; Enables pullup/pulldown resistor that locks in the voltage
				bis.b	#Butt2, &P2OUT		; Sets inactive state as 1 for button pin
				bic.b	#Butt2,	&P2IFG		; P2.7 IFG cleared
				bis.b	#Butt2,	&P2IE		; P2.7 Interrupt Enable

; Set up State machine Timer (TimerA0) to periodically trigger interrupt
SetupC0    		mov.w   #CCIE, &TA0CCTL0
				mov.w   #2000, &TA0CCR0   ; Load default speed into CCR0 Register

SetupTA    		mov.w   #TASSEL_2|MC_1, &TA0CTL ; Config Timer Control Register: SMCLK, /2

; Enable interrupts and return from subroutine
SetupGIE		nop							; NOP needed before and after GIE
				bis.w   #GIE, SR            ; Enable General Interrupts
				nop
				ret							; Return from the subroutine

;-------------------------------------------------------------------------------
; Main loop
;-------------------------------------------------------------------------------

; Main Loop (branches to: do_fsm, do_button, endmain)
mainLoop:
				nop
				jmp		mainLoop					;
				nop

;-------------------------------------------------------------------------------
; Interrupt Service Routines (ISR)
;-------------------------------------------------------------------------------

TA0_ISR:		; TimerA0 ISR
				inc		phaseState 					; Increment phase state
				cmp		#maxPhaseStates, phaseState	; Have we reached our maximum value?
				jl 		skipResetPhase				; Skip state reset if we haven't.
				clr 	phaseState					; Reset phase state
skipResetPhase	bit.b	#BIT0,	direction			; Are we facing reverse?
				jnz		reverse						; If true, jump to reverse
				mov.b	forwardTable(phaseState), phase	; Move table value to SIGNAL1
				mov.b  	phase, &P1OUT 	 			; Set CCR1's value
				clr	 	&TAIFG						; Clear interrupt flag
				reti
reverse			mov.b	reverseTable(phaseState), phase	; Move table value to SIGNAL1
				mov.b  	phase, &P1OUT 	 			; Set CCR1's value
				clr		&TAIFG						; Clear interrupt flag
				reti								; Return

P2_ISR:		; Button Press ISR
				bit.b	#BIT7,	&P2IFG				; Check to see if P2.7 was pressed
				jnz		P2_7						; Jump if true
				xor.b	#BIT0, direction			; Toggle direction
				bic.b	#BIT3,	&P2IFG				; Clear interrupt flag
				reti								; Return
P2_7		    incd	speedState 					; Increment speed state (double)
				cmp		#maxSpeedStates, speedState	; Have we reached our maximum value?
				jl		skipResetSpeed
				clr		speedState
skipResetSpeed	mov.w	speedTable(speedState), speed	; Move table value to SIGNAL1
				mov.w  	speed, &TA0CCR0		 		; Set CCR0's value
				bic.b   #BIT7, &P2IFG				; Clear interrupt flag
				reti
				nop

;-------------------------------------------------------------------------------
; Data Tables
;-------------------------------------------------------------------------------
				;.data					; Uncomment .data to store in RAM, otherwise comment to keep in ROM

; Phase Tables for Motor
forwardTable:	;	Clockwise; A1 = bit0 B1 = bit1 A2 = bit2 B2 = bit3
 				.byte	00000001b
 				.byte	00000011b
 				.byte	00000010b
 				.byte	00000110b
 				.byte	00000100b
 				.byte	00001100b
 				.byte	00001000b
 				.byte	00001001b
 				.byte	00001111b	; for debugging

reverseTable:
 				.byte	00001001b
				.byte	00001000b
 				.byte	00001100b
 				.byte	00000100b
 				.byte	00000110b
 				.byte	00000010b
 				.byte	00000011b
 				.byte	00000001b
 				.byte	00001111b	; for debugging

speedTable:
 				.word	2000
 				.word	3000
 				.word	4000
				.word	5000
				.word	100			; for debugging

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
				.sect   ".int56"				; TimerA0 CCR0 Vector
				.short  TA0_ISR
				.sect	".int41"				; P2.3 Vector
          	  	.short	P2_ISR
				.end
