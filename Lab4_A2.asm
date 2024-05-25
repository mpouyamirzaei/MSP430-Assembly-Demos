;-------------------------------------------------------------------------------
; Laboratory 4 (Input / Output Interfacing) Part 2, Question 1
; M. Pouya Mirzaei
;-------------------------------------------------------------------------------
            .cdecls C,LIST,"msp430.h"       ; Include device header file
            
;-------------------------------------------------------------------------------
            .def    RESET                   ; Export program entry-point to
                                            ; make it known to linker.
;-------------------------------------------------------------------------------
            .text                           ; Assemble into program memory.
            .retain                         ; Override ELF conditional linking
                                            ; and retain current section.
            .retainrefs                     ; And retain any sections that have
                                            ; references to current section.

;-------------------------------------------------------------------------------
; Parameters
;-------------------------------------------------------------------------------
maxStates		.equ	8				; 3 states * 2 bytes per states

state				.equ	R4				; Frequency State Index
freq				.equ	R5				; Frequency Value
duty				.equ	R6				; Duty Cycle Value

;-------------------------------------------------------------------------------
; Initialization
;-------------------------------------------------------------------------------
RESET       mov.w   #__STACK_END,SP         ; Initialize stackpointer
StopWDT     mov.w   #WDTPW|WDTHOLD,&WDTCTL  ; Stop watchdog timer
GPIO		bic.w	#LOCKLPM5, &PM5CTL0		; Unlock GPIO pins
SetupStateMachine
			clr		state					; set to 0
			mov.w	freqTable(state), freq	; load in default value
			mov.w	dutyTable(state), duty  ; load in default value
SetupTA
			mov.w   freq, &TA0CCR0         ; PWM period: 1 second with ACLK at 32768 Hz
            mov.w   #OUTMOD_7, &TA0CCTL1    ; CCR1 reset/set
            mov.w   duty, &TA0CCR1         ; CCR1 75% PWM duty cycle defaut
            mov.w   #TASSEL__ACLK|ID_2|MC__UP|TACLR,&TA0CTL		; ACLK, divide ACLK by 4, up mode, clear TAR
SetupP1
			bis.b	#BIT1,	&P1DIR			; P1.1 as OUTPUT
			bis.b	#BIT1,	&P1SEL1         ; P1.1 options select
SetupP2
			bic.b	#BIT3,	&P2DIR			; P2.3 as INPUT
			bis.b	#BIT3,	&P2REN			; P2.3 Pullup Resister
			bis.b	#BIT3,	&P2OUT			; P2.3 Pullup R Enable
			bis.b	#BIT3,	&P2IES			; P2.3 hi/low edge (button press)
			bic.b	#BIT3,	&P2IFG			; P2.3 IFG cleared
			bis.b	#BIT3,	&P2IE			; P2.3 Interrupt Enable
EnableGIE
			nop
			bis.w	#GIE,	SR				; enable interrupts in SR
			nop

;-------------------------------------------------------------------------------
; Main loop here
;-------------------------------------------------------------------------------
MainLoop:
			nop
			jmp		MainLoop
			nop

;-------------------------------------------------------------------------------
P2ISR: ; Toggle P1.0 LED
;-------------------------------------------------------------------------------
			incd	state					; Go to next state
			cmp.b	#maxStates, state		; Has the maximum number of valid states been exceeded?
			jl		skip_state_reset		; if not, skip ahead, else...
			clr.w	state					; go back to the initial state (state 0)
skip_state_reset
			mov.w	dutyTable(state), duty	; load new PWM value into PWMValue
			mov.w	duty, &TA0CCR1      	; CCR1: set current PWM duty cycle
			mov.w	freqTable(state), freq	; load new PWM value into PWMValue
			mov.w	freq, &TA0CCR0      	; CCR0: set current PWM duty cycle
			bic.b	#BIT3,	&P2IFG			; clear the BIT3 interrupt flag
			reti							; return from interrupt

;-------------------------------------------------------------------------------
; Tables
;-------------------------------------------------------------------------------

freqTable:
			.word	100
			.word	100
			.word	100
			.word	100
			.word	10		; include this dummy variable for debugging purposes
dutyTable:
			.word	10	; 10%
			.word	30	; 30%
			.word	60	; 65%
			.word	90	; 90%
			.word	10		; include this dummy variable for debugging purposes

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
            
