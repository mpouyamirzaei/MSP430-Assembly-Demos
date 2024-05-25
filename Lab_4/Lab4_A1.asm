;-------------------------------------------------------------------------------
; Laboratory 4 (Input / Output Interfacing) Part 2, Question 1
; M. Pouya Mirzaei
;-------------------------------------------------------------------------------
; Directives Section (define any constants with .equ)
;-------------------------------------------------------------------------------
            .cdecls C,LIST,"msp430.h"       ; Include device header file
            .def    	RESET               ; Export program entry-point to
                                            ; make it known to linker.
            .text                           ; Assemble into ROM.

bttnMask		.equ 	BIT3				; P2.3 for button input
pinLED			.equ	BIT0				; P1.0 for light output

;-------------------------------------------------------------------------------
; Initialization Section (setup pins, timers, etc.)
;-------------------------------------------------------------------------------
RESET			mov.w		#__STACK_END, SP			; Set stack pointer
				mov.w   	#WDTPW+WDTHOLD, &WDTCTL 	; Stop WDT
				bic.w		#LOCKLPM5, &PM5CTL0		; Unlock GPIO Pins

setupP2			bic.b	#bttnMask, &P2DIR 		; Configure Port 2 as input
				bis.b	#bttnMask, &P2REN		; Enables pullup/pulldown resistor that locks in the voltage
				bis.b	#bttnMask, &P2OUT		; Sets inactive state as 1 for button pin
				bis.b	#bttnMask,&P2IE			; P2.3 Interrupt enabled
				bis.b	#bttnMask,&P2IES		; P2.3 hi/low edge
				bic.b	#bttnMask,&P2IFG		; P2.3 IFG Cleared

setupP1			bis.b   #pinLED, &P1DIR   		; P1.0 as output
         	    bis.b   #pinLED, &P1OUT      	; SET P1.0 = 1 (LED ON)

;-------------------------------------------------------------------------------
; Main loop (Background Program)
;-------------------------------------------------------------------------------
loop			nop
				bis.w #LPM3+GIE,SR 			; LPM4, enable interrupts
				nop

;-------------------------------------------------------------------------------
; Interrupt Service Routines (ISRs) (Foreground Program(s))
;-------------------------------------------------------------------------------
btnPressISR
				xor.b #001h,&P1OUT ; P1.0 = toggle
				bic.b #088h,&P2IFG ; P2.3 IFG Cleared
				reti ; Return from ISR

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

				.sect   PORT2_VECTOR			; MSP430 P2 Vector
				.short  btnPressISR				; Go to this label when interrupt occurs
				.end
