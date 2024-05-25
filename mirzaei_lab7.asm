;-------------------------------------------------------------------------------
; Lab 7
; By M. Pouya Mirzaei
;-------------------------------------------------------------------------------
            .cdecls C,LIST,"msp430fr2433.h"       ; Include device header file
            
; RAM variables defined as global for debugging
			.global	TXptr
			.global RX_ind
			.global	RX_buf_st


;-------------------------------------------------------------------------------
            .def    RESET                   ; Export program entry-point to
                                            ; make it known to linker.
;-------------------------------------------------------------------------------
            .retain                         ; Override ELF conditional linking
                                            ; and retain current section.
            .retainrefs                     ; And retain any sections that have
                                            ; references to current section.

			.data

;-------------------------------------------------------------------------------
; VARIABLES AND CONSTANTS
;-------------------------------------------------------------------------------

; UART PARAMETERS
; Define UART Pins
TX_PIN4		.equ	BIT4	; P1.4: USART TX
RX_PIN5		.equ	BIT5	; P1.5: USART RX

; Communication (USART)
; USART 0 (BackDoor - TX: P1.4, RX: P1.5)
; Assumes MCLK is running at 16 MHz. BRCLK = SMCLK = MCLK.
; BRCLK 	Baud Rate 	UCOS16 	UCBRx 	UCBRFx 	UCBRSx (p 590 user guide)
; 8000000	9600		1		52		01		0x49

; RX DATA BUFFER VAR
RX_BUF_SIZE	.equ	10
TXptr 		.word	0						; Pointer to next address of data to send.
											; TX continues until null byte is detected.
RX_ind		.byte	0						; Byte Index of RX_BUF characters (buffer must be < 255 bytes in size)
			.bss	RX_buf_st, RX_BUF_SIZE	; Reserve an array in RAM (BSS will ALWAYS be saved in RAM)

; SPECIAL TEXT CHARACTERS
CR			.equ	0x0D		; Carriage Return
LF			.equ	0x0A		; Line Feed (New Line)

; ECG Monitor Parameters
thres		.equ	0x0286		; 1 V threshold (to detect QRS peak)
dispTime	.equ	3FFFh		; BPM Display time (4 seconds)
bufferMax	.equ	5000		; Maximum value of the beat buffer
multiFactor .equ	30			; multiply to calculate get bpm

; Variables
beatCount	.equ	R4
beatBuffer  .equ  	R5

;-------------------------------------------------------------------------------
; INITIALIZATION
;-------------------------------------------------------------------------------
			.text								; The following are saved to ROM (executable)

RESET       mov.w   #__STACK_END,SP         	; Initialize stackpointer
StopWDT     ; BASIC SETUP (DONE)
			mov.w   #WDTPW|WDTHOLD,&WDTCTL 		; Stop watchdog timer
			bic.w	#LOCKLPM5, &PM5CTL0			; Unlock GPIO (although not actually needed for UART)
			mov.w   #SELA__XT1CLK,&CSCTL4       ; Set ACLK = XT1; MCLK = SMCLK = DCO

SetupVar	; SETUP ALL VARIABLES
			clr		beatCount
			mov		#bufferMax, beatBuffer

InitClock	; SETUP CLOCK FOR USART
			bis.w	#SCG0, SR					; Disable FLL = Frequency Locked Loop to generate 8 MHz MCLK
			bis.w	#SELREF__REFOCLK, &CSCTL3	; FLL reference CLK = REFOCLK
			clr		&CSCTL0						; Clear tap and mod settings for FFL fresh start
			bic.w	#DCORSEL_7, &CSCTL1			; Clears all buts that control frequency
			bis.w	#DCORSEL_3, &CSCTL1			; Sets SMCLK to 8 MHz in the frequency locked loop
			mov.w	#FLLD_0+243, &CSCTL2		; FLLD = 1 (using 32768 Hz FLLREFCLK) and FLLN 'divider' 486 this divides by 2
			nop
			nop
			nop
			bic.w	#SCG0, SR					; Enable FLL
Unlock
			mov.w   &CSCTL7, R13					; brings whatever value that is in clock system control register 7 into register 13
            and.w   #FLLUNLOCK0|FLLUNLOCK1, R13 	; anded everything but basically cleared it all; set the Z bit (so no jump)
            jnz     Unlock 							; Clear fault flag

; Enable USART on port 1 pins = primary function (DONE)
			bis.b	#RX_PIN5+TX_PIN4, &P1SEL0
			bic.b	#RX_PIN5+TX_PIN4, &P1SEL1
Go_on
			mov.w   #SELMS__DCOCLKDIV+SELA__REFOCLK,&CSCTL4   ; set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz

SetupUSART
			bis.w 	#UCSWRST, &UCA0CTLW0			; Reset USART - all logic 0b
			bis.w	#UCSSEL__SMCLK, &UCA0CTLW0		; sets UART source BRCLK = SMCLK
			mov.b	#52, &UCA0BR0					; Baud Rate Control Word Register - From Table 22-5
			clr.b	&UCA0BR1
			bis.w   #0x0490+UCOS16+UCBRF_1,&UCA0MCTLW
			bic.b 	#UCSWRST, &UCA0CTLW0			; Release USART for operation
			bis.w	#UCRXIE, &UCA0IE				; Enable RX interrupt (Only enable TX interrupt (UCTXCPTIE) when needed)

; ENABLE INTERRUPTS (DONE)
			nop
			bis.w	#GIE, SR
			nop

; Startup Command
			mov.w	#HelloMSG, TXptr		; Load starting address of the message to TXptr
			call	#StartTX

; SETUP ADC (Done)
SetupP2		; Setup Port 2
			bis.b   #BIT2, &P2DIR                            ; P2.2 output
			bis.b   #BIT2, &P2REN                            ; P2.2 pullup/pulldown resistor enabled
            bic.b   #BIT2, &P2OUT                            ; Clear P2.2

SetupP1     bis.b   #BIT0,&P1DIR                            ; P1.0 output
            bic.b   #BIT0,&P1OUT                            ; Clear P1.0
            bis.w   #ADCPCTL1,&SYSCFG2                      ; ADC A1 pin
            bis.b   #BIT0|BIT1,&P2SEL0                      ; P2.0~P2.1: crystal pins
            bic.w   #LOCKLPM5,PM5CTL0                       ; Unlock I/O pins

SetupADC    ; Setup Analog-to-Digital Converter
			bis.w   #ADCSHT_2|ADCMSC|ADCON,&ADCCTL0         ; ADCON
            bis.w   #ADCSHP|ADCSHS_2|ADCCONSEQ_2,&ADCCTL1   ; rpt single ch; TA1.1 trigger
            bis.w   #ADCRES,&ADCCTL2                        ; 10-bit conversion results
            bis.w   #ADCINCH_1|ADCSREF_1,&ADCMCTL0          ; Vref+, A1
            mov.w   #thres, &ADCHI                          ; Window Comparator Hi-threshold ~1V (IMPORTANT)
            mov.w   #0,&ADCLO                               ; Window Comparator Lo-threshold ~0V (IMPORTANT)
            bis.w   #ADCHIIE|ADCLOIE|ADCINIE,&ADCIE         ; Enable ADC conv complete interrupt
SetupREF
			mov.b   #PMMPW_H, &PMMCTL0_H                    ; Unlock the PMM registers
            bis.w   #INTREFEN, &PMMCTL2                     ; Enable internal reference
            mov.w   #200,R15                                ; Delay ~400 cycles for reference settling
L2
			dec.w   R15                                     ; Decrement R14
            jnz     L2                                      ; Delay over?

; SETUP TIMER A (Done)
SetupTA0
			clr		&TA0CCTL0
			bis.w   #CCIE, &TA0CCTL0                       	; CCR0 interrupt enabled
            bis.w   #TACLR|TASSEL_1|MC_1|ID_2, &TA0CTL      ; ACLK, up mode, clear TAR
            mov.w   #dispTime, &TA0CCR0                     ; Display Time Period
SetupTA1
            mov.w   #3FFFh,&TA1CCR0
            mov.w   #1FFFh, &TA1CCR1                  	    ; Duty cycle TA1.1
            bis.w   #OUTMOD_4, &TA1CCTL1                    ; TA0CCR1 toggle
            bis.w   #TASSEL_1|MC_1|TACLR,&TA1CTL            ; ACLK, up mode

;-------------------------------------------------------------------------------
; Main Loop
;-------------------------------------------------------------------------------
Mainloop    bis.w   #ADCENC, &ADCCTL0                        ; Start sampling/conversion
            nop
            bis.w   #LPM3+GIE, SR                            ; Enter LPM3 w/ interrupts
            nop
            inc		beatBuffer
            jmp     Mainloop

StartTX		; A subroutine that sends out the text in the TXptr..
			push	R15
			mov.w	TXptr, R15								; Load TX pointer from memory
			mov.b	@R15+, &UCA0TXBUF						; Load char into buffer (triggers transmission)
			mov.w	R15, TXptr								; Return updated pointer to RAM
			pop		R15
			bis.w	#UCTXIE, &UCA0IE						; Enable TX Complete interrupt
			ret
;-------------------------------------------------------------------------------
; TABLES (ROM)
;-------------------------------------------------------------------------------
HelloMSG	.byte	"INITIALIZATION COMPLETE ", 0x00 		; CR = carriage return = always 0x0D
											 				; LF = Line Feed  = always 0x0A
											 				; 0x00 is null character that terminates the bite

;-------------------------------------------------------------------------------
; UART 0 ISR Handler
; Use UCA IV to determine if RX or TX event.
; RX ISR saves RX data to data buffer.
; R15 is used by the ISR but is pushed to the stack and popped when completed.
;-------------------------------------------------------------------------------
UART_ISR	push	R15										; Push R15 to the stack so we can use R15
			add.w	&UCA0IV, PC								; Add interrupt vector to PC (clears that flag)
			reti
            jmp     UARTev                  				; Vector  2: USCI_UART_UCRXIFG = receive buffer full = high priority interrupt flag
            jmp		TX_comp	                				; Vector  4: USCI_UART_UCTXIFG = transmit buffer empty = regular flag
            reti                            				; Vector  6: USCI_UART_UCSTTIFG = start bit received
            reti                   					        ; Vector  8: USCI_UART_UCTXCPTIFG = transmit complete

UARTev      ; Test oscilator fault flag ; if we are here but the IFG is clear, then we circle
			; back until IFG is set so Z is clear so we continue
			bit.w   #UCTXIFG,&UCA0IFG
            jnz    	UARTev

UART_DONE	; Exit code for all UART vectors
			pop		R15
            reti

; Transmission ------------------------------------------------------------------
TX_comp		mov.w	TXptr, R15								; Load TX pointer from memory
			tst.b	0(R15)									; Is the next requested char a NULL?
			jz		MSG_Done
			mov.b	@R15+, &UCA0TXBUF						; Load char into buffer (triggers transmission)
			mov.w	R15, TXptr								; Return updated pointer to memory
			jmp		UART_DONE

MSG_Done	mov.b	#CR, &UCA0TXBUF
			bic.w	#UCTXIE, &UCA0IE						; Disable TX interrupt
			jmp		UART_DONE
			nop

;-------------------------------------------------------------------------------
ADC_ISR;      ISR for ADC (DONE)
;-------------------------------------------------------------------------------
            add     &ADCIV,PC                               ; Add offset to PC
            reti                                            ; No Interrupt
            reti                                            ; Conversion result overflow
            reti                                            ; Conversion time overflow
            jmp      ADHI                                   ; A1 > 1V
            jmp      ADLO                                   ; A1 < 0.5V
            jmp      ADLO                                   ; A1 < 0.5V
            reti                                            ; ADCIFG0

ADHI        ; A1 > 1V
			cmp		 #bufferMax, beatBuffer					; see if the buffer has hit its max
			jge      beatInc
			inc		 beatBuffer
			jmp		 skipADHI
beatInc
			bit.b  	 #BIT0, &P1OUT                     		; Test to see if its still set
			jnz		 skipADHI		   						; Debounce
			bis.b    #BIT0, &P1OUT                          ; Toggle LED P1.0
			inc		 beatCount								; Increment the beat counter
			clr		 beatBuffer								; Clear the beat buffer
skipADHI
			bic.w    #ADCINIFG, &ADCIFG                     ; Clear interrupt flag
			reti

ADLO        ; A1 < 1V
			bic.b    #BIT0,&P1OUT
            bic.w    #ADCINIFG,&ADCIFG                      ; Clear interrupt flag
            reti

;-------------------------------------------------------------------------------
TA0_ISR;    ISR for TIMER A0
;-------------------------------------------------------------------------------
            xor.b   #BIT2, &P2OUT       ; Toggle LED P2.2
            clr 	R6					; clear R6
            clr 	R7					; clear R7
            mov		R4, R6      		; Load operand1 into R6
        	mov  	#multiFactor, R7    ; Load operand2 into R7
        	clr   	R8                 	; Clear R8 to store the result
multiply_loop
        	cmp   #0, R7           		; Check if operand2 is zero
        	jz    multiply_end       	; If it's zero, jump to the end of the multiplication
        	add   R6, R8             	; Add operand1 to the result
        	dec   R7                 	; Decrement operand2
        	jmp   multiply_loop      	; Jump back to multiply_loop
multiply_end	; Result is stored in R8
			clr	  beatCount
			mov.w	R8, TXptr			; Load starting address of the message to TXptr
			call	#StartTX
            reti

;-------------------------------------------------------------------------------
; Stack Pointer definition
;-------------------------------------------------------------------------------
            .global __STACK_END
            .sect   .stack
            
;-------------------------------------------------------------------------------
; Interrupt Vectors (DONE)
;-------------------------------------------------------------------------------
            .sect   ".reset"				                ; MSP430 RESET Vector
            .short  RESET
            .sect	USCI_A0_VECTOR							; USART
            .short	UART_ISR
            .sect   .int43					                ; ADC Vector
            .short  ADC_ISR
            .sect   TIMER0_A0_VECTOR 		 	       		; Timer_A0 Vector
            .short  TA0_ISR
            .end


