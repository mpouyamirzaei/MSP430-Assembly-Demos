;-------------------------------------------------------------------------------
; Lab 5, Part 1
; Written by M. Pouya Mirzaei
;-------------------------------------------------------------------------------
            .cdecls C,LIST,"msp430fr2433.h"       ; Include device header file
;-------------------------------------------------------------------------------
            .def    RESET                   ; Export program entry-point to
                                            ; make it known to linker.
;-------------------------------------------------------------------------------
            .global __STACK_END
            .global	txPtr
			.global rxInd
			.global	rxBufSt

            .sect   .stack                   ; Make stack linker segment ?known?
            .retain                          ; Ensure current section gets linked
            .retainrefs

; CONSTANTS --------------------------------------------------------------------
txPin		.equ	BIT4	; P1.4: USART TX (Transmitter)
rxPin		.equ	BIT5	; P1.5: USART RX (Reciever)

; Communication (USART)
; USART 0 (BackDoor - TX: P1.4, RX: P1.5)
; BRCLK 	Baud Rate 	UCOS16 	UCBRx 	UCBRFx 	UCBRSx (P590 of User Guide)
; 8000000	9600		0x0001	52		1		0x0490
U0_UCOS16	.equ	0x0001
U0_UCBR		.equ	52
U0_UCBRF	.equ	0x10
U0_UCBRS	.equ	0x0490

; Reciever Constants
rxBufSize	.equ	10

; Special Text Characters
CR			.equ	0x0D		; Carriage Return
LF			.equ	0x0A		; Line Feed (New Line)

; VARIABLES --------------------------------------------------------------------
txPtr		.equ	R4		; Pointer to the next address to transmit
							; TX continues until null byte is detected
rxFlag		.equ	R5
rxInd		.equ	R6
			.bss	rxBufSt, rxBufSize	; Reserve an array in RA

maxCount	.equ 	R7		; Toggle Maximum Count Value

;-------------------------------------------------------------------------------
; 			Initialization
;-------------------------------------------------------------------------------
           .text                           	 ; Saved to to FRAM (ROM)
RESET       mov.w   #__STACK_END,SP          ; Initialize stack pointer
            mov.w   #WDTPW+WDTHOLD,&WDTCTL   ; Stop WDT
            ; Disable the GPIO power-on default high-impedance mode
            ; to activate previously configured port settings
            bic.w   #LOCKLPM5,PM5CTL0

; INITIALIZE PORTS -------------------------------------------------------------

            bis.b   #0xFF,&P1DIR
            bis.b   #0xFF,&P1REN
            bic.b   #0xFF,&P1OUT
			bic.b   #BIT1, &P1SEL0
            bis.b   #BIT1, &P1SEL1
            bis.b   #txPin+rxPin, &P1SEL0

; INITIALIZE THE FLL DCO CLOCK -------------------------------------------------
; Setup the FLL DCO Clock at 8 MHz
            bis.w   #SCG0,SR                 ; Disable FLL
            bis.w   #SELREF__REFOCLK, &CSCTL3		; Set REFOCLK as FLL reference source
            clr		&CSCTL0           		 ; Clear DCO and MOD registers
            bic.w   #DCORSEL_7, &CSCTL1      ; Clear DCO frequency select bits first
            bis.w   #DCORSEL_3, &CSCTL1      ; Set DCOCLK = 8MHz
            mov.w   #FLLD_0+243, &CSCTL2     ; Set DCODIV = 8MHz
            nop
            nop
            nop
            bic.w   #SCG0, SR                ; Enable FLL
; Wait for the signal to lock
Unlock      bit.w	#FLLUNLOCK0+FLLUNLOCK1, &CSCTL7 ; Wait until FLL locks
            jnz     Unlock                  ; Check if FLL is locked

Continue    mov.w   #SELMS__DCOCLKDIV+SELA__REFOCLK, &CSCTL4
											; set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                            ; default DCODIV as MCLK and SMCLK source

; INITIALIZE AND CONFIGURE THE UART SYSTEM (UART 0) 9600 BAUD -----------------
            bis.w   #UCSWRST,&UCA0CTLW0     ; Configure UART
            bis.w   #UCSSEL__SMCLK,&UCA0CTLW0	; Set UART source BRCLK = SMCLK
            mov.b   #U0_UCBR, &UCA0BR0          ; Initialize the Control Word Register
            clr.b   &UCA0BR1
            bis.w   #U0_UCBRS+U0_UCOS16+U0_UCBRF, &UCA0MCTLW	; Initialize the Modulation Control Reg - From Table 22-5

            bic.w   #UCSWRST,&UCA0CTLW0     ; release from reset
            bis.w   #UCRXIE,&UCA0IE         ; enable RX interrupt

; ENABLE INTERRUPTS -----------------------------------------------------------
            nop
            bis.w   #GIE, SR            ; Enable Interrupts
            nop

; INITIALIZE TIMER A ----------------------------------------------------------
			mov		#1000,	rxInd				; Initialize rxInd
			mov		#1000,	maxCount			; Initialize maxCount
SetupC0		clr		&TA0CCTL0					; Set CCR0's control register (CTL) to default
			mov.w   #1000, &TA0CCR0  			; Set CCR0's value
SetupC1     mov.w   #OUTMOD_4, &TA0CCTL1    	; Set CCR1's control register to toggle pin mode
            mov.w   #1000, &TA0CCR1				; Set CCR1's value
SetupTA     mov.w   #TASSEL_1+MC_1+ID_3, &TA0CTL 	; Configure the timer tou use the ACLK and count in updown mode

; STARTUP TEXT ----------------------------------------------------------------
			mov.w	#SendMSG, txPtr				; Place the startup SendMSG into the txPtr
			push	R15							; Push R15 onto the stack pointer
			mov.w	txPtr,	R15					; Move the txPtr value to R15
			mov.b	@R15+, &UCA0TXBUF			; Move the value at R15 to the transmission buffer and add 1 to R15
			mov.w	R15, txPtr					; Move R15 to txPtr variable
			pop		R15							; Remove R15 from stack
			bis.w	#UCTXIE, &UCA0IE			; Enable Tx Complete interrupt

;------------------------------------------------------------------------------
;           Main Loop
;------------------------------------------------------------------------------
mainLoop	bit.b	#1,	rxFlag					; Check if RX has been recieved
			jz		mainLoop
update		cmp		#125, rxInd					; Ensure that timer A is at most 16 Hz
 			jge		skipMin
 			mov		#125, rxInd
skipMin		mov		rxInd, maxCount;			; Update blink timer
			mov		maxCount, &TA0CCR0
			mov		maxCount, &TA0CCR1

			mov.w	#SuccessMSG, txPtr				; Place the startup SendMSG into the txPtr
			push	R15							; Push R15 onto the stack pointer
			mov.w	txPtr,	R15					; Move the txPtr value to R15
			mov.b	@R15+, &UCA0TXBUF			; Move the value at R15 to the transmission buffer and add 1 to R15
			mov.w	R15, txPtr					; Move R15 to txPtr variable
			pop		R15							; Remove R15 from stack
			bis.w	#UCTXIE, &UCA0IE			; Enable Tx Complete interrupt

			clr		rxFlag
			jmp		mainLoop
            
;------------------------------------------------------------------------------
; 			USCI Interrupt Service Routine
;------------------------------------------------------------------------------
UART_ISR 	push	R15							; Push R15 to the stack so we can use R15
			add.w	&UCA0IV, PC					; Add interrupt vector to PC (clears that flag)
			reti
			jmp		UART_RX						; RX buffer full (RXIFG) (Get data out of buffer ASAP)
			jmp		TX_comp						; TX complete (TXCPTIFG) (Shift register is now empty)
			reti

UART_DONE	pop		R15							; Exit code for all UART vectors
			reti

; RECIEVE ---------------------------------------------------------------------
UART_RX		mov.b	rxInd, R15					; We already pushed R15 to stack, now load data from RAM to R15
			mov.b	&UCA0RXBUF, R15				; Move data from Rx buffer and save it in data buffer
			inc		R15							; Increment R15 to the next buffer address
			cmp.b	#rxBufSize, R15				; Check if we exceeded our buffer (this prevents a Buffer Overflow cyber attack!)
			jl		skip_RX_ind_reset
			clr		R15
skip_RX_ind_reset
			mov.b	R15, rxInd					; Our buffer index (in R15 still) is valid, move it back to RAM for storage.
			rla		rxInd
			rla		rxInd
			rla		rxInd
			rla		rxInd
			rla		rxInd
			rla		rxInd
			rla		rxInd
			mov		#1, rxFlag					; Flag RX
			jmp		UART_DONE					; Jump to exit code to pop R15 back to the way we found it!

; TRANSMISSION ----------------------------------------------------------------
TX_comp		mov.w	txPtr, R15					; Load TX pointer from memory
			tst.b	0(R15)						; Is the next requested char a NULL?
			jz		MSG_Done
			mov.b	@R15+, &UCA0TXBUF			; Load char into buffer (triggers transmission)
			mov.w	R15, txPtr					; Return updated pointer to memory
			jmp		UART_DONE
MSG_Done	mov.b	#CR, &UCA0TXBUF
			bic.w	#UCTXIE, &UCA0IE			; Disable TX interrupt, finish transmission
			jmp		UART_DONE
			nop

;-------------------------------------------------------------------------------
; 			Tables (ROM)
;-------------------------------------------------------------------------------
SendMSG:		.byte	"INPUT VALUE ", 0x00
SuccessMSG:		.byte	"CHANGE COMPLETED ", 0x00

;------------------------------------------------------------------------------
;           Interrupt Vectors
;------------------------------------------------------------------------------
            .sect   RESET_VECTOR            ; MSP430 RESET Vector
            .short  RESET                   ;
            .sect   USCI_A0_VECTOR          ; USCI_A0_VECTOR
            .short  UART_ISR                ; Send to UART ISR
            .end
