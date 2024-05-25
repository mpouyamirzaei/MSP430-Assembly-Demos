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
rxFlag		.equ	R5		; #1: RX is pending
							; #2: RX has been recieved
rxInd		.equ	R6
			.bss	rxBufSt, rxBufSize	; Reserve an array in RAM

maxCount	.equ 	R7		; Toggle Maximum Count Value
speedState	.equ	R8		; Speed Index #


;-------------------------------------------------------------------------------
; 			Table (RAM)
;-------------------------------------------------------------------------------
speedTable: 	.word	1000, 2500, 5000, 0, 0, 0, 0
; This stupid interrupt was messing up my table, so I had to add some zeros to
; the end so that it corrupts the zeros. This took me hours to figure out.

;-------------------------------------------------------------------------------
; 			Initialization
;-------------------------------------------------------------------------------
           .text                           	 ; Saved to to FRAM (ROM)
RESET       mov.w   #__STACK_END,SP          ; Initialize stack pointer
            mov.w   #WDTPW+WDTHOLD,&WDTCTL   ; Stop WDT
            bic.w   #LOCKLPM5,PM5CTL0

; INITIALIZE PORTS -------------------------------------------------------------
SetupP1		; Serial Comm. and Blinky Light
            bis.b   #0xFF, &P1DIR
            bis.b   #0xFF, &P1REN
            bic.b   #0xFF, &P1OUT
			bic.b   #BIT1, &P1SEL0
            bis.b   #BIT1, &P1SEL1
            bis.b   #txPin+rxPin, &P1SEL0
SetupP2 	; Buttons
			bic.b	#BIT3,	&P2DIR
			bis.b	#BIT3,	&P2REN
			bis.b	#BIT3,	&P2OUT
			bic.b	#BIT3,	&P2IES
			bic.b	#BIT3,	&P2IFG
			bis.b	#BIT3,	&P2IE

			bic.b	#BIT7,	&P2DIR
			bis.b	#BIT7,	&P2REN
			bis.b	#BIT7,	&P2OUT
			bic.b	#BIT7,	&P2IES
			bic.b	#BIT7,	&P2IFG
			bis.b	#BIT7,	&P2IE

; INTIALIZE VARIABLES ----------------------------------------------------------
			clr 	txPtr
			clr		speedState

; INITIALIZE THE FLL DCO CLOCK -------------------------------------------------
; Setup the FLL DCO Clock at 8 MHz
            bis.w   #SCG0, SR                 ; Disable FLL
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

;------------------------------------------------------------------------------
;           Main Loop
;------------------------------------------------------------------------------
mainLoop	; Loop (Wait for button press)
			nop
			bit.b	#BIT3,	&P2IFG			; Check to see if P2.3 was pressed
			jne		P2_3_db					; Jump if true
			bit.b	#BIT7,	&P2IFG			; Check to see if P2.7 was pressed
			jne		P2_7_db					; Jump if true
			bit.b	#2,	rxFlag				; Check if RX has been recieved
			jz		mainLoop				; Jump to mainLoop if it hasn't
update		cmp		#125, rxInd				; Ensure that timer A is at most 16 Hz
 			jge		skipMin
 			mov		#125, rxInd
skipMin		mov		rxInd, speedTable(speedState);		; Update blink timer
			mov		speedTable(speedState), maxCount
			mov		maxCount, &TA0CCR0
			mov		maxCount, &TA0CCR1

			mov.w	#SuccessMSG, txPtr		; Place the startup SendMSG into the txPtr
			push	R15						; Push R15 onto the stack pointer
			mov.w	txPtr,	R15				; Move the txPtr value to R15
			mov.b	@R15+, &UCA0TXBUF		; Move the value at R15 to the transmission buffer and add 1 to R15
			mov.w	R15, txPtr				; Move R15 to txPtr variable
			pop		R15						; Remove R15 from stack
			bis.w	#UCTXIE, &UCA0IE		; Enable Tx Complete interrupt
			clr		rxFlag
			jmp		mainLoop

; Button Press (BttnFlag = 1)
P2_3_db		; Button 2.3 debounce
			bit.b	#BIT3, &P2IN 			; Check to see if the button P2.3 is pressed
			jnz		P2_3					; Jump to P2_3 if false
			jz		P2_3_db					; Loop

P2_3		; Button 2.3 has been pressed
; Ask to input an index value
			mov.w	#SendMSG, txPtr				; Place the startup SendMSG into the txPtr
			push	R15							; Push R15 onto the stack pointer
			mov.w	txPtr,	R15					; Move the txPtr value to R15
			mov.b	@R15+, &UCA0TXBUF			; Move the value at R15 to the transmission buffer and add 1 to R15
			mov.w	R15, txPtr					; Move R15 to txPtr variable
			pop		R15							; Remove R15 from stack
			bis.w	#UCTXIE, &UCA0IE			; Enable Tx Complete interrupt
			bic.b	#BIT3,	&P2IFG				; P2.7 IFG cleared
			bis.b	#BIT3,	&P2IE				; P2.3 Interrupt Enabled
			bis.b	#BIT7,	&P2IE				; P2.7 Interrupt Enabled
			bis.b	#1,	rxFlag					; Set RX flag to be pending
			jmp 	mainLoop

P2_7_db		; Button 2.7 debounce
			bit.b	#BIT7, &P2IN 			; Check to see if the button P2.3 was pressed
			jnz		P2_7					; Jump to P2_3 if false
			jz		P2_7_db					; Loop

P2_7		; Button 2.7 has been pressed
			incd	speedState 				; Increment S1 State (double)
			cmp		#6, speedState		; Have we reached our maximum value?
			jl		P2_7_Skip				; Skip state reset if we haven't.
			clr		speedState			; Reset speedState

P2_7_Skip	; Skip state reset for P2.7
			mov.w	speedTable(speedState), maxCount	; Move table value to SIGNAL2
			mov.w   maxCount, &TA0CCR0  	; Set CCR0's value
			mov.w	maxCount, &TA0CCR1		; Set CCR1's value
			bic.b	#BIT7,	&P2IFG			; P2.7 IFG cleared
			bis.b	#BIT3,	&P2IE			; P2.3 Interrupt Enabled
			bis.b	#BIT7,	&P2IE			; P2.7 Interrupt Enabled
			jmp 	mainLoop
			nop
            
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
UART_RX		bit.b	#1, rxFlag					; Test if RX is pending
			jz		pressButtonFirst
			mov.b	rxInd, R15					; We already pushed R15 to stack, now load data from RAM to R15
			mov.b	&UCA0RXBUF, R15				; Move data from Rx buffer and save it in data buffer
			inc		R15							; Increment R15 to the next buffer address
			cmp.b	#rxBufSize, R15				; Check if we exceeded our buffer (this prevents a Buffer Overflow cyber attack!)
			jl		skip_RX_ind_reset
			mov.w	#InvalidMSG, txPtr			; Place the startup SendMSG into the txPtr
			push	R15							; Push R15 onto the stack pointer
			mov.w	txPtr,	R15					; Move the txPtr value to R15
			mov.b	@R15+, &UCA0TXBUF			; Move the value at R15 to the transmission buffer and add 1 to R15
			mov.w	R15, txPtr					; Move R15 to txPtr variable
			pop		R15							; Remove R15 from stack
			bis.w	#UCTXIE, &UCA0IE			; Enable Tx Complete interrupt
			clr		R15
			jmp		UART_DONE
skip_RX_ind_reset
			mov.b	R15, rxInd					; Our buffer index (in R15 still) is valid, move it back to RAM for storage.
			rla		rxInd
			rla		rxInd
			rla		rxInd
			rla		rxInd
			rla		rxInd
			rla		rxInd
			rla		rxInd
			mov		#2, rxFlag					; Flag RX
			jmp		UART_DONE					; Jump to exit code to pop R15 back to the way we found it!
pressButtonFirst
			mov.w	#PressBtnMSG, txPtr			; Place the startup SendMSG into the txPtr
			push	R15							; Push R15 onto the stack pointer
			mov.w	txPtr,	R15					; Move the txPtr value to R15
			mov.b	@R15+, &UCA0TXBUF			; Move the value at R15 to the transmission buffer and add 1 to R15
			mov.w	R15, txPtr					; Move R15 to txPtr variable
			pop		R15							; Remove R15 from stack
			bis.w	#UCTXIE, &UCA0IE			; Enable Tx Complete interrupt
			jmp		UART_DONE					; Start over

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

;------------------------------------------------------------------------------
;           Button Interrupt Service Routine
;------------------------------------------------------------------------------

P2ISR 		; Button pressed
			bic.b	#BIT7,	&P2IE			; P2.7 Interrupt Disable
			bic.b	#BIT3,	&P2IE			; P2.3 Interrupt Disable
			reti
			nop

;-------------------------------------------------------------------------------
; 			Table (ROM)
;-------------------------------------------------------------------------------
SendMSG:		.byte	"INPUT INDEX VALUE ", 0x00
SuccessMSG:		.byte	"CHANGE COMPLETED ", 0x00
PressBtnMSG:	.byte	"PRESS P2.3 TO CHANGE INDEX ", 0x00
InvalidMSG:		.byte	"INVALID INDEX, TRY AGAIN ", 0x00

;------------------------------------------------------------------------------
;           Interrupt Vectors
;------------------------------------------------------------------------------
            .sect   RESET_VECTOR            ; MSP430 RESET Vector
            .short  RESET                   ;
            .sect   USCI_A0_VECTOR          ; USCI_A0_VECTOR
            .short  UART_ISR                ; Send to UART ISR
            .sect	".int41"				; P2 interrupt vector
            .short	P2ISR					; Send to P2ISR
            .end


