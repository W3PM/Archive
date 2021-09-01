;
; WSPR audio tone generator to drive SSB transmitter using 16F628A PIC
; Gene Marcus W3PM GM4YRE
;
;  13 July, 2009
;  18 September, 2009 - modified to include NMEA GPS data stream as optional NMEA timing
;  07 November, 2009 -  added PC symbol data text file update routine
;  12 November, 2009 - revised symbol look-up routine to smooth out timing irregularities
;  
; External xtal  - 4 MHz
; UART rate      - 4800 baud
; TX  RB7 pin 13
;
;
;  Data table reference for nomimal 1500 Hz:
;  0  1497.803 
;  1  1499.268 
;  2  1500.732 
;  3  1502.197 
;
;  
;
; 					       PIC16F628A                             
;                          __________                                          
;     Not used--------RA2 |1       18| RA1---------Not used                   
;     LED1------------RA3 |2       17| RA0---------Not used                   
;     LED2------------RA4 |3       16| OSC1--------XTAL                        
;     +5V-----------!MCLR |4       15| OSC2--------XTAL                        
;     Ground----------Vss |5       14| VDD---------+5 V                        
;     Not used--------RB0 |6       13| RB7---------TX                    
;     RS232/NMEA in---RB1 |7       12| RB6---------Symbol 3    
;     RS232 out-------RB2 |8       11| RB5---------Symbol 2          
;     Symbol 0--------RB3 |9       10| RB4---------Symbol 1         
;                          ----------                                          
; 
;=====================================================================

		processor	pic16f628a
		include		"p16f628a.inc"
		__config		_CP_OFF & _LVP_OFF & _BODEN_OFF & _MCLRE_ON & _XT_OSC & _WDT_OFF & _PWRTE_ON
		movlw		0x07
		movwf		CMCON		; Turn off comparator

		list		b=4,n=70

;=====================================================================
;	Manifest Constants
;=====================================================================

TMR1IF		equ			D'00'
LED1		equ			D'03'		; PORTA pin2 RA3
LED2		equ			D'04'		; PORTA pin3 RA4
carry		equ			D'00'
zero		equ			D'02'

;=====================================================================
;	File register use
;=====================================================================
		cblock		H'20'

			wdata0
            wdata1
            wdata2
            wdata3
            wdata4
            wdata5
            wdata6
            wdata7
            wdata8
            wdata9
            wdata10
            wdata11
            wdata12
            wdata13
            wdata14
            wdata15
            wdata16
            wdata17
            wdata18
            wdata19
            wdata20
            wdata21
            wdata22
            wdata23
            wdata24
            wdata25
            wdata26
            wdata27
            wdata28
            wdata29
            wdata30
            wdata31
            wdata32
            wdata33
            wdata34
            wdata35
            wdata36
            wdata37
            wdata38
            wdata39
            wdata40
 			TableNum
			position
			CRLF
			sec_count
			min_count
			sendcode
			symbolcount
			symboltimer
			temp
			pad
			count
			loop_count
			w_temp
			status_temp
			two_sec
			evenmin
			onemin
			rxstate
			char
			tchar

		endc

		goto		start

;=====================================================================
;  Subroutines
;=====================================================================

	; Interrupt service routine

IRQSVC								; Interrupt 1PPS starts here
		org			H'04'
		movwf		w_temp			; Save off the W register
		swapf		STATUS,W		; And the STATUS (use swapf
		movwf		status_temp		; so as not to change STATUS)

		nop
		movlw		D'011'			; Reset TMR1 for 500 mS
		movwf		TMR1H
		movlw		D'221'
		movwf		TMR1L

		movlw		B'00001000'
		xorwf		PORTA
		decfsz		sec_count		; Counts 500 mS intervals
		goto		main3
		movlw		D'120'
		movwf		sec_count
		
		decfsz		min_count		
		goto		main3
		movlw		D'10'			; Set time interval here
		movwf		min_count

		clrf		sendcode

main3			
		bcf			PIR1,TMR1IF		; Clear the old interrupt

		swapf		status_temp,W   ; Restore the status
		movwf		STATUS			; register
		swapf		w_temp,F		; Restore W without disturbing
		swapf		w_temp,W		; the STATUS register

		retfie


;---------------------------------------------------------------------
; Subroutine to tramsmit symbol data
;---------------------------------------------------------------------

main								; Symbol send code starts here

		bsf 		STATUS, RP0		; Bank 1
		clrf		EEADR			; Start at EEPROM address 0
		bcf     	STATUS,RP0		; Back to bank 0
		movlw		D'163'
		movwf		symbolcount
		bcf 		PORTA,LED2 		; Turn on LED2
		movlw		B'10000000'		; Set RB7 to turn on TX	
		movwf		PORTB
		movlw		D'31'			; 2 sec delay before data start
		movwf		two_sec			; 
		movlw		D'04'			; Initiialize symbol group size
		movwf		position
		clrf		temp
		movlw		D'21'
		movwf		symboltimer
							
tdelay								; 
 		btfss 		INTCON,T0IF		; Did timer overflow?
		goto 		tdelay 			; No, hang around some more
		bcf 		INTCON,T0IF		; reset overflow flag
		decfsz 		two_sec,F 		; Count down
		goto 		tdelay			; Not time yet	
		movlw 		D'130' 			; Timer will count 	
		movwf 		TMR0			; 127 (256-129) counts

symbol_loop
		movlw		D'21'
		movwf		symboltimer
		decfsz		symbolcount
		goto		continue2
		goto		stop			; Yes, stop

continue2
		clrf		temp
		call		getsymbol
		movfw		tchar
		movwf		temp			; No, reload w from temp and continue
		sublw		D'0'			; Test for symbol 0 
		bz			zer0			; Yes, goto zer0
		movfw		temp			; No, reload w from temp and continue
		sublw		D'1'			; Test for symbol 1
		bz			one				; Yes, goto one
		movfw		temp			; No, reload w from temp and continue
		sublw		D'2'			; Test for symbol 2
		bz			two				; Yes, goto two
									; No, it must be symbol 3

three
		movlw		B'11000000'		;Set RB4	
		movwf		PORTB		
		goto		delay

zer0
		movlw		B'10001000'		;Set RB1	
		movwf		PORTB
		goto		delay
one
		movlw		B'10010000'		;Set RB2	
		movwf		PORTB
		goto		delay
two
		movlw		B'10100000'		;Set RB3	
		movwf		PORTB

delay	
 		btfss 		INTCON,T0IF 	; Did timer overflow?
		goto 		delay			; No, hang around some more
		movlw		d'39'			;
		movwf		pad				; Calibrate symbol timer
timepad	decfsz		pad,F			; to 680 mSec
		goto		timepad			;
		bcf 		INTCON,T0IF 	; reset overflow flag
		movlw 		D'130'			; Timer will count 	
		movwf 		TMR0			; 127 (256-129) counts
		decfsz 		symboltimer,F 	; Count down
		goto 		delay			; Not time yet											
		goto		symbol_loop
stop
		movlw		D'1'
		movwf		sendcode
		bsf 		PORTA,LED2		; Turn off LED2
		movlw		B'00000000'		; Set to symbol3 Turn off TX
		movwf		PORTB			

		return


;---------------------------------------------------------------------
; Subroutine to download symbol data from EEPROM and decompress
;---------------------------------------------------------------------

getsymbol
		movf		position,w
		xorlw		D'04'
		btfss		STATUS,zero	
		goto		pos3		
		bsf 		STATUS, RP0 	; Bank 1			
		bsf 		EECON1, RD		; EE Read		
		movf 		EEDATA, W		; W = EEDATA
		bcf 		STATUS, RP0 	; Bank 0
		movwf		char

		movlw		B'11000000'		; Mask for 1st symbol
		andwf		char,w
		movwf		temp 
		bcf			STATUS,C
		rrf			temp 
		rrf			temp 
		rrf			temp 
		rrf			temp 
		rrf			temp 
		rrf			temp			; First symbol is valid at this point
		movfw		temp
		movwf		tchar
		decf		position
		return

pos3
		movf		position,w
		xorlw		D'03'
		btfss		STATUS,zero	
		goto		pos2
		movlw		B'00110000'		; Mask for 2nd symbol
		andwf		char,w
		movwf		temp 
		bcf			STATUS,C
		rrf			temp 
		rrf			temp 
		rrf			temp 
		rrf			temp			; Second symbol is valid at this point
		movfw		temp
		movwf		tchar
		decf		position
		return

pos2
		movf		position,w
		xorlw		D'02'
		btfss		STATUS,zero	
		goto		pos1
		movlw		B'00001100'		; Mask for 3rd symbol
		andwf		char,w
		movwf		temp 
		bcf			STATUS,C
		rrf			temp 
		rrf			temp			; Third symbol is valid at this point
		movfw		temp
		movwf		tchar
		decf		position
		return

pos1
		movlw		D'04'
		movwf		position
		movlw		B'00000011'		; Mask for 4th symbol
		andwf		char,w
		movwf		tchar			; Fourth symbol is valid at this point

		bsf 		STATUS, RP0		; Bank 1
		incf		EEADR			; Increment	EEADR
		bcf     	STATUS,RP0		; Back to bank 0
		return

;
; *****************************************************************************
; *                                                                           *
; * Purpose:  Write the byte of data at EEDATA to the EEPROM at address       *
; *           EEADR.                                                          *
; *                                                                           *
; *   Input:  The values at EEDATA and EEADR.                                 *
; *                                                                           *
; *  Output:  The EEPROM value is updated.                                    *
; *                                                                           *
; *  NOTE:  ALL IN BANK 1                                                     *
; *****************************************************************************
;

write_EEPROM
        bsf     EECON1,WREN       	; Set the EEPROM write enable bit
        ; Start required sequence
;       bcf     INTCON,GIE        	; Disable interrupts
        movlw   0x55              	; Write 0x55 and 0xAA to EECON2
        movwf   EECON2            	; control register, as required
        movlw   0xAA              	;   
        movwf   EECON2            	;
        bsf     EECON1,WR         	; Set WR bit to begin write
        ; End required sequence
bit_check
        btfsc   EECON1,WR         	; Has the write completed?
        goto    bit_check         	; No, keep checking
;       bsf     INTCON,GIE        	; Enable interrupts      
        bcf     EECON1,WREN       	; Clear the EEPROM write enable bit
        incf    EEADR,f           	; Increment the EE write address
        return                    	; Return to the caller


;---------------------------------------------------------------------
; Subroutine to convert symbol to ASCII and send to PC
;---------------------------------------------------------------------

send	
		movlw		D'48'			; Add 48 to convert symbol number
		addwf		temp,0			; to ASCII character		
send2	
		btfsc		PIR1,TXIF		; TX buffer full?
		goto		loop2			; No: send character
		goto		send2			; Yes: go back
loop2
		movwf		TXREG			; Trnsmit character
		decfsz		position		; 4th symbol?
		return
		call		space2			; Yes: send a 'space' 
		decfsz		CRLF			; 10th column?
		return						; No: return
		call		CRLF2			; Yes: send 'CR' and 'LF'


;---------------------------------------------------------------------
; Subroutine to send a 'space' or CR and LF
;---------------------------------------------------------------------

space2
		movlw		D'04'			; Reset symbol counter
		movwf		position
		movlw		D'32'			; Send a 'space'
		goto		send4
		
CRLF2
		movlw		D'10'			; Reset column counter
		movwf		CRLF
		movlw		D'13'			; Send a 'CR'
send3	
		btfsc		PIR1,TXIF		; TX buffer full?
		goto		loop3			; No: send character
		goto		send3			; Yes: go back
loop3
		movwf		TXREG			; Trnsmit character
		movlw		D'10'			; Send a 'LF'
send4		
		btfsc		PIR1,TXIF		; TX buffer full?
		goto		loop4			; No: send character
		goto		send4			; Yes: go back
loop4
		movwf		TXREG			; Trnsmit character
		return


;---------------------------------------------------------------------
; Table containing intruction messages
;---------------------------------------------------------------------

Table								; WSPR symbol data starts here
				addwf 	PCL,F
				dt		"Send symbol file now:"
				dt		H'ff'		; End of message flag



;=====================================================================
;  Mailine begins here -- Initialization
;=====================================================================
start

;---------------------------------------------------------------------
;	Set up timer 
;---------------------------------------------------------------------
				
		movlw		B'00110001'		; Turn timer 1 on - 1:8 prescaler
		movwf		T1CON
		banksel		PIE1
		bsf			PIE1,TMR1IE

		banksel		TMR1H
		movlw		D'011'
		movwf		TMR1H
		movlw		D'221'
		movwf		TMR1L

		bsf			INTCON,PEIE
		bsf			INTCON,GIE

		banksel		OPTION_REG
		movlw		B'01010111'		; Select pull-ups, TOCS, 256 prescale
		movwf		OPTION_REG

;---------------------------------------------------------------------
;	Set up I/O 
;---------------------------------------------------------------------

		bcf	STATUS,IRP

		banksel	TRISA                        
        movlw   B'00000000'       ; Tristate PORTA (all outputs)  
        movwf   TRISA             ;
 		movlw	B'00000110'
        movwf   TRISB             ; Set RB1 & RB2 as input for UART I/O - all other bits as output
		clrf	EEADR
		banksel	PORTA
		clrf	PORTA
		clrf	PORTB


;---------------------------------------------------------------------
;	Initialize memory for asyncronous transmission at 4800 baud
;---------------------------------------------------------------------

		bsf			STATUS,RP0
		bcf			TXSTA,SYNC		; Clears TXSTA bits SYNC (async) & BRGH (low speed baudrate) 
		bsf			TXSTA,TXEN		; Transmit enabled

		movlw 		D'12'			; set UART for 4800 baud 
		movwf		SPBRG
		bcf			STATUS,RP0
		bsf			RCSTA,SPEN		; Enable serial port

		bcf			PIR1,RCIF

;---------------------------------------------------------------------
;	Initialize memory
;---------------------------------------------------------------------

		bsf 		PORTA,LED1 		; Turn off LED1
		bsf 		PORTA,LED2 		; Turn off LED2
		movlw		D'48'			; Defines even minute to transmit - 48=0, 50=2, 52=4, etc...
		movwf		evenmin
		bcf			STATUS,zero

		movlw		D'26'
		movwf		loop_count

		movlw		D'120'			; 500 mS interrupt interval
		movwf		sec_count

		movlw		D'10'
		movwf		min_count

		movlw		D'01'
		movwf		sendcode


;---------------------------------------------------------------------
;	Wait 6 seconds for for new PC symbol text file '++++' command, 
;	else continue
;---------------------------------------------------------------------
		movlw		D'04'			; Count 4 '++++'
		movwf		count
		movlw		D'90'			; 6 sec delay 
		movwf		temp			; 
		movlw		B'10010000'		; Set SPEN & CREN
		movwf		RCSTA
							
tdelay2								; 
		bcf			STATUS,carry
		btfsc		PIR1,RCIF		; Check bit 5 (RCIF) for any character received
		goto		getchar5
		bcf			STATUS,carry
		goto		continue

getchar5	
		movlw		D'06'			; Check for UART overrun or framing error
		andwf		RCSTA,W
		btfsc		STATUS,zero		; Check STATUS register bit 2 -	1=0
		goto		getchar6
		movf		RCREG,W			; W = RCREG
		bcf			RCSTA,CREN		; CREN = 0
		bsf			RCSTA,CREN		; CREN = 1
		goto		tdelay2

getchar6
		movf		RCREG,W			; W = RCREG - this clears RCIF
		movwf		char
		xorlw		D'43'			; test for ASCII "+"
		btfss		STATUS,Z 
		goto		continue
		decfsz		count
		goto		continue
		goto		sendinstr

continue
		btfss 		INTCON,T0IF		; Did timer overflow?
		goto 		tdelay2			; No, hang around some more
		bcf 		INTCON,T0IF		; reset overflow flag
		decfsz 		temp,F			; Count down
		goto 		tdelay2			; Not time yet	


;____________________________________________________________________________________
;
; Entry point for for transmit timing - test for NMEA input
;____________________________________________________________________________________
;


WaitForInt
		movfw		sendcode
		addlw		D'0'
		bz			loop
		btfsc		PIR1,RCIF		; Check bit 5 (RCIF) for any character received
		goto		nmeastart
		goto 		WaitForInt

loop	call		main
		movf		RCREG,W			; W = RCREG - this clears RCIF
		goto 		WaitForInt

;____________________________________________________________________________________
;
; Entry point for for NMEA transmit timing
;____________________________________________________________________________________
;

nmeastart

		bcf			INTCON,GIE

clear
		clrf		rxstate			; Start over
	
receive
		bcf			STATUS,carry
		btfsc		PIR1,RCIF		; Check bit 5 (RCIF) for any character received
		goto		getchar
		bcf			STATUS,carry
		goto		receive

getchar	
		movlw		D'06'			; Check for UART overrun or framing error
		andwf		RCSTA,W
		btfsc		STATUS,zero		; Check STATUS register bit 2 -	1=0
		goto		getchar2
		movf		RCREG,W			; W = RCREG
		bcf			RCSTA,CREN		; CREN = 0
		bsf			RCSTA,CREN		; CREN = 1
		goto		receive


getchar2
		movf		RCREG,W			; W = RCREG - this clears RCIF
		movwf		char			; Received character from buffer
		xorlw		D'36'
		btfss		STATUS,zero		; Does start character = "$"?
		goto		getchar3
		movlw		D'01'
		movwf		rxstate			; rxstate = 1
		bcf			STATUS,carry
		goto		receive			; Get next character

getchar3
		movlw		d'15'
		subwf		rxstate,w		; Check for illegal state - rxstate > 14
		btfss		STATUS,carry
	    goto		getchar4
		goto		receive

getchar4
		movlw		D'01'
		movwf		PCLATH
		movf		rxstate,w
		addwf		PCL,F			; Jump to rxstate number
		goto		receive			; idle
		goto		st1				; get_Gp
		goto		st2				; get_gP
		goto		st3				; get_gpGga_or_gpRmc
		goto		st4				; get_gpgGa
		goto		st5				; get_gpgga
		goto		st6				; get_gprMc
		goto		st7				; get_gprmC
		goto		st8				; get_comma
		goto		st9				; get_10h
		goto		st10			; get_1h
		goto		st11			; get_10m
		goto		st12			; get_1m
		goto		st13			; get_10s
		goto		st14			; get_1s

st1									; get_Gp
		movf		char,w
		xorlw		D'71'			; Is receive charactor G?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		clear			; If not, clear rxstate and wait for another '$'
		incf		rxstate
		goto		receive			; Get next character

st2									; get_gP
		movf		char,w
		xorlw		D'80'			; Is receive charactor P?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		clear			; If not, clear rxstate and wait for another '$'
		incf		rxstate,1
		goto		receive			; Get next character

st3									; get_gpGga_or_gpRmc
		movf		char,w
		xorlw		D'71'			; Is receive charactor G?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		st3b			; 
		incf		rxstate			; We have "GPG"
		goto		receive			; Get next character

st3b	movf		char,w
		xorlw		D'82'			; Is receive character R?
		btfss		STATUS,zero
		goto		clear			; Start over
		movlw		D'06'
		movwf		rxstate			; We have "GPR" rxstate=6
		goto		receive			; Get next character

st4									; get_gpgGa
		movf		char,w
		xorlw		D'71'			; Is receive charactor G?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		clear			; If not, clear rxstate and wait for another '$'
		incf		rxstate
		goto		receive			; Get next character

st5									; get_gpggA
		movf		char,w
		xorlw		D'65'			; Is receive charactor A?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		clear			; If not, clear rxstate and wait for another '$'
		movlw		D'08'
		movwf		rxstate
		goto		receive			; We have "GPGGA" - rxstate=8 - get next character

st6									; get_gprMc
		movf		char,w
		xorlw		D'77'			; Is receive charactor M?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		clear			; If not, clear rxstate and wait for another '$'
		incf		rxstate
		goto		receive			; Get next character

st7									; get_gprmC
		movf		char,w
		xorlw		D'67'			; Is receive charactor C?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		clear			; If not, clear rxstate and wait for another '$'
		incf		rxstate
		goto		receive			; Get next charactor

st8									; get_comma
		movf		char,w
		xorlw		D'44'			; Is receive charactor a comma?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		clear			; If not, clear rxstate and wait for another '$'
		incf		rxstate
		goto		receive			; Get next character

st9									; get_10h
		movlw		D'51'
		subwf		char,w			; Is character <=2?
		btfsc		STATUS,carry	
		goto		clear			; Start over if not valid
		incf		rxstate			; Increment and go to next character
		goto		receive

st10								; get_1h
		movlw		D'58'
		subwf		char,w			; Is character <=9?
		btfsc		STATUS,carry	
		goto		clear			; Start over if not valid
		incf		rxstate			; Increment and go to next character
		goto		receive

st11								; get_10m
		movlw		D'54'
		subwf		char,w			; Is character <=5?
		btfsc		STATUS,carry	
		goto		clear			; Start over if not valid
		incf		rxstate			; Increment and go to next character
		goto		receive

st12								; get_1m
		movf		char,w
		movwf		onemin
		movlw		D'58'
		subwf		char,w			; Is character <=9?
		btfsc		STATUS,carry	
		goto		clear			; Start over if not valid
		incf		rxstate			; Increment and go to next character
		goto		receive

st13								; get_10s
		movlw		D'54'
		subwf		char,w			; Is character <=5?
		btfsc		STATUS,carry	
		goto		clear			; Start over if not valid
		movlw		D'48'
		subwf		char,w			; Top of the minute?	
		btfsc		STATUS,zero
		goto		transmit		; Yes, see if it's time to transmit
st13b	incf		rxstate			; Increment and go to next character
		goto		receive

st14								; get_1s
		movlw		D'58'
		subwf		char,w			; Is character <=9?
		btfsc		STATUS,carry	
		goto		clear			; Start over if not valid
		movlw		B'00001000'	
		xorwf		PORTA			; Toggle LED (RA2)
		goto		clear			; start over

transmit	
		movf		evenmin,w		; Load selected minute to transmit
		subwf		onemin,w
		btfss		STATUS,zero		; Time to transmit?
		goto		st13b			; No - resume 		
		bsf 		PORTA,LED1 		; Turn off LED1
		clrf		PCLATH
		call		main
		goto		clear			; No - start over




;---------------------------------------------------------------------
;	Send instruction message to PC
;---------------------------------------------------------------------

sendinstr
		call		CRLF2			; Send a 'CR' and 'LF'
		call		CRLF2			; Send a 'CR' and 'LF'

Loop
		clrf		TableNum		; Assume msg starts at 0 

LLoop
		movf		TableNum,W		; Pick up address to read
		call 		Table
		movwf		TXREG			; Load transmit register
		sublw		H'ff'			; test for end of message
		btfsc		STATUS,Z		; End of message?	
		goto		rxstart			; Yes: time to load WSPR symbol data
									; No: send character 

LLoop1	btfss		PIR1,TXIF		; Check bit 5 (TXIF) for empty TXREG
		goto		LLoop1			; No: check again
		incf		TableNum		; Increment character counter
		goto		LLoop			; get next character



;------------------------------------------------------------------------------------
; This is the entry point for inputting WSPR symbol text file from PC 
;------------------------------------------------------------------------------------


rxstart
		bcf			INTCON,GIE
		movlw		B'10010000'		; Set SPEN & CREN
		movwf		RCSTA
		movf		RCREG,W			; W = RCREG - this clears RCIF
		movlw		D'04'
		movwf		position		; Symbol group size = 4
		clrf		temp 
		movlw		wdata0			; Start of temporary symbol data table
		movwf		FSR
	
receive2
		bcf			STATUS,carry
		btfsc		PIR1,RCIF		; Check bit 5 (RCIF) for any character received
		goto		getchar8
		bcf			STATUS,carry
		goto		receive2

getchar8	
		movlw		D'06'			; Check for UART overrun or framing error
		andwf		RCSTA,W
		btfsc		STATUS,zero		; Check STATUS register bit 2 -	1=0
		goto		getchar9
		movf		RCREG,W			; W = RCREG
		bcf			RCSTA,CREN		; CREN = 0
		bsf			RCSTA,CREN		; CREN = 1
		goto		receive2

getchar9
		movf		RCREG,W			; W = RCREG - this clears RCIF
		movwf		char
		xorlw		D'43'			; test for end of message "+"
		btfsc		STATUS,Z	
		goto		WriteEE	
		movfw		char
		xorlw		D'48'			; test for ASCII '0'
		btfsc		STATUS,Z
		goto		convert
		movfw		char
		xorlw		D'49'			; test for ASCII '1'
		btfsc		STATUS,Z
		goto		convert
		movfw		char
		xorlw		D'50'			; test for ASCII '2'
		btfsc		STATUS,Z
		goto		convert
		movfw		char
		xorlw		D'51'			; test for ASCII '3'
		btfsc		STATUS,Z
		goto		convert
		goto		receive2

convert
		movlw		D'48'			; Subtract 48 to convert
		subwf		char,1			; ASCII char to symbol number
		decfsz		position
		goto		compress
		goto		write

compress
		movfw		char
		addwf		temp 
		rlf			temp 
		rlf			temp 
		goto		receive2
write
		movfw		char
		addwf		temp,0 
		movwf		INDF
		incf		FSR
		movlw		D'04'			; Reset symbol group size
		movwf		position		
		clrf		temp
		goto		receive2

;------------------------------------------------------------------------------------
; This is the entry point for writing compressed WSPR symbol to EEPROM
;------------------------------------------------------------------------------------


WriteEE
		movlw		wdata0			; Get start of FSR
		movwf		FSR
		movlw		D'41'			; Load 41 symbol group counts
		movwf		count
		bsf 		STATUS, RP0 	; Bank 1
		clrf		EEADR			; Start at EEPROM address 0
		bcf     	STATUS,RP0		; Back to bank 0
EELoop
		movf		INDF,w
		bsf 		STATUS, RP0 	; Bank 1
		movwf   	EEDATA			; w to EEPROM Write register
		call		write_EEPROM	; Send character to EEPROM
		bcf     	STATUS,RP0		; Back to bank 0
		incf		FSR
		decfsz		count
		goto		EELoop

;------------------------------------------------------------------------------------
; This is the entry point for decompressing  & printing WSPR symbol to PC for verification
;------------------------------------------------------------------------------------

		call		CRLF2			; Send 'CR' and 'LF'
		call		CRLF2			; Send 'CR' and 'LF'
		movlw		D'32'			; Load ASCII 'space' required to align 1st row
		call		send4			; Send 'space' to PC
		bsf 		STATUS, RP0 	; Bank 1
		clrf		EEADR			; Start at EEPROM address 0
		bcf     	STATUS,RP0		; Back to bank 0
		movlw		D'41'			; Load 41 symbol group counts
		movwf		count
		movlw		D'04'			; Initiialize symbol group size
		movwf		position
		movlw		D'10'			; Initialize column counter
		movwf		CRLF
		clrf		temp

load1		
		bsf 		STATUS, RP0 	; Bank 1			
		bsf 		EECON1, RD 		; EE Read		
		movf 		EEDATA, W 		; W = EEDATA
		incf		EEADR			; Increment	EEADR
		bcf 		STATUS, RP0 	; Bank 0
		movwf		char
		movlw		B'11000000'		; Mask for 1st symbol
		andwf		char,w
		movwf		temp 
		rrf			temp 
		rrf			temp 
		rrf			temp 
		rrf			temp 
		rrf			temp 
		rrf			temp 			; First symbol is valid at this point
		call		send
		movlw		B'00110000'		; Mask for 2nd symbol
		andwf		char,w
		movwf		temp 
		rrf			temp 
		rrf			temp 
		rrf			temp 
		rrf			temp 			; Second symbol is valid at this point
		call		send
		movlw		B'00001100'		; Mask for 3rd symbol
		andwf		char,w
		movwf		temp 
		rrf			temp 
		rrf			temp 			; Third symbol is valid at this point
		call		send
		movlw		B'00000011'		; Mask for 4th symbol
		andwf		char,w
		movwf		temp 			; Fourth symbol is valid at this point
		call		send
		decfsz		count
		goto		load1
;		bsf			INTCON,GIE
;		goto		WaitForInt


Stop
		goto		Stop




		end

