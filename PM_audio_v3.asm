;
; PM_audio_v3
;
;
; A WSPR controller using pulse width modulation (PWM) to derive narrow-band 
; 4-FSK modulation for a voltage controlled crystal oscillator (VCXO). Three
; data ports are included to modulate an audio signal source.
;
; Copyright (C) 2010,  Gene Marcus W3PM GM4YRE
;
; Permission is granted to use, copy, modify, and distribute this software
; and documentation for non-commercial purposes.
;
; WSPR message generation algorithm is based upon the 'WSPR Coding Process'
; paper authored by Andy Talbot, G4JNT
;
; Portions of this project were influenced by MJB, the stand alone PC-less 
; MEPT_JT beacon controller by Johan Bodin, SM6LKM
;
; 28 March, 2010
; 5 Marcus 2011 - Modified to be compatible with all versions of GPS receivers 
;                 using the $GPGGA NMEA format
;  
; PIC            - 16F628A
; External xtal  - 4 MHz
; UART rate      - 4800 baud
; TX             - RB7 pin 13
; PWM output     - RB3 pin 9
;
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
;     PWM out---------RB3 |9       10| RB4---------Symbol 1         
;                          ----------                                          
; 
;=====================================================================

		processor	pic16f628a
		include		"p16f628a.inc"
		__config		_CP_OFF & _LVP_OFF & _BODEN_OFF & _MCLRE_ON & _XT_OSC & _WDT_OFF & _PWRTE_ON
		errorlevel	2
		movlw		0x07
		movwf		CMCON		; Turn off comparator

		list		b=4,n=70

;=====================================================================
;	Manifest Constants
;=====================================================================

TMR1IF		equ			D'00'
LED1		equ			D'03'		; PORTA pin2 RA3
LED2		equ			D'04'		; PORTA pin3 RA4
;carry		equ			D'00'
zero		equ			D'02'

;=====================================================================
;	File register use
;=====================================================================
		cblock		H'20'

			position
			CRLF
			sec_count
			min_count
			sendcode
			symbolcount
			symboltimer
			pad
			loop_count
			w_temp
			status_temp
			two_sec
			evenmin
			onemin
			rxstate
			char
			tchar
			TableNum
			temp
			temp1
			temp2
			count
			count1
			count2
			count3
			Product
			Product1
			Product2
			Product3
			Multiplier
			Multiplicant
			Multiplicant1
			Multiplicant2
			Multiplicant3
			MultiCount
			Lat10
			Lat1
			NS
			Lon100
			Lon10
			Lon1
			EW
			callsign0
			callsign1
			callsign2
			callsign3
			callsign4
			callsign5
			Loc1
			Loc2
			Loc3
			Loc4
			power0
			power1
			reg00
			reg01
			reg02
			reg03
			cc0
			cc1
			cc2
			cc3
			cc4
			cc5
			cc6
			cc7
			cc8
			cc9
			cc10
			poly10
			poly11
			poly12
			poly13
			poly00
			poly01
			poly02
			poly03
			andtemp00
			andtemp01
			andtemp02
			andtemp03
			parity
			BitCount
			flag
		endc

		cblock		H'a0'
			cdata20
			cdata19
			cdata18
			cdata17
			cdata16
			cdata15
			cdata14
			cdata13
			cdata12
			cdata11
			cdata10
			cdata9
			cdata8
			cdata7
			cdata6
			cdata5
			cdata4
			cdata3
			cdata2
			cdata1
			cdata0
			idata20
			idata19
			idata18
			idata17
			idata16
			idata15
			idata14
			idata13
			idata12
			idata11
			idata10
			idata9
			idata8
			idata7
			idata6
			idata5
			idata4
			idata3
			idata2
			idata1
			idata0
			itemp
			icount
			wmsg40
			wmsg39
			wmsg38
			wmsg37
			wmsg36
			wmsg35
			wmsg34
			wmsg33
			wmsg32
			wmsg31
			wmsg30
			wmsg29
			wmsg28
			wmsg27
			wmsg26
			wmsg25
			wmsg24
			wmsg23
			wmsg22
			wmsg21
			wmsg20
			wmsg19
			wmsg18
			wmsg17
			wmsg16
			wmsg15
			wmsg14
			wmsg13
			wmsg12
			wmsg11
			wmsg10
			wmsg9
			wmsg8
			wmsg7
			wmsg6
			wmsg5
			wmsg4
			wmsg3
			wmsg2
			wmsg1
			wmsg0
			msgtemp
			msgcount
			msgcount1
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
; Tables containing intruction messages and sync vector data
;---------------------------------------------------------------------

table0
		addwf 		PCL,F
		dt			"CALL:"			; Callsign
		dt			H'ff'			; End of message flag

		dt			"GRID:"			; Gridsquare
		dt			H'ff'			; End of message flag	

		dt			"POWER:"		; Power (dB)
		dt			H'ff'			; End of message flag	


; 162 bit Synchronization Vector follows:
table1
		addwf 		PCL,F
		dt			B'11000000',B'10001110',B'00100101',B'11100000',B'00100101'
		dt			B'00000010',B'11001101',B'00011010',B'00011010',B'10101001'
		dt			B'00101100',B'01101010',B'00100000',B'10010011',B'10110011'
		dt			B'01000111',B'00000101',B'00110000',B'00011010',B'11000110'
		dt			B'00000000'





;---------------------------------------------------------------------
; Subroutine to tramsmit symbol data
;---------------------------------------------------------------------

main								; Symbol send code starts here

		movlw		wmsg40			; Get start of FSR
		movwf		FSR
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
		movlw		D'231'
		movwf		CCPR1L
		goto		delay

zer0
		movlw		B'10001000'		;Set RB1	
		movwf		PORTB
		movlw		D'120'
		movwf		CCPR1L
		goto		delay
one
		movlw		B'10010000'		;Set RB2	
		movwf		PORTB
		movlw		D'157'
		movwf		CCPR1L
		goto		delay
two
		movlw		B'10100000'		;Set RB3	
		movwf		PORTB
		movlw		D'194'
		movwf		CCPR1L
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
		movfw		INDF
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
		incf		FSR

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
; Subroutine to send Table0 data to PC
;---------------------------------------------------------------------

PCsend		
		movf		TableNum,W		; Pick up address to read
		call 		table0
		movwf		temp
		sublw		H'ff'			; test for end of message
		btfsc		STATUS,Z		; End of message?
		goto		$+5
		movfw		temp
		call		txchar
		incf		TableNum
		goto		PCsend
		return



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
; Subroutine to send a character
;---------------------------------------------------------------------
txchar
		btfsc		PIR1,TXIF		; TX buffer full?
		goto		txchar1			; No: send character
		goto		txchar			; Yes: go back

txchar1
		movwf		TXREG			; Trnsmit character
txchar2
		btfsc		PIR1,TXIF		; TX buffer full?
		return						; No: return
		goto		txchar2			; Yes: go back

;---------------------------------------------------------------------
; Subroutine to receive character and write to callsign, grid square,
; and power file registers
;---------------------------------------------------------------------
rxstart
		movlw		B'10010000'		; Set SPEN & CREN
		movwf		RCSTA
		movf		RCREG,W			; W = RCREG - this clears RCIF
	
receive2
		bcf			STATUS,C
		btfsc		PIR1,RCIF		; Check bit 5 (RCIF) for any character received
		goto		getchar8
		bcf			STATUS,C
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
		xorlw		D'13'			; test for end of message "CR"
		btfsc		STATUS,Z	
		return		
		movfw		char
		bsf 		STATUS, RP0 	; Bank 1
		movwf		EEDATA
		call		write_EEPROM
		bcf 		STATUS, RP0 	; Bank 0
		movfw		char
		call		txchar			; Load transmit register
		goto		receive2




;---------------------------------------------------------------------
; Subroutine to gererate WSPR message using GPS Lat/Lon
;---------------------------------------------------------------------


; Subroutine convert ASCII callsign, grid square, and
; power data to basenumbers
cnvert
		movlw		D'12'			; Determines file length
		movwf		count
		movlw		callsign0
		movwf		FSR
		bsf 		STATUS, RP0 	; Bank 1
		clrf		EEADR
cnvert1
		bsf 		STATUS, RP0 	; Bank 1			
		bsf 		EECON1, RD 		; EE Read		
		movf 		EEDATA, W 		; W = EEDATA
		incf		EEADR			; Increment	EEADR
		bcf 		STATUS, RP0 	; Bank 0
		bcf			STATUS,C		; Ensure carry is cleared
		movwf		temp
		movlw		D'91'
		subwf		temp,w
		btfsc		STATUS,C 		; Was there a carry?
		goto		cap				; Yes, capitol letter - convert
		movlw		D'58'			; No, test for lowercase
		subwf		temp,w
		btfsc		STATUS,C 		; Was there a carry?
		goto		lc				; Yes, lowercase - convert
		movlw		D'48'			; No, it's a number - convert
		subwf		temp
		goto		se		
cap		movlw		D'87'
		subwf		temp
		goto		se
lc		movlw		D'55'
		subwf		temp
se		movfw		temp
		movwf		INDF			; Write converted callsign character 
		incf		FSR				; into callsign variables  
 		decf		count
		btfss		STATUS,Z		; End of data registers?
		goto		cnvert1			; No: pick up next character
		movlw		D'10'			; Yes: convert grid square letter
		subwf		Loc1			; values to basenumbers
		subwf		Loc2
		call		clrfile			; Now convert power file
		movfw		power0			; registers into base number
		movwf		Multiplicant
		movlw		D'10'
		movwf		Multiplier
		call		Multiply32x8
		movfw		Product
		addwf		power1			; Store result in power1
		return					

; Subroutine to multiply 32 x 8 bit array
Multiply32x8
		clrf		Product			; Clear Product
		clrf		Product+1
		clrf		Product+2
		clrf		Product+3
		movf		Multiplier, W	; Test for an 0 multiplier
		btfsc		STATUS, Z
		return
		movlw		0x08			; Setup the counter for 8 bits
		movwf		MultiCount
MultiplyLoop
		rlf			Multiplier, F
		btfss		STATUS, C
		goto		ShiftLoop
		movf		Multiplicant+0, W
		addwf 		Product+0, F
		btfsc 		STATUS, C
		call 		CarryByte1
		movf 		Multiplicant+1, W
		addwf 		Product+1, F
        btfsc 		STATUS, C
        call 		CarryByte2
		movf 		Multiplicant+2, W
		addwf 		Product+2, F
		btfsc 		STATUS, C
		call 		CarryByte3
		movf 		Multiplicant+3, W
		addwf 		Product+3, F
ShiftLoop
		decfsz 		MultiCount, F
		goto $+2
		return
		bcf 		STATUS, C
		rlf 		Product+0, F	
		rlf 		Product+1, F	
		rlf			Product+2, F	
		rlf 		Product+3, F	
		goto 		MultiplyLoop
CarryByte1
		incfsz 		Product+1, F
		return
CarryByte2
		incfsz 		Product+2, F
		return
CarryByte3
		incf 		Product+3, F
		return


; Subroutine to handle addition carries
carry
		bcf			STATUS,C		; Ensure carry is cleared
		incf		Product+1		; Add carry
		btfss		STATUS,C 		; Was there a carry?
		return						; No, return
		bcf			STATUS,C		; Ensure carry is cleared
		incf		Product+2		; Add carry
		btfss		STATUS,C 		; Was there a carry?
		return						; No, return
		incf		Product+3		; Yes, add carry
		return						

; Subroutine to clear Product and Multiplicant
clrfile
		clrf		Multiplicant
		clrf		Multiplicant1
		clrf		Multiplicant1
		clrf		Multiplicant2
		clrf		Multiplicant3
		clrf		Product
		clrf		Product1
		clrf		Product2
		clrf		Product3
		return
	
; Subroutine to Place Product into Multiplicant
movefiles
		movfw		Product
		movwf		Multiplicant
		movfw		Product1
		movwf		Multiplicant1
		movfw		Product2
		movwf		Multiplicant2
		movfw		Product3
		movwf		Multiplicant3
		return

; Subroutine to return parity for a 32 bit word
parity32
		movlw		D'00'		; Even parity = 0, Odd parity = 1
		movwf		parity
    	movlw		D'32' 		;Total # of bits in four bytes
    	movwf 		BitCount
Rotate
		bcf			STATUS,C
		rlf			andtemp03
		rlf			andtemp02
		rlf			andtemp01
		rlf			andtemp00
 		btfsc 		STATUS,C
 		comf 		parity 		;Complement parity bit store
 		decfsz 		BitCount	; 32 bits counted?
    	goto 		Rotate		; No: rotate and 
		btfsc 		parity,0	; Yes: process parity result
    	goto 		parity1
parity0
    	clrf		parity		;Parity = 0
 		return
parity1 
		clrf		parity
		bsf			parity,0	;Parity = 1
		return

; Subroutine to pack convolved data
cpack
		bsf 		STATUS, RP0 	; Bank 1
		movlw		cdata0		; Get 1st address for convolved data store
		movwf		FSR			; Set FSR to 1st address
		movlw		D'21'		; 21*8 = 168 bits (last 6 bits not used)
		movwf		icount
		bcf			STATUS,C
crotate
		rlf			INDF		; Rotate left through entire 168 bits
		decf		FSR
		decfsz		icount		; All 168 bits rotated?
		goto		$+5			; No: start again				
		movfw		parity		; Yes: get parity status		
		addwf		cdata0		; Place parity bit at bit 168
		bcf 		STATUS, RP0 	; Bank 0
		return
		goto		crotate

; Subroutine to bit reverse 8 bit address
bitrev
		clrf		count1
		btfsc		count,0
		bsf			count1,7
		btfsc		count,1
		bsf			count1,6
		btfsc		count,2
		bsf			count1,5		
		btfsc		count,3
		bsf			count1,4		
		btfsc		count,4
		bsf			count1,3
		btfsc		count,5
		bsf			count1,2
		btfsc		count,6
		bsf			count1,1
		btfsc		count,7
		bsf			count1,0
		return

;Soubroutine to extract sourcebit
sourcebit
		bcf			STATUS,C
		bsf 		STATUS, RP0 	; Bank 1
		movlw		cdata0		; Get 1st address for convolved data store
		movwf		FSR			; Set FSR to 1st address
		movlw		D'21'		; 21*8 = 168 bits (last 6 bits not used)
		movwf		itemp
rl
		rlf			INDF		; Rotate left through entire 168 bits
		decf		FSR
		decf		itemp
		btfss		STATUS,Z	; All 168 bits rotated?
		goto		rl			; No: start again
		bcf 		STATUS, RP0 	; Bank 0
		return					; Yes: return with carry = sourcebit


;Subroutine to write bit in calculated slot
;icount = bit number, INDF = destination register, itemp = bit status
putbit
		bcf			STATUS,Z	; clear zero status bit
		bcf			STATUS,C	; clear carry staus bit
		movlw		D'00'
		subwf		icount
		btfss		STATUS,Z	; Is it bit 0?		
		goto		$+7			; No: goto next test
		movlw		D'01'		; Yes: now check bit status		
		subwf		itemp
		btfss		STATUS,C	; Bit status = 1?
		return					; No: return
		bsf			INDF,0		; Yes: set bit 0
		return
		decfsz		icount		; Is it bit 1?		
		goto		$+7			; No: goto next test
		movlw		D'01'		; Yes: now check bit status
		subwf		itemp
		btfss		STATUS,C	; Bit status = 1?
		return					; No: return
		bsf			INDF,1		; Yes: set bit 1
		return
		decfsz		icount		; Is it bit 2?		
		goto		$+7			; No: goto next test
		movlw		D'01'		; Yes: now check bit status
		subwf		itemp
		btfss		STATUS,C	; Bit status = 1?
		return					; No: return
		bsf			INDF,2		; Yes: set bit 2
		return
		decfsz		icount		; Is it bit 3?		
		goto		$+7			; No: goto next test
		movlw		D'01'		; Yes: now check bit status
		subwf		itemp
		btfss		STATUS,C	; Bit status = 1?
		return					; No: return
		bsf			INDF,3		; Yes: set bit 3
		return
		decfsz		icount		; Is it bit 4?		
		goto		$+7			; No: goto next test
		movlw		D'01'		; Yes: now check bit status
		subwf		itemp
		btfss		STATUS,C	; Bit status = 1?
		return					; No: return
		bsf			INDF,4		; Yes: set bit 4
		return
		decfsz		icount		; Is it bit 5?		
		goto		$+7			; No: goto next test
		movlw		D'01'		; Yes: now check bit status
		subwf		itemp
		btfss		STATUS,C	; Bit status = 1?
		return					; No: return
		bsf			INDF,5		; Yes: set bit 5
		return	
		decfsz		icount		; Is it bit 6?		
		goto		$+7			; No: goto next test
		movlw		D'01'		; Yes: now check bit status
		subwf		itemp
		btfss		STATUS,C	; Bit status = 1?
		return					; No: return
		bsf			INDF,6		; Yes: set bit 6
		return
		decfsz		icount		; Is it bit 7?		
		goto		$+7			; No: return
		movlw		D'01'		; Yes: now check bit status
		subwf		itemp
		btfss		STATUS,C	; Bit status = 1?
		return					; No: return
		bsf			INDF,7		; Yes: set bit 7
		return



; Subroutine to convert Lat/Lon to gridsquare basenumbers
latlon
		movlw		D'16'
		movwf		count
		movlw		D'1'
		movwf		temp
		movlw		D'48'			; Load ASCII to number converion factor
		subwf		Lon100			; Convert longitude 100's value to basenumber
		subwf		Lon10			; Convert longitude 10's value to basenumber
		subwf		Lon1			; Convert longitude 1's value to basenumber
		subwf		Lat10			; Convert latitiude 10's value to basenumber
		subwf		Lat1			; Convert latitude 1's value to basenumber
		clrf		temp
		clrf		temp1
		btfss		Lon100,0
		goto		$+3
		movlw		D'100'
		addwf		temp
		movlw		D'10'
		call		clrfile
		movwf		Multiplier
		movfw		Lon10
		movwf		Multiplicant
		call		Multiply32x8
		movfw		Product
		addwf		temp
		movfw		Lon1
		addwf		temp			; temp now equals longitude				
		call		clrfile
		movlw		D'10'
		movwf		Multiplier
		movfw		Lat10
		movwf		Multiplicant
		call		Multiply32x8
		movfw		Product
		addwf		temp1
		movfw		Lat1
		addwf		temp1			; temp1 now equals latitude			

; Start longitude conversion
		clrf		count
		clrf		count1
		movlw		D'10'
		movwf		count2	
		clrf		count3
		movlw		D'20'
		movwf		temp2
		movlw		D'69'			; Set w for ASCII character "E"
		subwf		EW,w
		btfsc		STATUS,Z		; Does EW = "E"
		goto		eloncalc		; 
		movlw		D'9'			; 
		movwf		count2
		movlw		D'09'
		movwf		count3
		goto		wloncalc

eloncalc
		movfw		count
		subwf		temp,w
		btfsc		STATUS,Z
		goto		loncalc
		incf		count
		incf		count1
		btfsc		count,0
		goto		$+2
		incf		count3
		movfw		count3
		sublw		D'10'
		btfsc		STATUS,Z
		clrf		count3	
		movfw		temp2
		subwf		count1,w
		btfss		STATUS,Z
		goto		$+3
		incf		count2
		clrf		count1
		goto		eloncalc

wloncalc

		movfw		count
		subwf		temp,w
		btfsc		STATUS,Z
		goto		loncalc
		incf		count
		incf		count1
		btfsc		count,0
		goto		$+2
		decf		count3	
		movfw		count3
		sublw		D'255'
		btfss		STATUS,Z
		goto		$+3
		movlw		D'09'
		movwf		count3
		movfw		temp2
		subwf		count1,w
		btfss		STATUS,Z
		goto		$+3
		decf		count2
		clrf		count1
		goto		wloncalc

loncalc
		decf		count2			; Make compatible for N1 calculations
		movfw		count2
		movwf		Loc1
		movfw		count3
		movwf		Loc3

; Start latitude conversion
		clrf		count		
		movlw		D'10'
		movwf		temp2
		movwf		count2
		clrf		count1
		movlw		D'78'			; Set w for ASCII character "N"
		subwf		NS,w
		btfsc		STATUS,Z		
		goto		nlatcalc		; Yes:
		movlw		D'89'			; No
		movwf		count			
		clrf		count1
		movlw		D'01'
		movwf		count2
		goto		slatcalc
nlatcalc
		movfw		count
		subwf		temp1,w
		btfsc		STATUS,Z
		goto		latcalc1
		incf		count
		incf		count1
		movfw		temp2
		subwf		count1,w
		btfss		STATUS,Z
		goto		$+3
		incf		count2
		clrf		count1
		goto		nlatcalc

slatcalc
		movfw		count
		subwf		temp1,w
		btfsc		STATUS,Z
		goto		latcalc1
		decf		count
		incf		count1
		movfw		temp2
		subwf		count1,w
		btfss		STATUS,Z
		goto		$+3
		incf		count2
		clrf		count1
		goto		slatcalc
	
latcalc1
		decf		count2				; Make compatible for N1 calculations
		movfw		count2
		movwf		Loc2
		movfw		count1
		movwf		Loc4
		
		return
___________________________________________________________________________________
___________________________________________________________________________________

; This is the entry point for the auto WSPR memory generator subroutine

msggen
		movlw		0x51			; Load first parity gen feedback tap
		movwf		poly00			; 0xF2D05351
		movlw		0x53
		movwf		poly01
		movlw		0xd0
		movwf		poly02
		movlw		0xf2
		movwf		poly03
		movlw		0x47			; Load second parity gen feedback tap
		movwf		poly10			; 0xE4613C47
		movlw		0x3c
		movwf		poly11
		movlw		0x61
		movwf		poly12
		movlw		0xe4
		movwf		poly13

		bcf			STATUS,C

align	movlw		D'10'
		subwf		callsign2,w		; Is callsign2 a number?
		btfss		STATUS,C 		; Was there a carry?        
		goto		sp				; No: check for spaces
		movfw		callsign4		; Yes: realign callsign data
		movwf		callsign5		; to ensure number is in callsign3
		movfw		callsign3
		movwf		callsign4
		movfw		callsign2
		movwf		callsign3
		movfw		callsign1
		movwf		callsign2
		movfw		callsign0
		movwf		callsign1
		movlw		D'36'			; Set 1st  character (callsign0) to 'space' 
		movwf		callsign0
sp		bcf			STATUS,Z			
		movlw		D'0'
		subwf		callsign4,w
		btfsc		STATUS,Z 		; Is it zero? 
		goto		c4				; Yes: set callsign4 to 'space'
c3		bcf			STATUS,Z		; No: check callsign 5 for 'space'		
		movlw		D'0'		
		subwf		callsign5,w
		btfsc		STATUS,Z 		; Is it zero? 
		goto		c5				; Yes: set callsign5 to 'space'
		goto		Ncompress		; No: callsign conversion is complete time to calculate N
c4		movlw		D'36'
		movwf		callsign4		; 5th position of callsign is a 'space'
		goto		c3
c5		movlw		D'36'
		movwf		callsign5		; 6th position of callsign is a 'space'

; Calculate callsign compressed integer N
Ncompress
		call		clrfile
		movfw		callsign0
		movwf		Multiplicant+0
		movlw		D'36'
		movwf		Multiplier		; Multiply 1st callsign character by 36
		call 		Multiply32x8
		bcf			STATUS,C		; Ensure carry is cleared
		movfw		callsign1
		addwf		Product			; Add 2nd callsign character to the product
		btfsc		STATUS,C 		; Was there a carry?
		call		carry			; Yes: add the carry to the product
		call		movefiles		; No: place product into multiplicant
		movlw		D'10'
		movwf		Multiplier		; Multiply product by 10
		call 		Multiply32x8
		bcf			STATUS,C		; Ensure carry is cleared
		movfw		callsign2
		addwf		Product			; Add 3rd callsign character to the product
		btfsc		STATUS,C 		; Was there a carry?
		call		carry			; Yes: add the carry to the product
		call		movefiles		; No: place product into multiplicant
		movlw		D'27'
		movwf		Multiplier		; Multiply product by 27
		call 		Multiply32x8
		bcf			STATUS,C		; Ensure carry is cleared
		movlw		D'10'
		subwf		callsign3,w		; Subtract 10 from 4th callsign character
		addwf		Product			; Place result into the product
		btfsc		STATUS,C 		; Was there a carry?
		call		carry			; Yes: add the carry to the product
		call		movefiles		; No: place product into multiplicant
		movlw		D'27'
		movwf		Multiplier		; Multiply product by 27
		call 		Multiply32x8
		bcf			STATUS,C		; Ensure carry is cleared
		movlw		D'10'
		subwf		callsign4,w		; Subtract 10 from 5th callsign character
		addwf		Product			; Place result into the product
		btfsc		STATUS,C 		; Was there a carry?
		call		carry			; Yes: add the carry to the product
		call		movefiles		; No: place product into multiplicant
		movlw		D'27'
		movwf		Multiplier		; Multiply product by 27
		call 		Multiply32x8
		bcf			STATUS,C		; Ensure carry is cleared
		movlw		D'10'
		subwf		callsign5,w		; Subtract 10 from 6th callsign character
		addwf		Product			; Place result into the product
		btfsc		STATUS,C 		; Was there a carry?
		call		carry			; Yes: add the carry to the product
		call		movefiles		; No: place product into multiplicant
		movlw		D'16'			; Multpier to rotate data 4 bits left 
		movwf		Multiplier		; Multiply product by 16
		call 		Multiply32x8
		movfw		Product3		; Load first 28 bits of 50 bit word
		movwf		cc0				
		movfw		Product2
		movwf		cc1
		movfw		Product1
		movwf		cc2
		movfw		Product
		movwf		cc3

; Calculate location data for M1 
N1
		call 		clrfile
		movlw		D'10'
		movwf		Multiplier
		movfw		Loc1
		movwf		Multiplicant+0
		call		Multiply32x8
		movlw		D'179'
		movwf		temp
		movfw		Product
		subwf		temp
		call		clrfile
		movfw		Loc3
		subwf		temp,f
		movlw		D'10'
		movwf		Multiplier
		movfw		Loc2
		movwf		Multiplicant+0
		call		Multiply32x8		
		movfw		Product
		movwf		temp1
		call		clrfile
		movfw		temp
		movwf		Multiplier
		movlw		D'180'		
		movwf		Multiplicant+0
		call		Multiply32x8
		bcf			STATUS,C		; Ensure carry is cleared
		movfw		temp1
		addwf		Product
		btfsc		STATUS,C 		; Was there a carry?
		call		carry
		bcf			STATUS,C		; Ensure carry is cleare
		movfw		Loc4
		addwf		Product
		btfsc		STATUS,C 		; Was there a carry?
		call		carry
		call		movefiles

; Calculate M
		movlw		D'128'
		movwf		Multiplier
		call		Multiply32x8
		bcf			STATUS,C		; Ensure carry is cleared
		movfw		power1
		addwf		Product
		btfsc		STATUS,C 		; Was there a carry?
		call		carry
		bcf			STATUS,C		; Ensure carry is cleared
		movlw		D'64'
		addwf		Product
		btfsc		STATUS,C 		; Was there a carry?
		call		carry
		call		movefiles

; Load M into last 22 bits of 50 bit word
		movlw		D'4'			; Multplier to rotate data 2 bits left 
		movwf		Multiplier
		call 		Multiply32x8		
		swapf		Product2,f
		movlw		B'00001111'
		andwf		Product2,w
		addwf		cc3,f
		movlw		B'11110000'
		andwf		Product2,w
		movwf		cc4

		swapf		Product1,f
		movlw		B'00001111'
		andwf		Product1,w
		addwf		cc4,f
		movlw		B'11110000'
		andwf		Product1,w
		movwf		cc5

		swapf		Product,f
		movlw		B'00001111'
		andwf		Product,w
		addwf		cc5,f
		movlw		B'11110000'
		andwf		Product,w
		movwf		cc6

; Routine to shift 81 bit files cc0...cc10 left by 1 bit 
		movlw		D'82'
		movwf		count

clrloop
		decf		count
		btfsc		STATUS,Z
		goto		align1
		movlw		D'11'
		movwf		count1
		movlw		cc10				; Start of temporary symbol data table
		movwf		FSR
		bcf			STATUS,C
		
sftloop1	
		rlf			INDF
		decf		FSR
		decf		count1
		btfsc		STATUS,Z
		goto		loadreg
		goto		sftloop1		


; Routine to load shifted carry bit from cc0 and shift into two 32 bit registers 
loadreg
		rlf			reg03
		rlf			reg02
		rlf			reg01
		rlf			reg00


; AND contents of 32 bit "reg" registers with "poly" registers - save result in "andtemp"
		movfw		poly00
		andwf		reg03,w
		movwf		andtemp03
		movfw		poly01
		andwf		reg02,w
		movwf		andtemp02
		movfw		poly02
		andwf		reg01,w
		movwf		andtemp01
		movfw		poly03
		andwf		reg00,w
		movwf		andtemp00
		call		parity32
		call		cpack
		movfw		poly10
		andwf		reg03,w
		movwf		andtemp03
		movfw		poly11
		andwf		reg02,w
		movwf		andtemp02
		movfw		poly12
		andwf		reg01,w
		movwf		andtemp01
		movfw		poly13
		andwf		reg00,w
		movwf		andtemp00
		call		parity32
		call		cpack
		goto		clrloop

align1
		clrf		parity
		call		cpack
		call		cpack
		call		cpack
		call		cpack
		call		cpack
		call		cpack

; Interleave process
		clrf		count
		clrf		count1
		clrf		count2
		movlw		D'161'
		movwf		temp
aa
		call		bitrev
		bcf			STATUS,C
		movfw		count1
		subwf		temp,w
		btfsc		STATUS,C
		goto		$+3
		incf		count
		goto		aa
		call		sourcebit
		movlw		D'00'
		btfss		STATUS,C
		goto		$+2
		movlw		D'01'
		bsf 		STATUS, RP0 	; Bank 1			
		movwf		itemp
		movlw		idata20
		bcf 		STATUS, RP0 	; Bank 0
		movwf		FSR
ab
		bcf			STATUS,C
		movlw		D'08'
		subwf		count1
		btfss		STATUS,C
		goto		$+3
		incf		FSR
		goto		ab
		movlw		D'08'
		addwf		count1,w		; count1 now reflects bit number in INDF
		sublw		D'07'
		bsf 		STATUS, RP0 	; Bank 1
		movwf		icount
		call		putbit
		bcf 		STATUS, RP0 	; Bank 0
		incf		count
		incf		count2
		movfw		count2
		subwf		temp,w			; Check for 162nd iteration
		btfss		STATUS,C
		goto		merge
		goto		aa
		
; Merge with sync vector
merge
		movlw		D'164'			; 2 extra chars added to 162 char
		movwf		temp			; msg to align data
		clrf		TableNum
merge1		
		movf		TableNum,W		; Pick up address to read
		call 		table1
		movwf		temp1
		incf		TableNum
		movlw		D'09'
		movwf		count
merge6
		bcf			STATUS,C
		clrf		temp2			; Clear sync vector variable
		rlf			temp1
		btfss		STATUS,C
		goto		$+2
		bsf			temp2,0			; Sync vector set		
		decfsz		count		
		goto		$+2
		goto		merge1			; Get next 8 characters
		decfsz		temp			; Test for 162nd character		
		goto		$+3
		bcf 		STATUS, RP0 	; Bank 0
		return
		movfw		temp2			; get sync vector status
		bsf 		STATUS, RP0 	; Bank 1			
		movwf		itemp			; Move sync vector status to bank 1
		movlw		idata0
		movwf		FSR
		bcf			STATUS,C
		movlw		D'21'
		movwf		icount
merge3
		rlf			INDF
		decf		FSR
		decfsz		icount
		goto		merge3
		clrf		msgtemp
		btfss		STATUS,C
		goto		$+3
		movlw		D'02'
		addwf		msgtemp	
		movfw		itemp
		addwf		msgtemp,w
		addwf		wmsg0
		movlw		D'02'
		movwf		msgcount1
merge4
		movlw		D'41'
		movwf		msgcount
		movlw		wmsg0
		movwf		FSR
		bcf			STATUS,C
merge5			
		rlf			INDF
		decf		FSR
		decfsz		msgcount
		goto		merge5
		decfsz		msgcount1
		goto		merge4
		bcf 		STATUS, RP0 	; Bank 0		
		goto		merge6


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
		bsf			INTCON,GIE		; Enable interrupts

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

		call		cnvert
		call		msggen			; Called twice to clear residual
		call		msggen			; data from rolling file registers

		movlw		wmsg40			; Get start of FSR
		movwf		FSR
		movlw		D'41'			; Load 41 symbol group counts
		movwf		count
		bsf 		STATUS, RP0 	; Bank 1
		movlw		D'12'
		movwf		EEADR			; Start at EEPROM address 12
		bcf     	STATUS,RP0		; Back to bank 0
xfrloop
		bsf 		STATUS, RP0 	; Bank 1			
		bsf 		EECON1, RD		; EE Read		
		movf 		EEDATA, W		; W = EEDATA
		incf		EEADR
		bcf 		STATUS, RP0 	; Bank 0
		movwf		INDF
		incf		FSR
		decfsz		count
		goto		xfrloop

;---------------------------------------------------------------------
;	Initialize PWM output
;---------------------------------------------------------------------

; Set PWM period to 3.906 KHz
		banksel	PR2
		movlw	0xff
		movwf	PR2

; Set MSB bits of duty cycle for initial setting of 50%
		banksel	CCPR1L
		movlw	B'01111111'
		movwf	CCPR1L

; Set LSB 2 bits of duty cycle for initial setting of 50%
;  and turn on PWM
		movlw	B'00111100'
		movwf	CCP1CON

; Clear TRISB<3>
		banksel	TRISB
		bcf		TRISB,3			  

; Turn on timer 2, prescale=1, postscale=1
		banksel	T2CON
		movlw	B'00000100'
		movwf	T2CON


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
		bcf			STATUS,C
		btfsc		PIR1,RCIF		; Check bit 5 (RCIF) for any character received
		goto		getchar5
		bcf			STATUS,C
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

		org			H'500'
nmeastart
		bcf			INTCON,GIE

clear
		movlw		D'05'
		movwf		PCLATH

		clrf		rxstate			; Start over
	
receive
		bcf			STATUS,C
		btfsc		PIR1,RCIF		; Check bit 5 (RCIF) for any character received
		goto		getchar
		bcf			STATUS,C
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
		bcf			STATUS,C
		goto		receive			; Get next character

getchar3
		movlw		D'23'
		subwf		rxstate,w		; Check for illegal state - rxstate > 22
		btfss		STATUS,C
	    goto		getchar4
		goto		receive

getchar4
		movf		rxstate,w
		addwf		PCL,F			; Jump to rxstate number
		goto		receive			; idle
		goto		st1				; get_Gp
		goto		st2				; get_gP
		goto		st3				; get_gpGga
		goto		st4				; get_gpgGa
		goto		st5				; get_gpgga
		goto		st6				; get_comma
		goto		st7				; get_10h
		goto		st8 			; get_1h
		goto		st9		    	; get_10m
		goto		st10			; get_1m
		goto		st11			; get_10s
		goto		st12			; wait for a comma
		goto		st13			; get_tens deg lat
		goto		st14			; get_ones deg lat
		goto		st15			; wait for a comma
		goto		st16			; get_N/S latitude
		goto		st17			; get_comma
		goto		st18			; get_hundreds deg lon
		goto		st19			; get_tens deg lon
		goto		st20			; get_ones deg lon
		goto		st21			; wait for a comma
		goto		st22			; get_E/W longitude


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

st3									; get_gpGga
		movf		char,w
		xorlw		D'71'			; Is receive charactor G?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		clear			; 
		incf		rxstate			; We have "GPG"
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
		incf		rxstate
		goto		receive			; We have "GPGGA" - rxstate=8 - get next character

st6									; get_comma
		movf		char,w
		xorlw		D'44'			; Is receive charactor a comma?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		clear			; If not, clear rxstate and wait for another '$'
		incf		rxstate
		goto		receive			; Get next character

st7									; get_10h
		movlw		D'51'
		subwf		char,w			; Is character <=2?
		btfsc		STATUS,C	
		goto		clear			; Start over if not valid
		incf		rxstate			; Increment and go to next character
		goto		receive

st8 								; get_1h
		movlw		D'58'
		subwf		char,w			; Is character <=9?
		btfsc		STATUS,C	
		goto		clear			; Start over if not valid
		incf		rxstate			; Increment and go to next character
		goto		receive

st9 								; get_10m
		movlw		D'54'
		subwf		char,w			; Is character <=5?
		btfsc		STATUS,C	
		goto		clear			; Start over if not valid
		incf		rxstate			; Increment and go to next character
		goto		receive

st10								; get_1m
		movf		char,w
		movwf		onemin
		movlw		D'58'
		subwf		char,w			; Is character <=9?
		btfsc		STATUS,C	
		goto		clear			; Start over if not valid
		incf		rxstate			; Increment and go to next character
		goto		receive

st11								; get_10s
		movlw		D'54'
		subwf		char,w			; Is character <=5?
		btfsc		STATUS,C	
		goto		clear			; Start over if not valid
		movlw		D'48'
		subwf		char,w			; Top of the minute?	
		btfsc		STATUS,zero
		goto		transmit		; Yes, see if it's time to transmit
st11b	incf		rxstate			; Increment and go to next character
		goto		receive

st12								; wait for a comma
		movf		char,w
		xorlw		D'44'			; Is receive charactor a comma?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		receive			; If not, go back and wait for a comma
		incf		rxstate
		goto		receive

st13								; get_tens deg lat
		movf		char,w
		movwf		Lat10
		incf		rxstate
		goto		receive

st14								; get_ones deg lat
		movf		char,w
		movwf		Lat1
		incf		rxstate
		goto		receive

st15								; wait for a comma
		movf		char,w
		xorlw		D'44'			; Is receive charactor a comma?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		receive			; If not, go back and get next character
		incf		rxstate
		goto		receive

st16								; get_N/S latitude
		movf		char,w
		movwf		NS
		incf		rxstate
		goto		receive

st17
		movf		char,w
		xorlw		D'44'			; Is receive charactor a comma?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		clear			; If not, clear rxstate and wait for another '$'
		incf		rxstate
		goto		receive

st18								; get_hundreds deg lon
		movf		char,w
		movwf		Lon100
		incf		rxstate
		goto		receive

st19								; get_tens deg lon
		movf		char,w
		movwf		Lon10
		incf		rxstate
		goto		receive

st20								; get_ones deg lon
		movf		char,w
		movwf		Lon1
		incf		rxstate
		goto		receive

st21								; wait for a comma
		movf		char,w
		xorlw		D'44'			; Is receive charactor a comma?
		btfss		STATUS,zero		; If so, increment rxstate
		goto		receive			; If not, go back and wait for a comma
		incf		rxstate
		goto		receive

st22								; get_E/W longitude
		movf		char,w
		movwf		EW
		movlw		B'00001000'	
		xorwf		PORTA			; Toggle LED (RA2)
		goto		clear			; start over

transmit	
		movf		evenmin,w		; Load selected minute to transmit
		subwf		onemin,w
		btfss		STATUS,zero		; Time to transmit?
		goto		st11b			; No - resume 		
		bsf 		PORTA,LED1 		; Turn off LED1
		clrf		PCLATH
		call		cnvert
		call		latlon			; Yes: update Lat/Lon
		call		msggen			; Generate new msg		
		call		main			; Goto transmit
		goto		clear			; Start over

;---------------------------------------------------------------------
;	Send instruction message to PC and write callsign, grid square 
;   and power data EEPROM
;---------------------------------------------------------------------

sendinstr
		bcf			INTCON,GIE		; Disable interrupts
		call		CRLF2			; Send a 'CR' and 'LF'
		call		CRLF2			; Send a 'CR' and 'LF'

		clrf		TableNum		; Assume msg starts at 0 
		bsf 		STATUS, RP0 	; Bank 1
		movlw		D'06'
		movwf		icount
		clrf		EEADR			; Start at EEPROM address 0
LLoop

		movlw		"0"				; Ensure callsign registers are clear
		movwf		EEDATA			; w to EEPROM Write register
		call		write_EEPROM	; Send character to EEPROM
		incf		EEADR
		decfsz		icount
		goto		LLoop
		clrf		EEADR			; Set EEPROM address t0 0
		bcf 		STATUS, RP0 	; Bank 0

		call		PCsend			; Send callsign load instr to PC
		call		rxstart			; Get data and store in EEPROM

		call		CRLF2			; Send a 'CR' and 'LF'
		
		bsf 		STATUS, RP0 	; Bank 1
		movlw		D'06'
		movwf		EEADR			; Start at EEADR 6
		bcf 		STATUS, RP0 	; Bank 0
		incf		TableNum		; Get gridsquare load instr address

		call		PCsend			; Send gridsquare load instr to PC
		call		rxstart			; Get data and store in EEPROM

		call		CRLF2			; Send a 'CR' and 'LF'
		
		bsf 		STATUS, RP0 	; Bank 1
		movlw		D'10'
		movwf		EEADR			; Start at EEADR 10
		bcf 		STATUS, RP0 	; Bank 0
		incf		TableNum		; Get power load instr address

		call		PCsend			; Send power load instr to PC
		call		rxstart			; Get data and store in EEPROM

; Entry point to enerate WSPR message.
		call		cnvert
		call		msggen			; All data loaded - generate message


;------------------------------------------------------------------------------------
; This is the entry point for writing WSPR symbol message to EEPROM
;------------------------------------------------------------------------------------

WriteEE
		movlw		wmsg40			; Get start of FSR
		movwf		FSR
		movlw		D'41'			; Load 41 symbol group counts
		movwf		count
		bsf 		STATUS, RP0 	; Bank 1
		movlw		D'12'
		movwf		EEADR			; Start at EEPROM address 12
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
		movlw		D'12'
		movwf		EEADR			; Start at EEPROM address 12
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


Stop
		goto		Stop


		end

