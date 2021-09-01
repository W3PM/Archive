; wwvb_pwm_wspr
;
; A WSPR controller using pulse width modulation (PWM) to derive narrow-band 
; 4-FSK modulation from a voltage controlled crystal oscillator (VCXO).
; Minute and second data from WWVB receiver is used to update an internal 
; software clock to provide precise WSPR timing.
; 
; Copyright (C) 2010,  Gene Marcus W3PM GM4YRE
;
; Permission is granted to use, copy, modify, and distribute this software
; and documentation for non-commercial purposes.
;
; Portions of this project were influenced by MJB, the stand alone PC-less 
; MEPT_JT beacon controller by Johan Bodin, SM6LKM
;
; 15 March, 2011 Modified to incorporate two transmit timeslots
;
;  
; PIC            - 16F628A
; External xtal  - 4 MHz
; WWVB RX input  - RA4 pin 3
; TX             - RB7 pin 13
; PWM output     - RB3 pin 9

;
; Gene Marcus W3PM GM4YRE
;
;  20 December, 2010
;
;  
;
; 					       PIC16F628A                             
;                          __________                                          
;           	  ----RA2 |1       18| RA1---------                   
;                 ----RA3 |2       17| RA0---------                   
;     RX data in------RA4 |3       16| OSC1--------XTAL                        
;     +5V-----------!MCLR |4       15| OSC2--------XTAL                        
;     Ground----------Vss |5       14| VDD---------+5 V                        
;               ------RB0 |6       13| RB7---------TX                   
;     Valid time LED--RB1 |7       12| RB6---------    
;     RX data LED-----RB2 |8       11| RB5---------          
;     PWM output------RB3 |9       10| RB4---------         
;                          ----------                                          
; 
;=====================================================================

		processor	pic16f628a
		include		"p16f628a.inc"
		__config		_CP_OFF & _LVP_OFF & _BODEN_OFF & _MCLRE_OFF & _XT_OSC & _WDT_OFF & _PWRTE_ON

		errorlevel	2
		movlw		0x07
		movwf		CMCON		; Turn off comparator

		list		b=4,n=70

;=====================================================================
;	Manifest Constants
;=====================================================================

RX			equ			D'04'		; PORTA pin3 RA4


;=====================================================================
;	File register use
;=====================================================================
		cblock		H'20'
			sec_count_500ms		; interrupt seconds count
			min_count			; interrupt minutes count
			w_temp
			status_temp
			count_20ms	
			pulsecount
			wwvb_min
			tx_flag
			timeslot
			timeslot2
			countsync
			bitcount
			temp	
			position
			symbolcount
			symboltimer
			pad
			count
			two_sec
			char
			tchar
			bytecount
			txflagtimer			; Inhibit TX if software clock is not updated
								; for 36 hours 		

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

		movlw		D'011'			; Reset TMR1 for 500 mS
		movwf		TMR1H
		movlw		D'221'
		movwf		TMR1L

		decfsz		sec_count_500ms		; count 500 mS intervals
		goto		main3
		movlw		D'120'
		movwf		sec_count_500ms
		
		incf		min_count
		movlw		D'20'
		xorwf		min_count,w

		btfss		STATUS,Z
		goto		main3
		clrf		min_count
		decfsz		txflagtimer
		goto		main3
		bcf			tx_flag,0		;Inhibit TX until valid time update
		bsf			PORTB,1			;Turn off LED3

main3			
		bcf			PIR1,TMR1IF		; Clear the old interrupt
		swapf		status_temp,W   ; Restore the status
		movwf		STATUS			; register
		swapf		w_temp,F		; Restore W without disturbing
		swapf		w_temp,W		; the STATUS register

		retfie


;---------------------------------------------------------------------
;	Set WSPR symbol data
;
; WSPR Data Packed 4 symbols per byte - MSBits first
; W3PM  EM64 27         (Generated from 'GENWSPR.EXE'   G4JNT June 2009)
;
;---------------------------------------------------------------------

table
		
		addwf		PCL,F

  dt 0xD2, 0xA0, 0x40, 0xD6, 0xAC, 0x33, 0xF6, 0x20

  dt 0x2E, 0x19, 0xA8, 0xAC, 0xD8, 0xF1, 0xA1, 0x66

  dt 0xA9, 0x44, 0x66, 0x6B, 0x0E, 0x5A, 0x3C, 0xE4

  dt 0xA4, 0xAA, 0x41, 0xA5, 0xE5, 0x27, 0xB8, 0x9F

  dt 0x80, 0x39, 0x27, 0x00, 0xAB, 0xEC, 0x78, 0x1C

  dt 0xA0  ;  End of WSPR Data


;---------------------------------------------------------------------
; Subroutine to tramsmit symbol data
;---------------------------------------------------------------------

transmit							; Symbol send code starts here

		clrf		bytecount
		movlw		D'163'
		movwf		symbolcount
		bsf			PORTB,7			; Set RB7 to turn on TX
		movlw		D'31'			; 2 sec delay before data start
		movwf		two_sec			; 
		movlw		D'04'			; Initiialize symbol group size
		movwf		position
		clrf		temp
		movlw		D'21'
		movwf		symboltimer
							
tdelay								; 
 		btfss 		INTCON,T0IF 		; Did timer overflow?
		goto 		tdelay 			; No, hang around some more
		bcf 		INTCON,T0IF 		; reset overflow flag
		decfsz 		two_sec,F 		; Count down
		goto 		tdelay			; Not time yet	
		movlw 		D'130' 			; Timer will count 	
		movwf 		TMR0			; 127 (256-129) counts

symbol_loop
		movlw		D'21'
		movwf		symboltimer
		decfsz		symbolcount
		goto		continue2		; No; continue
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
		movlw		D'233'
		movwf		CCPR1L
		goto		delay

zer0
		movlw		D'123'
		movwf		CCPR1L
		goto		delay
one
		movlw		D'159'
		movwf		CCPR1L
		goto		delay
two
		movlw		D'196'
		movwf		CCPR1L

delay	
 		btfss 		INTCON,T0IF 	; Did timer overflow?
		goto 		delay 			; No, hang around some more
		movlw		d'39'			;
		movwf		pad				; Calibrate symbol timer
timepad	decfsz		pad,F			; to 680 mSec
		goto		timepad			;
		bcf 		INTCON,T0IF 	; reset overflow flag
		movlw 		D'130' 			; Timer will count 	
		movwf 		TMR0			; 127 (256-129) counts
		decfsz 		symboltimer,F 	; Count down
		goto 		delay			; Not time yet											
		goto		symbol_loop

stop
		bcf			PORTB,7			; Clear RB7 to turn off TX			

		return


;---------------------------------------------------------------------
; Subroutine to download symbol data from symbol table and decompress
;---------------------------------------------------------------------

getsymbol
		movf		position,w
		xorlw		D'04'
		btfss		STATUS,Z	
		goto		pos3		

		movfw		bytecount
		call		table
		incf		bytecount
		movwf		char
		movlw		B'11000000'		; Mask for 1st symbol
		andwf		char,w
		movwf		temp 
		bcf		STATUS,C
		rrf		temp 
		rrf		temp 
		rrf		temp 
		rrf		temp 
		rrf		temp 
		rrf		temp 			; First symbol is valid at this point
		movfw		temp
		movwf		tchar
		decf		position
		return

pos3
		movf		position,w
		xorlw		D'03'
		btfss		STATUS,Z	
		goto		pos2
		movlw		B'00110000'		; Mask for 2nd symbol
		andwf		char,w
		movwf		temp 
		bcf		STATUS,C
		rrf		temp 
		rrf		temp 
		rrf		temp 
		rrf		temp 			; Second symbol is valid at this point
		movfw		temp
		movwf		tchar
		decf		position
		return

pos2
		movf		position,w
		xorlw		D'02'
		btfss		STATUS,Z	
		goto		pos1
		movlw		B'00001100'		; Mask for 3rd symbol
		andwf		char,w
		movwf		temp 
		bcf		STATUS,C
		rrf		temp 
		rrf		temp 			; Third symbol is valid at this point
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

		return



;____________________________________________________________________
;____________________________________________________________________

start

;---------------------------------------------------------------------
;	Select transmit time slot and disable tranmitter until valid
;	WWVB time data is loaded into software clock
;---------------------------------------------------------------------
		movlw		D'08'
		movwf		timeslot
		movlw		D'18'
		movwf		timeslot2
		clrf		tx_flag

;---------------------------------------------------------------------
;	Set up I/O 
;---------------------------------------------------------------------
		banksel		TRISB
		clrw						; 
		movwf		TRISB			; Make all port B output
;		banksel		PORTA
		movlw		B'00010000'
		movwf		TRISA			; Make RA4 input

;=====================================================================
;	Set up timer0
;=====================================================================
		banksel		OPTION_REG
		movlw		B'11010111'		; Select TOCS, 256 prescale, pull-ups
		movwf		OPTION_REG


;=====================================================================
;	Set up timer1
;=====================================================================
		banksel		T1CON
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


;---------------------------------------------------------------------
;	Initialize memory
;---------------------------------------------------------------------
		banksel		PORTA
		clrf		PORTA

		movlw		D'120'
		movwf		sec_count_500ms
		clrf		min_count

		banksel		EEADR
		clrf		EEADR


;---------------------------------------------------------------------
;	Initialize PWM output
;---------------------------------------------------------------------

; Set PWM period to 3.906 KHz
		banksel	PR2
		movlw	0xff
		movwf	PR2

; Set MSB bits of duty cycle for ultimate goal of 50%
		banksel	CCPR1L
		movlw	B'01111111'
		movwf	CCPR1L

; Set LSB 2 bits of duty cycle for ultimate goal of 50%
;  and turn on PWM
		movlw	B'00111100'
		movwf	CCP1CON

		  
; Turn on timer 2, prescale=1, postscale=1
		banksel	T2CON
		movlw	B'00000100'
		movwf	T2CON

; turn off LEDs
		banksel	PORTB
		movlw	B'00001110'
		movwf	PORTB



;---------------------------------------------------------------------
;	Check for WWVB receive pulse start
;---------------------------------------------------------------------
clear
		clrf		wwvb_min
		clrf		bitcount
		clrf		countsync

waitforstart
		btfss		PORTA,RX		; Check for negative transition
		goto		timerstart		
		movfw		timeslot
		xorwf		min_count,w
		btfsc		STATUS,Z
		goto		$+5
	  	movfw		timeslot2
		xorwf		min_count,w
		btfss		STATUS,Z
		goto		waitforstart
		btfss		tx_flag,0
		goto		$+3
		clrf		PCLATH
		call		transmit
		goto		waitforstart	; No: loop and wait for start

timerstart
		movlw 		D'177'			; Set timer0 for 20ms count 	
		movwf 		TMR0			; (256-177) * 256 = 20ms
		clrf		count_20ms		; Start pulse width measurement process

waitforend
		btfss		PORTA,RX		; Check for positive transition
		goto		$+2
		goto		process
		btfss 		INTCON,T0IF		; Did timer overflow?
		goto		waitforend		; No: loop and wait for start
		bcf 		INTCON,T0IF		; reset overflow flag
		movlw 		D'177'			; Set timer0 for 50ms count 	
		movwf 		TMR0			; (256-177) * 256 = 20ms
		incf 		count_20ms,F	; count_20ms intervals
		goto 		waitforend		; No, hang around some more

process
		movlw		B'00000100'
		xorwf		PORTB
		movlw		D'2'			; look for double sync pulse
		xorwf		countsync,w
		btfsc		STATUS,Z	
		goto		process2		; Yes: proceed with processing
		movlw		D'38'			; No: look for double sync pulse
		subwf		count_20ms,W	; pulse < 760ms?
		btfss		STATUS,C
		goto		clear
		movlw		D'42'
		subwf		count_20ms,W	; pulse > 840ms?
		btfsc		STATUS,C
		goto		clear
		incf		countsync
		goto		waitforstart	

process2
		bsf			STATUS,C
		movlw		D'8'
		subwf		count_20ms,W	; pulse < 160ms?
		btfss		STATUS,C
		goto		clear			; Yes: start over	
		bsf			STATUS,C
		movlw		D'12'		
		subwf		count_20ms,W	; pulse > 240ms?
		btfss		STATUS,C
		goto		pulse0			; Yes: pulse = 0 			
		movlw		D'22'
		subwf		count_20ms,W	; pulse < 440ms?
		btfss		STATUS,C
		goto		clear			; Yes: invalid pulse - start over
		movlw		D'27'
		subwf		count_20ms,W	; pulse > 540ms?
		btfss		STATUS,C
		goto		pulse1			; Yes: pulse = 1
		movlw		D'38'
		subwf		count_20ms,W	; pulse < 740ms?
		btfss		STATUS,C
		goto		clear			; Yes: invalid pulse - start over
		movlw		D'42'
		subwf		count_20ms,W	; pulse > 840ms?
		btfsc		STATUS,C
		goto		clear			; No: invalid pulse - start over
		goto		pulse_sync		; pulse = sync - go to pulse_sync				

pulse0
		incf		bitcount
		rlf			wwvb_min
		bcf			wwvb_min,0
		goto		waitforstart    ; Get next bit
		
pulse1
		incf		bitcount
		rlf			wwvb_min
		bsf			wwvb_min,0
		goto		waitforstart    ; Get next bit

pulse_sync
		movlw		D'8'
		xorwf		bitcount,w
		btfss		STATUS,Z
		goto		clear			; Invalid - start over
		movlw		B'00100000'
		andwf		wwvb_min,w
		movwf		temp
		movlw		B'00001111'
		andwf		wwvb_min,f
		bsf			STATUS,C
		movlw		D'32'
		subwf		temp
		btfss		STATUS,C
		goto		$+3
		movlw		D'10'
		goto		$+2
		movlw		D'0'
		addwf		wwvb_min,f
		movlw		D'20'
		xorwf		wwvb_min,w
		btfsc		STATUS,Z
		clrf		wwvb_min
		movfw		wwvb_min		;Update software timer - minutes		
		movwf		min_count
		movlw		D'102'			;Update software timer using correction
		movwf		sec_count_500ms	;factor for 10 sec WWVB time sync 
		bsf			tx_flag,0		;Valid time update - OK to enable TX
		bcf			PORTB,1			;Turn on LED3
		movlw		D'216'
		movwf		txflagtimer		;Set timer to 36 hours
		goto		clear	




loop
		goto		loop
		end

