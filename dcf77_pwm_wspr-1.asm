; dcf77_pwm_wspr
;
; A WSPR controller using pulse width modulation (PWM) to derive narrow-band 
; 4-FSK modulation from a voltage controlled crystal oscillator (VCXO).
; Minute and second data from a DCF77 receiver is used to update an internal 
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
; 20 December, 2010
;
;  
; 15 May, 2012 modified for the DCF77 time signal.
; Copyright (C) 2012,  Allard Munters PE1NWL
;
; DCF77 transmits at 77.5 kHz from Frankfurt, Germany.
; The DCF77 signal is strong and covers almost all Europe.
; This design decodes only the minutes, the hours and date code is
; discarded. It can therefor be used anywhere in Europe,
; regardless of the timezone.
; A low cost DCF receiver PCB with a ferrite rod antenna is available 
; for approx. 10 Euros at Conrad, order number 641138.
; Use the non-inverting output (pin 3) from the receiver with a 10K 
; pull-up resistor to +5V.
;
; 22 May, 2012 added tx-fraction capability.
; similar to the original WSPR software by Joe Taylor - K1JT
;
; 23 May, 2012 corrected bug for TX control signal at RB7.
; inverted logic: bit 7 must be cleared, not set, to turn on TX
; RB7 must be set during initialization to prevent the TX to transmit while
; the time is not yet valid
;
; 26 May, 2012 added optional TX LED at RB0.
; during transmissions the RX data LED will stop flashing and the TX LED will be on
;
; Allard Munters - PE1NWL     pe1nwl@gooddx.net
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
;     TX LED ---------RB0 |6       13| RB7---------TX                   
;     Valid time LED--RB1 |7       12| RB6---------    
;     RX data LED-----RB2 |8       11| RB5---------          
;     PWM output------RB3 |9       10| RB4---------         
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

RX			equ			D'04'		; PORTA pin3 RA4


;=====================================================================
;	File register use
;=====================================================================
		cblock		H'20'
			sec_count_500ms		; interrupt seconds count
			min_count			; interrupt minutes count
			w_temp
			status_temp
			count_20ms			; pulse width
			Ncount_20ms			; time between pulses
			minute_detect
			pulsecount
			dcf77_min
			tx_flag
			timeslot
			timeslot2
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
			FRparity
			randomval			; 8-bits random number
			txf_flag			; 1: tx-fraction, 0: fixed timeslot
			tx_fraction			; tx-fraction percentage
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

		decfsz		sec_count_500ms	; count 500 mS intervals
		goto		main3

		clrc						; generate a new random number every minute
		rlf     	randomval,1
		swapf   	randomval,0
		andlw   	0xE0
		rrf     	randomval,1
		addwf   	randomval,0
		addwf   	randomval,0
		addwf   	randomval,0
		sublw   	0x35
		movwf   	randomval		; 8-bit random number

		bcf			tx_flag,1		; clear flag bit 1
		movfw		tx_fraction		; duty cycle
		subwf		randomval,0
		btfss		STATUS,C
		bsf			tx_flag,1		; set flag bit 1 only when txfraction>randomval

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
		bcf			tx_flag,0		; Inhibit TX until valid time update
		bsf			PORTB,1			; Turn off "valid time" LED

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
; PE1NWL JO22 20         (Generated from 'GENWSPR.EXE'   G4JNT June 2009)
;
;---------------------------------------------------------------------

table
		
		addwf		PCL,F

  dt 0xF2, 0x88, 0xE2, 0x54, 0x2E, 0x19, 0x54, 0x82

  dt 0x8E, 0x91, 0x88, 0x86, 0x7A, 0x79, 0x29, 0x6C

  dt 0x2B, 0x4E, 0xC6, 0x43, 0x0C, 0x50, 0xB4, 0x4C

  dt 0x04, 0xA0, 0x43, 0x27, 0x45, 0xAF, 0xB0, 0xBD

  dt 0x80, 0x99, 0xA7, 0x08, 0xA9, 0xE4, 0x70, 0x94

  dt 0x80  ;  End of WSPR Data

;---------------------------------------------------------------------
; Subroutine to check if it is time to transmit
;---------------------------------------------------------------------

check_ttt
		movlw		D'120'
		xorwf		sec_count_500ms,w	; check if it is the START of a minute		
		btfss		STATUS,Z
		return						; No: don't transmit now

		movfw		min_count		; Yes: check if it is an EVEN minute		
		andlw		B'00000001'
		btfss		STATUS,Z		
		return						; No: don't transmit now
			
		btfss		tx_flag,0		; Yes: check if TX is enabled
		return						; No: don't transmit now

		movlw		D'01'			; Yes
		xorwf		txf_flag,w		; fixed timeslot or random tx-fraction?		
		btfss		STATUS,Z
		goto		$+4	

		btfss		tx_flag,1		; Random: check if TX-fraction flag is set
		return						; No: don't transmit now
		goto		$+9				; Yes: start 2 mins WSPR transmission

		movfw		timeslot		; Fixed: check if it is time to transmit
		xorwf		min_count,w
		btfsc		STATUS,Z		; is current minute in first timeslot?
		goto		$+5				; Yes: start 2 mins WSPR transmission
	  	movfw		timeslot2
		xorwf		min_count,w		
		btfss		STATUS,Z		; No: is current minute in second timeslot?	
		return						; No: don't transmit now	

		clrf		PCLATH
		call		transmit		; Yes: start 2 mins WSPR transmission
		return



;---------------------------------------------------------------------
; Subroutine to tramsmit symbol data
;---------------------------------------------------------------------

transmit							; Symbol send code starts here
		bcf			PORTB,0			; Turn on "TX" LED
		clrf		bytecount
		movlw		D'163'
		movwf		symbolcount
		bcf			PORTB,7			; Clear RB7 to turn on TX
		movlw		D'31'			; 2 sec delay before data start
		movwf		two_sec			; 
		movlw		D'04'			; Initiialize symbol group size
		movwf		position
		clrf		temp
		movlw		D'21'
		movwf		symboltimer
							
tdelay								; 
 		btfss 		INTCON,T0IF 	; Did timer overflow?
		goto 		tdelay 			; No, hang around some more
		bcf 		INTCON,T0IF 	; reset overflow flag
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
		movlw		D'158'
		movwf		CCPR1L
		goto		delay

zer0
		movlw		D'003'
		movwf		CCPR1L
		goto		delay
one
		movlw		D'054'
		movwf		CCPR1L
		goto		delay
two
		movlw		D'105'
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
		bsf			PORTB,7			; Set RB7 to turn off TX			
		bsf			PORTB,0			; Turn off "TX" LED
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
		bcf			STATUS,C
		rrf			temp 
		rrf			temp 
		rrf			temp 
		rrf			temp 
		rrf			temp 
		rrf			temp 			; First symbol is valid at this point
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
		bcf			STATUS,C
		rrf			temp 
		rrf			temp 
		rrf			temp 
		rrf			temp 			; Second symbol is valid at this point
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
		bcf			STATUS,C
		rrf			temp 
		rrf			temp 			; Third symbol is valid at this point
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
;	Select transmit time slot or tx-fraction and disable tranmitter until
;	valid DCF77 time data is loaded into software clock
;---------------------------------------------------------------------
		movlw		D'08'			; choose an EVEN minute
		movwf		timeslot
		movlw		D'18'			; choose an EVEN minute
		movwf		timeslot2

;		movlw		D'00'			; set to 00 for fixed timeslot mode
		movlw		D'01'			; set to 01 for tx-fraction mode
		movwf		txf_flag

; If tx-fraction mode is used (txf_flag=01), uncomment one of the following
; lines to select the desired tx-fraction percentage:

;		movlw		D'00'			; tx_fraction 0%   (never transmit)
;		movlw		D'13'			; tx_fraction 5%   (avg 1.5  transmissions/hour)
;		movlw		D'25'			; tx_fraction 10%  (avg 3    transmissions/hour)
;		movlw		D'38'			; tx_fraction 15%  (avg 4.5  transmissions/hour)
		movlw		D'51'			; tx_fraction 20%  (avg 6    transmissions/hour)(recommended)
;		movlw		D'64'			; tx_fraction 25%  (avg 7.5  transmissions/hour)
;		movlw		D'76'			; tx_fraction 30%  (avg 9    transmissions/hour)
;		movlw		D'89'			; tx_fraction 35%  (avg 10.5 transmissions/hour)
;		movlw		D'102'			; tx_fraction 40%  (avg 12   transmissions/hour)
;		movlw		D'115'			; tx_fraction 45%  (avg 13.5 transmissions/hour)
;		movlw		D'127'			; tx_fraction 50%  (avg 15   transmissions/hour)
;		movlw		D'140'			; tx_fraction 55%  (avg 16.5 transmissions/hour)
;		movlw		D'153'			; tx_fraction 60%  (avg 18   transmissions/hour)
;		movlw		D'166'			; tx_fraction 65%  (avg 19.5 transmissions/hour)
;		movlw		D'178'			; tx_fraction 70%  (avg 21   transmissions/hour)
;		movlw		D'191'			; tx_fraction 75%  (avg 22.5 transmissions/hour)
;		movlw		D'204'			; tx_fraction 80%  (avg 24   transmissions/hour)
;		movlw		D'217'			; tx_fraction 85%  (avg 25.5 transmissions/hour)
;		movlw		D'229'			; tx_fraction 90%  (avg 27   transmissions/hour)
;		movlw		D'242'			; tx_fraction 95%  (avg 28.5 transmissions/hour)
;		movlw		D'255'			; tx_fraction 100% (always: 30 transmissions/hour)

		movwf		tx_fraction
		clrf		tx_flag			; disable transmitter

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
; and turn on PWM
		movlw	B'00111100'
		movwf	CCP1CON

		  
; Turn on timer 2, prescale=1, postscale=1
		banksel	T2CON
		movlw	B'00000100'
		movwf	T2CON

; turn off LEDs and turn off TX
		banksel	PORTB
		movlw	B'10001111'
		movwf	PORTB

;---------------------------------------------------------------------
;	Initialize random number generator
;---------------------------------------------------------------------

		movlw	D'100'				; seed random generator with whatever number
		movwf	randomval

;---------------------------------------------------------------------
;	Check for DCF77 receive pulse start
;---------------------------------------------------------------------

startover
		bcf			minute_detect,0
		clrf		Ncount_20ms		; Start time measurement between pulses
		clrf		dcf77_min
		clrf		bitcount

waitforstart
		btfsc		PORTA,RX		; Check for negative transition
		goto		timerstart		

		btfss 		INTCON,T0IF		; Did timer overflow?
		goto		$+5     		; No
		bcf 		INTCON,T0IF		; reset overflow flag
		movlw 		D'177'			; Set timer0 for 20ms count 	
		movwf 		TMR0			; (256-177) * 256 = 20ms
		incf 		Ncount_20ms,F	; count_20ms intervals between pulses

		call		check_ttt		; check if it is time to transmit
		goto		waitforstart


timerstart
		bsf			PORTB,2			; Turn off "RX data" LED
		movlw 		D'177'			; Set timer0 for 20ms count 	
		movwf 		TMR0			; (256-177) * 256 = 20ms
		clrf		count_20ms		; Start pulse width measurement process

waitforend
		btfss		PORTA,RX		; Check for positive transition
		goto		process

		call		check_ttt		; check if it is time to transmit

		btfss 		INTCON,T0IF		; Did timer overflow?
		goto		waitforend		; No: loop and wait for start
		bcf 		INTCON,T0IF		; reset overflow flag
		movlw 		D'177'			; Set timer0 for 20ms count 	
		movwf 		TMR0			; (256-177) * 256 = 20ms
		incf 		count_20ms,F	; count_20ms intervals
		goto 		waitforend		; No, hang around some more

process
		bcf			PORTB,2			; Turn on "RX data" LED
		movlw		D'47'			; look for missing pulse (=start of new minute)
		subwf		Ncount_20ms,W	; pulse < 920ms?
		btfss		STATUS,C
		goto		$+8
		movlw		D'96'
		subwf		Ncount_20ms,W	; pulse > 1920ms?
		btfsc		STATUS,C
		goto		$+4
		bsf			minute_detect,0	; start of new minute detected
		clrf		bitcount		; reset bit counter

		movlw 		D'177'			; Set timer0 for 20ms count 	
		movwf 		TMR0			; (256-177) * 256 = 20ms
		clrf		Ncount_20ms		; Start between pulses width measurement process

		btfss		minute_detect,0	; has start of new minute been detected?
		goto		waitforstart	; No: loop and wait for start

		movlw		D'3'			; Yes: check pulse width
		subwf		count_20ms,W	; pulse < 60ms?
		btfss		STATUS,C
		goto		startover       ; Yes: invalid pulse - start over
		movlw		D'13'		
		subwf		count_20ms,W	; pulse > 260ms?
		btfsc		STATUS,C
		goto		startover       ; Yes: invalid pulse - start over

		movlw		D'8'			; pulse is valid, is it "0" or "1" ?
		subwf		count_20ms,W	; pulse < 160ms?
		btfss		STATUS,C
		goto		pulse0			; Yes: pulse = 0
		goto		pulse1			; No: pulse = 1	

pulse0
		incf		bitcount
		movlw		D'22'			; discard bits 0-20
		subwf		bitcount,W
		btfss		STATUS,C
		goto		waitforstart    ; Get next bit
		movlw		D'30'			; collect bits 21-28
		subwf		bitcount,W
		btfsc		STATUS,C
		goto		par_check  	    ; all 8 minute bits complete, do a parity check
		rrf			dcf77_min		; reverse bit order
		bcf			dcf77_min,7		; insert a "0"
		goto		waitforstart    ; Get next bit
		
pulse1
		incf		bitcount
		movlw		D'22'			; discard bits 0-20
		subwf		bitcount,W
		btfss		STATUS,C
		goto		waitforstart    ; Get next bit
		movlw		D'30'			; collect bits 21-28
		subwf		bitcount,W
		btfsc		STATUS,C
		goto		par_check  	    ; all 8 minute bits complete, do a parity check
		rrf			dcf77_min		; reverse bit order
		bsf			dcf77_min,7		; insert a "1"
		goto		waitforstart    ; Get next bit

par_check
		bcf			minute_detect,0	; clear flag

		movfw		dcf77_min		; check for even parity
		movwf	  	FRparity
        swapf	  	FRparity,W
        xorwf 	 	FRparity,W
        movwf  		FRparity
        rrf    		FRparity,F
        rrf    		FRparity,F
        xorwf  		FRparity,W
        movwf  		FRparity
        rrf    		FRparity,F
        xorwf  		FRparity,F
        btfsc  		FRparity,0      ; Test parity bit 0
        goto   		startover       ; parity error - start over			
				
		movlw		B'00010000'		; parity OK
		andwf		dcf77_min,w		; convert BCD to HEX
		movwf		temp			; and to a 0-19 minute cycle
		movlw		B'00001111'
		andwf		dcf77_min,f
		movlw		B'00010000'
		xorwf		temp,w
		btfss		STATUS,Z				
		goto		$+3
		movlw		D'10'
		addwf		dcf77_min,f

		movfw		dcf77_min		; decoded time is for FOLLOWING minute
		btfsc		STATUS,Z		; so we must substract one minute
		goto		$+3
		decf		dcf77_min,0		; decrement minutes (if >0), and store in W	
		goto		$+2
		movlw		D'19'			; if minutes=0 then set W to 19

		movwf		min_count		; Update software timer - minutes
		movlw		D'63'			; Update software timer using correction
		movwf		sec_count_500ms	; factor for 29 sec DCF time sync 
		bsf			tx_flag,0		; Valid time update - OK to enable TX
		bcf			PORTB,1			; Turn on "valid time" LED
		movlw		D'216'
		movwf		txflagtimer		; Set timer to 36 hours
		goto		waitforstart	

loop
		goto		loop
		end