;
; DDS_WSPR - Uses AD9851 to tranmit WSPR code on 20% transmit cycle
;		   - Multi-band 160 thru 6 meters
;          - Transmit band scanning
; 				
; 10 July, 2009   - Modified to 6 band scan for auto tune box
;                 - Band frequencies re-calibrated
; 10 August, 2009 - Modified to use NMEA data stream for timing
; 13 August, 2009 - Streamlined timing to correct scanning bug
; 
;  Gene Marcus W3PM GM4YRE
;  10 August, 2009
;
;  Clock Frequency: 4 MHz
;
; 
;
; 					       PIC16F628A                             
;                          __________                                          
;     LED1------------RA2 |1       18| RA1---------Scan LED                   
;     Band BCD0-------RA3 |2       17| RA0---------Transmit                   
;     PB_1-Bandswitch-RA4 |3       16| OSC1--------XTAL                        
;     +5V-----------!MCLR |4       15| OSC2--------XTAL                        
;     Ground----------Vss |5       14| VDD---------+5 V                        
;     DDS_CLK---------RB0 |6       13| RB7---------DDS_LOAD                    
;     NMEA data-------RB1 |7       12| RB6---------Band BCD3    
;     Not used    ----RB2 |8       11| RB5---------Band BCD2           
;     DDS_DATA--------RB3 |9       10| RB4---------Band BCD1         
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

DDS_clk 	equ     0x00        ; AD9850/AD9851 write clock
DDS_dat 	equ     0x03        ; AD9850/AD9851 serial data input
DDS_load	equ     0x07        ; Update pin on AD9850/AD9851                      
pb_1        equ     0x04        ; Change band/band scan
TX          equ		0x00		; Transmit on/off
BCD_0		equ		0x03		; BCD outputs used to control (RA3)
BCD_1		equ		0x04		; CD4028 BCD to decimal switch
BCD_2		equ		0x05		; or directly for amplifier/filter
BCD_3		equ		0x06		; selection and/or LED band indicator
scan_LED	equ		0x01		; Used to indicate scan enabled
zero		equ		D'02'
carry		equ		D'00'
RB1 		equ 	D'01' 		; pin 7 RB1 on PORTB
LED1 		equ 	D'02' 		; LED 1 on PORTA (RA2)

;=====================================================================
;	File register use
;=====================================================================
		cblock		H'20'

			basefreq_0 			; Base frequency MSB
			basefreq_1 			; 
			basefreq_2 			; 
			basefreq_3 			; Base frequency LSB 
			offset				; Offset to add to base frequency
			offset_0			; WSPR offset for symbol o
			offset_1			; WSPR offset for symbol 1			
			offset_2			; WSPR offset for symbol 2
			offset_3			; WSPR offset for symbol 3
			DDSword_0			; (base freq + offset) MSB
			DDSword_1			;
			DDSword_2			;
			DDSword_3			; (base freq + offset) LSB
			DDSword_4			; x6 multiply for AD9851
			temp				
			temp1
			temp2
			temp3
    	   	bit_count           
        	byte2send           
			timer1				
			timer2				
			Inputs
			counter
			w_temp
			status_temp
			sec_count
			min_count
			symbolcount
			symboltimer
			pad
			two_sec
			band
			led_stat
			scan_flag
			char
			rxstate
			evenmin
			tenhour
			onehour
			tenmin
			onemin
			tensec
			onesec
		endc


		goto		start



;=====================================================================
;  Subroutines
;=====================================================================
;
;	
;_________________________________________________________________________________
;
;  STATION CALLSIGN, POWER LEVEL, AND GRIDSQURE IS CONTAINED IN THE FOLLOWING TABLE
;
;  Symbol table contains the station's callsign, powerlevel, and gridsquare
;  
;  The follwing data is for W3PM, 10 dBm, EM64
;  
;  Note: End of table flag is D'4'
;                                 
;_________________________________________________________________________________
;

;				org			H'A0'
Table								; WSPR symbol data starts here
				addwf 	PCL,F 
                retlw         D'3'
                retlw         D'1'
                retlw         D'0'
                retlw         D'2'
                retlw         D'2'
                retlw         D'0'
                retlw         D'0'
                retlw         D'2'
                retlw         D'1'
                retlw         D'2'
                retlw         D'0'
                retlw         D'2'
                retlw         D'3'
                retlw         D'1'
                retlw         D'1'
                retlw         D'0'
                retlw         D'2'
                retlw         D'2'
                retlw         D'3'
                retlw         D'2'
                retlw         D'0'
                retlw         D'1'
                retlw         D'0'
                retlw         D'1'
                retlw         D'3'
                retlw         D'1'
                retlw         D'1'
                retlw         D'2'
                retlw         D'0'
                retlw         D'0'
                retlw         D'0'
                retlw         D'2'
                retlw         D'0'
                retlw         D'2'
                retlw         D'3'
                retlw         D'2'
                retlw         D'0'
                retlw         D'3'
                retlw         D'2'
                retlw         D'3'
                retlw         D'2'
                retlw         D'2'
                retlw         D'2'
                retlw         D'2'
                retlw         D'2'
                retlw         D'2'
                retlw         D'3'
                retlw         D'2'
                retlw         D'3'
                retlw         D'1'
                retlw         D'2'
                retlw         D'0'
                retlw         D'3'
                retlw         D'1'
                retlw         D'0'
                retlw         D'1'
                retlw         D'2'
                retlw         D'0'
                retlw         D'0'
                retlw         D'1'
                retlw         D'1'
                retlw         D'2'
                retlw         D'1'
                retlw         D'0'
                retlw         D'2'
                retlw         D'2'
                retlw         D'2'
                retlw         D'3'
                retlw         D'1'
                retlw         D'0'
                retlw         D'1'
                retlw         D'0'
                retlw         D'1'
                retlw         D'0'
                retlw         D'1'
                retlw         D'0'
                retlw         D'1'
                retlw         D'2'
                retlw         D'2'
                retlw         D'3'
                retlw         D'0'
                retlw         D'0'
                retlw         D'3'
                retlw         D'0'
                retlw         D'1'
                retlw         D'3'
                retlw         D'2'
                retlw         D'0'
                retlw         D'0'
                retlw         D'1'
                retlw         D'3'
                retlw         D'0'
                retlw         D'3'
                retlw         D'2'
                retlw         D'1'
                retlw         D'0'
                retlw         D'2'
                retlw         D'2'
                retlw         D'1'
                retlw         D'2'
                retlw         D'2'
                retlw         D'2'
                retlw         D'2'
                retlw         D'0'
                retlw         D'1'
                retlw         D'2'
                retlw         D'0'
                retlw         D'1'
                retlw         D'2'
                retlw         D'2'
                retlw         D'1'
                retlw         D'3'
                retlw         D'3'
                retlw         D'2'
                retlw         D'1'
                retlw         D'3'
                retlw         D'0'
                retlw         D'0'
                retlw         D'1'
                retlw         D'3'
                retlw         D'2'
                retlw         D'3'
                retlw         D'2'
                retlw         D'0'
                retlw         D'2'
                retlw         D'3'
                retlw         D'3'
                retlw         D'1'
                retlw         D'2'
                retlw         D'0'
                retlw         D'0'
                retlw         D'0'
                retlw         D'0'
                retlw         D'1'
                retlw         D'2'
                retlw         D'3'
                retlw         D'0'
                retlw         D'0'
                retlw         D'1'
                retlw         D'3'
                retlw         D'0'
                retlw         D'2'
                retlw         D'0'
                retlw         D'0'
                retlw         D'2'
                retlw         D'2'
                retlw         D'2'
                retlw         D'3'
                retlw         D'3'
                retlw         D'2'
                retlw         D'3'
                retlw         D'0'
                retlw         D'1'
                retlw         D'1'
                retlw         D'2'
                retlw         D'2'
                retlw         D'0'
                retlw         D'3'
                retlw         D'3'
                retlw         D'0'
                retlw         D'2'
                retlw         D'2'
				retlw         D'4' ; end_of_table flag

;
; ****************************************************************************  
; * band table.  												             *
; *																			 *
; * Example: Fout   = 14.0971 MHz											 *
; *          Fclock = 180 MHz												 *
; *          basefreq = (14.0971*10^6) * (2^32) / (180*10^6) = 336369908	 *
; *	   		 336369908 = 14 0c 98 f4 hex									 *
; *                                                                          *
; * Each entry is four instructions long, with each group of four literals   *
; * representing the frequency as a 32 bit integer. 						 *
; *                                                                          *
; ****************************************************************************
;
band_table
        addwf   PCL,f;_________________ 
        retlw   0x28 ; 10 meters MSB 0		
        retlw   0x00 ; 							
        retlw   0x83 ; band 6						
        retlw   0x52 ;_________________			
        retlw   0x23 ; 12 meters MSB 4					
        retlw   0x73 ; 							
        retlw   0x6a ; band 5						
        retlw   0x92 ;_________________		
        retlw   0x19 ; 17 meters MSB 8				
        retlw   0xc0 ; 					
        retlw   0x4d ; band 4				
        retlw   0x78 ;_________________			
        retlw   0x14 ; 20 meters MSB 12	
        retlw   0x0c ; 				
        retlw   0xa7 ; band 3			
        retlw   0x61 ;_________________ 			
        retlw   0x0e ; 30 meters MSB 16					
        retlw   0x6b ; 							
        retlw   0xf9 ; band 2						
        retlw   0x76 ;_________________  			
        retlw   0x0a ; 40 meters MSB 20					
        retlw   0x03 ; 							
        retlw   0x3f ; band 1                      
        retlw   0x93 ;_________________



;
; *****************************************************************************
; * main                                                                      *
; *                                                                           *
; * Purpose:  This routine retrives WSPR symbols, determines symbol frequency *
; *			  offset and symbol transmit delay timing.                        *
; *                                                                           *
; *   Input:  Symbol data from symbol table                                   *
; *                                                                           *
; *  Output:  Subroutine calls to calc_DDSword and send_DDS-word   		      *
; *                                                                           *
; *****************************************************************************
;
main								; Symbol send code starts here
		movlw		D'0'
		movwf		symbolcount
		bsf			PORTA,TX		; Turn on transmitter
		call    	get_band 
		movlw		D'30'			; 2 sec delay before data start
		movwf		two_sec			; 
							
tdelay								; 
 		btfss 		INTCON,T0IF 	; Did timer overflow?
		goto 		tdelay 			; No, hang around some more
		bcf 		INTCON,T0IF 	; reset overflow flag
		decfsz 		two_sec,F 		; Count down
		goto 		tdelay			; Not time yet	

symbol_loop
		movlw		D'21'
		movwf		symboltimer
		movfw		symbolcount
		call		Table
		movwf		temp
		incf		symbolcount
		sublw		D'4'			; Test for end_of_table flag
        bz		   	stop     	    ; Yes, stop
		movfw		temp			; No, reload w from temp and continue
		sublw		D'0'			; Test for symbol 0 
		bz			zero1			; Yes, goto zero
		movfw		temp			; No, reload w from temp and continue
		sublw		D'1'			; Test for symbol 1
		bz			one				; Yes, goto one
		movfw		temp			; No, reload w from temp and continue
		sublw		D'2'			; Test for symbol 2
		bz			two				; Yes, goto two
									; No, it must be symbol 3
		movfw		offset_3		; Load the offset for symbol 3
		movwf		offset			; into offset
		call		calc_DDSword	; Calculate the word to send to the DDS
		call		send_dds_word	; Transmit symbol
		goto		delay			; Wait for 680 mSec

zero1
		movfw		offset_0		; Load the offset for symbol 0
		movwf		offset			; into offset
		call		calc_DDSword	; Calculate the word to send to the DDS
		call		send_dds_word	; Transmit symbol
		goto		delay			; Wait for 680 mSec
one
		movfw		offset_1		; Load the offset for symbol 1
		movwf		offset			; into offset
		call		calc_DDSword	; Calculate the word to send to the DDS
		call		send_dds_word	; Transmit symbol
		goto		delay			; Wait for 680 mSec
two
		movfw		offset_2		; Load the offset for symbol 2
		movwf		offset			; into offset
		call		calc_DDSword	; Calculate the word to send to the DDS
		call		send_dds_word	; Transmit symbol

delay	
		btfss 		INTCON,T0IF 	; Did timer overflow?
		goto 		delay 			; No, hang around some more
		movlw		d'39'			;
		movwf		pad				; Calibrate symbol timer
timepad	decfsz		pad,F			; to 680 mSec
		goto		timepad			;
		bcf 		INTCON,T0IF 	; Reset overflow flag
		movlw 		D'130' 			; Timer will count 	
		movwf 		TMR0			; 127 (256-129) counts
		decfsz 		symboltimer,F 	; Count down
		goto 		delay			; Not time yet											
		goto		symbol_loop
stop
		bcf			PORTA,TX		; Turn off tranmitter

		return


;
; *****************************************************************************
; * calc_DDSword                                                              *
; *                                                                           *
; * Purpose:  This routine calculates the DDSword control word to be sent     *
; *			  to the DDS.              					                      *
; *                                                                           *
; *   Input:  basefreq_3 ... basefreq_0, offset                               *
; *                                                                           *
; *  Output:  DDSword_3 ... DDSword_0   		                              *
; *                                                                           *
; *****************************************************************************
;

calc_DDSword
		movf 		basefreq_0,w 	; LSD freq byte operand
		addwf 		offset,w 		; Add offset to LSDfreq byte operand
		movwf 		DDSword_0 		; Store result
		movf 		basefreq_1,w 	; Pick up next operand
		movwf		temp3			; Temporarily store
		btfss		STATUS,C 		; Was there a carry?
		goto		a01				; No, skip to a01
		movlw 		0x01 			; Yes, add in carry
		addwf		temp3,w 		; Add to temp

a01		movwf 		DDSword_1 		; Store result
		movf 		basefreq_2,w 	; Pick up next operand
		movwf		temp3			; Temporarily store
		btfss		STATUS,C 		; Was there a carry?
		goto		a02				; No, skip to a02
		movlw 		0x01 			; Yes, add in carry
 		addwf		temp3,W 		; Add to temp

a02		movwf 		DDSword_2 		; Store result
		movf 		basefreq_3,w 	; Pick MSB freq byte operand
		movwf		temp3			; Temporarily store
		btfss		STATUS,C 		; Was there a carry?
		goto		a03				; No, skip to a03
		movlw 		0x01 			; Yes, add in carry
		addwf		temp3,w 		; Add to temp

a03		movwf 		DDSword_3 		; Store result 
		
		return

;
; *****************************************************************************
; * send_dds-word                                                             *
; *                                                                           *
; * Purpose:  This routine sends the DDSword control word to the DDS          *
; *           using a serial data transfer.                                   *
; *                                                                           *
; *   Input:  DDSword_4 ... DDSword_0                                         *
; *                                                                           *
; *  Output:  The DDS chip register is updated.                               *
; *                                                                           *
; *****************************************************************************
;

send_dds_word
        movlw   DDSword_0         ; Point FSR at Least Significant Byte       
        movwf   FSR               ; 
next_byte
        movf    INDF,w            ; 
        movwf   byte2send         ; 
        movlw   0x08              ; Set counter to 8
        movwf   bit_count         ; 
next_bit
        rrf     byte2send,f       ; Test if next bit is 1 or 0
        btfss   STATUS,C          ; Was it zero?
        goto    send0             ; Yes, send zero
        bsf     PORTB,DDS_dat     ; No, send one                               
        bsf     PORTB,DDS_clk     ; Toggle write clock                         
        bcf     PORTB,DDS_clk     ;                                            
        goto    break             ; 
send0
        bcf     PORTB,DDS_dat     ; Send zero                                  
        bsf     PORTB,DDS_clk     ; Toggle write clock                         
        bcf     PORTB,DDS_clk     ;                                            
break
        decfsz  bit_count,f       ; Has the whole byte been sent?
        goto    next_bit          ; No, keep going.
        incf    FSR,f             ; Start the next byte unless finished
        movlw   DDSword_4+1       ; Next byte (past the end)
        subwf   FSR,w             ; 
        btfss   STATUS,C          ;
        goto    next_byte         ;
        bsf     PORTB,DDS_load    ; Send load signal to the AD9850/DDSword             
        bcf     PORTB,DDS_load    ;                                            
        return  
                  ;
 
;
; *****************************************************************************
; * get_band                                                                  *
; *                                                                           *
; * Purpose:  This routine reads the frequency value of a band table entry    *
; *           pointed to by band and returns it in freq_3...freq_0.           *
; *                                                                           *
; *   Input:  band must contain the index of the desired band entry * 4       *
; *           (with the entries numbered from zero).                          *
; *                                                                           *
; *  Output:  The band frequency in freq.                                     *
; *                                                                           *
; *****************************************************************************
;

get_band
        movf    band,w            ; Get the index of the high byte 
        call    band_table        ; Get the value into W
        movwf   basefreq_3        ; Save it in basefreq_3
        incf    band,f            ; Increment index to next byte
        movf    band,w            ; Get the index of the next byte
        call    band_table        ; Get the value into W
        movwf   basefreq_2        ; Save it in basefreq_2
        incf    band,f            ; Increment index to the next byte
        movf    band,w            ; Get the index to the next byte
        call    band_table        ; Get the value into W
        movwf   basefreq_1        ; Save it in basefreq_1
        incf    band,f            ; Increment index to the low byte
        movf    band,w            ; Get the index to the low byte
        call    band_table        ; Get the value into W
        movwf   basefreq_0        ; Save it in basefreq_0
        movlw   0x03              ; Get a constant three
        subwf   band,f            ; Restore original value of band
        return                    ; Return to the caller

;
; *****************************************************************************
; * LED_status                                                                *
; *                                                                           *
; * Purpose:  Reads the variable led_stat and outputs BCD word to ports.      *
; *           BCD ports used to indicate band number selected.	              *
; *                                                                           *
; *   Input:  led_stat													      *
; *           										                          *
; *                                                                           *
; *  Output:  Ports RB1,RB4,RB5, and RB6.                                     *
; *                                                                           *
; *****************************************************************************
;

LED_status
		btfss	led_stat,0
		goto	$+3
		bsf		PORTA,BCD_0
		goto	$+2
		bcf		PORTA,BCD_0
		btfss	led_stat,1
		goto	$+3
		bsf		PORTB,BCD_1
		goto	$+2
		bcf		PORTB,BCD_1
		btfss	led_stat,2
		goto	$+3
		bsf		PORTB,BCD_2
		goto	$+2
		bcf		PORTB,BCD_2
		btfss	led_stat,3
		goto	$+3
		bsf		PORTB,BCD_3
		goto	$+2
		bcf		PORTB,BCD_3
		return

;
; *****************************************************************************
; * scan                                                                      *
; *                                                                           *
; * Purpose:  Increment start time, led_stat, and band                        *
; *                                                                           *
; *   Input:  evenmin, led_stat, band                                         *
; *           										                          *
; *  Output:  evenmin, led_stat, band                                         *
; *                                                                           *
; *****************************************************************************
;
scan
		incf		led_stat		; Increment band LED
		bcf			STATUS,carry
		movlw   	0x04      		; get 4 bytes to subtract
        subwf   	band,f  		; Move down to MSB in band list
		btfss   	STATUS,carry	; Off the bottom?
        goto    	reset_var       ; Yes, reset variables
		call    	LED_status		; Set up BCD outputs
		bcf			STATUS,carry
		movlw		D'02'
		addwf		evenmin
		movlw		D'58'
		subwf		evenmin,w
		btfss		STATUS,carry
		return
		movlw		D'48'
		movwf		evenmin	
		return
reset_var       
		movlw		D'24'			; Set counter to end of band table
		movwf		band
		movlw		D'0'			; Initialize LED (BCD) status
		movwf		led_stat               
        goto    	scan  

;
; *****************************************************************************
; * bandchange                                                                *
; *                                                                           *
; * Purpose:  Increment led_stat and band                                     *
; *                                                                           *
; *   Input:  led_stat, band                                                  *
; *           										                          *
; *  Output:  led_stat, band                                                  *
; *                                                                           *
; *****************************************************************************
;
bandchange
		bcf			STATUS,carry
		bcf			STATUS,zero 
bc2		movlw   	0x04      		; get 4 bytes to subtract
        subwf   	band,f  		; Move down to MSB in band list
		incf		led_stat		; Increment band LED
        btfss   	STATUS,carry	; Off the bottom?
        goto    	reset_var2      ; Yes, reset variables
		call    	LED_status		; Set up BCD outputs
		return
reset_var2       
		movlw		D'24'			; Set counter to end of band table
		movwf		band
		movlw		D'0'			; Initialize LED (BCD) status
		movwf		led_stat               
        goto    	bc2  


; End of subroutines
;______________________________________________________________________




start


;---------------------------------------------------------------------
;	Set up timer 
;---------------------------------------------------------------------
		errorlevel	-302
		banksel		INTCON
		bcf			INTCON,GIE		; Disable all interrups

		banksel		OPTION_REG
		bcf			OPTION_REG,T0CS	; Select timer
		bcf			OPTION_REG,PSA	; Prescaler to timer
		bsf			OPTION_REG,PS2	; \
		bsf			OPTION_REG,PS1	;  >- 1:256 prescale
		bsf			OPTION_REG,PS0	; /

;---------------------------------------------------------------------
;	Set up I/O 
;---------------------------------------------------------------------

		banksel	TRISA                        
        movlw   B'11010000'       ; Tristate PORTA (all Inputs except RA0,RA1,RA3)  
        movwf   TRISA             ;
 		movlw	B'00000110'
        movwf   TRISB             ; Set RB1 & RB2 as input for UART I/O (per SM6LKM) - all other bits as output
		banksel	PORTA
		clrf	PORTA
		clrf	PORTB

;---------------------------------------------------------------------
;	Initialize memory
;---------------------------------------------------------------------

		bsf			STATUS,RP0
		clrf		TXSTA			; Clears TXSTA bits SYNC (async) & BRGH (low speed baudrate) 
		movlw 		D'12'			; set UART for 4800 baud 
		movwf		SPBRG
		bcf			STATUS,RP0
		movlw		B'10010000'		; Set SPEN & CREN
		movwf		RCSTA
		clrf		rxstate

		bcf 		PORTA,LED1 		; Turn off LED1
		movlw		D'48'			; Defines even minute to transmit - 48=0, 50=2, 52=4, etc...
		movwf		evenmin
		bcf			STATUS,zero

;		Set default basefreq to 10.1042 MHz
        movlw		D'16'			; MSB for band (refer to band table) 
		movwf		band 
		movlw		D'2'			; band number from band table
		movwf		led_stat		; set up LED (BCD) status
		call  		LED_status

;		Set DDSword_4 to turn on AD9851 6x clock multiplier
        movlw   	0x01            ; Turn on 6x clock multiplier (AD9851)      
        movwf   	DDSword_4       ; Last byte to be sent                      
                                  	; Mult answer is in bytes _3 .. _0  
;_________________________________________________________________________________
;
;  offset - N*12000*2^32 / 8192*Fclock
;  Example: (for symbol 2) 
;           Fclock = 180 MHz
;           offset = 2*12000*2^32 / 8192*(180*10^6) = 69.905
;			69.905 ~ 70 = 46 Hex	  
;__________________________________________________________________________________

;		Load WSPR offsets 
		movlw		0x00			; 0.00 Hz
		movwf		offset_0
		movlw		0x23			; 1.46 Hz
		movwf		offset_1
		movlw		0x46			; 2.93 Hz
		movwf		offset_2
		movlw		0x69			; 4.39 Hz
		movwf		offset_3

		clrf		scan_flag
		clrf		bandchange

		btfsc	 	PORTA,pb_1 		; PB1 down?
		goto 		clear			; No - scan not selected - goto transmit timing loop

TestPB1up
		btfss 		PORTA,pb_1 		; PB1 up?
		goto 		TestPB1up 		; No, wait for release
		bsf			scan_flag,0
		bsf			PORTA,scan_LED	; Turn on scan LED

;____________________________________________________________________________________
;
; This is the entry point for for transmit timing
;____________________________________________________________________________________
;

clear
		clrf		rxstate		; Start over
	
receive
		bcf			STATUS,carry
		btfsc		PIR1,RCIF	; Check bit 5 (RCIF) for any character received
		goto		getchar
		bcf			STATUS,carry
		goto		receive

getchar	
		movlw		D'06'		; Check for UART overrun or framing error
		andwf		RCSTA,W
		btfsc		STATUS,zero	; Check STATUS register bit 2 -	1=0
		goto		getchar2
		movf		RCREG,W		; W = RCREG
		bcf			RCSTA,CREN	; CREN = 0
		bsf			RCSTA,CREN	; CREN = 1
		goto		receive


getchar2
		movf		RCREG,W		; W = RCREG - this clears RCIF
		movwf		char		; Received character from buffer
		xorlw		D'36'
		btfss		STATUS,zero	; Does start character = "$"?
		goto		getchar3
		movlw		D'01'
		movwf		rxstate		; rxstate = 1
		bcf			STATUS,carry
		goto		receive		; Get next character

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
		addwf		PCL,F		; Jump to rxstate number
		goto		receive		; idle
		goto		st1			; get_Gp
		goto		st2			; get_gP
		goto		st3			; get_gpGga_or_gpRmc
		goto		st4			; get_gpgGa
		goto		st5			; get_gpgga
		goto		st6			; get_gprMc
		goto		st7			; get_gprmC
		goto		st8			; get_comma
		goto		st9			; get_10h
		goto		st10		; get_1h
		goto		st11		; get_10m
		goto		st12		; get_1m
		goto		st13		; get_10s
		goto		st14		; get_1s

st1								; get_Gp
		movf		char,w
		xorlw		D'71'		; Is receive charactor G?
		btfss		STATUS,zero	; If so, increment rxstate
		goto		clear		; If not, clear rxstate and wait for another '$'
		incf		rxstate
		goto		receive		; Get next character

st2								; get_gP
		movf		char,w
		xorlw		D'80'		; Is receive charactor P?
		btfss		STATUS,zero	; If so, increment rxstate
		goto		clear		; If not, clear rxstate and wait for another '$'
		incf		rxstate,1
		goto		receive		; Get next character

st3								; get_gpGga_or_gpRmc
		movf		char,w
		xorlw		D'71'		; Is receive charactor G?
		btfss		STATUS,zero	; If so, increment rxstate
		goto		st3b		; 
		incf		rxstate		; We have "GPG"
		goto		receive		; Get next character

st3b	movf		char,w
		xorlw		D'82'		; Is receive character R?
		btfss		STATUS,zero
		goto		clear		; Start over
		movlw		D'06'
		movwf		rxstate		; We have "GPR" rxstate=6
		goto		receive		; Get next character

st4								; get_gpgGa
		movf		char,w
		xorlw		D'71'		; Is receive charactor G?
		btfss		STATUS,zero	; If so, increment rxstate
		goto		clear		; If not, clear rxstate and wait for another '$'
		incf		rxstate
		goto		receive		; Get next character

st5								; get_gpggA
		movf		char,w
		xorlw		D'65'		; Is receive charactor A?
		btfss		STATUS,zero	; If so, increment rxstate
		goto		clear		; If not, clear rxstate and wait for another '$'
		movlw		D'08'
		movwf		rxstate
		goto		receive		; We have "GPGGA" - rxstate=8 - get next character

st6								; get_gprMc
		movf		char,w
		xorlw		D'77'		; Is receive charactor M?
		btfss		STATUS,zero	; If so, increment rxstate
		goto		clear		; If not, clear rxstate and wait for another '$'
		incf		rxstate
		goto		receive		; Get next character

st7								; get_gprmC
		movf		char,w
		xorlw		D'67'		; Is receive charactor C?
		btfss		STATUS,zero	; If so, increment rxstate
		goto		clear		; If not, clear rxstate and wait for another '$'
		incf		rxstate
		goto		receive		; Get next charactor

st8								; get_comma
		movf		char,w
		xorlw		D'44'		; Is receive charactor a comma?
		btfss		STATUS,zero	; If so, increment rxstate
		goto		clear		; If not, clear rxstate and wait for another '$'
		incf		rxstate
		goto		receive		; Get next character

st9								; get_10h
		movf		char,w
		movwf		tenhour
		movlw		D'51'
		subwf		char,w		; Is character <=2?
		btfsc		STATUS,carry	
		goto		clear		; Start over if not valid
		incf		rxstate		; Increment and go to next character
		goto		receive

st10							; get_1h
		movf		char,w
		movwf		onehour
		movlw		D'58'
		subwf		char,w		; Is character <=9?
		btfsc		STATUS,carry	
		goto		clear		; Start over if not valid
		incf		rxstate		; Increment and go to next character
		goto		receive

st11								; get_10m
		movf		char,w
		movwf		tenmin
		movlw		D'54'
		subwf		char,w		; Is character <=5?
		btfsc		STATUS,carry	
		goto		clear		; Start over if not valid
		incf		rxstate		; Increment and go to next character
		goto		receive

st12							; get_1m
		movf		char,w
		movwf		onemin
		movlw		D'58'
		subwf		char,w		; Is character <=9?
		btfsc		STATUS,carry	
		goto		clear		; Start over if not valid
		incf		rxstate		; Increment and go to next character
		goto		receive

st13								; get_10s
		movf		char,w
		movwf		tensec
		movlw		D'54'
		subwf		char,w		; Is character <=5?
		btfsc		STATUS,carry	
		goto		clear		; Start over if not valid
		movlw		D'48'
		subwf		char,w		; Top of the minute?	
		btfsc		STATUS,zero
		goto		transmit	; Yes, see if it's time to transmit
st13b	incf		rxstate		; Increment and go to next character
		goto		receive

st14							; get_1s
		movf		char,w
		movwf		onesec
		movlw		D'58'
		subwf		char,w		; Is character <=9?
		btfsc		STATUS,carry	
		goto		clear		; Start over if not valid
		movlw		B'00000100'	
		xorwf		PORTA		; Toggle LED (RA2)
		btfsc	 	PORTA,pb_1 	; PB1 down?
		goto 		clear		; No - bandchange not selected - start over
		call		bandchange	; Yes - increment band
		goto		clear		; start over

transmit	
		movf		evenmin,w	; Load selected minute to transmit
		subwf		onemin,w
		btfss		STATUS,zero	; Time to transmit?
		goto		st13b		; No - resume 		
		bcf 		PORTA,LED1 	; Turn off LED1
		clrf		PCLATH
		call		main
		btfsc		scan_flag,0	; Scan flag set?
        call		scan		; Yes - increment variables
		goto		clear		; No - start over


;___________________________________________________________________________________



		end
