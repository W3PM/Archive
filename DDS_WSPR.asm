;
; DDS_WSPR - Uses AD9851 to tranmit WSPR code on 20% transmit cycle
;		   - Multi-band 160 thru 6 meters
;          - Transmit band scanning
; 				
;  
;  Gene Marcus W3PM GM4YRE
;  16 February, 2009
;
;  Clock Frequency: 4 MHz
;
; 
;
; 					       PIC16F628A                             
;                          __________                                          
;     Not used	  ----RA2 |1       18| RA1---------Scan LED                   
;     Not used    ----RA3 |2       17| RA0---------Transmit                   
;     PB_1-Bandswitch-RA4 |3       16| OSC1--------XTAL                        
;     +5V-----------!MCLR |4       15| OSC2--------XTAL                        
;     Ground----------Vss |5       14| VDD---------+5 V                        
;     1 PPS-----------RB0 |6       13| RB7---------DDS_LOAD                    
;     Band BCD0-------RB1 |7       12| RB6---------Band BCD3    
;     DDS_CLK---------RB2 |8       11| RB5---------Band BCD2           
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

DDS_clk 	equ     0x02        ; AD9850/AD9851 write clock
DDS_dat 	equ     0x03        ; AD9850/AD9851 serial data input
DDS_load	equ     0x07        ; Update pin on AD9850/AD9851                      
pb_1        equ     0x04        ; Change band/band scan
TX          equ		0x00		; Transmit on/off
BCD_0		equ		0x01		; BCD outputs used to control
BCD_1		equ		0x04		; CD4028 BCD to decimal switch
BCD_2		equ		0x05		; or directly for amplifier/filter
BCD_3		equ		0x06		; selection and/or LED band indicator
scan_LED	equ		0x01		; Used to indicate scan enabled

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
			sendcode
			pad
			two_sec
			band
			led_stat
			scan_flag
		endc


		goto		start



;=====================================================================
;  Subroutines
;=====================================================================

	; Interrupt service routine - triggered by 1 PPS GPS on pin 6 (RB0)

IRQSVC								
		org			H'04'			; Interrupt 1PPS starts here
		movwf		w_temp			; Save off the W register
		swapf		STATUS,W		; And the STATUS (use swapf
		movwf		status_temp		; so as not to change STATUS)

		decfsz		sec_count		; End of even 2 minute iterval?
		goto		main3			; No, end interrupt service routine 
		movlw		D'120'			; Yes, reset 2 minute counter
		movwf		sec_count		;
		clrf		scan_flag		; start scan transmit		
		
		decfsz		min_count		; End of 10 minute interval?
		goto		main3			; No, end interrupt service routine 
		movlw		D'5'			; Yes, reset 10 minute counter 
		movwf		min_count		; 
		clrf		sendcode		; Flag to transmit

main3	
		bcf			INTCON,T0IF		; Clear the old interrupt
		bcf			INTCON,INTF

		swapf		status_temp,W	; Restore the status
		movwf		STATUS			; register
		swapf		w_temp,F		; Restore W without disturbing
		swapf		w_temp,W		; the STATUS register

		retfie

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
				retlw         D'0'
				retlw         D'2'
				retlw         D'0'
				retlw         D'0'
				retlw         D'2'
				retlw         D'1'
				retlw         D'0'
				retlw         D'0'
				retlw         D'0'
				retlw         D'3'
				retlw         D'1'
				retlw         D'1'
				retlw         D'0'
				retlw         D'2'
				retlw         D'2'
				retlw         D'3'
				retlw         D'0'
				retlw         D'0'
				retlw         D'3'
				retlw         D'0'
				retlw         D'3'
				retlw         D'3'
				retlw         D'3'
				retlw         D'1'
				retlw         D'2'
				retlw         D'0'
				retlw         D'0'
				retlw         D'0'
				retlw         D'2'
				retlw         D'0'
				retlw         D'2'
				retlw         D'3'
				retlw         D'0'
				retlw         D'0'
				retlw         D'1'
				retlw         D'2'
				retlw         D'1'
				retlw         D'2'
				retlw         D'0'
				retlw         D'2'
				retlw         D'2'
				retlw         D'2'
				retlw         D'0'
				retlw         D'3'
				retlw         D'0'
				retlw         D'3'
				retlw         D'3'
				retlw         D'2'
				retlw         D'2'
				retlw         D'3'
				retlw         D'1'
				retlw         D'0'
				retlw         D'3'
				retlw         D'2'
				retlw         D'2'
				retlw         D'0'
				retlw         D'3'
				retlw         D'1'
				retlw         D'2'
				retlw         D'1'
				retlw         D'0'
				retlw         D'2'
				retlw         D'2'
				retlw         D'2'
				retlw         D'1'
				retlw         D'1'
				retlw         D'0'
				retlw         D'1'
				retlw         D'2'
				retlw         D'1'
				retlw         D'2'
				retlw         D'1'
				retlw         D'0'
				retlw         D'1'
				retlw         D'2'
				retlw         D'2'
				retlw         D'1'
				retlw         D'0'
				retlw         D'0'
				retlw         D'3'
				retlw         D'2'
				retlw         D'1'
				retlw         D'1'
				retlw         D'2'
				retlw         D'0'
				retlw         D'0'
				retlw         D'1'
				retlw         D'3'
				retlw         D'0'
				retlw         D'3'
				retlw         D'0'
				retlw         D'1'
				retlw         D'2'
				retlw         D'2'
				retlw         D'2'
				retlw         D'1'
				retlw         D'2'
				retlw         D'2'
				retlw         D'2'
				retlw         D'2'
				retlw         D'0'
				retlw         D'1'
				retlw         D'0'
				retlw         D'0'
				retlw         D'3'
				retlw         D'2'
				retlw         D'0'
				retlw         D'1'
				retlw         D'3'
				retlw         D'3'
				retlw         D'0'
				retlw         D'1'
				retlw         D'3'
				retlw         D'0'
				retlw         D'2'
				retlw         D'1'
				retlw         D'3'
				retlw         D'2'
				retlw         D'1'
				retlw         D'2'
				retlw         D'2'
				retlw         D'2'
				retlw         D'3'
				retlw         D'3'
				retlw         D'1'
				retlw         D'2'
				retlw         D'0'
				retlw         D'0'
				retlw         D'0'
				retlw         D'0'
				retlw         D'3'
				retlw         D'2'
				retlw         D'1'
				retlw         D'0'
				retlw         D'0'
				retlw         D'1'
				retlw         D'1'
				retlw         D'0'
				retlw         D'2'
				retlw         D'0'
				retlw         D'2'
				retlw         D'2'
				retlw         D'2'
				retlw         D'2'
				retlw         D'3'
				retlw         D'3'
				retlw         D'0'
				retlw         D'3'
				retlw         D'0'
				retlw         D'1'
				retlw         D'3'
				retlw         D'2'
				retlw         D'2'
				retlw         D'0'
				retlw         D'3'
				retlw         D'3'
				retlw         D'2'
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
        retlw   0x47 ; 6 meters  MSB 0
        retlw   0x87 ;
        retlw   0xab ; band 10
        retlw   0x2a ;____________LSB__
        retlw   0x28 ; 10 meters MSB 4		
        retlw   0x00 ; 							
        retlw   0x66 ; band 9						
        retlw   0x87 ;_________________			
        retlw   0x23 ; 12 meters MSB 8					
        retlw   0x73 ; 							
        retlw   0x50 ; band 8						
        retlw   0xe8 ;_________________		
        retlw   0x1e ; 15 meters MSB 12				
        retlw   0x00 ; 						
        retlw   0xdb ; band 7				
        retlw   0x09 ;_________________    
        retlw   0x19 ; 17 meters MSB 16				
        retlw   0xc0 ; 					
        retlw   0x3a ; band 6				
        retlw   0xd6 ;_________________			
        retlw   0x14 ; 20 meters MSB 20	
        retlw   0x0c ; 				
        retlw   0x98 ; band 5			
        retlw   0xf4 ;_________________ 			
        retlw   0x0e ; 30 meters MSB 24					
        retlw   0x6b ; 							
        retlw   0xef ; band 4						
        retlw   0x24 ;_________________  			
        retlw   0x0a ; 40 meters MSB 28					
        retlw   0x03 ; 							
        retlw   0x38 ; band 3                      
        retlw   0xe1 ;_________________
        retlw   0x05 ; 80 meters MSB 32
        retlw   0x1c ; 
        retlw   0x92 ; band 2
        retlw   0x66 ;_________________ 
        retlw   0x02 ; 160 meters MSB 40
        retlw   0x9d ; 
        retlw   0x3b ; band 1
        retlw   0x56 ;_________________ 


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
		bz			zero			; Yes, goto zero
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

zero
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
		movlw		D'39'			;
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
		movlw		D'1'
		movwf		sendcode		; End of transmit
		movwf		scan_flag		; End of transmit for scan function
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
		bsf		PORTB,BCD_0
		goto	$+2
		bcf		PORTB,BCD_0
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

;______________________________________________________________________




start


;---------------------------------------------------------------------
;	Set up timer 
;---------------------------------------------------------------------
		errorlevel	-302
		banksel		INTCON
		bsf			INTCON,GIE
		bcf			INTCON,T0IE		; Mask timer interrupt
		bsf			INTCON,INTE

		banksel		OPTION_REG
		bsf			OPTION_REG,INTEDG
		bcf			OPTION_REG,T0CS	; Select timer
		bcf			OPTION_REG,PSA	; Prescaler to timer
		bsf			OPTION_REG,PS2	; \
		bsf			OPTION_REG,PS1	;  >- 1:256 prescale
		bsf			OPTION_REG,PS0	; /

;---------------------------------------------------------------------
;	Set up I/O 
;---------------------------------------------------------------------

		banksel	TRISA                        
        movlw   B'11011100'       ; Tristate PORTA (all Inputs except RA0,RA1)  
        movwf   TRISA             ;
 		movlw	B'00000001'
        movwf   TRISB             ; Set port B to all outputs except RB0
		banksel	PORTA
		clrf	PORTA
		clrf	PORTB

;---------------------------------------------------------------------
;	Initialize memory
;---------------------------------------------------------------------


;		Set default basefreq to 10.1042 MHz
        movlw		D'24'			; MSB for band (refer to band table) 
		movwf		band 
		movlw		D'4'			; band number from band table
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

		movlw		D'120'			; for 2 minute intervals
		movwf		sec_count

		movlw		D'5'
		movwf		min_count		; for 10 minute intervals

		clrf		sendcode
		clrf		scan_flag

		btfsc	 	PORTA,pb_1 		; PB1 down?
		goto 		WaitForInt		; No, skip to WaitForInt

TestPB1up
		btfss 		PORTA,pb_1 		; PB1 up?
		goto 		TestPB1up 		; No, wait for release
		goto		start_scan


WaitForInt
		movfw		sendcode		; Will go to transmit on the
		addlw		D'0'			; default frequency upon reset then 
		bz			transmit		; transmit on a 10 min. interval. 
				

; Note: pushbutton is only active when not transmitting

		btfsc	 	PORTA,pb_1 		; PB1 down?
		goto 		WaitForInt		; No, skip to WaitForInt

TestPB1u
		btfss 		PORTA,pb_1 		; PB1 up?
		goto 		TestPB1u 		; No, wait for release

PB_yes1
		bcf			STATUS,C
        movlw   	0x04      		; get 4 bytes to subtract
        subwf   	band,f  		; Move down to MSB in band list
		incf		led_stat		; Increment band LED
        btfss   	STATUS,C       	; Off the bottom?
        goto    	reset_var       ; Yes, reset variables
		call    	LED_status		; Set up BCD outputs
		movlw		D'1'
		movwf		sendcode		; Ensure timing integity
        goto    	WaitForInt

start_scan
		movlw		D'1'
		movwf		scan_flag		; Set scan flag	
		bsf			PORTA,scan_LED	; Turn on scan LED

loop	
		movfw		scan_flag		; Interrupt routine will
		addlw		D'0'			; clear scan_flag at 120 sec interval
		bz			transmit2		; Time to transmit?
		goto		loop			; No, wait for even minute		

transmit	
		call		main			; Start transmit routine
		goto 		WaitForInt		; Start over and wait for interrupts


reset_var       
		movlw		D'40'			; Set counter to end of band table
		movwf		band
		movlw		D'0'			; Initialize LED (BCD) status
		movwf		led_stat               
        goto    	PB_yes1      

transmit2	
	 	bcf			STATUS,C
	    movlw   	0x04      		; Get 4 bytes to subtract
        subwf   	band,f  		; Move down to MSB in band list
		incf		led_stat		; Increment band LED
        btfss   	STATUS,C       	; Off the bottom?
        goto	   	reset_var2      ; Yes, reset variables
		call    	LED_status		; Set up BCD outputs
		call		main			; Start transmit routine
		goto 		loop			; Start over and wait for interrupt
	

reset_var2       
		movlw	D'40'				; Set counter to end of band table
		movwf	band
		movlw	D'0'				; Initialize LED (BCD) status
		movwf	led_stat               
        goto	transmit2

		end
