/*
Mega 2560 DDS-60 WSPR/QRSS Controller 
 Generates QRSS and WSPR coordinated frequency hopping transmissions 
 on 6 thru 160 meters synchronized by either GPS or WWVB time data.
 
 Copyright (C) 2011,  Gene Marcus W3PM GM4YRE
 
 25 August  2011
 
 Permission is granted to use, copy, modify, and distribute this software
 and documentation for non-commercial purposes.
 
 
 Updates:
 21 September 2011: Modified to include 12 QRSS graphical patterns
                    and independent timeslot WSPR power allocations
 28 September 2011: Corrected WWVB LCD print conflict
 16 December  2011: Updated for Arduino 1.0 compatibility
  5 January   2012: Added selective band transmit algorithm
 18 March     2012: Added $GPRMC GPS decode function
 20 March     2012: Added x10 and /10 power option
 18 June      2012: Corrected EEPROM R/W in calibration routine using code provided by G0MGX                   
 18 September 2012  Added witched external 30 MHz input with switched ground 
                    on pin 14 when active. 
 21 September 2012  Modified to use GPS 1pps to generate 30MHz DDS clock correction factor
                    Note: 'Band 6 select' moved from pin 47 in previous versions to pin 33.
                    pin 47 is now used for DDS clock/8 input.
 28 December  2012  Added type 2 station ID (eg W4/GM4YRE EM64OR) capabilities.
 12 March     2013  Fixed grid square calculation bug
 16 April     2013  Fixed satellite display bug
 19 June      2013  Fixed LF, MF frequency display bug
 
 Note to GPS users:
 The GPS algorith detects both $GPGGA and $GPRMC NMEA sentences. If your GPS receiver outputs $GPRMC data 
 the LED display will always indicate 0 satellites in view. If your GPS receiver is sending both $GPGGA 
 and $GPRMC data you may either reconfigure your receiver to output only $GPGGA data or change $GPRMC to
 $AAAAA in the variables configuration portion of this sketch.

 
 The WSPR special message alorithm was derived from Fortran and C files found the K1JT WSPR source code. 
 Portions of the WSPR message algorithm is the work of Andy Talbot, G4JNT. Portions of the GPS receive 
 code were influenced by Igor Gonzalez Martin's Arduino tutorial. Special thanks to G0MRX for providing
 the updated EEPROM R/W algorithm for manual frequency calibration. 
 
 ------------------------------------------------------------
 Mega 2560 Digital Pin allocation:
 
 0 
 1                    19 GPS RX             37 Band 1 select
 2 Encoder0 A         20 GPS 1pps input     38 DDS data
 3 Transmit Inhibit   21 Band Change        39 Band 2 select
 4 Encoder0 B         22 LCD D7             40 CAL Enable/TX set
 5 Divide power by 10 23                    41 Band 3 select
 6 Mult power by 10   24 LCD D6             42 Clock Set
 7                    25                    43 Band 4 select
 8                    26 LCD D5             44 Hours Set/CAL +
 9                    27                    45 Band 5 select
 10                   28 LCD D4             46 Minutes Set/CAL -
 11                   29                    47 Ext input from DDS clock/8
 12                   30 LCD E              48 TX
 13 On board LED      31                    49 Band 7 select
 14 Ext clock sense   32 LCD RS             50 WWVB RX
 15                   33 Band 6 select      51 Band 8 select 
 16                   34 DDS load           52 QRSS TX/band select
 17                   35 Band 0 select      53 Band 9 select
 18                   36 DDS clock                   
 
 note: pin 47 is timer 5 input
 ------------------------------------------------------------
 */
// include the library code:
#include <LiquidCrystal.h>
#include <string.h>
#include <ctype.h>
#include <EEPROM.h>
#include <avr/interrupt.h>  
#include <avr/io.h>
#include <math.h>
struct CalFact {
  union {
    long value;
    struct {
      unsigned char b1;
      unsigned char b2;
      unsigned char b3;
      unsigned char b4;
    }
    __attribute__((packed));
  }
  __attribute__((packed));
}
__attribute__((packed));
struct CalFact CalFactor;


//_________________________Enter callsign and grid square below:_________________________
char call2[13] = "W4/GM4YRE"; //e.g. "W3PM" or "W4/GM4YRE"
char locator[7] = "EM64";  // Use 4 or 6 character locator e.g. "EM64" or "EM64or"
//_______________________________________________________________________________________


//_________________________Enter timeslot data below:____________________________________
// column 1 = frequency; Column 2 = mode; Column 3 = power
// frequency: Hz
// mode: ( 0=idle, 1=WSPR, 2=QRSS )
// power: Min = 0 dBm, Max = 43 dBm, steps 0,3,7,10,13,17,20,23,27,30,33,37,40,43
// Note: Ensure QRSS message length does not exceed QRSS allocated transmit window(s)
unsigned long timeslot_array [10] [5] = { 
  { // timeslot 0  00,20,40 minutes after hour
    1838120,1,13                                   }
  ,  
  { // timeslot 1  02,22,42 minutes after hour
    3594120,1,13                                   }
  ,  
  { // timeslot 2  04,24.44 minutes after hour
    474200,1,37                                    }
  , 
  { // timeslot 3  06,26,46 minutes after hour
    7040120,1,13                                   }
  ,  
  { // timeslot 4  08,28,48 minutes after hour
    10140220,1,13                                   }
  , 
  { // timeslot 5  10,30,50 minutes after hour
    14097120,1,13                                   }
  , 
  { // timeslot 6  12,32,52 minutes after hour
    18106120,1,13                                   }
  , 
  { // timeslot 7  14,34,54 minutes after hour
    21096120,1,10                                   }
  , 
  { // timeslot 8  16,36,56 minutes after hour
    24926120,1,10                                   }
  , 
  { // timeslot 9  18,38,58 minutes after hour
    28126120,1,10                       } 
  };


//__________________________________________________________________________________________________
// ENTER QRSS DATA:

// QRSS frequency (Hz)
unsigned long QRSSfreq = 10140060;

// QRSS message
// A "space" should be added at the end of message to separate message when looping
const char QRSSmsg[ ] = "W3PM ";

// QRSS frequency shift in Hz
const byte QRSSshift = 4;

// QRSS dot length in seconds
const byte QRSSdot = 3;

// QRSS patterns (patterns not available with auto calibration due to timing conflicts):
// 0 = ID loop; 1 = sinewave; 2 = interrupted sinewave; 3 = sawtooth
// 4 = waves; 5 = hills; 6 = herringbone; 7 = upramp ; 8 = downramp
// 9 = upramp sawtooth; 10 = downramp sawtooth; 11 = M's; 12 = W's
byte Pattern = 4;

//__________________________________________________________________________________________________

const char SyncVec[162] = {
  1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,0,0,1,0,
  1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,1,1,0,0,0,1,1,0,1,0,1,0,
  0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,1,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,
  0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,0,0
};

/*
Load Morse code send data. 
 The first five bits are data representing the Morse character. 
 1 = dit
 0 = dah
 The last three bits represent the number of bits in the Morse character.
 */
const byte Morse[37] = {
  B11111101,	//	0		
  B01111101,	//	1	
  B00111101,	//	2		
  B00011101,	//	3		
  B00001101,	//	4	
  B00000101,	//	5		
  B10000101,	//	6		
  B11000101,	//	7		
  B11100101,	//	8		
  B11110101,	//	9		
  B01000010,	//	A		
  B10000100,	//	B		
  B10100100,	//	C		
  B10000011,	//	D		
  B00000001,	//	E		
  B00100100,	//	F		
  B11000011,	//	G		
  B00000100,	//	H		
  B00000010,	//	I		
  B01110100,	//	J		
  B10100011,	//	K		
  B01000100,	//	L		
  B11000010,	//	M		
  B10000010,	//	N		
  B11100011,	//	O		
  B01100100,	//	P		
  B11010100,	//	Q		
  B01000011,	//	R		
  B00000011,	//	S		
  B10000001,	//	T		
  B00100011,	//	U		
  B00010100,	//	V		
  B01100011,	//	W		
  B10010100,	//	X		
  B10110100,	//	Y		
  B11000100,	//	Z
  B00000000     //      sp		
};

// define pins
#define DATA          38 //DDS DATA 
#define CLOCK         36 // DDS CLOCK 
#define LOAD          34 // DDS LOAD
#define encoder0PinA   2
#define encoder0PinB   4
#define InhibitButton  3
#define CalSetButton  40 // Go to calibrate pin (hold LOW on reset to calibrate)
#define CalUpButton   44
#define CalDwnButton  46          
#define TimeSetPin    42 // Go to clock set pin (hold LOW on reset to clock set)
#define HourSetPin    44 // LOW to set hours
#define MinuteSetPin  46 // Low to set minutes (seconds will go to zero)
#define ledPin        13 // on board LED
#define GPSpin        19 // RX PIN 
#define wwvbPin       50 // WWVB receiver input pin
#define txPin         48 // TX (HIGH on transmit)
#define div10Pin       5 // Low = power divided by 10
#define mult10Pin      6 // Low = power multiplied by 10
#define extclockPin   14 // Low = external clock
#define ppsPin        20 // 1pps input pin
#define QRSStxPin     52 // QRSS TX/band select (HIGH on transmit)
#define BandButton    40 // used to set timeslot TX in bandselect operation

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(32, 30, 28, 26, 24, 22);

// Load WSPR symbol frequency offsets
unsigned int OffsetFreq[4] = {0,35,70,105};

// configure variables
int bandPin[10]={35,37,39,41,43,45,33,49,51,53};
int IndiceCount=0,StartCount=0,counter=0,val = 0,MsgLength;
int indices[13],byteGPS=-1,second=0,minute=0,hour=0;
int pulseWidth=0,bitcount=0,countSync=0,wwvbData=0,wwvbHour=0,wwvbMin=0;
int validGPSflag=0,GPScount=0,mSecTimer,mSecTimer2,PulsePeriod,milliwatts;
int Lat10,Lat1,NS,lat,sat,inc = 3,mult;
int Lon100,Lon10,Lon1,EW,sat10,sat1,lon;
int dLon1,dLon2,dLat1,dLat2,dlon,dlat;
byte ndbm,pinState = 0,w0 = 1,count = 0,start = 0; 
byte c[11],sym[170],symt[170],symbol[162],calltype,RXflag=1;                
volatile int jj;
volatile byte bb,ii,i,j,timeslot,BandLockSlot;
volatile byte QRSScount,MorseByte,ByteMask,MorseBitCount,QRSSflag,GPSflag;
volatile byte MorseBit,MorseChar,SpaceCount,CharFlag,WordSpaceFlag,WWVBflag1,WWVBflag2;
volatile byte InhibitFlag = 0,temp=1,GPSinhibitFlag = 0,BandLockFlag = 0,WSPR15flag = 0; 
unsigned long fCLK=180000000,mask = 1; // bitmask
unsigned long FreqWord,TempWord,TempFreq,temp_array [5] [5];
unsigned long TenMHz = 10000000,n1,m1;
unsigned char cc1;
char GPSlocator[7],buffer[300] = "",StartCommand[7] = "$GPGGA",StartCommand2[7] = "$GPRMC",buf[10],cnt1;
float pi=3.14,wavecount=0;
double wave;
char grid4[5],grid6[7],call1[7];
long MASK15=32767,ihash;
unsigned long t1,ng,n2;
int nadd,nc,n,ntype; 


//******************************************************************
// Clock - interrupt routine used as master timekeeper
// Toggle LED on pin 13 1PPS
//Timer1 Overflow Interrupt Vector, called every second
ISR(TIMER1_COMPA_vect) {
  static boolean output = HIGH;
  digitalWrite(13, output);
  output = !output;
  second++ ;
  if (second == 60) {
    minute++ ;
    second=0 ;
  }
  if (minute == 60) {
    hour++;
    minute=0 ;
  }
  if (hour == 24) {
    hour=0 ;
  }
  displaytime();
  if(InhibitFlag == 0)
  {
    if(bitRead(minute,0) == 0 & second == 0) 
    {
      if (minute < 20) {
        timeslot = minute/2;
      }
      else{
        if (minute < 40) {
          timeslot = (minute-20)/2;
        }
        else {
          timeslot = (minute -40)/2;
        }
      }
      setfreq();
      transmit();
    }

    
 //QRSS transmit routine begins here.
    if(QRSSflag == 1)
    {
      if(start < 4)
      { // Transmit QRSS base frequency for four seconds to start QRSS transmission
        TempFreq = QRSSfreq;
        TempWord = TempFreq*pow(2,32)/180000000;
        DisplayFreq();
        TempWord = (TempFreq+(CalFactor.value*(TempFreq/pow(10,7))))*pow(2,32)/180000000;
        TransmitSymbol();
        start++;
        QRSScount=0;
        MorseChar=0;
        MorseBit=0;
        CharFlag=0; 
      }
      else
      {
        MsgLength = (strlen(QRSSmsg));
        if (MorseChar>MsgLength-1)
        {
          if(Pattern >= 1)
          {
            i=0;
            ii=0;
            j=0;
            wavecount = 0;          
            if(Pattern == 1) wavecount = 270;//Offset to start on mark frequency
            QRSSflag = 0;      //Disable QRSS process
            TIMSK4 = 2;        //Timer4 enabled
          }
          else
            QRSScount=0;
          MorseChar=0;
          MorseBit=0;
          CharFlag=0;
        }
        else
          if((QRSSmsg[MorseChar] >= 97) & (QRSSmsg[MorseChar] <= 122)) 
          {
            temp = QRSSmsg[MorseChar]-87;          
          }
          else
            if((QRSSmsg[MorseChar] >= 65) & (QRSSmsg[MorseChar] <= 90))
            { 
              temp = QRSSmsg[MorseChar] - 55;  
            }
            else
              temp = QRSSmsg[MorseChar] - 48; 
        ByteMask = B00000111;
        MorseBitCount = ByteMask & Morse[temp]; 
        if(QRSSmsg[MorseChar] == 32)
        {
          WordSpace(); 
        }
        else
          if(bitRead(Morse[temp],(7-MorseBit)) == HIGH)
          {
            Dah();
          }
          else
          {
            Dit();
          }
        if (CharFlag >= 1)
        {
          CharSpace(); 
        }
      }
    }
  }
  else{
  }
};



//******************************************************************
//Timer3 Overflow Interrupt Vector, called every mSec for WWVB timing
ISR(TIMER3_COMPA_vect) {
  mSecTimer++;
};  


//******************************************************************
//Timer4 Overflow Interrupt Vector, called every mSec to increment
//the mSecTimer that is used for WSPR processing & QRSS pattterns

ISR(TIMER4_COMPA_vect) {

  if ((timeslot_array [timeslot] [1]) ==2)
    //  if (TransmitFlag[timeslot] == 2)
  { //QRSS pattern generator routines start here 
    ii++;
    if(ii > (QRSSdot*15))
    {
      TempFreq = QRSSfreq;
      TempWord = TempFreq*pow(2,32)/180000000;
      DisplayFreq();
      TempWord = (TempFreq+(CalFactor.value*(TempFreq/pow(10,7))))*pow(2,32)/fCLK;
      switch(Pattern){
      case 1:  // sinewave
        wave = sin(wavecount*(pi/180));
        wave = (wave+1)/2;
        break;
      case 2:  // interrupted sinewave
        if(j == HIGH)
        {
          wave = 0.25;
        }
        else
          wave = sin(wavecount*(pi/180));
        wave = (wave+1)/2;
        break;        
      case 3:  // sawtooth
        if(wavecount < 180)
        {
          wave = wavecount/180;
        }
        else
          wave = 2-(wavecount/180);
        break;
      case 4:  // waves
        if(wavecount < 180)
        {
          wave = pow((wavecount/180),2);
        }
        else
          wave = pow((2-(wavecount/180)),2);
        break;         
      case 5:  // hills
        if(wavecount < 180)
        {
          wave = sqrt(wavecount/180);
        }
        else
          wave = sqrt(2-(wavecount/180));
        break; 
      case 6:  // herringbone
        if(wavecount < 180)
        {
          wave = wavecount/180;
        }
        else
          wave = 1.2 -(wavecount/180);
        break; 
      case 7:  // upramp
        wave = wavecount/360;
        break; 
      case 8:  // downramp
        wave = 1-(wavecount/360);
        break; 
      case 9:  // upramp sawtooth
        if(wavecount < 300)
        {
          wave = wavecount/300;
        }
        else
          wave = 1-((wavecount-300)/60);
        break; 
      case 10:  // downramp sawtooth
        if(wavecount < 60)
        {
          wave = wavecount/60;
        }
        else
          wave = 1-((wavecount-60)/300);          
        break;    
      case 11:  // M's
        if(i == 2)
        {
          wave = 0;
        }
        else
        {
          if(wavecount < 180)
          {
            wave = wavecount/180;
          }
          else
            wave = 2-(wavecount/180);
        }
        break;
      case 12:  // W's
        if(i == 2)
        {
          wave = 1;
        }
        else
        {
          if(wavecount < 180)
          {
            wave = 1-(wavecount/180);
          }
          else
            wave = (wavecount-180)/180;
        }
        break;
      }
      TempFreq = (QRSSshift*wave)*(QRSSfreq/pow(10,7)*pow(2,32)/fCLK);
      //      TempFreq = QRSSshift*(wave*QRSSfreq/pow(10,7))*pow(2,32)/fCLK;
      TempWord = TempWord + TempFreq;
      TransmitSymbol();
      ii=0;
      wavecount++;
      if(wavecount>359)
      {
        wavecount=0;
        i++;
        if(i > 2) i=0;
        j=!j;
      }
    } 
  }
  else
  {// WSPR symbol tranmission routine begins here:
    mSecTimer2++;
    if(mSecTimer2 > 681){
      mSecTimer2 = 0;
      if(bb < 3) { // Begin 2 second delay - actually 0.682mSec * 3
        TempWord = FreqWord;
        TransmitSymbol();
        bb++;
      }
      else
      {
        // Begin 162 WSPR symbol transmission
        if (count < 162) 
        {
          TempWord = FreqWord+OffsetFreq[sym[count]];
          TransmitSymbol();
          count++;             //Increments the interrupt counter
        }
        else
        {
          TIMSK4 = 0;   // Disable WSPR timer
          digitalWrite(txPin,LOW); // External transmit control OFF 
          TempWord=0;            // Turn off transmitter
          TransmitSymbol(); 
          if(TIMSK1 == 2)RXflag = 1;          //Turn GPS/WWVB receiver back on 
          lcd.setCursor(0,3);      
          lcd.print("IDLE         ");
        }
      }
    }
  }
};  


void setup()
{
  TIMSK0 = 0;        //Disable timer0

    //Setup Timer1 to fire every second (Master Clock)
  TCCR1B = 0;        //Disable Timer1 during setup
  TIMSK1 = 2;        //OCR1A Interrupt Enable
  //TIMSK1 = 0;        //OCR1A Interrupt Disable
  TCCR1A = 0;        //Normal port operation, Wave Gen Mode normal
  TCCR1B = 12;       //Timer Prescaler set to 256 - CTC mode
  OCR1A = 62487;     // Timer set for 1000 mSec using correction factor
  // 62500 is nominal for 1000 mSec. Decrease OCR1A to increase clock speed

  //Setup Timer3 to fire every mSec (WWVB)
  TCCR3B = 0;        //Disable timer during setup
  TIMSK3 = 0;        //OCR3A Interrupt disable
  TCCR3A = 0;        //Normal port operation, Wave Gen Mode normal
  TCCR3B = 9;        //Timer prescaler to 1 - CTC mode
  OCR3A = 16000;     //Timer set for 1 mSec

  //Setup Timer4 to fire every mSec (WSPR processing & QRSS pattterns)
  TCCR4B = 0;        //Disable timer during setup
  TIMSK4 = 0;        //Timer4 Interrupt disable
  TCCR4A = 0;        //Normal port operation, Wave Gen Mode normal
  TCCR4B = 9;        //Timer prescaler to 1 - CTC mode
  OCR4A = 16000;     //Timer set for 1 mSec

  //Set up Timer5 as a frequency counter - input at pin 47
  TCCR5B = 0;        //Disable Timer5 during setup
  TCCR5A = 0;        //Reset
  TCNT5  = 0;        //Reset counter to zero
  TIFR5  = 1;        //Reset overflow

  // Ensure input data is upper case
  for(i=0;i<13;i++)if(call2[i]>=97&&call2[i]<=122)call2[i]=call2[i]-32;
  for(i=0;i<7;i++)if(locator[i]>=97&&locator[i]<=122)locator[i]=locator[i]-32;
  
  // Initialize LED pin
  pinMode(ledPin, OUTPUT);         

  // Inititalize divide power by 10 input
  pinMode(div10Pin, INPUT);        // Initialize divide by 10 pin
  digitalWrite(div10Pin, HIGH);    // internal pull-up enabled

  // Inititalize multiply power by 10 input
  pinMode(mult10Pin, INPUT);       // Initialize multiply by 10 pin
  digitalWrite(mult10Pin, HIGH);   // internal pull-up enabled

  // Inititalize external 30 MHz clock input sense
  pinMode(extclockPin, INPUT);
  digitalWrite(extclockPin, HIGH); // internal pull-up enabled

  // Inititalize GPS 1pps input
  pinMode(ppsPin, INPUT);
  // digitalWrite(ppsPin, HIGH); // internal pull-up enabled

  // Inititalize GPS input
  pinMode(GPSpin, INPUT);
  digitalWrite(GPSpin, HIGH); // internal pull-up enabled
  Serial1.begin(4800);

  // Setup band select pins
  for(i=0;i<10;i++)
  {
    pinMode(bandPin[i],OUTPUT);
  }

  // Set TX pins for output
  pinMode(txPin, OUTPUT);
  pinMode(QRSStxPin, OUTPUT);


  // Set 1PPS pin 20 for external interrupt input
  mult = 0;
  attachInterrupt(3, PPSinterrupt, RISING);

  // Initialize a buffer for received data
  for (int i=0;i<300;i++){    
    buffer[i]=' ';
  }   

  // Initialize WWVB input
  pinMode(wwvbPin, INPUT);
  digitalWrite(wwvbPin, HIGH); // internal pull-up enabled

  // set up the LCD's number of columns and rows 
  lcd.begin(20,4); 

  // Set up transmit inhibit interrupt
  pinMode(InhibitButton, INPUT);
  digitalWrite(InhibitButton, HIGH); // internal pull-up enabled
  attachInterrupt(1, TXinhibit, LOW);  // encoder pin on interrupt 1 - pin 3

  // Set up band change interrupt
  pinMode(BandButton, INPUT);
  digitalWrite(BandButton, HIGH); // internal pull-up enabled

  // Set up rotary encoder
  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 
  attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2

  // Set up calibration pins 
  pinMode(CalSetButton, INPUT);    // declare pushbutton as input 
  digitalWrite(CalSetButton, HIGH); // internal pull-up enabled
  pinMode(CalUpButton, INPUT);    // declare pushbutton as input 
  digitalWrite(CalUpButton, HIGH); // internal pull-up enabled
  pinMode(CalDwnButton, INPUT);    // declare pushbutton as input 
  digitalWrite(CalDwnButton, HIGH); // internal pull-up enabled

  // Set up TimeSet pins
  pinMode(TimeSetPin, INPUT);    // declare pushbutton as input 
  digitalWrite(TimeSetPin, HIGH); // internal pull-up enabled  
  pinMode(HourSetPin, INPUT);     // declare pushbutton as input
  digitalWrite(HourSetPin, HIGH); // internal pull-up enabled
  pinMode(MinuteSetPin, INPUT);   // declare pushbutton as input
  digitalWrite(MinuteSetPin, HIGH); // internal pull-up enabled

  // Set up DDS-60
  pinMode (DATA, OUTPUT);   // sets pin 38 as OUPUT
  pinMode (CLOCK, OUTPUT);  // sets pin 37 as OUTPUT
  pinMode (LOAD, OUTPUT);   // sets pin 36 as OUTPUT
  // pinMode (pin13, OUTPUT);

  Serial.begin(9600);        // connect to the serial port
  lcd.display();       

  // turn off transmitter
  TempWord = 0;
  TransmitSymbol();

  // Get calfactor data
  if(digitalRead(extclockPin) == LOW) {
    CalFactor.value = 0;
  }
  else
  CalFactor.b1 = EEPROM.read(50);
  CalFactor.b2 = EEPROM.read(51);
  CalFactor.b3 = EEPROM.read(52);
  CalFactor.b4 = EEPROM.read(53);

  // Set power level to default timeslot
  if(digitalRead(div10Pin) == LOW)
  {
    ndbm = (timeslot_array [timeslot] [2])-10;
  }
  else
    if(digitalRead(mult10Pin) == LOW)
    {
      ndbm = (timeslot_array [timeslot] [2])+10;
    } 
    else
      ndbm = timeslot_array [timeslot] [2];

  // Display data on LCD
  screendata();  

  // WSPR message calculation
  wsprGenCode(); 
  
  // Calibration process follows:
  if(digitalRead(CalSetButton) == LOW)
  {
    FreqWord = TenMHz*pow(2,32)/fCLK;
    TempWord = FreqWord;
    TransmitSymbol();
    CalFactor.value = 0;
    //CalTemperature = 0;
    TIMSK1 = 0;          // Disable Timer1 Interrupt 
    TIMSK3 = 0;          // Disable Timer3 Interrupt 
    QRSSflag = 0;        // Disable QRSS process
    lcd.clear(); 
    lcd.setCursor(0,0);
    lcd.print("Adjust to 10MHz");
    lcd.setCursor(0,1);
    lcd.print("Cal Factor= ");
    lcd.setCursor(12,1);
    lcd.print(CalFactor.value);
    calibrate();
  }

  // Manual time set process follows:
  if(digitalRead(TimeSetPin) == LOW)
  {
    TIMSK1 = 0;        // Interrupt disable to stop clock
    hour = 0;
    minute = 0;
    second = 0;
    displaytime();
    timeset();
  }
}

//******************************************************************
void wsprGenCode()
{
  for(i=0;i<13;i++){if(call2[i] == 47)calltype=2;};
  if(calltype == 2) type2();
  else
  {
  for(i=0;i<7;i++){call1[i] = call2[i]; };
  for(i=0;i<5;i++){
  grid4[i] = locator[i];
  };
  packcall();
  packgrid();
  n2=ng*128+ndbm+64;
  pack50();
  encode_conv();
  interleave_sync();
  }
}

//******************************************************************
void type2()
{
  if(timeslot_array [timeslot] [4] == LOW)
  {
  packpfx();
  ntype=ndbm + 1 + nadd;
  n2 = 128*ng + ntype + 64;
  pack50();
  encode_conv();
  interleave_sync();  
  }
  else
  {
    hash();
    for(ii=1;ii<6;ii++)
    {
      call1[ii-1]=locator[ii];
    };
    call1[5]=locator[0];
    packcall();
    ntype=-(ndbm+1);
    n2=128*ihash + ntype +64;
    pack50();
    encode_conv();
    interleave_sync();
  };

}

//******************************************************************
void packpfx()
{
  char pfx[3];
  int Len;
  int slash;

  for(i=0;i<7;i++)
  {
    call1[i]=0;
  };
  Len = strlen(call2);
  for(i=0;i<13;i++)
  {
    if(call2[i] == 47) slash = i;
  };
  if(call2[slash+2] == 0)
  {//single char add-on suffix
    for(i=0;i<slash;i++)
    {
      call1[i] = call2[i];
    };
    packcall();
    nadd=1;
    nc=int(call2[slash+1]);
    if(nc>=48 && nc<=57) n=nc-48;
    else if(nc>=65 && nc<=90) n=nc-65+10;
    else if (nc>=97 && nc<=122) n=nc-97+10;
    else n=38;
    ng=60000-32768+n;
  }
  else
    if(call2[slash+3] == 0)
    {
     for(i=0;i<slash;i++)
    {
      call1[i] = call2[i];
    };
    packcall();
    n=10*(int(call2[slash+1])-48) +  int(call2[slash+2])-48;
    nadd=1;
    ng=60000 + 26 + n;
    }
    else
    {
     for(i=0;i<slash;i++)
    {
     pfx[i] = call2[i];
    };
    if (slash == 2)
    {
     pfx[2] = pfx[1];
     pfx[1] = pfx[0];
     pfx[0] = ' ';
    };
    if (slash == 1)
    {
     pfx[2] = pfx[0];
     pfx[1] = ' ';
     pfx[0] = ' ';
    };
     ii=0;
     for(i=slash+1;i<Len;i++)
    {
      call1[ii] = call2[i];
      ii++;
     };
    packcall();
    ng=0;
    for(i=0;i<3;i++)
    {
     nc=int(pfx[i]);
    if(nc>=48 && nc<=57) n=nc-48;
    else if(nc>=65 && nc<=90) n=nc-65+10;
    else if (nc>=97 && nc<=122) n=nc-97+10;
    else n=36;     
    ng=37*ng+n;
    };
    nadd=0;
    if(ng >= 32768)
    {
     ng=ng-32768;
     nadd=1; 
    };
   }
 }

//******************************************************************
void packcall()
{
  // coding of callsign
  if (chr_normf(call1[2]) > 9) 
  {
    call1[5] = call1[4];
    call1[4] = call1[3]; 
    call1[3] = call1[2];
    call1[2] = call1[1];
    call1[1] = call1[0];
    call1[0] = ' ';
  }

  n1=chr_normf(call1[0]);
  n1=n1*36+chr_normf(call1[1]);
  n1=n1*10+chr_normf(call1[2]);
  n1=n1*27+chr_normf(call1[3])-10;
  n1=n1*27+chr_normf(call1[4])-10;
  n1=n1*27+chr_normf(call1[5])-10;
}

//******************************************************************
void packgrid()
{
  // coding of grid4
  ng=179-10*(chr_normf(grid4[0])-10)-chr_normf(grid4[2]);
  ng=ng*180+10*(chr_normf(grid4[1])-10)+chr_normf(grid4[3]);
}

//******************************************************************
void pack50()
{
  // merge coded callsign into message array c[]
  t1=n1;
  c[0]= t1 >> 20;
  t1=n1;
  c[1]= t1 >> 12;
  t1=n1;
  c[2]= t1 >> 4;
  t1=n1;
  c[3]= t1 << 4;
  t1=n2;
  c[3]= c[3] + ( 0x0f & t1 >> 18);
  t1=n2;
  c[4]= t1 >> 10;
  t1=n2;
  c[5]= t1 >> 2;
  t1=n2;
  c[6]= t1 << 6;
}

//******************************************************************
//void hash(string,len,ihash)
void hash()
{
  int Len;
  uint32_t jhash;
  int *pLen = &Len;
  Len = strlen(call2); 
  byte IC[12];
  byte *pIC = IC;
  for (i=0;i<12;i++)
  {
    pIC + 1;
    &IC[i];
  }
  uint32_t Val = 146;
  uint32_t *pVal = &Val;
  for(i=0;i<Len;i++)
  {
    IC[i] = int(call2[i]);
  };
  jhash=nhash_(pIC,pLen,pVal);
  ihash=jhash&MASK15;
  return;
}
//******************************************************************
// normalize characters 0..9 A..Z Space in order 0..36
char chr_normf(char bc ) 
{
  char cc=36;
  if (bc >= '0' && bc <= '9') cc=bc-'0';
  if (bc >= 'A' && bc <= 'Z') cc=bc-'A'+10;
  if (bc >= 'a' && bc <= 'z') cc=bc-'a'+10;  
  if (bc == ' ' ) cc=36;

  return(cc);
}


//******************************************************************
// convolutional encoding of message array c[] into a 162 bit stream
void encode_conv()
{
  int bc=0;
  int cnt=0;
  int cc;
  unsigned long sh1=0;

  cc=c[0];

  for (int i=0; i < 81;i++) {
    if (i % 8 == 0 ) {
      cc=c[bc];
      bc++;
    }
    if (cc & 0x80) sh1=sh1 | 1;

    symt[cnt++]=parity(sh1 & 0xF2D05351);
    symt[cnt++]=parity(sh1 & 0xE4613C47);

    cc=cc << 1;
    sh1=sh1 << 1;
  }
}

//******************************************************************
byte parity(unsigned long li)
{
  byte po = 0;
  while(li != 0)
  {
    po++;
    li&= (li-1);
  }
  return (po & 1);
}

//******************************************************************
// interleave reorder the 162 data bits and and merge table with the sync vector
void interleave_sync()
{
  int ii,ij,b2,bis,ip;
  ip=0;

  for (ii=0;ii<=255;ii++) {
    bis=1;
    ij=0;
    for (b2=0;b2 < 8 ;b2++) {
      if (ii & bis) ij= ij | (0x80 >> b2);
      bis=bis << 1;
    }
    if (ij < 162 ) {
      sym[ij]= SyncVec[ij] +2*symt[ip];
      ip++;
    }
  }
}

/*
-------------------------------------------------------------------------------
lookup3.c, by Bob Jenkins, May 2006, Public Domain.

These are functions for producing 32-bit hashes for hash table lookup.
hashword(), hashlittle(), hashlittle2(), hashbig(), mix(), and final() 
are externally useful functions.  Routines to test the hash are included 
if SELF_TEST is defined.  You can use this free for any purpose.  It's in
the public domain.  It has no warranty.

You probably want to use hashlittle().  hashlittle() and hashbig()
hash byte arrays.  hashlittle() is is faster than hashbig() on
little-endian machines.  Intel and AMD are little-endian machines.
On second thought, you probably want hashlittle2(), which is identical to
hashlittle() except it returns two 32-bit hashes for the price of one.  
You could implement hashbig2() if you wanted but I haven't bothered here.

If you want to find a hash of, say, exactly 7 integers, do
  a = i1;  b = i2;  c = i3;
  mix(a,b,c);
  a += i4; b += i5; c += i6;
  mix(a,b,c);
  a += i7;
  final(a,b,c);
then use c as the hash value.  If you have a variable length array of
4-byte integers to hash, use hashword().  If you have a byte array (like
a character string), use hashlittle().  If you have several byte arrays, or
a mix of things, see the comments above hashlittle().  

Why is this so big?  I read 12 bytes at a time into 3 4-byte integers, 
then mix those integers.  This is fast (you can do a lot more thorough
mixing with 12*3 instructions on 3 integers than you can with 3 instructions
on 1 byte), but shoehorning those bytes into integers efficiently is messy.
-------------------------------------------------------------------------------
*/

//#define SELF_TEST 1

//#include <stdio.h>      /* defines printf for tests */
//#include <time.h>       /* defines time_t for timings in the test */
//#ifdef Win32
//#include "win_stdint.h"	/* defines uint32_t etc */
//#else
//#include <stdint.h>	/* defines uint32_t etc */
//#endif

//#include <sys/param.h>  /* attempt to define endianness */
//#ifdef linux
//# include <endian.h>    /* attempt to define endianness */
//#endif

#define HASH_LITTLE_ENDIAN 1

#define hashsize(n) ((uint32_t)1<<(n))
#define hashmask(n) (hashsize(n)-1)
#define rot(x,k) (((x)<<(k)) | ((x)>>(32-(k))))

/*
-------------------------------------------------------------------------------
mix -- mix 3 32-bit values reversibly.

This is reversible, so any information in (a,b,c) before mix() is
still in (a,b,c) after mix().

If four pairs of (a,b,c) inputs are run through mix(), or through
mix() in reverse, there are at least 32 bits of the output that
are sometimes the same for one pair and different for another pair.
This was tested for:
* pairs that differed by one bit, by two bits, in any combination
  of top bits of (a,b,c), or in any combination of bottom bits of
  (a,b,c).
* "differ" is defined as +, -, ^, or ~^.  For + and -, I transformed
  the output delta to a Gray code (a^(a>>1)) so a string of 1's (as
  is commonly produced by subtraction) look like a single 1-bit
  difference.
* the base values were pseudorandom, all zero but one bit set, or 
  all zero plus a counter that starts at zero.

Some k values for my "a-=c; a^=rot(c,k); c+=b;" arrangement that
satisfy this are
    4  6  8 16 19  4
    9 15  3 18 27 15
   14  9  3  7 17  3
Well, "9 15 3 18 27 15" didn't quite get 32 bits diffing
for "differ" defined as + with a one-bit base and a two-bit delta.  I
used http://burtleburtle.net/bob/hash/avalanche.html to choose 
the operations, constants, and arrangements of the variables.

This does not achieve avalanche.  There are input bits of (a,b,c)
that fail to affect some output bits of (a,b,c), especially of a.  The
most thoroughly mixed value is c, but it doesn't really even achieve
avalanche in c.

This allows some parallelism.  Read-after-writes are good at doubling
the number of bits affected, so the goal of mixing pulls in the opposite
direction as the goal of parallelism.  I did what I could.  Rotates
seem to cost as much as shifts on every machine I could lay my hands
on, and rotates are much kinder to the top and bottom bits, so I used
rotates.
-------------------------------------------------------------------------------
*/
#define mix(a,b,c) \
{ \
  a -= c;  a ^= rot(c, 4);  c += b; \
  b -= a;  b ^= rot(a, 6);  a += c; \
  c -= b;  c ^= rot(b, 8);  b += a; \
  a -= c;  a ^= rot(c,16);  c += b; \
  b -= a;  b ^= rot(a,19);  a += c; \
  c -= b;  c ^= rot(b, 4);  b += a; \
}

/*
-------------------------------------------------------------------------------
final -- final mixing of 3 32-bit values (a,b,c) into c

Pairs of (a,b,c) values differing in only a few bits will usually
produce values of c that look totally different.  This was tested for
* pairs that differed by one bit, by two bits, in any combination
  of top bits of (a,b,c), or in any combination of bottom bits of
  (a,b,c).
* "differ" is defined as +, -, ^, or ~^.  For + and -, I transformed
  the output delta to a Gray code (a^(a>>1)) so a string of 1's (as
  is commonly produced by subtraction) look like a single 1-bit
  difference.
* the base values were pseudorandom, all zero but one bit set, or 
  all zero plus a counter that starts at zero.

These constants passed:
 14 11 25 16 4 14 24
 12 14 25 16 4 14 24
and these came close:
  4  8 15 26 3 22 24
 10  8 15 26 3 22 24
 11  8 15 26 3 22 24
-------------------------------------------------------------------------------
*/
#define final(a,b,c) \
{ \
  c ^= b; c -= rot(b,14); \
  a ^= c; a -= rot(c,11); \
  b ^= a; b -= rot(a,25); \
  c ^= b; c -= rot(b,16); \
  a ^= c; a -= rot(c,4);  \
  b ^= a; b -= rot(a,14); \
  c ^= b; c -= rot(b,24); \
}

/*
-------------------------------------------------------------------------------
hashlittle() -- hash a variable-length key into a 32-bit value
  k       : the key (the unaligned variable-length array of bytes)
  length  : the length of the key, counting by bytes
  initval : can be any 4-byte value
Returns a 32-bit value.  Every bit of the key affects every bit of
the return value.  Two keys differing by one or two bits will have
totally different hash values.

The best hash table sizes are powers of 2.  There is no need to do
mod a prime (mod is sooo slow!).  If you need less than 32 bits,
use a bitmask.  For example, if you need only 10 bits, do
  h = (h & hashmask(10));
In which case, the hash table should have hashsize(10) elements.

If you are hashing n strings (uint8_t **)k, do it like this:
  for (i=0, h=0; i<n; ++i) h = hashlittle( k[i], len[i], h);

By Bob Jenkins, 2006.  bob_jenkins@burtleburtle.net.  You may use this
code any way you wish, private, educational, or commercial.  It's free.

Use for hash table lookup, or anything where one collision in 2^^32 is
acceptable.  Do NOT use for cryptographic purposes.
-------------------------------------------------------------------------------
*/

//uint32_t hashlittle( const void *key, size_t length, uint32_t initval)
#ifdef STDCALL
uint32_t __stdcall NHASH( const void *key, size_t *length0, uint32_t *initval0)
#else
uint32_t nhash_( const void *key, int *length0, uint32_t *initval0)
#endif
{
  uint32_t a,b,c;                                          /* internal state */
  size_t length;
  uint32_t initval;
  union { const void *ptr; size_t i; } u;     /* needed for Mac Powerbook G4 */

  length=*length0;
  initval=*initval0;

  /* Set up the internal state */
  a = b = c = 0xdeadbeef + ((uint32_t)length) + initval;

  u.ptr = key;
  if (HASH_LITTLE_ENDIAN && ((u.i & 0x3) == 0)) {
    const uint32_t *k = (const uint32_t *)key;         /* read 32-bit chunks */
    const uint8_t  *k8;

    k8=0;                                     //Silence compiler warning
    /*------ all but last block: aligned reads and affect 32 bits of (a,b,c) */
    while (length > 12)
    {
      a += k[0];
      b += k[1];
      c += k[2];
      mix(a,b,c);
      length -= 12;
      k += 3;
    }

    /*----------------------------- handle the last (probably partial) block */
    /* 
     * "k[2]&0xffffff" actually reads beyond the end of the string, but
     * then masks off the part it's not allowed to read.  Because the
     * string is aligned, the masked-off tail is in the same word as the
     * rest of the string.  Every machine with memory protection I've seen
     * does it on word boundaries, so is OK with this.  But VALGRIND will
     * still catch it and complain.  The masking trick does make the hash
     * noticably faster for short strings (like English words).
     */
#ifndef VALGRIND

    switch(length)
    {
    case 12: c+=k[2]; b+=k[1]; a+=k[0]; break;
    case 11: c+=k[2]&0xffffff; b+=k[1]; a+=k[0]; break;
    case 10: c+=k[2]&0xffff; b+=k[1]; a+=k[0]; break;
    case 9 : c+=k[2]&0xff; b+=k[1]; a+=k[0]; break;
    case 8 : b+=k[1]; a+=k[0]; break;
    case 7 : b+=k[1]&0xffffff; a+=k[0]; break;
    case 6 : b+=k[1]&0xffff; a+=k[0]; break;
    case 5 : b+=k[1]&0xff; a+=k[0]; break;
    case 4 : a+=k[0]; break;
    case 3 : a+=k[0]&0xffffff; break;
    case 2 : a+=k[0]&0xffff; break;
    case 1 : a+=k[0]&0xff; break;
    case 0 : return c;              /* zero length strings require no mixing */
    }

#else /* make valgrind happy */

    k8 = (const uint8_t *)k;
    switch(length)
    {
    case 12: c+=k[2]; b+=k[1]; a+=k[0]; break;
    case 11: c+=((uint32_t)k8[10])<<16;  /* fall through */
    case 10: c+=((uint32_t)k8[9])<<8;    /* fall through */
    case 9 : c+=k8[8];                   /* fall through */
    case 8 : b+=k[1]; a+=k[0]; break;
    case 7 : b+=((uint32_t)k8[6])<<16;   /* fall through */
    case 6 : b+=((uint32_t)k8[5])<<8;    /* fall through */
    case 5 : b+=k8[4];                   /* fall through */
    case 4 : a+=k[0]; break;
    case 3 : a+=((uint32_t)k8[2])<<16;   /* fall through */
    case 2 : a+=((uint32_t)k8[1])<<8;    /* fall through */
    case 1 : a+=k8[0]; break;
    case 0 : return c;
    }

#endif /* !valgrind */

  } else if (HASH_LITTLE_ENDIAN && ((u.i & 0x1) == 0)) {
    const uint16_t *k = (const uint16_t *)key;         /* read 16-bit chunks */
    const uint8_t  *k8;

    /*--------------- all but last block: aligned reads and different mixing */
    while (length > 12)
    {
      a += k[0] + (((uint32_t)k[1])<<16);
      b += k[2] + (((uint32_t)k[3])<<16);
      c += k[4] + (((uint32_t)k[5])<<16);
      mix(a,b,c);
      length -= 12;
      k += 6;
    }

    /*----------------------------- handle the last (probably partial) block */
    k8 = (const uint8_t *)k;
    switch(length)
    {
    case 12: c+=k[4]+(((uint32_t)k[5])<<16);
             b+=k[2]+(((uint32_t)k[3])<<16);
             a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 11: c+=((uint32_t)k8[10])<<16;     /* fall through */
    case 10: c+=k[4];
             b+=k[2]+(((uint32_t)k[3])<<16);
             a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 9 : c+=k8[8];                      /* fall through */
    case 8 : b+=k[2]+(((uint32_t)k[3])<<16);
             a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 7 : b+=((uint32_t)k8[6])<<16;      /* fall through */
    case 6 : b+=k[2];
             a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 5 : b+=k8[4];                      /* fall through */
    case 4 : a+=k[0]+(((uint32_t)k[1])<<16);
             break;
    case 3 : a+=((uint32_t)k8[2])<<16;      /* fall through */
    case 2 : a+=k[0];
             break;
    case 1 : a+=k8[0];
             break;
    case 0 : return c;                     /* zero length requires no mixing */
    }

  } else {                        /* need to read the key one byte at a time */
    const uint8_t *k = (const uint8_t *)key;

    /*--------------- all but the last block: affect some 32 bits of (a,b,c) */
    while (length > 12)
    {
      a += k[0];
      a += ((uint32_t)k[1])<<8;
      a += ((uint32_t)k[2])<<16;
      a += ((uint32_t)k[3])<<24;
      b += k[4];
      b += ((uint32_t)k[5])<<8;
      b += ((uint32_t)k[6])<<16;
      b += ((uint32_t)k[7])<<24;
      c += k[8];
      c += ((uint32_t)k[9])<<8;
      c += ((uint32_t)k[10])<<16;
      c += ((uint32_t)k[11])<<24;
      mix(a,b,c);
      length -= 12;
      k += 12;
    }

    /*-------------------------------- last block: affect all 32 bits of (c) */
    switch(length)                   /* all the case statements fall through */
    {
    case 12: c+=((uint32_t)k[11])<<24;
    case 11: c+=((uint32_t)k[10])<<16;
    case 10: c+=((uint32_t)k[9])<<8;
    case 9 : c+=k[8];
    case 8 : b+=((uint32_t)k[7])<<24;
    case 7 : b+=((uint32_t)k[6])<<16;
    case 6 : b+=((uint32_t)k[5])<<8;
    case 5 : b+=k[4];
    case 4 : a+=((uint32_t)k[3])<<24;
    case 3 : a+=((uint32_t)k[2])<<16;
    case 2 : a+=((uint32_t)k[1])<<8;
    case 1 : a+=k[0];
             break;
    case 0 : return c;
    }
  }

  final(a,b,c);
  return c;
}

//uint32_t __stdcall NHASH(const void *key, size_t length, uint32_t initval)



//******************************************************************
//******************************************************************
void loop()
{
  if (TIFR5 & 1){                   //We have overlflow
    mult++;                         //Increment multiplier
    TIFR5 = (1<<TOV5);              //Clear overlow flag
  }
  if (RXflag == 1)
  {   
    if (digitalRead(wwvbPin) == LOW) wwvbStart() ;
    else
      TIMSK3 = 0; //Turn off timer3
    GPSprocess();
  }
}

//******************************************************************
// Determine time slot and load band frequency data. Display 
// frequency and calculate frequency word for DDS
void setfreq()
{
  for(i=0; i<10; i++)
  {
    digitalWrite(bandPin[i],LOW); // Clear all band select pins
  }
  digitalWrite(bandPin[timeslot],HIGH); // Turn corressponding band select pin ON

  // Print frequency to the LCD
  lcd.setCursor(0,0);

  ltoa((timeslot_array [timeslot] [0]),buf,10);
  
  if (timeslot_array [timeslot] [0] < 1000000)
    {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print('.');
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print(" KHz  ");         
    }
    
    if (timeslot_array [timeslot] [0] >= 1000000 && timeslot_array [timeslot] [0] < 10000000)
    {
    lcd.print(buf[0]);
    lcd.print(',');
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print('.');
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(" KHz ");
    }
    
    if (timeslot_array [timeslot] [0] >= 10000000)
    {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(',');
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print('.');
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(buf[7]);
    lcd.print(" KHz ");
   }


  FreqWord = ((timeslot_array[timeslot] [0])+(CalFactor.value*((timeslot_array[timeslot] [0])/pow(10,7))))*pow(2,32)/fCLK;

  lcd.setCursor(15,0);
  lcd.print(" "); 
  if(timeslot_array [timeslot] [3] == 1)
  {
    lcd.setCursor(15,0);
    lcd.print("*");
  } 
  return;
}


//******************************************************************
// Determine if it is time to transmit. If so, determine if it is
// time to transmit the QRSS or the WSPR message

void transmit()
{
  if(BandLockFlag == 1 && timeslot_array [timeslot] [3] == 0)return;
  else
    if(GPSinhibitFlag == 1 || InhibitFlag ==1){
      TIMSK3 = 0; //Turn off timer3
      TIMSK4 = 0; //Turn off timer4
      return;
    }
    else
      if(WWVBflag1 == 1 && WWVBflag2 != 1)return;
      else
        if(validGPSflag ==1)calcGridSquare();
  RXflag = 0; // Disable GPS/WWVB receiver
  // Set power level to default timeslot value
  if(digitalRead(div10Pin) == LOW){
    ndbm = (timeslot_array [timeslot] [2])-10; 
  }
  else
    if(digitalRead(mult10Pin) == LOW){
      ndbm = (timeslot_array [timeslot] [2])+10; 
    } 
    else
      ndbm = timeslot_array [timeslot] [2];
  wsprGenCode(); 
  if (timeslot_array [timeslot] [0] == timeslot_array [timeslot+1] [0]) timeslot_array [timeslot+1] [4] = !timeslot_array [timeslot+1] [4];
  timeslot_array [timeslot] [4] = !timeslot_array [timeslot] [4];
  screendata();
  if ((timeslot_array [timeslot] [1]) == 1)
  { // Start WSPR transmit process
    start = 0;
    QRSSflag = 0; // Disable QRSS process
    TIMSK3 = 0; //Turn off timer3
    TIMSK4 = 0; //Turn off timer4
    lcd.setCursor(0,3); 
    lcd.print("WSPR Transmit");
    digitalWrite(txPin,HIGH); // External transmit control ON
    digitalWrite(QRSStxPin,LOW); // External QRSS transmit control OFF
    bb =0;
    count = 0;            // Start WSPR symbol transmit process
    detachInterrupt(0);   // Disable rotary encoder when transmitting
    TIMSK4 = 2;           //Enable Timer4 interrupt 
    return;
  }
  else
    if ((timeslot_array [timeslot] [1]) == 2)
    { // Start QRSS transmit process
      lcd.setCursor(0,3); 
      lcd.print("QRSS Transmit"); 
      if(TIMSK4 == 2)return; // Do not disturb while transmitting patterns
      else
        digitalWrite(txPin,HIGH); // External transmit control ON
      digitalWrite(QRSStxPin,HIGH); // External QRSS transmit control ON
      TIMSK3 = 0; //Turn off timer3
      detachInterrupt(0);  // Disable rotary encoder when transmitting
      QRSSflag = 1;  // Enable QRSS process
      return;
    }
    else  
  { // Turn off transmitter and idle
    TIMSK4 = 0; //Turn off WSPR timer
    QRSSflag = 0; //Disable QRSS process
    RXflag = 1; //Enable GPS/WWVB receiver     
    start = 0;
    lcd.setCursor(0,3);      
    lcd.print("IDLE         ");
    digitalWrite(txPin,LOW); // External transmit control OFF
    digitalWrite(QRSStxPin,LOW); // External QRSS transmit control OFF
    attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
    TempWord=0;
    TransmitSymbol();
    return;
  }
}

//******************************************************************
void TransmitSymbol()
{
  for (mask = 1; mask>0; mask <<= 1) { //iterate through bit mask
    if (TempWord & mask){ // if bitwise AND resolves to true
      digitalWrite(DATA,HIGH); // send 1
    }
    else{ //if bitwise and resolves to false
      digitalWrite(DATA,LOW); // send 0
    }
    digitalWrite(CLOCK,HIGH); // Clock data in
    digitalWrite(CLOCK,LOW);
  }
  for (temp = 1; temp>0; temp <<= 1) { //iterate through bit mask
    if (w0 & temp){ // if bitwise AND resolves to true
      digitalWrite(DATA,HIGH); // send 1
    }
    else{ //if bitwise and resolves to false
      digitalWrite(DATA,LOW); // send 0
    } 
    digitalWrite(CLOCK,HIGH); // Clock data in
    digitalWrite(CLOCK,LOW);
  } 
  digitalWrite (LOAD, HIGH); // Turn on pin 36/1 LOAD FQ-UD
  digitalWrite (LOAD, LOW); // Turn pin 36 off
  return;
}


//******************************************************************
void doEncoder()
{
  ii++;
  if(ii>=12)ii=0;// Set up encoder rate of change
  if (digitalRead(encoder0PinA) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      if(ii == 0)(timeslot_array [timeslot] [0])--; //CCW
    } 
    else {
      if(ii == 0)(timeslot_array [timeslot] [0])++; //CW
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(encoder0PinB) == LOW) {   // check channel B to see which way
      // encoder is turning  
      if(ii == 0)(timeslot_array [timeslot] [0])++; //CW
    } 
    else {
      if(ii == 0)(timeslot_array [timeslot] [0])--; //CCW
    }
  }
  setfreq(); 
}

//******************************************************************
void toggle(int pinNum) {
  // set the LED pin using the pinState variable:
  digitalWrite(pinNum, pinState); 
  // if pinState = 0, set it to 1, and vice versa:
  pinState = !pinState;
}


/*_______________________________________________________________
 
 WWVB timing process starts here
 _______________________________________________________________
 */
void wwvbStart() {
  if(WWVBflag1==0)
  {
    lcd.setCursor(16,0);
    lcd.print("WWVB");
    lcd.setCursor(5,2);
    lcd.print("Pulse(mSec)=");
  }
  TIMSK3 = 2;   //Enable timer3 interrupt
  wwvbData = 0;
  WWVBflag1 = 1;
  bitcount=0;
  wwvbLoop() ;
}

void pulsedetect() {
  while (digitalRead(wwvbPin) == HIGH) {
  };
  if (countSync >= 2) process();
}

void wwvbLoop() {
  // Search for double sync pulse to start process:
  mSecTimer = 0;
  while (digitalRead(wwvbPin) == LOW) {
  };
  PulsePeriod = mSecTimer;
  lcd.setCursor(17,2);
  lcd.print(PulsePeriod);
  if (PulsePeriod > 780 && PulsePeriod < 820) {
    countSync++ ;
    if (countSync == 2){
      second = 1 ; // Set to top of minute plus  one sync pulse delay time
      pulsedetect(); 
    } 
  }
  else
  {
    countSync=0;
    TIMSK3 = 0; //Turn off timer3
    loop() ; // Timing error - start over
  }
}

void process() {
  // Double sync pulse found - continue with processing:
  mSecTimer = 0;
  while (digitalRead(wwvbPin) == LOW) {
  };
  PulsePeriod = mSecTimer;
  lcd.setCursor(17,2);
  lcd.print(PulsePeriod);
  bitcount++ ;
  if (bitcount == 10) pulsedetect() ;//Discard unwanted pulse
  if (PulsePeriod > 780 && PulsePeriod < 820){ 
    pulseWidth = 3; 
    pulse3() ; 
  }
  else
    if (PulsePeriod > 480 && PulsePeriod < 520) {
      wwvbData = wwvbData << 1;
      bitSet(wwvbData,0);
      pulsedetect(); // Get next bit 
    }
    else
      if (PulsePeriod > 180 && PulsePeriod < 220) {
        wwvbData = wwvbData << 1;
        bitClear(wwvbData,0);
        pulsedetect(); // Get next bit
      }
      else
      {
        TIMSK3 = 0; //Turn off timer3
        loop(); // Timing error - start over
      }
}

// Sync pulse detected 
void pulse3() {
  countSync++ ; 
  if (countSync < 4 ) pulsedetect();
  else
    if (countSync == 4)
    { 
      // If countSynce = 4 then hour and minute data is valid so update software clock 
      wwvbHour = 0 ;
      wwvbMin = 0 ;
      if (bitRead(wwvbData,0)==1) wwvbHour +=1 ;
      if (bitRead(wwvbData,1)==1) wwvbHour +=2 ;
      if (bitRead(wwvbData,2)==1) wwvbHour +=4 ;
      if (bitRead(wwvbData,3)==1) wwvbHour +=8 ;
      if (bitRead(wwvbData,5)==1) wwvbHour +=10 ;
      if (bitRead(wwvbData,6)==1) wwvbHour +=20 ;

      if (bitRead(wwvbData,8)==1) wwvbMin +=1 ;
      if (bitRead(wwvbData,9)==1) wwvbMin +=2 ;
      if (bitRead(wwvbData,10)==1) wwvbMin +=4 ;
      if (bitRead(wwvbData,11)==1) wwvbMin +=8 ;
      if (bitRead(wwvbData,13)==1) wwvbMin +=10 ;
      if (bitRead(wwvbData,14)==1) wwvbMin +=20 ;
      if (bitRead(wwvbData,15)==1) wwvbMin +=40 ;
    }
    else
    {
      if (countSync == 7) // 50 seconds of valid data required before time update
      {
        WWVBflag2 = 1;
        if (hour != wwvbHour) hour = wwvbHour ;
        if (minute != wwvbMin) minute = wwvbMin ;
        TIMSK3 = 0; //Turn off timer3
      }
    } 
  pulsedetect();
}

/*
_______________________________________________________________
 
 GPS timing process starts here
 _______________________________________________________________
 */
void GPSprocess()
{
  byte pinState = 0;
  byteGPS=Serial1.read();         // Read a byte of the serial port
  if (byteGPS == -1) {           // See if the port is empty yet
    delay(100); 
  } 
  else {
    buffer[counter]=byteGPS;        // If there is serial port data, it is put in the buffer
    counter++;                      
    if (byteGPS==13){            // If the received byte is = to 13, end of transmission
      IndiceCount=0;
      StartCount=0;
      for (int i=1;i<7;i++){     // Verifies if the received command starts with $GPGGA
        if (buffer[i]==StartCommand[i-1]){
          StartCount++;
        }
      }
      if(StartCount==6){      // If yes, continue and process the data
        for (int i=0;i<300;i++){
          if (buffer[i]==','){    // check for the position of the  "," separator
            indices[IndiceCount]=i;
            IndiceCount++;
          }
          if (buffer[i]=='*'){    // ... and the "*"
            indices[12]=i;
            IndiceCount++;
          }
        }
        // Load time data
        temp = indices[0];
        hour = (buffer[temp+1]-48)*10 + buffer[temp+2]-48;
        minute = (buffer[temp+3]-48)*10 + buffer[temp+4]-48;
        second = (buffer[temp+5]-48)*10 + buffer[temp+6]-48; 
        // Load latitude and logitude data          
        temp = indices[1];
        Lat10 = buffer[temp+1]-48;
        Lat1 = buffer[temp+2]-48;
        dLat1 = buffer[temp+3]-48;
        dLat2 = buffer[temp+4]-48;
        temp = indices[2];
        NS = buffer[temp+1];
        temp = indices[3];
        Lon100 = buffer[temp+1]-48;
        Lon10 = buffer[temp+2]-48;
        Lon1 = buffer[temp+3]-48; 
        dLon1 = buffer[temp+4]-48;
        dLon2 = buffer[temp+5]-48;        
        temp = indices[4];
        EW = buffer[temp+1];
        temp = indices[5];
        validGPSflag = buffer[temp+1]-48;
        temp = indices[6];
        sat10 = buffer[temp+1]-48;
        sat1 = buffer[temp+2]-48;        
      }

      else
      {
        IndiceCount=0;
        StartCount=0;
        for (int i=1;i<7;i++){     // Verifies if the received command starts with $GPRMC
          if (buffer[i]==StartCommand2[i-1]){
            StartCount++;
          }
        }       
        if(StartCount==6){      // If yes, continue and process the data
          for (int i=0;i<300;i++){
            if (buffer[i]==','){    // check for the position of the  "," separator
              indices[IndiceCount]=i;
              IndiceCount++;
            }
            if (buffer[i]=='*'){    // ... and the "*"
              indices[12]=i;
              IndiceCount++;
            }
          }
          // Load time data
          temp = indices[0];
          hour = (buffer[temp+1]-48)*10 + buffer[temp+2]-48;
          minute = (buffer[temp+3]-48)*10 + buffer[temp+4]-48;
          second = (buffer[temp+5]-48)*10 + buffer[temp+6]-48;
          temp = indices[1]; 
          if (buffer[temp+1] == 65){
            validGPSflag = 1; 
          }
          else
          {
            validGPSflag = 0;
          }        
          // Load latitude and logitude data          
          temp = indices[2];
          Lat10 = buffer[temp+1]-48;
          Lat1 = buffer[temp+2]-48;
          dLat1 = buffer[temp+3]-48;
          dLat2 = buffer[temp+4]-48;
          temp = indices[3];
          NS = buffer[temp+1];
          temp = indices[4];
          Lon100 = buffer[temp+1]-48;
          Lon10 = buffer[temp+2]-48;
          Lon1 = buffer[temp+3]-48;
          dLon1 = buffer[temp+4]-48;
          dLon2 = buffer[temp+5]-48;         
          temp = indices[5];
          EW = buffer[temp+1];
        }
      }

      if(validGPSflag ==1)GPSinhibitFlag = 0;
      else
      {
        GPSinhibitFlag = 1;
      }
      counter=0;                  // Reset the buffer
      for (int i=0;i<300;i++){    //  
        buffer[i]=' '; 
      }
    }
  }
}


//******************************************************************
void displaytime()
{
  lcd.setCursor(0,1);
  if (hour < 10) lcd.print ("0");
  lcd.print (hour);
  lcd.print (":");
  if (minute < 10) lcd.print ("0");
  lcd.print (minute);
  lcd.print (":");
  if (second < 10) lcd.print ("0");
  lcd.print (second);
  lcd.print (" "); 
  return;  
}

//******************************************************************
void DisplayFreq()
{
  // Print frequency to the LCD
  lcd.setCursor(0,0);

  ltoa(TempFreq,buf,10);
  
    if (TempFreq < 1000000)
    {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print('.');
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print(" KHz  ");         
    }
    
    if (TempFreq >= 1000000 && TempFreq < 10000000)
    {
    lcd.print(buf[0]);
    lcd.print(',');
    lcd.print(buf[1]);
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print('.');
    lcd.print(buf[4]);
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(" KHz ");
    }
    
    if (TempFreq >= 10000000)
    {
    lcd.print(buf[0]);
    lcd.print(buf[1]);
    lcd.print(',');
    lcd.print(buf[2]);
    lcd.print(buf[3]);
    lcd.print(buf[4]);
    lcd.print('.');
    lcd.print(buf[5]);
    lcd.print(buf[6]);
    lcd.print(buf[7]);
    lcd.print(" KHz ");
   }
}

//******************************************************************
void Dah()
{
  TempFreq = QRSSfreq + QRSSshift;
  TempWord = TempFreq*pow(2,32)/180000000;
  DisplayFreq();
  TempWord = (TempFreq+(CalFactor.value*(TempFreq/pow(10,7))))*pow(2,32)/fCLK;
  TransmitSymbol();
  QRSScount++;
  if(QRSScount > (QRSSdot*3))
  {
    TempFreq = QRSSfreq;
    TempWord = TempFreq*pow(2,32)/180000000;
    DisplayFreq();
    TempWord = (TempFreq+(CalFactor.value*(TempFreq/pow(10,7))))*pow(2,32)/fCLK;
    TransmitSymbol();
    SpaceCount++;
    if(SpaceCount > QRSSdot)
    {
      QRSScount=0;
      MorseBit++;
      SpaceCount=0;
    }
  }
  if(MorseBit > MorseBitCount-1)
  {
    MorseBit=0;
    MorseChar++;
    CharFlag=1;
  }
  return;
}

//******************************************************************
void Dit()
{
  TempFreq = QRSSfreq + QRSSshift;
  TempWord = TempFreq*pow(2,32)/180000000;
  DisplayFreq();
  TempWord = (TempFreq+(CalFactor.value*(TempFreq/pow(10,7))))*pow(2,32)/fCLK;
  TransmitSymbol();
  QRSScount++;
  if(QRSScount > QRSSdot)
  {
    TempFreq = QRSSfreq;
    TempWord = TempFreq*pow(2,32)/180000000;
    DisplayFreq();
    TempWord = (TempFreq+(CalFactor.value*(TempFreq/pow(10,7))))*pow(2,32)/fCLK;
    TransmitSymbol();
    SpaceCount++;
    if(SpaceCount > QRSSdot)
    {
      QRSScount=0;
      MorseBit++;
      SpaceCount=0;
    }
  }
  if(MorseBit > MorseBitCount-1)
  {
    MorseBit=0;
    MorseChar++;
    CharFlag=1;
  }
  return;
}

//******************************************************************
void CharSpace()
{
  TempFreq = QRSSfreq;
  TempWord = TempFreq*pow(2,32)/180000000;
  DisplayFreq();
  TempWord = (TempFreq+(CalFactor.value*(TempFreq/pow(10,7))))*pow(2,32)/fCLK;
  TransmitSymbol();
  CharFlag++;
  if(CharFlag > QRSSdot*2)
  {
    CharFlag=0;
    MorseBit=0;
    QRSScount=0;
    SpaceCount=0;
  }
  return;
}

//******************************************************************
void WordSpace()
{
  TempFreq = QRSSfreq;
  TempWord = TempFreq*pow(2,32)/180000000;
  DisplayFreq();
  TempWord = (TempFreq+(CalFactor.value*(TempFreq/pow(10,7))))*pow(2,32)/fCLK;
  TransmitSymbol();
  CharFlag=0;
  WordSpaceFlag++;
  if(WordSpaceFlag > (QRSSdot*6))
  {
    MorseChar++;
    WordSpaceFlag=0;
    QRSScount=0;
    MorseBit=0;
  }
  return;
}

//******************************************************************
void TXinhibit()
{
  for(TempWord=0; TempWord < 750000; TempWord++) {
  }; //crude debounce delay
  InhibitFlag = !InhibitFlag;
  if(InhibitFlag == 1) // 1 turns OFF transmitter
  {
    lcd.setCursor(0,3);
    lcd.print("TX OFF       ");
    TIMSK3 = 0; //Turn off WSPR timer
    QRSSflag = 0; //Disable QRSS process
    TIMSK4 = 0; // Turn patterns off
    start = 0;
    digitalWrite(txPin,LOW); // External transmit control OFF
    digitalWrite(QRSStxPin,LOW); // External QRSS transmit control OFF
    attachInterrupt(2, BandChange, LOW);  // encoder pin on interrupt 2 - pin 21
    RXflag = 1; //Enable GPS/WWVB receiver
    TempWord=0; //Set DDS60 to 0Hz
    TransmitSymbol();
    return;      
  }
  else
    detachInterrupt(2);   // Disable band change interrupt
  lcd.setCursor(0,3);      
  lcd.print("Standby      ");
}

//******************************************************************
void BandChange()
{
  timeslot++;
  if(timeslot > 9) timeslot = 0;
  for(i=0; i<10; i++)
  {
    digitalWrite(bandPin[i],LOW); // Clear all band select pins
  }
  digitalWrite(bandPin[timeslot],HIGH); // Turn corressponding band select pin ON
  setfreq();
  for(TempWord=0; TempWord < 300000; TempWord++) {
    if(digitalRead(CalSetButton) == LOW)
    {
      if(timeslot_array [timeslot] [3] == 1) {
        timeslot_array [timeslot] [3] = 0;
        lcd.setCursor(15,0); 
        lcd.write(42);
        return; 
      }
      else
        BandLockFlag = 1;
      lcd.setCursor(15,0);      
      lcd.write(42);
      BandLockSlot = timeslot;
      digitalWrite(bandPin[timeslot],HIGH); // Turn corressponding band select pin ON
      timeslot_array [timeslot] [3] = 1; //Set transmit flag for selected band  
      timeslot_array [timeslot] [1] = 1; //Force selected band to WSPR mode   
      return;
    } 
  }
  return;
}

//******************************************************************
void screendata()
{
  lcd.clear();
  setfreq();
  lcd.setCursor(16,0);
  if(validGPSflag ==1)lcd.print("GPS ");
  else
    if(WWVBflag1 ==1) lcd.print("WWVB");
    else
      lcd.print("INT ");

  // Display callsign on LCD
  lcd.setCursor(10,1);
  lcd.print(call2);

  // Display "Standby" on LCD
  lcd.setCursor(0,3);
  lcd.print("Standby      ");

  // Calculate and display power level on LCD
  milliwatts = pow(10,(float(ndbm+0.015)/10));
  lcd.setCursor(14,3);
  if(milliwatts >= 1000)
  {
    milliwatts = milliwatts/1000;
    lcd.print(milliwatts);
    lcd.print(" W");
  }
  else
  {
    lcd.print(milliwatts);
    lcd.print(" mW");
  }

  // Display locator on LCD
  lcd.setCursor(0,2);
  lcd.print(locator);  

  // Display GPS data on LCD
  if(validGPSflag==1)
  {
    lcd.setCursor(7,2);
    lcd.print(Lat10);
    lcd.print(Lat1);
    lcd.write(NS);
    lcd.print(" ");
    lcd.print(Lon100);
    lcd.print(Lon10);
    lcd.print(Lon1);
    lcd.write(EW);
    lcd.setCursor(15,2);
    lcd.print(" s=");
    lcd.print(sat10);
    lcd.print(sat1);
  }

  if(WWVBflag1 ==1)
  {
    lcd.setCursor(5,2);
    lcd.print("Pulse(mSec)=0");
  }   
}

//******************************************************************
void calcGridSquare()
{
  unsigned long latitude, longitude;
  float temp3,tempLat,tempLong;
  longitude = Lon100*100000000L+Lon10*10000000L+Lon1*1000000L+dLon1*100000L+dLon2*10000L;
  latitude = Lat10*10000000L+Lat1*1000000L+dLat1*100000L+dLat2*10000L;
  tempLong=longitude;
  tempLong=1000000*int(tempLong/1000000)+ ((tempLong-1000000*int(tempLong/1000000))/0.6); 
  if (EW == 69)tempLong=(tempLong)+180000000;
  if (EW == 87)tempLong=180000000-(tempLong);
  tempLat=latitude;
  tempLat=1000000*int(tempLat/1000000)+((tempLat-1000000*int(tempLat/1000000))/0.6);   
  if (NS==78)tempLat=tempLat+90000000;
  if (NS==83)tempLat=90000000-tempLat;
  GPSlocator[0]=(65+int(tempLong/20000000));
  GPSlocator[1]=(65+int(tempLat/10000000));
  temp3=tempLong-(20000000*int(tempLong/20000000)); 
  GPSlocator[2]=(48+int(temp3*10/20/1000000)); 
  temp3=tempLat-(10000000*int(tempLat/10000000));  
  GPSlocator[3]=(48+int(temp3/1000000)); 
  temp3=(tempLong/2000000)-(int(tempLong/2000000));
  GPSlocator[4]=(65+int(temp3*24));
  temp3=(tempLat/1000000)-(int(tempLat/1000000));
  GPSlocator[5]=(65+int(temp3*24));
  if(strcmp(locator, GPSlocator)  != 0)  
  {
   for (int i = 0; i < 7; i++){
   locator[i] = GPSlocator[i];
   }
  }
}
//******************************************************************
void timeset()
{ // Process to manually set master time clock
  lcd.setCursor(0,2);
  lcd.print("SetTime  ");
  while(digitalRead(TimeSetPin) == LOW)
  {
    if (digitalRead(MinuteSetPin) == LOW)
    {
      for(TempWord=0; TempWord < 500000; TempWord++) {
      }; //crude debounce/time delay
      minute++;
      if(minute > 59) minute = 0;
      displaytime();
    };
    if (digitalRead(HourSetPin) == LOW)
    {
      for(TempWord=0; TempWord < 500000; TempWord++) {
      }; //crude debounce/time delay
      hour++;
      if(hour > 23) hour = 0;
      displaytime();
    };
  }

  TempWord = 0;
  TransmitSymbol();
  lcd.setCursor(0,2);
  lcd.print("         ");
  TIMSK1 = 2;          // Enable Timer1 Interrupt 
}

//******************************************************************
void calibrate()
{ // Process to determine frequency calibration factor
  while(digitalRead(CalSetButton) == LOW)
  {
    if (digitalRead(CalUpButton) == LOW)
    {
      for(TempWord=0; TempWord < 350000; TempWord++) {
      }; //crude debounce delay
      CalFactor.value++;
    };
    if (digitalRead(CalDwnButton) == LOW)
    {
      for(TempWord=0; TempWord < 350000; TempWord++) {
      }; //crude debounce delay
      CalFactor.value--;
    };
    TempWord = (TenMHz+CalFactor.value)*pow(2,32)/fCLK;
    TransmitSymbol();
    lcd.setCursor(12,1);
    lcd.print(CalFactor.value);
    lcd.print("   ");
  }

  // Writes CalFactor to address 50 + 3 bytes of EEprom
  EEPROM.write(50,CalFactor.b1);
  EEPROM.write(51,CalFactor.b2);
  EEPROM.write(52,CalFactor.b3);
  EEPROM.write(53,CalFactor.b4);

  lcd.setCursor(9,1);
  lcd.print("IDLE   ");
  TempWord = 0; // turn off 10 MHz calibrate signal
  TransmitSymbol();
  setfreq();  
  TIMSK1 = 2;          // Enable Timer1 Interrupt 
  screendata();
}   


//******************************************************************
// Clock - GPS 1PPS interrupt routine used as master timekeeper
// Called every second by GPS 1PPS on pin 20
// This interrupt is also used for QRSS timing
void PPSinterrupt()
{
  second++ ;
  if (second == 54 && (bitRead(minute,0) == 1)) 
  {
    TCCR5B = 7;                          //Clock on rising edge of pin 47
  }
  if (second == 58 && bitRead(minute,0) == 1) 
  {     
    TCCR5B = 0;                          //Turn off counter
    fCLK = ((mult * 0x10000 + TCNT5)*12);  //Calculate frequency
    TCNT5 = 0;                           //Reset count to zero
    mult = 0;                            //Reset mult to one
    RXflag = 1; //Enable GPS receiver
  }
  if (second == 53 && (bitRead(minute,0) == 1)) 
  {
    RXflag = 0; //Disable GPS receiver
  }
  TIMSK1 = 0;        //Disable Interrupt for internal master clock
  CalFactor.value = 0;
  if (second == 60) {
    minute++ ;
    second=0 ;
  }
  if (minute == 60) {
    hour++;
    minute=0 ;
  }
  if (hour == 24) {
    hour=0 ;
  }
  displaytime();
  if(InhibitFlag == 0)
  {
    if(bitRead(minute,0) == 0 & second == 0) 
    {
      if (minute < 20) {
        timeslot = minute/2;
      }
      else{
        if (minute < 40) {
          timeslot = (minute-20)/2;
        }
        else {
          timeslot = (minute -40)/2;
        }
      }
      setfreq();
      transmit();
    }

    
 //QRSS transmit routine begins here. 
 
    if(QRSSflag == 1)
    {
      if(start < 4)
      { // Transmit QRSS base frequency for four seconds to start QRSS transmission
        TempFreq = QRSSfreq;
        TempWord = TempFreq*pow(2,32)/180000000;
        DisplayFreq();
        TempWord = (TempFreq+(CalFactor.value*(TempFreq/pow(10,7))))*pow(2,32)/fCLK;
        TransmitSymbol();
        start++;
        Pattern = 0; //Default ot ID repeat when in GSP calibrate mode to avoid timer conflicts.
        QRSScount=0;
        MorseChar=0;
        MorseBit=0;
        CharFlag=0; 
      }
      else
      {
        MsgLength = (strlen(QRSSmsg));
        if (MorseChar>MsgLength-1)
        {
          if(Pattern >= 1)
          {
            i=0;
            ii=0;
            j=0;
            wavecount = 0;          
            if(Pattern == 1) wavecount = 270;//Offset to start on mark frequency
            QRSSflag = 0;      //Disable QRSS process
            TIMSK4 = 2;        //Timer4 enabled
          }
          else
            QRSScount=0;
          MorseChar=0;
          MorseBit=0;
          CharFlag=0;
        }
        else
          if((QRSSmsg[MorseChar] >= 97) & (QRSSmsg[MorseChar] <= 122)) 
          {
            temp = QRSSmsg[MorseChar]-87;          
          }
          else
            if((QRSSmsg[MorseChar] >= 65) & (QRSSmsg[MorseChar] <= 90))
            { 
              temp = QRSSmsg[MorseChar] - 55;  
            }
            else
              temp = QRSSmsg[MorseChar] - 48; 
        ByteMask = B00000111;
        MorseBitCount = ByteMask & Morse[temp]; 
        if(QRSSmsg[MorseChar] == 32)
        {
          WordSpace(); 
        }
        else
          if(bitRead(Morse[temp],(7-MorseBit)) == HIGH)
          {
            Dah();
          }
          else
          {
            Dit();
          }
        if (CharFlag >= 1)
        {
          CharSpace(); 
        }
      }
    }
  }
  else{
  }
};































































