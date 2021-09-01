/*
Arduino Uno/Nano signal generator
 A VFO project that uses an Arduino Uno or Nano to control a AD9850 
 Direct Digital synthesis board. The VFO may be used in stand-alone mode 
 or with a GPS receiver to software calibrate the DDS clock. 
 
 Copyright (C) 2013,  Gene Marcus W3PM GM4YRE
 
 25 August  2013
 
 Permission is granted to use, copy, modify, and distribute this software
 and documentation for non-commercial purposes.
 
 ------------------------------------------------------------------------
 Uno/Nano Digital Pin Allocation
 D0  GPS RX (this pin may differ on the Nano board)
 D1 
 D2  GPS 1pps input 
 D3  Rotary encoder pin A
 D4  Rotary encoder pin B
 D5  Freq counter input from divide by 32
 D6  DDS Clock
 D7  DDS FU_UD(Load)
 D8  DDS data
 D9  Resolution select button
 D10 Decrease frequency button
 D11 Increase frequency button
 D12 offset enable button
 D13 not used
 A0/D14 LCD D7
 A1/D15 LCD D6
 A2/D16 LCD D5
 A3/D17 LCD D4
 A4/D18 LCD enable
 A5/D19 LCD RS 
 
 ------------------------------------------------------------
 */


//_________________________Enter start frequency Hz below:____________________________________
uint64_t frequency = 10000000; 

//_________________________Enter clock frequency (Hz) below:___________________________________
uint64_t fClk = 125000000;

//_________________________Enter offset frequency (Hz) below:__________________________________
int fOffset = -600;

//_________________________GPS 1pps calibrate flag (0 = NO, 1 = YES)___________________________
int GPSflag = 1;

//_________________________If GPSflag = 0 Enter calibration factor below:______________________
// Connect RF output to frequency counter; adjust for 10 MHz on counter; subtract 10 MHz
// from the dial frequency and enter difference.
int fCal = -113;

//_________________________Select AD9850 or AD9851 board below:_________________________________
//                                (0 = AD9850, 1 = AD9851)
byte w0 = 0;              


// include the library code:
#include <LiquidCrystal.h>
#include <string.h>
#include <ctype.h>
#include <avr/interrupt.h>  
#include <avr/io.h>

// Set up MCU pins
#define ppsPin                   2 // External interrupt 0
#define encoderPinA              3 
#define encoderPinB              4
#define CLOCK                    6 // DDS
#define LOAD                     7 // DDS FQ_UD 
#define DATA                     8 // DDS
#define Resolution               9
#define FreqDown                10
#define FreqUp                  11
#define Offset                  12
#define DB7                     14 // A0
#define DB6                     15 // A1
#define DB5                     16 // A2
#define DB4                     17 // A3
#define E                       18 // A4
#define RS                      19 // A5


// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);

// configure variables 
byte temp=1,fStepcount,offsetFlag=0,cf;
byte pinState,encoderA,encoderB,encoderC;
unsigned long mask,debounce,time,fStep=1;
uint64_t two_e32 = pow(2,32);
uint64_t TempWord,TempFreq,FreqWord,freqcount;
volatile byte i,ii;
volatile boolean triggered;
String resolution = "1 Hz  ";
char buffer[300] = "",locator[7];
char GPSlocator[7],StartCommand[7] = "$GPGGA",StartCommand2[7] = "$GPRMC",buf[10];
int IndiceCount=0,StartCount=0,counter=0,indices[13];
int Lat10,Lat1,NS,Lon100,Lon10,Lon1,EW,validGPSflag;
int dLon1,dLon2,dLat1,dLat2,dlon,dlat,lon,lat;
int mult=0,byteGPS=-1,tcount,second=0,minute=0,hour=0,sat1,sat10;
int divideCount;

//******************************************************************
// Clock - interrupt routine used as master timekeeper
// Called every second by GPS 1PPS on MCU pin 2
void PPSinterrupt()
{
  tcount++;
  if (tcount == 4)
  {
    TCCR1B = 7;                          //Clock on rising edge of pin 5
    loop();
  }
  if (tcount == divideCount)
  {     
    TCCR1B = 0;                          //Turn off counter
    freqcount = mult * 0x10000 + TCNT1 - cf;  //Calculate frequency
    TCNT1 = 0;                           //Reset count to zero
    mult = 0;
    setfreq();
    if (GPSflag == 0) TempWord = (frequency+(fCal*(frequency/pow(10,7))))*pow(2,32)/fClk;
    else TempWord = frequency*two_e32/freqcount;
    updateDDS();
    tcount = 0;
  }
  if(validGPSflag == 1)
  {
    second++;
    if (second == 60) 
    {
      minute++ ;
      second=0 ;
    }
    if (minute == 60) 
    {
      hour++;
      minute=0 ;
    }
    if (hour == 24) hour=0 ;
    lcd.setCursor(8,1);
    if (hour < 10) lcd.print ("0");
    lcd.print (hour);
    lcd.print (":");
    if (minute < 10) lcd.print ("0");
    lcd.print (minute);
    lcd.print (":");
    if (second < 10) lcd.print ("0");
    lcd.print (second);
    lcd.print (" ");  
    if((millis() > time)&&(millis()<= time+1000))
    {
      lcd.setCursor(0,1);
      lcd.print(locator);
    }
  }
}

ISR(TIMER1_OVF_vect) 
{
  mult++;                         //Increment multiplier
  TIFR1 = (1<<TOV1);              //Clear overlow flag 
}

void setup()
{
  //Set up Timer1 as a frequency counter - input at pin 5
  TCCR1B = 0;        //Disable Timer5 during setup
  TCCR1A = 0;        //Reset
  TCNT1  = 0;        //Reset counter to zero
  TIFR1  = 1;        //Reset overflow
  TIMSK1 = 1;        //Turn on overflow flag

  // Inititalize GPS 1pps input
  pinMode(ppsPin, INPUT);

  // Set 1PPS pin 2 for external interrupt input
  attachInterrupt(0, PPSinterrupt, RISING);   

  // set up the LCD's number of columns and rows 
  lcd.begin(16,2); 

  // Set up rotary encoder
  pinMode(encoderPinA, INPUT);
  digitalWrite(encoderPinA, HIGH); // internal pull-up enabled 
  pinMode(encoderPinB, INPUT);
  digitalWrite(encoderPinB, HIGH); // internal pull-up enabled  

  // Set up DDS board
  pinMode (DATA, OUTPUT);   
  pinMode (CLOCK, OUTPUT);  
  pinMode (LOAD, OUTPUT); 

  // Set up push buttons
  pinMode(Resolution, INPUT);
  digitalWrite(Resolution, HIGH); // internal pull-up enabled
  pinMode(FreqUp, INPUT);
  digitalWrite(FreqUp, HIGH);     // internal pull-up enabled
  pinMode(FreqDown, INPUT);
  digitalWrite(FreqDown, HIGH);   // internal pull-up enabled
  pinMode(Offset, INPUT);
  digitalWrite(Offset, HIGH);     // internal pull-up enabled

  // Set up divide count for DDS type. Note: divide count begins at 4
  if (w0 == 1)
  {
    divideCount = 52;
    cf = 4;
  }
  else 
  {
    divideCount = 36;
    cf = 3;
  }

  Serial.begin(4800);        // connect to the serial port
  lcd.display(); 
  lcd.setCursor(0,1);
  if (GPSflag == 1)
  {
    lcd.print("Waiting for GPS");
  }
  else
  {
    lcd.print("Step:");
    lcd.setCursor(6,1);
    lcd.print(resolution);
  } 
  TCCR1B = 0;
  setfreq();

  if (GPSflag == 0) TempWord = (frequency+(fCal*(frequency/pow(10,7))))*pow(2,32)/fClk;
  else TempWord = 0;   
  updateDDS();

}

//******************************************************************
//******************************************************************
void loop()
{  
  if(GPSflag == 1 && validGPSflag == 0)GPSprocess( );
  else
  {
    encoderA = digitalRead(encoderPinA); 
    encoderB = digitalRead(encoderPinB);
    if ((encoderA == HIGH) && (encoderC == LOW))
    {
      if (encoderB == LOW)
      {
        frequency-=fStep;
      }
      else
      {
        frequency+=fStep;
      }
      setfreq();
      if (GPSflag == 0) lcd.setCursor(6,1);
      else lcd.setCursor(0,1);
      lcd.print(resolution);
      time = millis()+10000;
    }
    encoderC = encoderA;
    if(digitalRead(Resolution) == LOW)
    {
      for(debounce=0; debounce < 400000; debounce++) { 
      };
      fStepcount++;
      if(fStepcount>6)fStepcount=0;
      ResDisplay();
    }

    if(digitalRead(FreqUp) == LOW)
    {
      for(debounce=0; debounce < 400000; debounce++) { 
      }; 
      frequencyUp();
    }

    if(digitalRead(FreqDown) == LOW)
    {
      for(debounce=0; debounce < 400000; debounce++) { 
      }; 
      frequencyDown();
    }
    if(digitalRead(Offset) == LOW && offsetFlag == 0)
    {
      for(debounce=0; debounce < 40000; debounce++) { 
      }; 
      offsetFlag = 1;
      frequency += fOffset;
      lcd.setCursor(15,0);
      lcd.print("*");
      setfreq();
    }
    if(digitalRead(Offset) == HIGH && offsetFlag == 1)
    {
      for(debounce=0; debounce < 40000; debounce++) { 
      };
      offsetFlag = 0; 
      frequency -= fOffset;
      lcd.setCursor(15,0);
      lcd.print(" ");
      setfreq();
    }
  }
}


//******************************************************************
// Display frequency and calculate frequency word for DDS
void setfreq()
{
  // Print frequency to the LCD
  lcd.setCursor(0,0);

  ltoa(frequency,buf,10);

  if (frequency < 1000000)
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

  if (frequency >= 1000000 && frequency < 10000000)
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

  if (frequency >= 10000000)
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
  if (GPSflag == 0) TempWord = (frequency+(fCal*(frequency/pow(10,7))))*pow(2,32)/fClk;  
  else TempWord = frequency*two_e32/freqcount;
  updateDDS();
}

//******************************************************************
void updateDDS()
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
void ResDisplay()
{
  switch(fStepcount)
  {
  case 0:
    fStep=1;
    resolution = "1 Hz  ";
    break;
  case 1:
    fStep=10;
    resolution = "10 Hz  ";
    break;
  case 2:
    fStep=100;
    resolution = "100 Hz ";
    break;
  case 3:
    fStep=1000;
    resolution = "1 KHz  ";
    break;
  case 4:
    fStep=10000;
    resolution = "10 KHz ";
    break;
  case 5:
    fStep=100000;
    resolution = "100 KHz";
    break;
  case 6:
    fStep=1000000;
    resolution = "1 MHz  ";
    break;
  }
  if (GPSflag == 0) lcd.setCursor(6,1);
  else lcd.setCursor(0,1);
  lcd.print(resolution);
  time = millis()+10000;
}

//******************************************************************
void frequencyUp()
{
  frequency += fStep;
  setfreq();
  if (GPSflag == 0) lcd.setCursor(6,1);
  else lcd.setCursor(0,1);
  lcd.print(resolution);
  time = millis()+10000;
}

//******************************************************************
void frequencyDown()
{   
  frequency -= fStep;
  setfreq();
  if (GPSflag == 0) lcd.setCursor(6,1);
  else lcd.setCursor(0,1);
  lcd.print(resolution);
  time = millis()+10000;
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
  GPSlocator[4]=(97+int(temp3*24));
  temp3=(tempLat/1000000)-(int(tempLat/1000000));
  GPSlocator[5]=(97+int(temp3*24));
  if(strcmp(locator, GPSlocator)  != 0)  
  {
    for (int i = 0; i < 7; i++){
      locator[i] = GPSlocator[i];
    }
  }
}


/*
_______________________________________________________________
 
 GPS NMEA processing starts here
 _______________________________________________________________
 */
void GPSprocess()
{
  byte pinState = 0;
  byteGPS=Serial.read();         // Read a byte of the serial port
  if (byteGPS == -1) {           // See if the port is empty yet
    delay(100); 
  } 
  else { // NMEA $GPGGA data begins here
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
        // Load data
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
      { // NMEA $GPRMC data begins here
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
      if(validGPSflag == 1) 
      {
        calcGridSquare();
        lcd.setCursor(0,1);
        lcd.print(locator);
        lcd.print("  ");
      }
      counter=0;                  // Reset the buffer
      for (int i=0;i<300;i++){    //  
        buffer[i]=' '; 
      }
    }
  }
}












