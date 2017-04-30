#include <SPI.h>
#include <Wire.h>
#include "TimerOne.h"


uint8_t asciiTable[128];            

uint8_t digTable[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66,
            0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x80};



const int pinRCK = 10;
const int pinMCRCK = A3;

const int pinDig[] = {9, 8, 7, 6, 5, 4, 3, 2};

uint64_t lastRTCPollTime=0;


const int butDownPin = A0;
const int butUpPin = A2;
const int butLeftPin = A1;
const int butRightPin = 12;

//const int pinLed = 13;

uint64_t lastBlinkTime=0;
int digitBlinkInd = -1;

byte mcVal = 0x01;

void setup() {
  fillAsciiTable();

  
  pinMode(pinRCK, OUTPUT); //RCK
  digitalWrite(pinRCK, LOW);    // turn the LED off by making the voltage LOW

  pinMode(pinMCRCK, OUTPUT); //RCK for Motro Control Shift Reg
  digitalWrite(pinMCRCK, LOW);    // turn the LED off by making the voltage LOW
  
  //pinMode(pinLed, OUTPUT);
  //digitalWrite(pinLed, HIGH);    // turn the LED off by making the voltage LOW
  //pinMode(PC6, OUTPUT); //RCK for Motro Control Shift Reg 
//  digitalWrite(pinDebug, LOW);    // turn the LED off by making the voltage LOW

  pinMode(butDownPin, INPUT_PULLUP); //RCK    
  pinMode(butUpPin, INPUT_PULLUP); //RCK    
  pinMode(butLeftPin, INPUT_PULLUP); //RCK    
  pinMode(butRightPin, INPUT_PULLUP); //RCK    

  for(int i=0; i<8; i++){
    pinMode(pinDig[i], OUTPUT); //RCK
    digitalWrite(pinDig[i], LOW);    // turn the LED off by making the voltage LOW
  }

  SPSR = 0x01;
  SPI.begin();
  Wire.begin();    
  
//  Wire.beginTransmission(0x68); // transmit to device #9
//  Wire.write(0x04); 
//  Wire.write(0x7);   
//  Wire.endTransmission(); // stop transmitting
//  Wire.beginTransmission(0x68); // transmit to device #9
//  Wire.write(0x03); 
//  Wire.write(0x2);   
//  Wire.endTransmission(); // stop transmitting

//  Wire.beginTransmission(0x68); // transmit to device #9
//  Wire.write(0x01); 
//  Wire.write(0x25);   
//  Wire.endTransmission(); // stop transmitting
//
//  Wire.beginTransmission(0x68); // transmit to device #9
//  Wire.write(0x02); 
//  Wire.write(0x23);   
//  Wire.endTransmission(); // stop transmitting
  
  
  Serial.begin(19200);  // start serial for output

  Timer1.initialize(10);     
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
}

byte counter; 

volatile byte motor1Val = 0x4;
int motor1Cnt = 0;
void callback()
{   
  //PORTC |= 0x08;
  //SPDR = mcVal;
  
  counter++;  
  if(motor1Cnt == 0){
    //mcVal = 0x01;  
    SPDR = 0x01;
    motor1Cnt = motor1Val;
  }   
  else{
    motor1Cnt--;                    
  }

  //SPI.transfer(mcVal);
  
  while (!(SPSR & _BV(SPIF))) ; // wait     
  PORTC |= 0x08;
  PORTC &= ~0x08;
  
  SPDR = 0x0;
  while (!(SPSR & _BV(SPIF))) ; // wait      
  PORTC |= 0x08;
  PORTC &= ~0x08;
  
  
}

typedef enum{
  unknownState,
  powerUpState,
  idleState,
  initState,
  disableState,
  acceptingState,
  stackingState,
  rejectingState,
} TCashCodeState;

typedef enum{
  idleExchState,
  readState0,
  readState1,
  readDataState,
  sendState,
  sendAckState  
} TCashCodeExchState;

//TCashCodeState 
TCashCodeState  ccState = unknownState;
TCashCodeExchState ccExchState = idleExchState;
uint64_t lastBVPollTime=0;
int iBytesToTransfer = 0;
int iCurInd = 0;
int data[60];
byte pollReqArr[] = {0x02, 0x03, 0x06, 0x33, 0xda, 0x81};
byte resetReqArr[] = {0x02, 0x03, 0x06, 0x30, 0x41, 0xb3};
byte writeBillTypeArr[] = {0x02, 0x03, 0x0C, 0x34, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xb5, 0xc1};
byte ackArr[] = {0x02, 0x03, 0x06, 0x00, 0xc2, 0x82};
byte *sendArr;

int cashCount = 0;

char dispArr[8];
int curDig = 0;

uint64_t lastMotorControlTime = 0;


typedef enum{
  speedUp,
  speedDown
} TMotorControlState;

TMotorControlState mcState = speedUp;

void loop() {
  for(int i=0; i<8; i++){
    digitalWrite(pinDig[i], LOW);    // turn the LED off by making the voltage LOW
  }  
  
  setDigit(curDig, dispArr[curDig]);
    
  curDig++;
  if(curDig > 7)
    curDig = 0;  
     
  if((millis()-lastRTCPollTime) > 1000){
    lastRTCPollTime = millis();
      
    Wire.beginTransmission(0x68); // transmit to device #9
    Wire.write(0x0); // sends x
    Wire.endTransmission(); // stop transmitting
    Wire.requestFrom(0x68, 7);    // request 6 bytes from slave device #8

    uint8_t c;
    uint8_t ddd[7];
    int dInd=0;         
    while (Wire.available()) { // slave may send less than requested
      c = Wire.read(); // receive a byte as character    
      ddd[dInd] = c;
      dInd++;      
    }
    char str[30] ;
 
//    sprintf(str,"y=%x M=%x  d=%x wd=%x h=%x m=%x s=%x dbi=%d", ddd[6], ddd[5], ddd[4], ddd[3],
//                                                        ddd[2], ddd[1], ddd[0], digitBlinkInd);
//    Serial.println(str); 

//      int secCnt = ddd[0];
//      int sec0 = secCnt%10;
//      int sec1 = (secCnt%100)/10;
//
//      int minCnt = ddd[1];
//      int min0 = minCnt%10;
//      int min1 = (minCnt%100)/10;
//

      //for(int i=0; i<8; i++){
      for(int i=0; i<4; i++){
        dispArr[i] = 0;
      }
      //dispArr[counter] = digTable[counter];
//      dispArr[7] = digTable[ddd[0]&0xf];
//      dispArr[6] = digTable[(ddd[0]&0xf0)>>4];
//      dispArr[5] = digTable[ddd[1]&0xf];
//      dispArr[4] = digTable[(ddd[1]&0xf0)>>4];
      dispArr[3] = digTable[ddd[1]&0xf];
      dispArr[2] = digTable[(ddd[1]&0xf0)>>4];
      dispArr[1] = digTable[ddd[2]&0xf];
      dispArr[0] = digTable[(ddd[2]&0xf0)>>4];
  }

  if(digitalRead(butDownPin)==0){
    
      
  }
  else if(digitalRead(butUpPin)==0){
      
  }
  else if(digitalRead(butLeftPin)==0){
    digitBlinkInd--;
    if(digitBlinkInd < -1)
      digitBlinkInd = -1;
      
  }
  else if(digitalRead(butRightPin)==0){
    digitBlinkInd++;      
    if(digitBlinkInd >= 7)
      digitBlinkInd = 7;    
  }

  if(digitBlinkInd > 0){
    if((lastBlinkTime-millis()) > 250){
      lastBlinkTime = millis();       
      static bool blinkState = true;
      if(blinkState == true){
        blinkState = false;        
        dispArr[digitBlinkInd] = 0xff;
      }
      else{        
        blinkState = true;        
        dispArr[digitBlinkInd] = 0x00;
      }
    }
  }
  else{
    
  }

  mcVal ^= 0xff;
  //setMCVal(mcVal);

  //processCC();


  if((millis()- lastMotorControlTime) > 250){
    lastMotorControlTime = millis();    
    //Serial.println("ctrl");
    Serial.println(motor1Val);
//    digitalWrite(pinLed, !digitalRead(pinLed));   
    switch(mcState){
      case speedUp:
        //motor1Val = 0;
        //mcState = speedDown;         
        
        if(motor1Val == 0){
          mcState = speedDown;         
        }
        else{
          motor1Val--;
        }
         
        break;
      case speedDown:
        //motor1Val = 0x20;
        //mcState = speedUp;  
        if(motor1Val == 0x30){
          mcState = speedUp;         
        }
        else{
          motor1Val++;
        }

        break;   
    }   
  }   
}


void setDigit(int dn, uint8_t ch)
{
  //SPI.transfer(ch);
  //digitalWrite(pinRCK, HIGH);
  //digitalWrite(pinDig[dn], HIGH);    // turn the LED off by making the voltage LOW 
  //digitalWrite(pinRCK, LOW); 
}

void setMCVal(int val)
{
  SPI.transfer(val);
  //digitalWrite(pinMCRCK, HIGH);     
  //digitalWrite(pinMCRCK, LOW);  
}
 


void fillAsciiTable()
{
  asciiTable['A'] = 0x77;
  asciiTable['C'] = 0x39;
  asciiTable['E'] = 0x79;
  asciiTable['S'] = 0x6D;
  asciiTable['T'] = 0x3E;
  asciiTable['0'] = 0x3f;
  asciiTable['1'] = 0x06;
  
}


