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

uint64_t lastBlinkTime=0;
int digitBlinkInd = -1;

byte mcVal = 0x01;

void setup() {
  fillAsciiTable();
  
  pinMode(pinRCK, OUTPUT); //RCK
  digitalWrite(pinRCK, LOW);    // turn the LED off by making the voltage LOW

  pinMode(pinMCRCK, OUTPUT); //RCK for Motro Control Shift Reg
  digitalWrite(pinMCRCK, LOW);    // turn the LED off by making the voltage LOW
  

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

  Timer1.initialize(5);         // initialize timer1, and set a 1/2 second period 
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
}

byte counter; 
void callback()
{   
  PORTC |= 0x08;
  SPDR = mcVal;
  counter++;

  if((counter&0x01) == 0){
    mcVal = 0x01;            
  }
  else{
    mcVal = 0x00;        
  }
  
  //SPI.transfer(mcVal);
  
     
//  PORTC |= 0x08;
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


  

void processCC()
{
  int rb = 0;
  switch(ccExchState){
    case idleExchState:
      if(((millis() - lastBVPollTime) > 200) && (Serial.availableForWrite())){        
        lastBVPollTime = millis();

        if(ccState == powerUpState){
          iCurInd = 0;
          iBytesToTransfer = 6;
          sendArr = &(resetReqArr[0]);               
          ccExchState = sendState;  
          ccState = unknownState;         
        }
        else if(ccState == disableState){  
          iCurInd = 0;
          iBytesToTransfer = 12;
          sendArr = &(writeBillTypeArr[0]);               
          ccExchState = sendState;        
          ccState = unknownState;                  
        }
        else{
          iCurInd = 0;
          iBytesToTransfer = 6;
          sendArr = &(pollReqArr[0]);               
          ccExchState = sendState;    
        }
      }
      else if(Serial.available()){
        int rb = Serial.read();                   
        if (rb == 0x02){
          ccExchState = readState0;
        }        
      }
    break;
    
    case readState0:
      rb = Serial.read();
      if(rb != -1){
        if(rb == 0x03){
          ccExchState = readState1;       
        }
        else{
          ccExchState = idleExchState;
        }      
      }
    break;

    case readState1: 
      rb = Serial.read();
      if(rb != -1){
        iBytesToTransfer = rb-3;
        iCurInd = 0;
        ccExchState = readDataState;   
//        dispArr[7] = digTable[iBytesToTransfer&0xf];
//        dispArr[6] = digTable[(iBytesToTransfer&0xf0)>>4];      
//        dispArr[5] = digTable[0];
//        dispArr[4] = digTable[0];           
  
      }
    break;

    case readDataState:
      rb = Serial.read();
      if(rb != -1){     
        data[iCurInd++] = rb;
        //bToRead--;
        if((iCurInd == iBytesToTransfer)){
          ccExchState = idleExchState; 
          if(iBytesToTransfer == 0x03){
            if(data[0]==0x10){  //power up state                     
              ccState = powerUpState;
              ccExchState = sendAckState;
            }  
            else if(data[0]==0x13){  //init state                     
              ccState = initState;
              ccExchState = sendAckState;
            }     
            else if(data[0]==0x19){  //disable state                     
              ccState = disableState;
              ccExchState = sendAckState;
            } 
            else if(data[0]==0x14){  //idle state                     
              ccState = idleState;
              ccExchState = sendAckState;            
            }
            else if(data[0]==0x15){  //accepting state                     
              ccState = acceptingState;
              ccExchState = sendAckState;   
              dispArr[0] = asciiTable['A'];
              dispArr[1] = asciiTable['C'];
              dispArr[2] = asciiTable['C'];
              dispArr[3] = asciiTable['0'];                  
            }
            else if(data[0]==0x17){  //stacking state                     
              ccState = stackingState;
              ccExchState = sendAckState;                                    
              dispArr[0] = asciiTable['A'];
              dispArr[1] = asciiTable['C'];
              dispArr[2] = asciiTable['C'];
              dispArr[3] = asciiTable['1'];                  
            }


          }
          else if(iBytesToTransfer == 0x04){
            if(data[0]==0x81){      //rubls packed
              if(data[1]==0x02)
                cashCount +=10;
              if(data[1]==0x03)
                cashCount +=50;
              if(data[1]==0x04)
                cashCount +=100; 

              iCurInd = 0;
              iBytesToTransfer = 6;
              sendArr = &(ackArr [0]);               
              ccExchState = sendState;
            } 
            else if(data[0]==0x1c){  //rejecting state                    
              ccState = rejectingState;
              ccExchState = sendAckState;   
            }           
          }
          dispArr[7] = digTable[(cashCount%10)&0xf];
          dispArr[6] = digTable[(int)(cashCount/10)%10];      
          dispArr[5] = digTable[(int)(cashCount/100)%10];
          dispArr[4] = digTable[(int)(cashCount/1000)%10];
        }              
      }                  
              
    break;

    case sendAckState:
      iCurInd = 0;
      iBytesToTransfer = 6;
      sendArr = &(ackArr [0]);                            
      ccExchState = sendState;
    break;

    case sendState:
    if(Serial.availableForWrite()){
      Serial.write(sendArr[iCurInd++]);  
      //iCurInd++;
      if((iCurInd == iBytesToTransfer)){
        ccExchState = idleExchState; 
      }
    }
    break;

  }
 
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


