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

#define driveCount 2

#define motorLowestSpeed 252

typedef enum{
  idle,
  speedUp,
  speedIdle,
  speedDown,
  dirChange
} TMotorControlState;

typedef struct{
  TMotorControlState state;
  byte maxSpeed;
  byte speedUpEndPos;
  byte speedDownStartPos;
  byte speedInc;
  bool bCont;
  uint32_t contPos;
    
} TMotorStatus;


volatile byte motorVal[driveCount];
byte motorCnt[driveCount];
volatile uint32_t motorPos[driveCount];
volatile byte motorDir[driveCount];

volatile TMotorStatus mcStatus[driveCount];


String inString = ""; 


int cashCount = 0;

char dispArr[8];
int curDig = 0;

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

  //Wire.beginTransmission(0x68); // transmit to device #9
  //Wire.write(0x01); 
  //Wire.write(0x30);   
  //Wire.endTransmission(); // stop transmitting
//
//  Wire.beginTransmission(0x68); // transmit to device #9
//  Wire.write(0x02); 
//  Wire.write(0x23);   
//  Wire.endTransmission(); // stop transmitting
  
  
  Serial.begin(19200);  // start serial for output

  for(int i=0; i<driveCount; i++){
    motorVal[i]=3;
    motorCnt[i]=0;
    motorPos[i]=0;
    mcStatus[driveCount].state = speedIdle;
    mcStatus[driveCount].maxSpeed = 0;
    mcStatus[driveCount].speedUpEndPos = 0;
    mcStatus[driveCount].speedDownStartPos = 0;
    mcStatus[driveCount].speedInc = 0;
    mcStatus[driveCount].bCont = false;
    
  }
  //Timer1.initialize(30);     
  Timer1.initialize(30);     
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
}


volatile byte spiVal = 0; 
void callback()
{   
  //PORTC |= 0x08;
  //SPDR = mcVal;
  
  SPDR = spiVal;
  spiVal = 0;
  for(int i=0; i<driveCount; i++){
    if(motorCnt[i] == 0){
      if(motorPos[i] > 0 ){        
        spiVal |= (1<<(2*i));
        spiVal |= (motorDir[i]<<(2*i+1));        
        motorCnt[i] = motorVal[i];
        motorPos[i]--;
      }
    }   
    else{
      motorCnt[i]--;                    
    } 
       
  }

  
  //SPI.transfer(mcVal);
  
  while (!(SPSR & _BV(SPIF))) ; // wait     
  PORTC |= 0x08;
  PORTC &= ~0x08;
  
  SPDR = 0x0;
  while (!(SPSR & _BV(SPIF))) ; // wait      
  PORTC |= 0x08;
  PORTC &= ~0x08;


//  for(int i=0; i<8; i++){
//    digitalWrite(pinDig[i], LOW);    // turn the LED off by making the voltage LOW
//  }  
//  
//  //setDigit(curDig, dispArr[curDig]);
//  SPI.transfer(dispArr[curDig]);
//  digitalWrite(pinRCK, HIGH);
//  digitalWrite(pinDig[curDig], HIGH);    // turn the LED off by making the voltage LOW 
//  digitalWrite(pinRCK, LOW); 
//    
//  curDig++;
//  if(curDig > 7)
//    curDig = 0;  
  
}



uint64_t lastMotorControlTime = 0;





void loop() {
  
  for(int i=0; i<8; i++){
    digitalWrite(pinDig[i], LOW);    // turn the LED off by making the voltage LOW
  }                                   //const int pinDig[] = {9, 8, 7, 6, 5, 4, 3, 2};    
  noInterrupts();
  setDigit(curDig, dispArr[curDig]);
  interrupts();    
  
  curDig++;
  if(curDig > 7)
    curDig = 0;  
     
  if((millis()-lastRTCPollTime) > 3000){
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
  if(readSerial() == true){
    if(inString.startsWith("p") == true){
      Serial.println(inString);  
      inString.remove(0, 1);    
      //Serial.println(inString);  
      int mcInd = inString.substring(0, 2).toInt();      
      inString.remove(0, 3);
      //Serial.println(inString);  
      
      uint32_t motorPosLoc = inString.toInt();
      uint32_t motorPosLocDivThr = (motorPosLoc/4)<4000 ? (motorPosLoc/4) : 4000;            
      mcStatus[mcInd].speedUpEndPos = motorPosLoc - motorPosLocDivThr;
      mcStatus[mcInd].speedDownStartPos = motorPosLoc + motorPosLocDivThr;
      mcStatus[mcInd].speedInc = 63;
      //motorVal[mcInd] = motorLowestSpeed;
      mcStatus[mcInd].state = speedUp;      
      
      motorPos[mcInd] = motorPosLoc;      
    }   
    else if(inString.startsWith("v") == true){
      Serial.println(inString);  
      inString.remove(0, 1);    
      //Serial.println(inString);  
      int mcInd = inString.substring(0, 2).toInt();      
      inString.remove(0, 3);
      //Serial.println(inString);  
      motorVal[mcInd] = inString.toInt();
      mcStatus[mcInd].maxSpeed = inString.toInt();
    }   
    else if(inString.startsWith("d") == true){
      Serial.println(inString);  
      inString.remove(0, 1);    
      //Serial.println(inString);  
      int mcInd = inString.substring(0, 2).toInt();      
      inString.remove(0, 3);
      //Serial.println(inString);  
      motorDir[mcInd] = inString.toInt();      
    } 
    else if(inString.startsWith("r") == true){
      Serial.println(inString);  
      inString.remove(0, 1);    
      //Serial.println(inString);  
      int mcInd = inString.substring(0, 2).toInt();      
      inString.remove(0, 3);
      //Serial.println(inString);  
      mcStatus[mcInd].contPos = inString.toInt();      
    }     
    else if(inString.startsWith("c") == true){
      Serial.println(inString);  
      inString.remove(0, 1);    
      //Serial.println(inString);  
      int mcInd = inString.substring(0, 2).toInt();      
      inString.remove(0, 3);
      //Serial.println(inString); 
       
      mcStatus[mcInd].bCont = !mcStatus[mcInd].bCont;
      if(mcStatus[mcInd].bCont == true ){
        motorPos[mcInd] = mcStatus[mcInd].contPos;      
        mcStatus[mcInd].state = speedIdle;
      }
      else{
        mcStatus[mcInd].state = idle;
        
      }
    }   
    inString = ""; 
  }


  int pos7seg[2] = {0, 0};  
  for(int i=0; i<2; i++){
    if(motorPos[i]>0){
      int motorPosDiv4 = motorPos[i]/4;
      if(motorPosDiv4 > 99999){
        pos7seg[i] = motorPosDiv4/100;        
      }
      else if(motorPosDiv4 > 9999){
        pos7seg[i] = motorPosDiv4/10;        
      } 
      else{
        pos7seg[i] = motorPosDiv4;                   
      }
      
      break;
    }
    else{
      pos7seg[i] = 0;     
    }        
  }  
  if((pos7seg[0]>0) && (pos7seg[1]>0)){
    dispArr[7] = digTable[(pos7seg[0]%10)&0xf];
    dispArr[6] = digTable[(int)(pos7seg[0]/10)%10];      
    dispArr[5] = digTable[(int)(pos7seg[0]/100)%10];
    dispArr[4] = digTable[(int)(pos7seg[0]/1000)%10];    

    dispArr[3] = digTable[(pos7seg[1]%10)&0xf];
    dispArr[2] = digTable[(int)(pos7seg[1]/10)%10];      
    dispArr[1] = digTable[(int)(pos7seg[1]/100)%10];
    dispArr[0] = digTable[(int)(pos7seg[1]/1000)%10];    

  }
  else if((pos7seg[0]>0) || (pos7seg[1]>0) ){
    if(pos7seg[1]>0) 
      pos7seg[0] = pos7seg[1];
    dispArr[7] = digTable[(pos7seg[0]%10)&0xf];
    dispArr[6] = digTable[(int)(pos7seg[0]/10)%10];      
    dispArr[5] = digTable[(int)(pos7seg[0]/100)%10];
    dispArr[4] = digTable[(int)(pos7seg[0]/1000)%10];
  }
  else{
    
    dispArr[7] = 0;
    dispArr[6] = 0;
    dispArr[5] = 0;
    dispArr[4] = 0;

//    dispArr[3] = 0;
//    dispArr[2] = 0;
//    dispArr[1] = 0;
//    dispArr[0] = 0;
  }
      
  if((millis()- lastMotorControlTime) > 250){
    lastMotorControlTime = millis();    
    //Serial.println("ctrl");
    for(int i=0; i<driveCount; i++){
      Serial.print(motorPos[i]);
      Serial.print("[");
      Serial.print(motorVal[i]);
      Serial.print(",");
      Serial.print(motorDir[i]);
      Serial.print(",");
      Serial.print(mcStatus[i].state);
      Serial.print("]");
      Serial.print(" ");

      


      switch(mcStatus[i].state){
        case idle:
        break;
//        case speedUp:    
//          //motor1Val = 0;
//          //mcState = speedDown;         
//  
//          if(motorPos[i] > mcStatus[i].speedUpEndPos){
//            if(motorVal[i] == mcStatus[i].maxSpeed){
//              mcStatus[i].state = speedIdle;         
//            }
//            else{
//              motorVal[i]-=mcStatus[i].speedInc;
//            }
//            
//          }
//          else{
//             mcStatus[i].state = speedIdle;                   
//          }
//           
//          break;
//  
          case speedIdle:
//          if(motorVal[i] == motorLowestSpeed){}
//          else{
//            if(motorPos[i] < mcStatus[i].speedDownStartPos){        
//              mcStatus[i].state = speedDown;
//            }
//          }
            if(mcStatus[i].bCont && (motorPos[i] == 0))
              mcStatus[i].state = dirChange;
            
          break;
//          
//        case speedDown:
//          if(motorVal[i] == motorLowestSpeed){
//           mcStatus[i].state = speedIdle;         
//          }
//          else{
//            motorVal[i]+=mcStatus[i].speedInc;
//          }
//  
//          break;   
          case dirChange:
            motorDir[i] ^= 0x1;
            motorPos[i] =  mcStatus[i].contPos;
            mcStatus[i].state = speedIdle;
          break;
      }
    
    }
    Serial.println(" ");
//    digitalWrite(pinLed, !digitalRead(pinLed));   
       
  }


}


void setDigit(int dn, uint8_t ch)
{
  SPI.transfer(ch);
  digitalWrite(pinRCK, HIGH);
  digitalWrite(pinDig[dn], HIGH);    // turn the LED off by making the voltage LOW 
  digitalWrite(pinRCK, LOW); 
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


bool readSerial()
{  
  bool ret = false;
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    //if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    //}

    if (inChar == '\n') {
      //andrCpuTemp = inString.toInt();
      //inString = "";
      ret = true;
      break;              
    }        
  }
  return ret;
}


