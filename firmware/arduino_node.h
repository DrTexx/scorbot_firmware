#ifndef ARDUINO_NODE_H
#define ARDUINO_NODE_H

//boolean initMTRCCW[6] = {false, false, false,false,true,true}; //specifying the direction of rotation of motors
boolean initMTRCCW[6] = {false, true, true,true,true,true}; //specifying the direction of rotation of motors
const int MS[6] = {2,3,19,21,20}; //pins for the microswitches
volatile boolean  MSpressed[6]={0,0,0,0,0,0}; //to store the state of the mss
const int pin_P0[6] = {51,49,53,50,48,46};  // Encoder input pins pinA
const int pin_P1[6] = {39,37,41,43,45,47};  // Encoder input pins pinB

int FW[6] = {0,0,0,0,0,0}; 
volatile long  counter[5]={0,0,0,0,0}; //counter for the encoder
volatile long  encoder_reference[5]={0,0,0,0,0}; //counter for the encoder

 
//——————————  Motors
const int M1[6] = {29,30,27,35,23,33}; //pins for the 6 motors
const int M2[6] = {28,31,26,34,22,32}; // the other terminal
const int Mpwm[6] = {8,9,7,11,6,10}; // pins for speed control


  

void resetCounter0(){   
  counter[0]=0;
  MSpressed[0] = !MSpressed[0];
  
}
void resetCounter1(){  
  counter[1]=0;
  MSpressed[1]=!MSpressed[1];
  
}
void resetCounter2(){  
  counter[2]=0;
  MSpressed[2]=!MSpressed[2];
  
}
void resetCounter3(){  
  counter[3]=0;
  MSpressed[3]=!MSpressed[3];
 
}
void resetCounter4(){  
  counter[4]=0;
  MSpressed[4]=!MSpressed[4];
  
}

#endif