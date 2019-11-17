// Lab10_Debugmain.c
// Runs on MSP432
// Student version to Debug lab
// Daniel and Jonathan Valvano
// September 4, 2017
// Interrupt interface for QTRX reflectance sensor array
// Pololu part number 3672.
// Debugging dump, and Flash black box recorder

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, 0 rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/
// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)



#include "msp.h"
#include "..\inc\bump.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\FlashProgram.h"

////To help with debugging... many errors
//#include <stdint.h>
#include "..\inc\TExaS.h"

int starting;

int debug_array_x[256];
int debug_array_y[256];
int debug_array_element;

const uint32_t PulseBuf[100]={
    5000, 5308, 5614, 5918, 6219, 6514, 6804, 7086, 7361, 7626,
    7880, 8123, 8354, 8572, 8776, 8964, 9137, 9294, 9434, 9556,
    9660, 9746, 9813, 9861, 9890, 9900, 9890, 9861, 9813, 9746,
    9660, 9556, 9434, 9294, 9137, 8964, 8776, 8572, 8354, 8123,
    7880, 7626, 7361, 7086, 6804, 6514, 6219, 5918, 5614, 5308,
    5000, 4692, 4386, 4082, 3781, 3486, 3196, 2914, 2639, 2374,
    2120, 1877, 1646, 1428, 1224, 1036,  863,  706,  566,  444,
     340,  254,  187,  139,  110,  100,  110,  139,  187,  254,
     340,  444,  566,  706,  863, 1036, 1224, 1428, 1646, 1877,
    2120, 2374, 2639, 2914, 3196, 3486, 3781, 4082, 4386, 4692};
int H,L;
uint32_t i=0;
uint32_t Input;


void Debug_Init(void){
  // write this as part of Lab 10
    debug_array_element= 0;
//    debug_array_x= {0};
//    debug_array_y= {0};
}
void Debug_Dump(uint8_t x, uint8_t y){
  // write this as part of Lab 10
    debug_array_x[debug_array_element]= x;
    debug_array_y[debug_array_element]= y;

    debug_array_element+= 1;
    debug_array_element%= 256;
}
void Debug_FlashInit(void){ 
  // write this as part of Lab 10
}
void Debug_FlashRecord(uint16_t *pt){
  // write this as part of Lab 10
}

volatile uint32_t Counts;
int Step_Size= 1000;//rate of change of lights

uint32_t Total= 480000;
uint32_t High=120000, Low=360000;
//uint32_t High=48, Low=480000;

void PWM_Example(void){
    H+= Step_Size;
    if (H >= Total || H<= 0){
        Step_Size*= -1;
        H+= Step_Size;
    }

    L= Total- H;


    P1->OUT^= 0x01; //toggle red LED
    if (P1->OUT&= 0x01){ //red on
        SysTick->LOAD= H; //time while high
    }else{ //red off
        SysTick->LOAD= L; //time while low
    }
    SysTick->VAL= 0; //value written to counter clears
}

void PWM_Init(void){
    P1->DIR |= 0x01; //LED pin out
    P1->OUT |= 0x01; //red LED on
    H= High;
}

void SysTick_Handler(void){ // every 1ms
//    PWM_Example();
}


int main(void){
  // write this as part of Lab 10
  Clock_Init48MHz();
  Debug_Init();
  LaunchPad_Init();   // buttons and LEDs

  SysTick_Init(High, 2); //initialize the sys tick cycle; change the period between the reflectance here
  EnableInterrupts();

  while(1){
  }
}

int Program10_1(void){ uint8_t data=0;
  Clock_Init48MHz();
  Debug_Init();
  LaunchPad_Init();
  Bump_Init();
  while(1){
    P1->OUT |= 0x01;
    Debug_Dump(data,data+1);// linear sequence
    P1->OUT &= ~0x01;
    data=data+2;
  }
}


// Driver test
#define SIZE 256  // feel free to adjust the size
uint16_t Buffer[SIZE];
int Program10_2(void){ uint16_t i;
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  for(i=0;i<SIZE;i++){
    Buffer[i] = (i<<8)+(255-i); // test data
  }
  i = 0;
  while(1){
    P1->OUT |= 0x01;
    Debug_FlashInit();
    P1->OUT &= ~0x01;
    P2->OUT |= 0x01;
    Debug_FlashRecord(Buffer); // 114us
    P2->OUT &= ~0x01;
    i++;
  }
}


int Program10_3(void){ uint16_t i;
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  for(i=0;i<SIZE;i++){
    Buffer[i] = (i<<8)+(255-i); // test data
  }
  P1->OUT |= 0x01;
  Debug_FlashInit();
  P1->OUT &= ~0x01;
  i = 0;
  while(1){
    P2->OUT |= 0x01;
    Debug_FlashRecord(Buffer);
    P2->OUT &= ~0x01;
    i++;
  }
}

/*
uint8_t Buffer[1000];
uint32_t I=0;
uint8_t *pt;
void DumpI(uint8_t x){
  if(I<1000){
    Buffer[I]=x;
    I++;
  }
}
void DumpPt(uint8_t x){
  if(pt<&Buffer[1000]){
    *pt=x;
    pt++;
  }
}
void Activity(void){
  DumpI(5);
  DumpI(6);
  pt = Buffer;
  DumpPt(7);
  DumpPt(8);

}
*/
