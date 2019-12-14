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
#include <stdint.h>
#include "..\inc\TExaS.h"

int starting;

#define SIZE 100
uint32_t Debug_Buffer[SIZE][2];
unsigned int Debug_Cnt= 0;


int H,L;
uint32_t i=0;
uint32_t Input;

uint32_t Reflectance_Counter= 0;
uint32_t Position= 0;
uint32_t power_percentage= 0;
uint32_t bump_data=0;

void Debug_Init(void){int i;int j;
  // write this as part of Lab 10
    for(i=0; i<SIZE; i++) {
       for(j=0;j<2;j++) {
           Debug_Buffer[i][j]= 0;
       }
    }
}

void Debug_Dump(uint32_t x, uint32_t y){
    if (Debug_Cnt< SIZE){
        Debug_Buffer[Debug_Cnt][0]= x; //P1->IN;
        Debug_Buffer[Debug_Cnt][1]= y; //P2->OUT;
        Debug_Cnt++;
        Debug_Cnt %= SIZE; //Roll over
        Debug_FlashRecord(Debug_Buffer[Debug_Cnt]);
    }
}


// Flash ROM addresses must be 4k byte aligned, e.g., 0x00020000, 0x00021000, 0x00022000...
#define FLASH                   0x00020000  // location in flash to write; make sure no program code is in this block; 0x00020000 ~ 0x0003FFFF

void Debug_FlashInit(void){ 
  // write this as part of Lab 10
}

////------------Flash_WriteArray------------
//// Write an array of 32-bit data to flash starting at given address.
//// Parameter 'addr' must be in flash Bank 1.
//// Input: source pointer to array of 32-bit data
////        addr   4-byte aligned flash memory address to start writing
////        count  number of 32-bit writes
//// Output: number of successful writes; return value == count if completely successful
//// Note: at 48 MHz, it takes 612 usec to write 10 words
//// Note: This function is not interrupt safe.
//int Flash_WriteArray(uint32_t *source, uint32_t addr, uint16_t count){
//  uint16_t successfulWrites = 0;
//  while((successfulWrites < count) && (Flash_Write(addr + 4*successfulWrites, source[successfulWrites]) == NOERROR)){
//    successfulWrites = successfulWrites + 1;
//  }
//  return successfulWrites;
//}

void Debug_FlashRecord(uint64_t *pt){
  // write this as part of Lab 10
    /* Example:
     *     SuccessfulWrites = Flash_WriteArray(DataArray, FLASH + 12, 10); // use scope to measure J4.37 (TM4C123 PC4, MSP432 P5.6) high time (610 usec)
     *     LaunchPad_Output(0);                           // red LED off; green LED off; blue LED off
     */
    uint16_t successfulWrites = 0;
    successfulWrites= Flash_WriteArray(pt, FLASH+12, 10); //ugh just copying
    LaunchPad_Output(0);                           // red LED off; green LED off; blue LED off
    P2->OUT= successfulWrites;
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

//Assumes opacity is between 0 and 1
void PWM_LED(uint32_t percentage){
    //Between 0 and 1, change the strength of the LED light
    H= (Total * percentage) / 100;
    L= Total- H;

//    P2->OUT^= 0X06; //Toggle sky blue LED
    P1->OUT^= 0x01; //toggle red LED
    if (P2->OUT&= 0x02){ //red on
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

void Scaled_Green_LED(int percentage){
    //Do PWD on a reflectance counter? Assuming between 0 and 100
    if (percentage> 0){
        if (percentage > Reflectance_Counter){
            P2->OUT|= 0x02; //Green LED ON
        }else{
            P2->OUT&= ~0x02; //Green LED OFF
        }
    }
}

void Bump_LED(int percentage){
    if (percentage > 0){
        if (percentage > Reflectance_Counter){
            P1->OUT|= 0x01; //Green LED ON
        }else{
            P1->OUT&= ~0x01; //Green LED OFF
        }
    }
}

void Reflectance_Handler(void){
    Reflectance_Counter%= 100;

    //P1->OUT&= ~0x01; //Red LED OFF

    if (Reflectance_Counter == 0){
        Reflectance_Start();
    }else if (Reflectance_Counter==1){
        P7->DIR &= 0x00; //Set p7 to Input (0)
    }
    else if (Reflectance_Counter >= 99){
        Position= Reflectance_Position(Reflectance_End());
        bump_data= Bump_Read();
        power_percentage = abs(Position)*100/334;
        Debug_Dump(Position, bump_data);
    }
    Scaled_Green_LED(power_percentage);
    Bump_LED(bump_data*100/64);

    Reflectance_Counter+= 1;
    //printf("Counter: %d, Status: %d, data: %d, Position: %d, Power Percentage: %d\n", Reflectance_Counter, status, data, position, power_percentage);
}

void SysTick_Handler(void){ // every 1ms
//    PWM_Example();
//    PWM_LED(2);
    Reflectance_Handler();
}


int main(void){
  // write this as part of Lab 10
  Clock_Init48MHz();
  Debug_Init();
  LaunchPad_Init();   // buttons and LEDs
  Reflectance_Init();
  Bump_Init();

  SysTick_Init(480, 2); //initialize the sys tick cycle; change the period between the reflectance here
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


int main_(void){ uint16_t i;
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
