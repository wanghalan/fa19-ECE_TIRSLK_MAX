// Lab13_Timersmain.c
// Runs on MSP432
// Student starter code for Timers lab
// Daniel and Jonathan Valvano
// July 11, 2019
// PWM output to motor
// Second Periodic interrupt

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

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

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include "msp.h"
#include "..\tirslk_max_1_00_00\inc\bump.h"
#include "..\tirslk_max_1_00_00\inc\Clock.h"
#include "..\tirslk_max_1_00_00\inc\SysTick.h"
#include "..\tirslk_max_1_00_00\inc\CortexM.h"
#include "..\tirslk_max_1_00_00\inc\LaunchPad.h"
#include "..\tirslk_max_1_00_00\inc\Motor.h"
#include "..\tirslk_max_1_00_00\inc\TimerA0.h"
#include "..\tirslk_max_1_00_00\inc\TimerA1.h"
#include "..\tirslk_max_1_00_00\inc\TimerA2.h"
#include "..\tirslk_max_1_00_00\inc\TExaS.h"
#include "..\tirslk_max_1_00_00\inc\Reflectance.h"
#include "..\tirslk_max_1_00_00\inc\Nokia5110.h"


uint16_t stop_flag= 0; //Whether or not to stop the motors
uint32_t Reflectance_Counter= 0; //?
uint32_t Position= 0; //Position as detected by the line sensor

// Linked data structure
struct State {
  char *name;
  uint32_t out;                // 2-bit output
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center    &fsm[0]
#define Left      &fsm[1]
#define Right     &fsm[2]
#define Left2      &fsm[3]
#define Right2     &fsm[4]
#define E_Right      &fsm[5]
#define E_Left     &fsm[6]
#define Rec_Center &fsm[7]
#define Stop &fsm[8]


State_t fsm[9]={ //Keeping each state string to 6 characters
  {"Center", 0x01, 500, { Right, Left, Right,  Center }},  // Center, red
  {"Left 1", 0x02, 500, { E_Right,  Left2, Right,  Center }},  // Left, green
  {"Right1", 0x03, 500, { E_Left, Left,   Right2, Center }},   // Right, yellow

  {"Left 2", 0x04, 500, { E_Right, Left,   Right, Center }},   // Right2, blue
  {"Right2", 0x04, 500, { E_Left, Left,   Right, Center }},   // Left2, blue

  {"ERight", 0x05, 500, { Rec_Center, Left,   Right, Center }},   // E_Right, pink
  {"E Left", 0x05, 500, { Rec_Center, Left,   Right, Center }},   // E_Left, pink

  {"RecCen", 0x06, 500, { Stop, Left,   Right, Center }},   // Recover_Center, sky blue
  {"-Stop-", 0x07, 500, { Stop, Stop,   Stop, Stop }},   // Stop, white
};

State_t *Spt;  // pointer to the current state
uint32_t Input;
uint32_t Output;
/*Run FSM continuously
1) Output depends on State (LaunchPad LED)
2) Wait depends on State
3) Input (LaunchPad buttons)
4) Next depends on (Input,State)
 */

// Driver test
void TimedPause(uint32_t time){
  //P2->OUT|=0x02;
  Clock_Delay1ms(time);          // run for a while and stop
  Motor_Stop();
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
  //P2->OUT^=0x06;
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
    }
    //Scaled_Green_LED(abs(Position)*100/334); //power percentage; if distance = 0, LED will turn off

    Reflectance_Counter+= 1;
    //printf("Counter: %d, Status: %d, data: %d, Position: %d, Power Percentage: %d\n", Reflectance_Counter, status, data, position, power_percentage);
}

void BumpCheck(void){
    //P2->OUT = 0x06;
    if (Bump_Read()>0){
        //P2->OUT = 0x03;
        Motor_Stop();
        stop_flag= 1; //This way it won't keep going
        Nokia5110_SetCursor(0, 5);
        Nokia5110_OutString("-BUMP-");
    }else{
        stop_flag= 0;
    }
}

int main(void){ uint32_t heart=0; //FMS check
  Clock_Init48MHz();
  LaunchPad_Init();
  Nokia5110_Init();
  Nokia5110_ClearBuffer();
  Nokia5110_Clear();


  Bump_Init();
  Motor_Init();
  Reflectance_Init();

  PWM_Init34(15000, 5000, 5000); //10 ms period motor set up
  TimerA1_Init(&BumpCheck,500);  // 1000 Hz bump check
  TimerA2_Init(&Reflectance_Handler,480);  // Can't use timer A0, because it is being used for the motor PWM; reflectance checking

  EnableInterrupts();

  Spt = Center;
  while(1){
    if (stop_flag== 0){
        Output = Spt->out;            // set output from FSM
        LaunchPad_Output(Output);     // do output to two motors
        TExaS_Set(Input<<2|Output);   // optional, send data to logic analyzer
        Clock_Delay1ms(Spt->delay);   // wait
        Input = LaunchPad_Input();    // read sensors
        Spt = Spt->next[Input];       // next depends on input and state
        heart = heart^1;
        LaunchPad_LED(heart);         // optional, debugging heartbeat

        Nokia5110_SetCursor(0, 5);         // five leading spaces, bottom row
        Nokia5110_OutString(Spt->name);
    }
  }
}


int main_(void){
    // write a main program that uses PWM to move the robot
    // like Program13_1, but uses TimerA1 to periodically
    // check the bump switches, stopping the robot on a collision

    Clock_Init48MHz();
    LaunchPad_Init(); // built-in switches and LEDs
    Bump_Init();      // bump switches
    Motor_Init();
    Reflectance_Init();

    PWM_Init34(15000, 5000, 5000); //10 ms period
    TimerA1_Init(&BumpCheck,500);  // 1000 Hz
    TimerA2_Init(&Reflectance_Handler,480);  // Can't use timer A0, because it is being used for the motor PWM

    EnableInterrupts();

    while(1){

        //TimedPause(1000);
        //P2->OUT= 0x02; //Green
        if (stop_flag== 0){
            //P2->OUT= 0x01; //Red
            Spt = Spt->next[Input]; // next depends on input and state
            //Motor_Forward(1000,1000);  // your function
        }
    }
}

