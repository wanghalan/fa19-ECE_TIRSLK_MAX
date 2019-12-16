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
int32_t Position= 0; //Position as detected by the line sensor



uint32_t speedMax= 2000;//Test speed //14998; max speed
uint32_t speedMin= 1000;
uint32_t failure_count= 0; //once failure count exceeds a value, then stop
uint32_t failure_threshold= 1000;

//typedef void (*MotorFunctions)();

// Linked data structure
struct State {
  char *name;
  char uid;
  uint32_t out;                // 2-bit output
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;
#define NULL (0)
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
  {"Center", 'c', 0x01, 500, { Right, Left, Right,  Center }},  // Center, red
  {"Left 1", 'l', 0x02, 500, { E_Right,  Left2, Right,  Center }},  // Left, green
  {"Right1", 'r', 0x03, 500, { E_Left, Left,   Right2, Center }},   // Right, yellow

  {"Left 2", 'L', 0x04, 500, { E_Right, Left,   Right, Center }},   // Left 2, blue
  {"Right2", 'R', 0x04, 500, { E_Left, Left,   Right, Center }},   // Right 2, blue

  {"ERight", 'E', 0x05, 500, { Rec_Center, Left,   Right, Center }},   // E_Right, pink
  {"E Left", 'Q', 0x05, 500, { Rec_Center, Left,   Right, Center }},   // E_Left, pink

  {"RecCen", 'C', 0x06, 500, { Stop , Left,   Right, Center }},   // Recover_Center, sky blue
  {"-Stop-", 's', 0x07, 500, { Stop, Left,   Right, Center}},   // Stop, white
};

State_t *Spt;  // pointer to the current state
State_t *pSpt;  // pointer to the previous state
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

uint16_t center_thresh= 97; //7; 190
//uint16_t *position_stream[3];
uint16_t position_counter= 0;
uint16_t position_total= 0;
void Reflectance_to_input(int16_t position){

//    position_total+= position;
//    position_counter++;
//
//    if (position_counter==3){
//
//        position_counter= 0;
//        position_total= 0;
//    }
    if (position > -center_thresh && position < center_thresh){
        Input= 3;
    }else if (position <= -center_thresh && position >= -190){
        Input= 2;
    }else if (position >= center_thresh && position <= 190){
        Input= 1;
    }else{
        Input= 0;
    }

    SetFSM();//!!! PUT THIS BAKC Only change state once reflectance is set
}

void Position_To_Movement(int16_t position){

//    position_total+= position;
//    position_counter++;
//
//    if (position_counter==3){
//
//        position_counter= 0;
//        position_total= 0;
//    }
    if (stop_flag== 0){
        if (position==-999 || (position > -center_thresh && position < center_thresh)){
            Motor_Forward(speedMax, speedMax);

            Nokia5110_SetCursor(0, 5);
            Nokia5110_OutString("-FORW-");

        }else if (position <= -center_thresh && position >= -193){ //Right
            Motor_Forward(speedMin, speedMax);
            Nokia5110_SetCursor(0, 5);
            Nokia5110_OutString("-LEFT-");
        }else if (position >= center_thresh && position <= 193){ //Left
            Motor_Forward(speedMax, speedMin);
            Nokia5110_SetCursor(0, 5);
            Nokia5110_OutString("-RIGHT-");
        }else if (position < -190){
            Motor_Right(speedMax, speedMax);
            Nokia5110_SetCursor(0, 5);
            Nokia5110_OutString("-MLEFT-");
        }else if (position > 190){
            Motor_Right(speedMax, speedMax);
            Nokia5110_SetCursor(0, 5);
            Nokia5110_OutString("-MRIGHT-");
        }

        Nokia5110_SetCursor(0, 4);         // five leading spaces, bottom row
        Nokia5110_OutString("Pos: ");
        Nokia5110_OutSDec(Position);
    }
}

uint32_t ref_latency= 140; //140; //160 for table, 295 for the groundfrom testing
uint32_t tmp_ref_latency= 0; //160 for table, 295 for the groundfrom testing


uint32_t snail_counter= 0;
uint32_t snail_trail[5];

uint32_t average(uint32_t *array){uint16_t i=0; uint16_t n= 0;uint16_t sum= 0;
    n= sizeof(array);
    for (i = 0; i < n; ++i) {
        sum += array[i];
    }
    return sum / n;
}
const uint32_t errorThreshold= 3; //Once it passes the error threshold, then it really is zero
const uint16_t threshold_step= 5;
uint32_t error_count= 0;

uint32_t Try_Tune_Reflectance(uint8_t data){uint32_t count=0;
    count= countSetBits(data);
    if (count < 2){
        ref_latency-= threshold_step;
    }else if (count >4){
        ref_latency+= threshold_step;
    }

    return count;
}
uint8_t tuned_flag= 0;

void Tune_Handler(void){uint8_t data= 0; uint32_t bit_count=0;
//    if (tuned_flag== 0){
//        tmp_ref_latency= ref_latency;
//        tuned_flag++;
//    }

    Reflectance_Counter%= tmp_ref_latency;

    if (Reflectance_Counter == 0){
        P2->OUT= 0x01;//Check that something is being shined
        Reflectance_Start();
    }else if (Reflectance_Counter==1){
        P2->OUT&= ~0x01;//close it
        P7->DIR &= 0x00; //Set p7 to Input (0)
    }
    else if (Reflectance_Counter >= tmp_ref_latency- 1){
        data= Reflectance_End();
        Position= Reflectance_Position(data);
        //Threshold_Position_Finding(Position); //C'mon gimme that sum!

        bit_count= countSetBits(data);
        if (bit_count< 2 || bit_count > 4){
            P2->OUT= 0x01; //red LED
            error_count++;
            bit_count= Try_Tune_Reflectance(data);
            if (error_count> errorThreshold){ //Essentially discounts the reading until tuned
                error_count= 0;
            }
        }else{
            P2->OUT= 0x02; //green LEF
            tuned_flag++;
        }
        Nokia5110_SetCursor(0, 3);         // five leading spaces, bottom row
        Nokia5110_OutString(Reflectance_String(data));
    }
    Reflectance_Counter+= 1; //for the iterating
    P2->OUT= 0x00; //Turn off lights
}

void Reflectance_Handler_old(void){uint8_t data= 0; uint32_t bit_count=0;
    Reflectance_Counter%= ref_latency;
    if (Reflectance_Counter == 0){
        //P2->OUT= 0x01;//Check that something is being shined
        Reflectance_Start();
    }else if (Reflectance_Counter==1){
        //P2->OUT&= ~0x01;//close it
        P7->DIR &= 0x00; //Set p7 to Input (0)
    }
    else if (Reflectance_Counter >= ref_latency- 1){
        P2->OUT= 0x06;
        data= Reflectance_End();
        Position= Reflectance_Position(data);
        Reflectance_to_input(Position);

        Nokia5110_SetCursor(0, 3);         // five leading spaces, bottom row
        Nokia5110_OutString("L: ");
        Nokia5110_OutString(Reflectance_String(data));
        P2->OUT= ~0x06;
        Nokia5110_SetCursor(0, 4);         // five leading spaces, bottom row
        Nokia5110_OutString("Pos: ");
        Nokia5110_OutSDec(Position);
    }
    Reflectance_Counter+= 1;
}

void Reflectance_Handler(void){uint8_t data= 0; uint32_t bit_count=0; //The new one
    Reflectance_Counter%= ref_latency;
    if (Reflectance_Counter == 0){
        P2->OUT= 0x01;//Check that something is being shined
        Reflectance_Start();
        P7->DIR &= 0x00; //Set p7 to Input (0)
    }
    else if (Reflectance_Counter >= ref_latency- 1){
        //P2->OUT= 0x06;
        data= Reflectance_End();
        Position= Reflectance_Position(data);
        position_total+= Position;

        //Reflectance_to_input(position_total/ref_latency);
//        Position_To_Movement(position_total/ref_latency);
//        Position_To_Movement(Position);
        Reflectance_to_input(Position);

        Nokia5110_SetCursor(0, 3);         // five leading spaces, bottom row
        Nokia5110_OutString("L: ");
        Nokia5110_OutString(Reflectance_String(data));
        P2->OUT= ~0x06;
        Nokia5110_SetCursor(0, 4);         // five leading spaces, bottom row
        Nokia5110_OutString("Pos: ");
        Nokia5110_OutSDec(Position);
    }
    else if (Reflectance_Counter> ref_latency){
        data= Reflectance_End();
        Position= Reflectance_Position(data);
        position_total+= Position;
    }
    Reflectance_Counter+= 1;
}



void BumpCheck(void){
    //P2->OUT = 0x06;
    if (Bump_Read()>0 && stop_flag!= 1){
        //P2->OUT = 0x03;
        Motor_Stop();
        stop_flag= 1; //This way it won't keep going
        Nokia5110_SetCursor(0, 5);
        Nokia5110_OutString("-BUMP-");
    }else{
        //stop_flag= 0;
    }
}


void hand_tuning_module(void){uint8_t touch= 0;//TEST MODE
    Nokia5110_SetCursor(0, 1);         // five leading spaces, bottom row
    Nokia5110_OutString("Lat: ");
    Nokia5110_OutUDec(ref_latency);

    touch= LaunchPad_Input();
    if (touch== 1){
        ref_latency+= 1;
    }else if (touch== 2){
        ref_latency-= 1;
    }else if (touch== 3){
        Spt= &fsm[0];
        stop_flag= 0;
    }
}


int main_t(void){
    Clock_Init48MHz();
    LaunchPad_Init();
    Nokia5110_Init();
    Nokia5110_ClearBuffer();
    Nokia5110_Clear();


    Bump_Init();
    Motor_Init();
    Reflectance_Init();

    PWM_Init34(15000, 5000, 5000); //10 ms period motor set up
    //TimerA1_Init(&BumpCheck,500);  // 1000 Hz bump check

    TimerA1_Init(&hand_tuning_module, 48000);
    TimerA2_Init(&Reflectance_Handler,4800);  // Can't use timer A0, because it is being used for the motor PWM; reflectance checking

    EnableInterrupts();

    Nokia5110_SetCursor(0, 0);         // five leading spaces, bottom row
    Nokia5110_OutString("-Test mode-");

    Spt = Center;

    stop_flag= 1;
    while (LaunchPad_Input()!= 3){
        //Motor_Right(speedMax, speedMax);
    }
    TimerA1_Init(&BumpCheck,480);  // 1000 Hz bump check
    stop_flag= 0;
    while(1){

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
    //TimerA1_Init(&BumpCheck,500);  // 1000 Hz bump check

    TimerA1_Init(&hand_tuning_module, 48000);
    TimerA2_Init(&Reflectance_Handler,480);  // Can't use timer A0, because it is being used for the motor PWM; reflectance checking

    EnableInterrupts();

    Nokia5110_SetCursor(0, 0);         // five leading spaces, bottom row
    Nokia5110_OutString("-Test mode-");

    Spt = Center;


    while (LaunchPad_Input()!= 3){
        //Motor_Right(speedMax, speedMax);
    }

    Nokia5110_SetCursor(0, 0);         // five leading spaces, bottom row
    Nokia5110_OutString("-Act  mode-");

    TimerA1_Init(&BumpCheck,480);  // 1000 Hz bump check
//    TimerA2_Init(&Tune_Handler,480);  // Can't use timer A0, because it is being used for the motor PWM; reflectance checking

    //tmp_ref_latency= ref_latency;
    while (1){
//      while (tuned_flag< 5){ //Tune the line sensor first; assuming the lighting conditions would be pretty even
//      }
//      TimerA2_Init(&Reflectance_Handler,480);  // Can't use timer A0, because it is being used for the motor PWM; reflectance checking
//      tuned_flag= 0;

      Spt = Center;
      while(1){
        //test_module();
        if (stop_flag== 0){
            Output = Spt->out;            // set output from FSM
            pSpt= Spt;

            switch(Spt->uid) {
               case 'c': //0 to 14,998
                  Motor_Forward(speedMax, speedMax);
                  break;

               case 'C':
                   Motor_Forward(speedMin, speedMin);
                   break;

               case 'r': //Right 1
                  Motor_Forward(speedMin/3, speedMin);
                  break;

               case 'R': //Right 2
                   Motor_Forward(speedMin/3, speedMin);
                  break;

               case 'l': //Left 1
                  Motor_Forward(speedMin, speedMin/3);
                  failure_count= 0;
                  break;

               case 'L': //Left 2
                   Motor_Forward(speedMin, speedMin/3);
                  break;

               case 'Q': //E Left
                  Motor_Left(speedMax/2, speedMax/2);
                  break;

               case 'E': //E RIght
                  Motor_Right(speedMax/2, speedMax/2);
                  break;

//               case 's':
////                   Motor_Stop();
//                   Motor_Right(speedMax/2, speedMax/2);
//                   break;
//               default :
//                   Motor_Stop();
//                   break;
            }

            //LaunchPad_Output(Output);     // do output to two motors
            //TExaS_Set(Input<<2|Output);   // optional, send data to logic analyzer

            //Clock_Delay1ms(Spt->delay);   // wait
            //Input = LaunchPad_Input();    // read sensors
        }else{
            if (LaunchPad_Input()== 3){
                stop_flag= 0;
            }
        }
      }
    }
}

void SetFSM(void){uint32_t heart=0;
    //P2->OUT= 0x02;
    Output = Spt->out;            // set output from FSM
    //Clock_Delay1ms(Spt->delay);   // wait
    //Input = LaunchPad_Input();    // read sensors

    Nokia5110_SetCursor(0, 2);         // five leading spaces, bottom row
    Nokia5110_OutString("In:  ");
    Nokia5110_OutUDec(Input);

    Spt = Spt->next[Input];       // next depends on input and state
    heart = heart^1;
    LaunchPad_LED(heart);         // optional, debugging heartbeat

    Nokia5110_SetCursor(0, 5);         // five leading spaces, bottom row
    Nokia5110_OutString(Spt->name);
    //P2->OUT&= ~0x02;
}

int main_(void){
  Clock_Init48MHz();
  LaunchPad_Init();
  Nokia5110_Init();
  Nokia5110_ClearBuffer();
  Nokia5110_Clear();


  Bump_Init();
  Motor_Init();
  Reflectance_Init();

  PWM_Init34(15000, 5000, 5000); //10 ms period motor set up
  //TimerA1_Init(&BumpCheck,500);  // 1000 Hz bump check

  TimerA1_Init(&hand_tuning_module, 48000);
  TimerA2_Init(&Reflectance_Handler,4800);  // Can't use timer A0, because it is being used for the motor PWM; reflectance checking

  EnableInterrupts();

  Nokia5110_SetCursor(0, 0);         // five leading spaces, bottom row
  Nokia5110_OutString("-Test mode-");

  Spt = Center;

  while(1){
  }
}


