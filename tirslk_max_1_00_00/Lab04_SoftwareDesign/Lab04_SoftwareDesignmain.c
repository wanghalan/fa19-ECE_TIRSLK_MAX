// Lab04_SoftwareDesignmain.c
// Runs on MSP432
// Solution to Software Design lab
// Daniel and Jonathan Valvano
// July 11, 2019

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
#include <stdint.h>
enum scenario {
    Error = 0,
    LeftTooClose = 1,
    RightTooClose = 2,
    CenterTooClose = 4,
    Straight = 8,
    LeftTurn = 9,
    RightTurn = 10,
    TeeJoint = 11,
    LeftJoint = 12,
    RightJoint = 13,
    CrossRoad = 14,
    Blocked = 15
};

enum dir_state{
    Open= 0,
    InRange= 1,
    Close= 2,
};

typedef enum scenario scenario_t;

#define SIDEMAX 354    // largest side distance to wall in mm
#define SIDEMIN 212    // smallest side distance to wall in mm
#define CENTEROPEN 600 // distance to wall between open/blocked
#define CENTERMIN 150  // min distance to wall in the front

#define MININPUT 50 //min input value
#define MAXINPUT 800 //max input value

scenario_t Classify(int32_t Left, int32_t Center, int32_t Right){
  scenario_t result=Error;
  // write this code
  int danger_condition= 0;
  int right_state;
  int left_state;
  int center_state;

  if ((MININPUT <= Left && Left <= MAXINPUT) && (MININPUT <= Right && Right <= MAXINPUT) && (MININPUT <= Center && Center <= MAXINPUT)){
      //if left, right, or center are within error range

      if (Left < SIDEMIN){
          danger_condition+= LeftTooClose;
      }
      if (Right < SIDEMIN){
          danger_condition+= RightTooClose;
      }
      if (Center < CENTERMIN){
          danger_condition+= CenterTooClose;
      }

      if (danger_condition!= 0){
          result= danger_condition;
      }else{//Neither error, nor danger condition

          //Setting up Side and center condition states
          if (Left >= SIDEMAX){
              left_state= Open;
          }else if (SIDEMIN <= Left && Left < SIDEMAX){
              left_state= InRange;
          }
          if (Right >= SIDEMAX){
              right_state= Open;
          }else if (SIDEMIN <= Right && Right < SIDEMAX){
              right_state= InRange;
          }
          if (Center >= CENTEROPEN){
              center_state= Open;
          }else if (CENTERMIN <= Center && Center < CENTEROPEN){
              center_state= InRange;
          }


          if (center_state == Open){
              //Straight, right joint, left joint, cross road
              //If the robot were in the middle of a road with the straight or blocked scenarios, both side sensors (placed at 45 degrees) would read 283 mm (In Range)
              if (left_state== InRange && right_state== InRange){
                  return Straight;
              }
              if (left_state== InRange && right_state==Open){
                  return RightJoint;
              }
              if (right_state== InRange && left_state== Open){
                  return LeftJoint;
              }
              if (right_state== Open && left_state== Open){
                  return CrossRoad;
              }

          }else if (center_state== InRange){
              //Blocked, right turn, left turn, tee joint
              if (left_state==InRange && right_state==InRange){
                  return Blocked;
              }
              if (left_state== InRange && right_state==Open){
                  return RightTurn;
              }
              if (right_state== InRange && left_state== Open){
                  return LeftTurn;
              }
              if (right_state== Open && left_state== Open){
                  return TeeJoint;
              }
          }
      }
  }
  return result;
}

#define IRSlope 1195172
#define IROffset -1058
#define IRMax 2552

int32_t Convert(int32_t n){
    // write this code
  //return 0; // replace this line
    if (n < IRMax){
        return 800;
    }else
        return IRSlope/(n+IROffset);
//    return IRSlope/(n+IROffset);
}
// ***********testing of Convert*********
int32_t const ADCBuffer[16]={2000, 2733, 3466, 4199, 4932, 5665, 6398, 7131, 7864, 8597, 9330, 10063, 10796, 11529, 12262, 12995};
int32_t const DistanceBuffer[16]={800, 713, 496, 380, 308, 259, 223, 196, 175, 158, 144, 132, 122, 114, 106, 100};
void Program4_1(void){
  int i;
  int32_t adc,distance,errors,diff;
  errors = 0;
  for(i=0; i<16; i++){
    adc = ADCBuffer[i];
    distance = Convert(adc); // call to your function
    diff = distance-DistanceBuffer[i];
    if((diff<-1)||(diff>1)){
      errors++;
    }
  }
  while(1){
      //printf("%d\n", errors);
  };
}
// ***********end of testing of Convert*********



// ***********testing of classify
scenario_t Solution(int32_t Left, int32_t Center, int32_t Right);
int32_t const CornerCases[18]={49,50,51,149,150,151,211,212,213,353,354,355,599,600,601,799,800,801};
int32_t errors;
void Program4_2(void){
  enum scenario result,truth;
  int i,j,k;
  int32_t left, right, center; // sensor readings
  errors = 0;
  for(i=0; i<18; i++){
    left = CornerCases[i];
    for(j=0; j<18; j++){
      center = CornerCases[j];
      for(k=0; k<18; k++){
        right = CornerCases[k];
        result = Classify(left,center,right); // your solution
        truth = Solution(left,center,right);  // correct answer

        if(result != truth){
           errors++;
           //printf("%d | %d\n", result, truth);
        }
      }
    }
  }
  while(1){
      printf("%d\n", errors);
  }
}

void Program4_3(void){ // will take over 16 hours to complete
  enum scenario result,truth;
  int32_t left, right, center; // sensor readings
  errors = 0;
  for(left=0; left<1000; left++){
    for(center=0; center<1000;  center++){
      for(right=0; right<1000; right++){
        result = Classify(left,center,right); // your solution
        truth = Solution(left,center,right);  // correct answer
        if(result != truth){
           errors++;
        }
      }
    }
  }
  while(1){
  }
}

void main(void){
  // run one of these
//  Program4_1();
  Program4_2();
//  Program4_3();
}
