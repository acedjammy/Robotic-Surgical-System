#include "System.h"

System* Tool1 = new System;
System* Tool2 = new System;
System* Tool3 = new System;


hw_timer_t *timer = NULL;
uint32 loop_index = 0;
float Time = 0;
float Ts = 0.0005; //Second
float referenceT = 5;
uint32 powerT = 0;
float referenceR = 90;
uint32 powerR = 0;
volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;


void IRAM_ATTR onTimer() {
//  Tool1->motorT->Spin(100);
//  Time = Ts * loop_index++;

  if (loop_index % 12000 == 0) referenceT = referenceT + pow(-1,powerT++)*-1;  
  Tool1->translate(referenceT);

  if (loop_index++ % 10000 == 0) referenceR = referenceR + pow(-1,powerR++)*-10;  
  Tool1->rotate(referenceR);

//  Tool1->rotate(90);
//  Tool1->KFT->predict();
}



void setup(){
  Serial.begin(115200);
  
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000 / (1 / Ts), true);
  timerAlarmEnable(timer);

  Tool1->Setup(25,2,15,26,14,12,33,32,1,2);
  
}

void loop(){
//  Serial.print(Tool1->fT->y);
//  Serial.print(", ");
//  Serial.print(Tool1->KFT->xk_k[0]);
//  Serial.print(", ");
//  Serial.println(Tool1->LPM_value);

//  Serial.print(Tool1->fR->y);
//  Serial.print(", ");
//  Serial.print(Tool1->KFR->xk_k[0]);
//  Serial.print(", ");
//  Serial.println(Tool1->RPM_value);
}
