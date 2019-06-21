#include "System.h"
extern float Ts;

void System::Setup(int motorT_EN, int motorT_dir1, int motorT_dir2, int motorR_EN, int motorR_dir1, int motorR_dir2, int linear_sensor, int rotary_sensor, float zeta, float omega){
  motorT = new Motor(motorT_EN, motorT_dir1, motorT_dir2);
  motorT->Setup();
  
  motorR = new Motor(motorR_EN, motorR_dir1, motorR_dir2);
  motorR->Setup();
  
  LPM_pin = linear_sensor;
  RPM_pin = rotary_sensor;  

  KFT = new Kalman;
  KFT->sensor_kf = linear_sensor;
  KFT->KF_Initialize();
  
  KFR = new Kalman;
  KFR->sensor_kf = rotary_sensor;
  KFR->KF_Initialize();

  fT = new Filter;
  fT->dt = Ts;
  fT->filter_setup(zeta,omega);

  fR = new Filter;
  fR->dt = Ts;
  fR->filter_setup(zeta,omega);

  DRC_translate = new Controller;
  DRC_translate->DRC_translate_setup();
  
  DRC_rotate = new Controller;
  DRC_rotate->DRC_rotate_setup();
}

void System::translate(float set){
  LPM_value = analogRead(LPM_pin);
  KFT->predict(LPM_value/4096*10);
  fT->filter(set); 
  DRC_translate->controlT(fT->y,fT->ydot,fT->yddot,KFT->xk_k[0],KFT->xk_k[1]);

  if (set - fT->y < 0.1){
    if ((KFT->xk_k[0] - fT->y) < 0.01 && (KFT->xk_k[0] - fT->y) > -0.01) {
      DRC_translate->u = 0;

    }
  }    
  
  if (DRC_translate->u >= 5) DRC_translate->u = 5;
  if (DRC_translate->u <= -5) DRC_translate->u = -5;  
  if (safe_TM(DRC_translate->u))UTQ = DRC_translate->u/9.6*255;
  if (!safe_TM(DRC_translate->u))UTQ = 0;

  motorT->Spin(UTQ); 

  KFT->uk1=DRC_translate->u;
  
  for (int i=0;i<3;i++){
    KFT->xk1_k1[i]=KFT->xk_k[i];
    for (int j=0;j<3;j++){
      KFT->zk1[i][j] = KFT->zk[i][j];
    }
  }
}



void System::rotate(float set){
  RPM_value = analogRead(RPM_pin);
  KFR->predict(RPM_value/4096*180);  
  fR->filter(set);
  DRC_rotate->controlR(fR->y*PI/180,fR->ydot*PI/180,fR->yddot*PI/180,KFR->xk_k[0]*PI/180,KFR->xk_k[1]*PI/180);

  if (DRC_rotate->u > 5) DRC_rotate->u = 5;
  if (DRC_rotate->u < -5) DRC_rotate->u = -5;  
  if (safe_RM(DRC_rotate->u))URQ = DRC_rotate->u/9.6*255;
  if (!safe_RM(DRC_rotate->u))URQ = 0;

  motorR->Spin(URQ); 


  KFR->uk1=DRC_rotate->u;
  
  for (int i=0;i<3;i++){
    KFR->xk1_k1[i]=KFR->xk_k[i];
    for (int j=0;j<3;j++){
      KFR->zk1[i][j] = KFR->zk[i][j];
    }
  }
}
