#include "Arduino.h"
#include "motor.h"
#include "Controller.h"
#include "Kalman.h"



#ifndef uint8
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long uint32;
#endif //


class Filter{
  protected:
    float zeta;
    float omega;   

    uint8 type;
  public:
    float y,ydot,yddot;
    float dt; 
    
    void filter_setup(float zeta, float omega){
      this->zeta = zeta;
      this->omega = omega;
      y=ydot=yddot=0;
    }
    
    void filter(float uc){
      yddot = -omega*omega*y - 2*zeta*omega*ydot + omega*omega*uc;
      y = ydot * dt + y;
      ydot = yddot * dt + ydot;
    }
};


class System{
  public: 
    float Position, Speed, Position_ref, Speed_ref;
    int LPM_pin, RPM_pin;
    float LPM_value, RPM_value;
    float UT,UR,UTQ,URQ;
    
    void Setup(int motorT_EN, int motorT_dir1, int motorT_dir2, int motorR_EN, int motorR_dir1, int motorR_dir2, int linear_sensor, int rotary_sensor, float zeta, float omega);   
    void translate(float set);
    void rotate(float set);

    uint8 safe_TM(float effort) {
      if ((LPM_value < 4090 && effort > 0) || (LPM_value > 0 && effort < 0) || (LPM_value > 0 && LPM_value < 4090)) {
        return 1;
      }
      else {
        return 0;
      }
    }
    uint8 safe_RM(float effort) {
      if  (RPM_value < 4090 && effort > 0) {
        return 1;
      }
      if (RPM_value > 500 && effort < 0) {
        return 1;
      }
      if (RPM_value > 500 && RPM_value < 4090) {
        return 1;
      }
      else return 0;
    }

    Motor* motorT;
    Motor* motorR;
    Kalman *KFT;  
    Kalman *KFR;
    Filter *fT;
    Filter *fR;  
    Controller *DRC_translate;
    Controller *DRC_rotate;
    
    
};
