#include "math.h"
#include "Arduino.h"

class Controller{
  public:
    float m, b, Ja, R, L, Kt,Kb, n1, n2, Le, phi_p, Cf, ga, M, B, K, p, Asc, d0, Fc,k1,k2,h,ua,us,U;
    float phi[4], theta[4], theta_max[4], theta_min[4],Theta1,Theta2,Theta3,Theta4,Theta1max,Theta1min,Theta2max,Theta2min,Theta3max,Theta3min,Theta4max,Theta4min,FFgain;  
    float u;
    
    void DRC_translate_setup(){
      M=18.1137;B=65.1571;K=26.5823;k1=50;k2=5;
      Theta1=B;Theta2=0.0042;Theta3=0.0021;Theta4=M;
      Theta1max=Theta1*1.5;Theta2max=Theta2*1.5;Theta3max=Theta3*1.5;Theta4max=Theta4*1.5;  
      Theta1min=Theta1/1.5;Theta2min=Theta2/1.5;Theta3min=Theta3/1.5;Theta4min=Theta4/1.5;  
      theta_max[0]=Theta1max;theta_max[1]=Theta2max;theta_max[2]=Theta2max;theta_max[3]=Theta4max;
      theta_min[0]=Theta1min;theta_min[1]=Theta2min;theta_min[2]=Theta2min;theta_min[3]=Theta4min;  
      theta[0]=Theta1;theta[1]=Theta2;theta[2]=Theta3;theta[3]=Theta4;
    }

    void DRC_rotate_setup(){
      M=2.4787;B=8.9159;K=26.5823;k1=50;k2=1;
      Theta1=B;Theta2=0.0232;Theta3=1;Theta4=M;
      Theta1max=Theta1*1.5;Theta2max=Theta2*1.5;Theta3max=Theta3*1.5;Theta4max=Theta4*1.5;  
      Theta1min=Theta1/1.5;Theta2min=Theta2/1.5;Theta3min=Theta3/1.5;Theta4min=Theta4/1.5;  
      theta_max[0]=Theta1max;theta_max[1]=Theta2max;theta_max[2]=Theta2max;theta_max[3]=Theta4max;
      theta_min[0]=Theta1min;theta_min[1]=Theta2min;theta_min[2]=Theta2min;theta_min[3]=Theta4min;  
      theta[0]=Theta1;theta[1]=Theta2;theta[2]=Theta3;theta[3]=Theta4;
    }
    
    void controlT(float Set,float Set_dot, float Set_ddot,float Get, float Get_dot){
      p=(Get_dot-Set_dot)+k1*(Get-Set);
      phi[0]=-Get_dot;phi[1]=-tanh(Get_dot*1000);phi[2]=-2;phi[3]=-(Set_ddot-k1*(Get_dot-Set_dot));
      
      h=ua=0;
      for (int index = 0; index<4; index++){
        h += (theta_max[index]/(K*0.8)-theta_min[index]/(K*1.3))*abs(phi[index]);
        ua += -theta[index]*phi[index];
      }
      ua/=K;    
      us=-tanh(p)*(h+5); 
      u = ua-k2*p/K+us;
    }


    void controlR(float Set,float Set_dot, float Set_ddot,float Get, float Get_dot){           
      p=(Get_dot-Set_dot)+k1*(Get-Set);
      phi[0]=-Get_dot;phi[1]=-tanh(Get_dot*1000);phi[2]=-2;phi[3]=-(Set_ddot-k1*(Get_dot-Set_dot));
      
      h=ua=0;
      for (int index = 0; index<4; index++){
        h += (theta_max[index]/(K*0.8)-theta_min[index]/(K*1.3))*abs(phi[index]);
        ua += -theta[index]*phi[index];
      }
      ua/=K;    
      us=-tanh(p)*(h+0.05); 
      u = ua-k2*p/K+us;
    }

};
