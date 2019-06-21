#include "math.h"
#include "Arduino.h"

 

class Kalman{
  public:
    int sensor_kf;
    
    void KF_Initialize();
    float* predict(float sysout);
    
    float Ad[3][3] = { { 1,0.0004995508423249883,0.0000001004338470130844 },{ 0,0.998203501502157,0.0002018996031454493 },{ 0,-0.002030160743219,-0.0000004106263379766955 } };
    float Bd[3] = { 0.000001053424329572821,0.004237715063843,0.126573711825744 }; float Cd[3] = { 1,0,0 };
    float eye[3][3]={{1,0,0},{0,1,0},{0,0,1}}; 
    float QQ[3][3] = {{500, 500, 500},{500, 550, 500},{500, 500, 500}};
    float Mkf[3][3]={{0, 0, 0},{0, 0, 0},{0, 0, 0}};float zk1[3][3]={{0.1,0,0},{0,0.1,0},{0,0,0.1}};float zk[3][3]={{0, 0, 0},{0, 0, 0},{0, 0, 0}};
    float xk1_k1[3]={0,0,0};float xk_k1[3]={0,0,0};float xk_k[3]={0,0,0};float Lkf[3];
    float uk1, V;
    
    float temp1[3], temp2[3],temp3[3][3],temp4[3][3],temp5[3],temp6,temp7[3],temp8;
    float temp9[3],temp10[3][3],temp11[3][3];//,temp7[3],temp5[3],temp6[3],temp7[3],
};

void sum31(float A[3], float B[3], float C[3], float sign);

void sum33(float A[3][3], float B[3][3], float C[3][3], float sign);

void mults31(float A[3], float a, float B[3]);

void mults33(float A[3][3], float a, float B[3][3]);

void mult33(float A[3][3], float B[3][3], float C[3][3]);

void mult31(float A[3][3], float B[3], float C[3]);

void mult13(float A[3], float B[3][3], float C[3]);

void mult1331(float A[3], float B[3], float *C);

void mult3113(float A[3], float B[3], float C[3][3]);

void transpose(float A[3][3], float B[3][3]);

void inverse(float A[3][3], float B[3][3]);
