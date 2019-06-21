#include "Kalman.h"

void Kalman::KF_Initialize(){
  uk1=0; V=0.000692; xk1_k1[0]=xk_k1[0]=xk_k[0]=analogRead(sensor_kf)/4096*10;
}

float* Kalman::predict(float sysout){
  mult31(Ad,xk1_k1,temp1);
  mults31(Bd,uk1,temp2);
  sum31(temp1,temp2,xk_k1,1);//step 1
  
  mult33(Ad,zk1,temp3);
  transpose(Ad,temp4);
  mult33(temp3,temp4,Mkf);//step 2
  sum33(Mkf,QQ,Mkf,1);

  mult13(Cd,Mkf,temp5);
  mult1331(temp5,Cd,&temp6);
  mult31(Mkf,Cd,temp7);
  mults31(temp7,1/(temp6+V),Lkf);//step 3

  mult1331(Cd,xk_k1,&temp8);
  mults31(Lkf, sysout - temp8, temp9);
  sum31(xk_k1,temp9,xk_k,1);//step 4

  mult3113(Lkf,Cd,temp10);
  sum33(eye,temp10,temp11,-1);
  mult33(temp11,Mkf,zk);//step 5


  return xk_k;
}


void sum31(float A[3], float B[3], float C[3], float sign) {
  for (int i = 0; i<3; i++) {
    C[i] = A[i] + B[i] * sign;
  }
}

void sum33(float A[3][3], float B[3][3], float C[3][3], float sign) {
  for (int i = 0; i<3; i++) {
    for (int j = 0; j<3; j++) {
      C[i][j] = A[i][j] + B[i][j] * sign;
    }
  }
}

void mults31(float A[3], float a, float B[3]) {
  for (int i = 0; i<3; i++) {
    B[i] = A[i] * a;
  }
}

void mults33(float A[3][3], float a, float B[3][3]) {
  for (int i = 0; i<3; i++) {
    for (int j = 0; j<3; j++) {
      B[i][j] = A[i][j] * a;
    }
  }
}

void mult33(float A[3][3], float B[3][3], float C[3][3]) {
  for (int i = 0; i<3; i++) {
    for (int j = 0; j<3; j++) {
      C[i][j] = 0;
      for (int k = 0; k<3; k++) {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}

void mult31(float A[3][3], float B[3], float C[3]) {
  for (int i = 0; i<3; i++) {
    C[i] = 0;
    for (int j = 0; j<3; j++) {
      C[i] += A[i][j] * B[j];
    }
  }
}

void mult13(float A[3], float B[3][3], float C[3]) {
  for (int i = 0; i<3; i++) {
    C[i] = 0;
    for (int j = 0; j<3; j++) {
      C[i] += A[j] * B[j][i];
    }
  }
}


void mult1331(float A[3], float B[3], float *C) {
  *C = 0;
  for (int i = 0; i<3; i++) {
    *C += A[i] * B[i];
  }
}

void mult3113(float A[3], float B[3], float C[3][3]) {
  for (int i = 0; i<3; i++) {
    for (int j = 0; j<3; j++) {
      C[i][j] = A[i] * B[j];
    }
  }
}


void transpose(float A[3][3], float B[3][3]) {
  for (int i = 0; i< 3; i++) {
    for (int j = 0; j< 3; j++) {
      B[i][j] = A[j][i];
    }
  }
}

void inverse(float A[3][3], float B[3][3]) {
  float determinant = 0;
  for (int i = 0; i < 3; i++)
    determinant = determinant + (A[0][i] * (A[1][(i + 1) % 3] * A[2][(i + 2) % 3] - A[1][(i + 2) % 3] * A[2][(i + 1) % 3]));

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      B[i][j] = ((A[(j + 1) % 3][(i + 1) % 3] * A[(j + 2) % 3][(i + 2) % 3]) - (A[(j + 1) % 3][(i + 2) % 3] * A[(j + 2) % 3][(i + 1) % 3])) / determinant;
  }
}
