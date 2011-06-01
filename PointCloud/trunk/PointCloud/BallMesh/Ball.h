#ifndef BALL_H
#define BALL_H

#include <math.h>

//for spline patched Gaussian
#define EE16 2.1653645317858030703

class Ball{
public:
  float cx, cy, cz;	//+��������+
  float px, py, pz;// ������ v
  float r;	//+��뾶+
  
  Ball(){
  }
  
  Ball(float x, float y, float z){
    cx = x;
    cy = y;
    cz = z;
  }
  //+�������еĵ��Ȩ��+
  inline double weight(float x, float y, float z){
    float vx = x - cx;
    float vy = y - cy;
    float vz = z - cz;
    
    return weight(sqrt(vx*vx+vy*vy+vz*vz), r);
  }
  //+�������еĵ��Ȩ��   GR(d)  +
  static inline double weight(double d, float R){
    //spline patched Gaussian  
    if(R < d)   //+λ������ʱΪ0+
      return 0;
    else{
      d /= R;
      if(d < 0.5f)
        return exp(-8*d*d);
      else{
        d = 1.0 - d;
        d = d*d;
        return EE16*d*d;
      }
    }
  }
};

#endif