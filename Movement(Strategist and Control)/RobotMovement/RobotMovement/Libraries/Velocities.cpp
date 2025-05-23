#include "Arduino.h"

class velocities
{
  public:
  float _x;
  float _y;
  float _z;
  float Magnitude()
  {
    return sqrt(pow (_x, 2) + pow(_y, 2));
  }
  //Get the angle of a vector if it is in a (x, y) form as I didnt find a atan2 function
  void setAngule(){ 
    if(_x  == 0) {      // if _x is 0 check wether _y is positive or negative
      _z = _y > 0 ? PI/2 : 3*PI/2;
    }else if(_y == 0){  // if _y is 0 check wether _x is positive or negative
      _z = _x > 0 ? 0 : PI;
    }else{  //As the atan function return between -PI < z < PI. if x is negative, the angule should be added PI and if not, just make the negative angle positive 
        _z = atan(_y/_x);                
        _z = _x > 0? (_z < 0? 2*PI + _z : _z) : PI + _z;   
    }

  }
  //Get the least difference between angles
  //It finds the lowest between the two rotational option
  float getThetaDif(float desire, float actual){
      float dif = desire - actual;
      while(abs(dif) > 2*PI){
        dif /= 2*PI;
      }
      if(abs(dif) < PI){
        return dif;
      }else{
        return (signbit(dif) ? 1 : -1) * 2*PI + dif;
      }
  }

  void Print(){
    Serial.print(_x,4);Serial.print(" ");Serial.print(_y,4);Serial.print(" ");Serial.print(_z,4);
  }

  velocities(){
    _x = 0;
    _y = 0;
    _z = 0;
  }
  

};
