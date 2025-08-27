#ifndef PID_H
#define PID_H
#include "Arduino.h"

//Constant Deffinition


class PID{
    public: 
        float previo;
        float acumulativo;
        float diferencial;
        PID();
        float GetCorrection(float error){
            acumulativo += error; // Integral
            if ((error*acumulativo)<0) acumulativo=0;  // corrige el overshooting 
            diferencial = error - previo; // diferential
            previo = error;             
            return kp * error + ki * acumulativo + kd * diferencial;
        };
    private:
        const float kp =  0.1;
        const float kd = 0.003;
        const float ki = 0.045;

};



#endif