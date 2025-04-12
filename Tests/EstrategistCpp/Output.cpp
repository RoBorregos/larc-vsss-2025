#include "Output.h"
#include <cmath>
using namespace std;

Output::Output(){
    a = 0;
    b = 0;
}
Output::Output(float a_, float b_){
    a = a_;
    b = b_;
}
// This function serves as a way to normalize the output of the robot to a certain scale
void Output::Scale(float scale){
    if(abs(a) > scale || abs(b) > scale){
        if(abs(a) > abs(b)){
            float scale = scale/abs(a);
            a = scale * (signbit(a) ? -1 : 1);
            b = b * scale;
        }else{
            float scale = scale/abs(b);
            b = scale * (signbit(b) ? -1 : 1);
            a = a * scale;
        }
    }
}