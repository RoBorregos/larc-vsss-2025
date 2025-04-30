#include "Output.h"
#include <cmath>

Output::Output(){
    a = 0;
    b = 0;
}
Output::Output(float a_, float b_){
    a = a_;
    b = b_;
}
// This function serves as a way to normalize the output of the robot to a certain scale
void Output::Scale(float MAX){
    if(abs(a) > MAX || abs(b) > MAX){
        if(abs(a) > abs(b)){
            float scale = MAX/abs(a);
            a = MAX * (std::signbit(a) ? -1 : 1);
            b = b * scale;
        }else{
            float scale = MAX/abs(b);
            b = MAX * (std::signbit(b) ? -1 : 1);
            a = a * scale;
        }
    }
}

ostream& operator<<(ostream& os, const Output& output)
{
    os << "Output(a: " << output.a << ", b: " << output.b << ")";
    return os;
}