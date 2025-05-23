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


/*
 * This function normalizes the output values a and b to a specified maximum scale.
 * It takes the following parameter:
 *   - MAX: the maximum allowed absolute value for a or b.
 * If either a or b exceeds MAX in absolute value, both are scaled proportionally so that the largest equals MAX.
 */

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