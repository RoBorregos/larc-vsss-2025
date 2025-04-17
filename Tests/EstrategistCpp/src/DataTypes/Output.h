#ifndef OUTPUT_H
#define OUTPUT_H


//class used as a data type to transmite rpm directly to the robot
class Output
{
    public:
        float a;
        float b;
        Output();
        Output(float a_, float b_);
        void Scale(float scale);
};

#endif