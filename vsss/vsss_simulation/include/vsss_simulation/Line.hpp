#pragma once
#include <iostream>
#include <tf2/LinearMath/Vector3.h>
#include <math.h>
using namespace std;
using namespace tf2;


class Line{
    private:
        float m;
        float b;
        float theta;
        Vector3 Init;
        Vector3 End;
    public:
        Line();
        Line(Vector3 pos, Vector3 obj);
        Line(Vector3 init, float theta);
        void Refresh(Vector3 pos, Vector3 obj);
        float Dist(Vector3 pos);
        float Evaluate(float x);
        float getTheta();
        string print();
        pair<int, Vector3> Intersect(Line other); //The int explains the edge cases of a intersection
        
};
