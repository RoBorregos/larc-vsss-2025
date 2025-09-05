#pragma once
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <tf2/LinearMath/Vector3.h>
using namespace tf2;
using namespace std;

class Polygon{
    public:
        Polygon();
        Polygon(vector<Vector3> );
        bool isInside(Vector3);
        vector<Vector3> poly;
};