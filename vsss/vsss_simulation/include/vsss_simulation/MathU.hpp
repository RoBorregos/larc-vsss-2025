#pragma once
#include <cmath>
#include <vector>
#include <iostream>
#include <tf2/LinearMath/Vector3.h>
#include <math.h>
using namespace tf2;

using namespace std;

float wrapToPI(float a);

Vector3 Theta2Vector(float a);

float gaussian(float r, float constant);

vector<Vector3> Rectangle(Vector3 origin, float width, float hight);

