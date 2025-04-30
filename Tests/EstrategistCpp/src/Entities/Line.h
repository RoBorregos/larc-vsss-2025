#ifndef LINE_H
#define LINE_H
#include "Transform.h"

#include <utility>

class Line {
public:
    Vector2 start;
    Vector2 end;
    float m;
    float b;
    Vector2 Intersect(Transform ball);
    Line(Vector2 s,  Vector2 f);

};

#endif // LINE_H