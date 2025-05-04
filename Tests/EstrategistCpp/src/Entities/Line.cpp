#include "Line.h"
#include <iostream>
#include <cmath>

using namespace std;

Line::Line(Vector2 s, Vector2 f) : start(s), end(f) {
    if(s.x > f.x){
        Vector2 t = s;
        s = f;
        f = t;
    }
    Vector2 dif = start -end;
    m = dif.y / dif.x;
    b = start.y - m* start.x;
    cout<<"Line ----------- m: "<<m<< " -- b: "<<b<<endl;
}

Vector2 Line::Intersect(Transform ball)  {
    
    cout<<"\n///////////cheking intersect to a horizontalLine: \n"<<endl;
    Vector2 fi(-1000,-1000);
    float _m = ball.velocities.y/ ball.velocities.x;
    float _b = ball.position.y - _m*ball.position.x;
    cout<<"Ball----------- m: "<<_m<< "-- b: "<<_b<<endl;
    if(end.y == start.y ){
        float intersection_y = _m ? (end.y-_b)/_m: -1000;
        if(start.x <  intersection_y && intersection_y < end.x){
            fi = Vector2(intersection_y, end.y);
        }
    }else{
        float fy = (m - _m) ?  (-b*_m + _b*m) / (m - _m): -1000;
        float fx = (fy - b)/m;
        fi = Vector2(fx, fy);
    }
    
    cout<<"Intersection at: "<< fi <<endl;
    return fi;
}

