#include "Line.h"
#include <iostream>
#include <cmath>

using namespace std;

Line::Line(Vector2 s, Vector2 f) : start(s), end(f) {
    if(s.y == f.y && s.x > f.x){
        Vector2 t = s;
        s = f;
        f = t;
    }
    if(s.x == f.x && s.y > f.y){
        Vector2 t = s;
        s = f;
        f = t;
    }
    Vector2 dif = start -end;
    m = dif.x ? dif.y / dif.x: 0;
    b = start.y - m* start.x;
    cout<<"Line ----------- m: "<<m<< " -- b: "<<b<<endl;
}

Line::Line() : start(0, 0), end(0, 0), m(0), b(0) {
    cout<<"Line ----------- m: "<<m<< " -- b: "<<b<<endl;
}
// Calculates the intersection point between the line and the ball's trajectory (represented by a Transform).
// Returns a Vector2 with the intersection position, or (-1000, -1000) if there is no valid intersection.

// Calculates the intersection point between the line and the ball's trajectory (represented by a Transform).
// Returns a Vector2 with the intersection position, or (-1000, -1000) if there is no valid intersection.

Vector2 Line::Intersect(Transform ball)  {
    // Prints info about the line intersection
    cout<<"\n///////////cheking intersect to a horizontalLine: \n"<<endl;
    Vector2 fi(-1000,-1000); // Default value if no intersection
    float _m = ball.velocities.y/ ball.velocities.x; // Get the slope of the ball trajectory
    float _b = ball.position.y - _m*ball.position.x; // Calculates the y-intercept of the ball's trajectory.
    cout<<"Ball----------- m: "<<_m<< "-- b: "<<_b<<endl;

    // If the line is horizontal.
    if(end.y == start.y ){
        float intersection_y = _m ? (end.y-_b)/_m: -1000; // Calculates the x-coordinate of the intersection.
        if(start.x <  intersection_y && intersection_y < end.x){
            fi = Vector2(intersection_y, end.y); // If it's within the segment, save the point.
        }
    // If the line is vertical.
    else if(end.x == end.y){
        float intersection_x = _m* end.x +_b; // Calculates the y-coordinate of the intersection.
        if(start.y < intersection_x && intersection_x < end.y){
            fi = Vector2(start.x, intersection_y); // If it's within the segment, save the point.
        }
    }
    }else{
        // For two lines with slope != ( 0 || INF ) , calculate the intersection between two lines.
        float fy = (m - _m) ?  (-b*_m + _b*m) / (m - _m): -1000;
        float fx = (fy - b)/m;
        fi = Vector2(fx, fy);
    }
    
    cout<<"Intersection at: "<< fi <<endl; // Prints the result.
    return fi;
}

// Sets the initial and final points of the line and calculates its slope and y-intercept.
void Line::SetLine(Vector2 s, Vector2 f) {
    start = s;
    end = f;
    // Depending if the rect is horizontal or vertical change the positions of the points to mach a sequence start < end
   if(s.y == f.y && s.x > f.x){
        Vector2 t = s;
        s = f;
        f = t;
    }
    if(s.x == f.x && s.y > f.y){
        Vector2 t = s;
        s = f;
        f = t;
    }
    Vector2 dif = start -end; // Calculates the difference between the points.
    m = dif.y / dif.x;        // Calculates the slope.
    b = start.y - m* start.x; // Calculates the y-intercept.
    cout<<"Line ----------- m: "<<m<< " -- b: "<<b<<endl; // Prints debug information.
}

// Calculates and returns the midpoint of the line segment.
Vector2 Line::MidPoint() {
    return Vector2((start.x + end.x) / 2, (start.y + end.y) / 2);
}