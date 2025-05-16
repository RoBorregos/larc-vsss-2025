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

// Calcula el punto de intersección entre la línea y la trayectoria de la pelota (representada por un Transform).
// Devuelve un Vector2 con la posición de la intersección, o (-1000, -1000) si no hay intersección válida.
Vector2 Line::Intersect(Transform ball)  {
    // Imprime información de depuración sobre la intersección.
    cout<<"\n///////////cheking intersect to a horizontalLine: \n"<<endl;
    Vector2 fi(-1000,-1000); // Valor por defecto si no hay intersección.
    float _m = ball.velocities.y/ ball.velocities.x; // Calcula la pendiente de la trayectoria de la pelota.
    float _b = ball.position.y - _m*ball.position.x; // Calcula la ordenada al origen de la trayectoria de la pelota.
    cout<<"Ball----------- m: "<<_m<< "-- b: "<<_b<<endl;

    // Si la línea es horizontal.
    if(end.y == start.y ){
        float intersection_y = _m ? (end.y-_b)/_m: -1000; // Calcula la abscisa de intersección.
        if(start.x <  intersection_y && intersection_y < end.x){
            fi = Vector2(intersection_y, end.y); // Si está dentro del segmento, guarda el punto.
        }
    // Si la línea es vertical.
    else if(end.x == end.y){
        float intersection_x = _m* end.x +_b; // Calcula la ordenada de intersección.
        if(start.y < intersection_x && intersection_x < end.y){
            fi = Vector2(start.x, intersection_y); // Si está dentro del segmento, guarda el punto.
        }
    }
    }else{
        // Para líneas inclinadas, calcula la intersección entre dos rectas.
        float fy = (m - _m) ?  (-b*_m + _b*m) / (m - _m): -1000;
        float fx = (fy - b)/m;
        fi = Vector2(fx, fy);
    }
    
    cout<<"Intersection at: "<< fi <<endl; // Imprime el resultado.
    return fi;
}

// Establece los puntos inicial y final de la línea y calcula su pendiente y ordenada al origen.
void Line::SetLine(Vector2 s, Vector2 f) {
    start = s;
    end = f;
    // Si el punto inicial está a la derecha, intercambia los puntos para mantener el orden.
    if(s.x > f.x){
        Vector2 t = s;
        s = f;
        f = t;
    }
    Vector2 dif = start -end; // Calcula la diferencia entre los puntos.
    m = dif.y / dif.x;        // Calcula la pendiente.
    b = start.y - m* start.x; // Calcula la ordenada al origen.
    cout<<"Line ----------- m: "<<m<< " -- b: "<<b<<endl; // Imprime información de depuración.
}

// Calcula y devuelve el punto medio del segmento de línea.
Vector2 Line::MidPoint() {
    return Vector2((start.x + end.x) / 2, (start.y + end.y) / 2);
}

