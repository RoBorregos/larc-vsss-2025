#include "vsss_simulation/Polygon.hpp"

Polygon::Polygon(){}

Polygon::Polygon(vector<Vector3> n){
    figure = n;
    poly = figure;
    origin = Vector3(0,0,0);
    rotation = 0;
}


bool Polygon::isInside(Vector3 p){
    bool inside = false;
    float minX = poly[0].x(), maxX = poly[0].x();
    float minY = poly[0].y(), maxY = poly[0].y();
    for(auto point: poly){
        minX = min((float)point.x(), minX);
        maxX = max((float)point.x(), maxX);
        minY = min((float)point.y(), minY);
        maxY = max((float)point.y(), maxY);
    }
    if(p.x() < minX || p.x() > maxX || p.y() < minY || p.y() > maxY ){
        return false;
    }
    int j = poly.size() - 1;
    for(int i =0; i < poly.size();  i++ ){
        if((poly[i].y() > p.y()) != (poly[j].y() > p.y()) // create a line of the polygon and check if it the point is in its y range
                && p.x() < (poly[j].x()- poly[i].x() )* (p.y() - poly[i].y()) / (poly[j].y() - poly[i].y()) + poly[i].x()){ //Valid inside flip
                    inside = !inside;
        }
        j = i;
    }
    return inside;

}

float newX (Vector3 point, float angle){
    return point.x() * cos(angle) - point.y() * sin(angle);
}

float newY (Vector3 point, float angle){
    return point.x() * sin(angle) + point.y() * cos(angle);
}

void Polygon::translade(){
    poly = figure;

    // cout<<"Robot in: "<<origin.x()<<" "<<origin.y()<<endl;
    for(int i = 0; i < poly.size(); i ++){
        float X = newX(poly[i], rotation);
        float Y = newY(poly[i], rotation);
        poly[i] = Vector3(X,Y, 0) + origin;
        // cout<<"vertex: "<<i<<" -> "<<poly[i].x()<<" "<<poly[i].y()<<endl;
    }


}

bool Polygon::fullInside(Polygon& another){
    another.translade();
    for(auto point : another.poly){
        if(!this->isInside(point)){
            // cout<<"Robot not in field"<<endl;
            // cout<<point.x()<<" "<<point.y()<<endl;
            return false;
        }
    }
    // cout<<"Robot in field"<<endl;
    return true;
}

