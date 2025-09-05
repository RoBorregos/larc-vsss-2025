#include "vsss_simulation/Polygon.hpp"

Polygon::Polygon(){}

Polygon::Polygon(vector<Vector3> n){
    poly = n;
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
    int i =0, j = poly.size() - 1;
    for(; i < poly.size(); j == i++ ){
        if((poly[i].y() > p.y()) != (poly[j].y() > p.y()) // between two points check if line intersect with the point
                && p.x() < (poly[j].x()- poly[i].x() )* (p.y() - poly[i].y()) / (poly[j].y() - poly[i].y()) + poly[i].x()){
                    inside = !inside;
        }
    }

}