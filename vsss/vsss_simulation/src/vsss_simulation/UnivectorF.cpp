#include "vsss_simulation/UnivectorF.hpp"

float de = 0.8/3;
float kr = 0.2/3;
float ko = 0.5/3;
float d_min = 0.2/3;
float delta__ = 0.43/3;
float phiH(float rho,  float theta, bool cw)    //Hyperbolic angle
{
    float angle = 0;
    if (rho > de){
        angle = (M_PI / 2) * (2 - ((de + kr) / (rho + kr)));
    }
    else if(0 <= rho && rho <= de){
        angle = (M_PI / 2) * sqrt(rho / de);
    } 

    if (cw){
        return wrapToPI(theta + angle);
    }
    else{
        return wrapToPI(theta - angle);
    }
}


float phiTuf(float theta, Vector3 p ,Vector3 b,  Line& trajectory) { // Move to Goal
    /*
    Merges a clockwise and a counterclockwise hyperbolic spiral and returns a coefficient of 
    movement that guides the robot to the ball, following the smallest path 
    */
    p[2] = 0; b[2] = 0;
    float spiralTheta = wrapToPI(trajectory.getTheta() + M_PI/2);
    Vector3 SpiralPos (de * cos(spiralTheta), de * sin(spiralTheta) ,0);
    Vector3 LeftSp = b+SpiralPos;  //y_r
    Vector3 RightSp = b-SpiralPos ; //y_l

    //Distance to the two spirals
    float ro_l = LeftSp.distance(p);
    float ro_r = RightSp.distance(p);
    
    //objective of each spiral
    float phi_ccw = phiH(ro_l, theta, true);
    float phi_cw = phiH(ro_r, theta, false);

    Vector3 nh_ccw = Theta2Vector(phi_ccw);
    Vector3 nh_cw = Theta2Vector(phi_cw);


    /*
    Returns a coefficient of a hyperbolic spiral that guides the robot to the ball
    '''
    '''
    The direction of rotation of the spiral has been inverted, cause by passing as in the article, 
    the clockwise direction becomes counterclockwise and vice versa
    '''*/


    //Distance from the robot to the trayectory line
    float distance_from_trayectory = trajectory.Dist(p);
    if(ro_l >= ro_r){
        distance_from_trayectory *= -1;
    }
    //merge in between
    Vector3 spiral_merge = (abs(distance_from_trayectory+de) * nh_ccw + abs(distance_from_trayectory-de) * nh_cw) / (2 * de) ;
    //Final angle depending from the distance to the trayectory
    float phi_tuf;
    if (-de <= distance_from_trayectory && distance_from_trayectory < de){
        phi_tuf = atan2(spiral_merge[1], spiral_merge[0]);
    }else if( distance_from_trayectory < -de){
        phi_tuf = phiH(ro_r, theta, false);
    }else{
        phi_tuf = phiH(ro_l, theta, true);
    }

    return wrapToPI(phi_tuf);



}


Vector3 getImagePos(Kinematic main, Kinematic obst){
    Vector3 imag = ko*(obst.velocity - main.velocity);
    imag[2] = 0;
    float distImag = imag.length();
    float distReal = (main.transform.getOrigin()-obst.transform.getOrigin()).length();
    Vector3 imagPos;
    if(distReal >= distImag){
        imagPos = obst.transform.getOrigin() + imag;   
    }else{
        imagPos = obst.transform.getOrigin() + (distReal/distImag) * imag ;
    }
    return imagPos;
}

float phiAuf(Kinematic main, Kinematic obst){
    Vector3 Imag = getImagePos(main, obst);
    Vector3 result = main.transform.getOrigin() - Imag;
    return wrapToPI(atan2(result[1], result[0]));
}



float phiCompose(float ball_c, float enemy_c, float distance_){
    if(distance_ <= d_min){
        return enemy_c;
    }
    float guass = gaussian(distance_ - d_min, delta__);
    float diff = wrapToPI(enemy_c - ball_c);
    return wrapToPI(guass * diff + ball_c);

}