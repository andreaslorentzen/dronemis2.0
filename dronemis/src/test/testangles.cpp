#include <iostream>
#include <stdio.h>
#include <math.h>

/* demo.c:  My first C program on a Linux */

float radiansBetweenTwoPoints(float x0, float y0, float x1, float y1);
double getRotaionalSpeed(double target_deg, double ori_deg);

using namespace std;

int main(void) {
    double degrees = 90;
    double ori_deg = 60.0123;// navData->rotation;
    double target_deg = ori_deg+degrees;;
    int iterations;

    iterations = ((int)degrees/30)+1;

    cout << "iterations = " << iterations << endl;

    float offset = 5;



    for(int i = 1; i < iterations; i++){

        if(i == iterations-1 && ((int)degrees % 30) != 0)
            target_deg = ori_deg + ((int)degrees%30);
        else
            target_deg = ori_deg + 30;

        if (target_deg > 360)
            target_deg = target_deg - 360;

        cout << "target deg = " << target_deg << endl;


        ori_deg = target_deg;

        /*do {
            ori_deg = navData->rotation;
            cmd.angular.z = getRotationalSpeed(target_deg, ori_deg);
            pub_control.publish(cmd);
        } while (ori_deg < target_deg - offset or ori_deg > target_deg + offset);
        hover(1);*/
    }


}

/*double getRotaionalSpeed(double target_deg, double ori_deg) {
    float dir; // direction
    float rot_speed; // calculated rotational speed
    float target_angle = atan2(y, x); // angle towards waypoint position
    float target_deg = target_angle * 180 / M_PI; // radians to degrees
    //printf("Orientation:\t%6.2f deg\n", ori_deg);
    //printf("Target angle:\t%6.2f deg\n", target_deg);
    target_deg = target_deg - ori_deg; // angle between drone orientation and angle towards waypoint
    if (target_deg < 0) target_deg = 360+target_deg; // "translate" orientation to 0
    printf("target_deg:\t%6.2f deg\n", target_deg);
    if (target_deg < 180) { // if angle < 180 turn left
        dir = -1;
        printf("Turn left\n");
    } else { // if angle >= 180 turn right
        dir = 1;
        target_deg = 360 - target_deg;
        printf("Turn right\n");
    }
   // printf("Angular difference:\t%6.2f deg\n", target_deg);
    rot_speed = target_deg*target_deg/200;
   // printf("Rotational speed:\t%6.2f deg\n", rot_speed);
    if (rot_speed > 0.5) rot_speed = 0.5;
    rot_speed = rot_speed * dir;
    //printf("Rotational speed:\t%6.2f deg\n", rot_speed);
    return rot_speed;

}


float radiansBetweenTwoPoints(float x0, float y0, float x1, float y1) {
    float len1 = sqrt(x0 * x0 + y0 * y0);
    float len2 = sqrt(x1 * x1 + y1 * y1);
    float dot = x0 * x1 + y0 * y1;
    float a = dot / (len1 * len2);
    float r;
    if (a >= 1.0)
        return 0.0;
    else if (a <= -1.0)
        return M_PI;
    else
        return acos(a);
}*/


