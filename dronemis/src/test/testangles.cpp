#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
/* demo.c:  My first C program on a Linux */


using namespace std;

int angleDirection(double a1, double a2);

double angleDifference(double a1, double a2);

double formatAngle(double angle);

void turnDegrees(double orientation , double degrees);

int main(void) {
    turnDegrees(90, 270);
}

void turnDegrees(double orientation , double degrees) {
    srand (time(NULL));
    if (orientation == 361)
        orientation = rand() % 360 + 1;
    if (degrees == 361)
        degrees = ((rand() % 2 - 1)) * (rand() % 361);
    double target = formatAngle(orientation + degrees);
    target = 270;
    double difference = angleDifference(orientation, target);
    int direction = angleDirection(orientation, target);
    int last_direction = direction;
    int last_ts = 0;
    int time_counter = 0;


    orientation = orientation + direction * 1;
    int debug_counter = 0;
    while (true) {
        int time = last_ts + rand() % 5 + 5;
        time_counter += (time - last_ts);
        orientation = formatAngle(orientation + direction * (rand() % 5 + 1));
        direction = angleDirection(orientation, target);
        difference = angleDifference(orientation, target);
        printf("orientation: %6.1f \t target: %6.1f \t direction: %d \t difference: %6.1f \t done-tolerance: %6.1f  \n", orientation, target, direction, difference, (float) ((time_counter * time_counter) / 2000 + 4));
        if (time_counter > 100) {
            //printf("HOVER\n");
            time_counter = 0;
            last_ts = last_ts + 1000;
            orientation = orientation + ((rand() % 2 - 1)) * ((rand() % 10 + 5));
            //printf("%d\n",((rand() % 2 - 1)) );
        }

        if (difference < (time_counter * time_counter) / 2000 + 4) {
            //hoverDuration(3);
            break;
        }


    }
    printf("DONE! orientation: %6.1f \t target: %6.1f \t direction: %d \t difference: %6.1f \n  ", orientation, target, direction, difference);
}

void turnDegrees(double orientation , double degrees) {
    srand (time(NULL));
    if (orientation == 361)
        orientation = rand() % 360 + 1;
    if (degrees == 361)
        degrees = ((rand() % 2 - 1)) * (rand() % 361);
    double target = formatAngle(orientation + degrees);
    target = 270;
    double difference = angleDifference(orientation, target);
    int direction = angleDirection(orientation, target);
    int last_direction = direction;
    int last_ts = 0;
    int time_counter = 0;


    orientation = orientation + direction * 1;
    int debug_counter = 0;
    while (true) {
        int time = last_ts + rand() % 5 + 5;
        time_counter += (time - last_ts);
        orientation = formatAngle(orientation + direction * (rand() % 5 + 1));
        direction = angleDirection(orientation, target);
        difference = angleDifference(orientation, target);
        printf("orientation: %6.1f \t target: %6.1f \t direction: %d \t difference: %6.1f \t done-tolerance: %6.1f  \n", orientation, target, direction, difference, (float) ((time_counter * time_counter) / 2000 + 4));
        if (time_counter > 100) {
            //printf("HOVER\n");
            time_counter = 0;
            last_ts = last_ts + 1000;
            orientation = orientation + ((rand() % 2 - 1)) * ((rand() % 10 + 5));
            //printf("%d\n",((rand() % 2 - 1)) );
        }

        if (difference < (time_counter * time_counter) / 2000 + 4) {
            //hoverDuration(3);
            break;
        }


    }
    printf("DONE! orientation: %6.1f \t target: %6.1f \t direction: %d \t difference: %6.1f \n  ", orientation, target, direction, difference);
}




double formatAngle(double angle) {
    double result = angle;
    if (result < 0)
        result += 360;
    else if (result > 360)
        result -= 360;
    // ROS_INFO("FORMAT ANGLE: %6.2f\t RESULT ANGLE: %6.2f", angle, result);
    return result;
}

double angleDifference(double a1, double a2) {
    double angle = a1 - a2;
    if (angle < -180) {
        angle = 360 + angle;
    } else if (angle > 180)
        angle = 360 - angle;
    if (angle < 0)
        angle *= -1;
    return angle;
}

int angleDirection(double a1, double a2) {
    int direction = -1;
    double difference = a2 - a1; // calculate difference
    if ((difference < 180 && difference > 0) || (difference < -180))
        direction = 1;
    return direction;
}





