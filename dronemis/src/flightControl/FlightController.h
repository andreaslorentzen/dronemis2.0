//
// Created by mathias on 6/4/16.
//

#ifndef PROJECT_FLIGHTCONTROLLER_H
#define PROJECT_FLIGHTCONTROLLER_H

#include <ros/ros.h>
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "Command.h"
#include "Route.h"
#include "../navdata/Nav.h"
#include "../OpenCv/CV_Handler.h"

struct MyVector{
    double x;
    double y;
    double z;
    MyVector(){

    }
    MyVector(double newX, double newY, double newZ): x(newX), y(newY), z(newZ){

    }
};

class FlightController{
public:
    FlightController();
    FlightController(int loopRate, ros::NodeHandle *nh, ros::MultiThreadedSpinner spinner);
    ~FlightController();
    void goToWaypoint(Command newWaypoint);
    void turnTowardsPoint(Command waypoint);
    void hover(int time);
    void takeOff();
    void land();
    void reset();
    void setStraightFlight(bool newState);
    void run();
    void startProgram(void);
    void resetProgram(void);
    void abortProgram(void);
private:
    double baseSpeed;
    bool started;
    int LOOP_RATE;
    int takeoff_time;
    double precision;
    bool straightFlight;
    double maxSpeed;
    ros::Publisher pub_takeoff;
    ros::Publisher pub_land;
    ros::Publisher pub_control;
    ros::Publisher pub_reset;
    Nav *navData;
    CV_Handler *cvHandler;
    double getSpeed(double distance);
    geometry_msgs::Twist cmd;
    MyVector transformCoordinates(MyVector incomingVector);
    double getRotationalSpeed(double target_deg, double ori_deg);
};

#endif //PROJECT_FLIGHTCONTROLLER_H
