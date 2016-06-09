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
    FlightController(int loopRate, ros::NodeHandle nh);
    ~FlightController();
    void goToWaypoint(Command newWaypoint);
    void turnDrone(double degrees);
    void hover(int time);
    void takeOff();
    void land();
    void reset();
    void setStraightFlight(bool newState);
    void run(Nav *navdata, CV_Handler *cv_handler);
private:
    // drone possition
    double x;
    double y;
    double z;
    double rotation;
    double baseSpeed;
    int LOOP_RATE;
    int takeoff_time;
    double precision;
    bool straightFlight;
    ros::Publisher pub_takeoff;
    ros::Publisher pub_land;
    ros::Publisher pub_control;
    ros::Publisher pub_reset;

    geometry_msgs::Twist cmd;
    void publishToControl(double timeToFly);
    MyVector transformCoordinates(MyVector incomingVector);
};

#endif //PROJECT_FLIGHTCONTROLLER_H
