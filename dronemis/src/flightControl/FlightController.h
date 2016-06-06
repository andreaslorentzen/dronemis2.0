//
// Created by mathias on 6/4/16.
//

#ifndef PROJECT_FLIGHTCONTROLLER_H
#define PROJECT_FLIGHTCONTROLLER_H

#include <ros/ros.h>
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "Command.h"

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
private:
    // drone possition
    double x;
    double y;
    double z;
    double baseSpeed;
    int LOOP_RATE;
    int takeoff_time;
    ros::Rate loop_rate;
    ros::Publisher pub_takeoff;
    ros::Publisher pub_land;
    ros::Publisher pub_control;

    geometry_msgs::Twist cmd;
};

#endif //PROJECT_FLIGHTCONTROLLER_H
