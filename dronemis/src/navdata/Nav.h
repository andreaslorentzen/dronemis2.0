//
// Created by andreas on 6/9/16.
//

#ifndef PROJECT_NAV_H
#define PROJECT_NAV_H

#include "ros/ros.h"
#include <ardrone_autonomy/Navdata.h>
#include <std_msgs/Empty.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"

class Nav {
private:
    bool running;
    double last;

    double current_time();
public:
    unsigned int state;
    struct {
        float x;
        float y;
        int z;

    } position;
    struct {
        float x;
        float y;
        float z;
    } rotation;
    struct {
        float x;
        float y;
        float z;
    } mag;

    void navdataCallback(const ardrone_autonomy::Navdata::ConstPtr &msg);
    void initCallback(const std_msgs::Empty::ConstPtr &msg);
    Nav();
    void run(ros::NodeHandle n);

};


#endif //PROJECT_NAV_H
