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
    void navdataCallback(const ardrone_autonomy::Navdata::ConstPtr &msg);
    void resetCallback(const std_msgs::Empty::ConstPtr &msg);
    bool landed = 1;
    double last;
    unsigned int oldState;
    unsigned int state;
    double current_time();
public:
    Nav();
    virtual ~Nav();
    void run(ros::NodeHandle n);
    struct {
        float x = 0.0;
        float y = 0.0;
        int z;

        float rotX;
        float rotY;
        float rotZ;
        int magX;
        int magY;
        int magZ;
    } position;
};


#endif //PROJECT_NAV_H
