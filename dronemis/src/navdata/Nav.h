//
// Created by andreas on 6/9/16.
//

#ifndef PROJECT_NAV_H
#define PROJECT_NAV_H

#include "ros/ros.h"
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/navdata_magneto.h>
#include <std_msgs/Empty.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "ros/callback_queue.h"

class Nav {
private:
    bool running;
    double last;

    double current_time();
public:
    unsigned int state;
    struct positionStruct{
        float x;
        float y;
        int z;

    } position;
    float rotation;

    void navdataCallback(const ardrone_autonomy::Navdata::ConstPtr &msg);
    void magnetoCallback(const ardrone_autonomy::navdata_magneto::ConstPtr &msg);
    void initCallback(const std_msgs::Empty::ConstPtr &msg);
    Nav();
    void run(ros::NodeHandle *n);
    int getHeight(){return position.z;}
};


#endif //PROJECT_NAV_H
