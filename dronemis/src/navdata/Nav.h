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
#include <sensor_msgs/Imu.h>
#include "../Vector3.h"
#include "../debug.h"

class Nav {
private:
    int start_time;
    bool running;
    double last;
    double current_time();
    double lastvX;
    double lastaX;
    bool rotinit;
    double rotoffset;

    double QRx;
    double QRy;
    double QRheading;
    double QRoffset;

    float rotation;

    Vector3 position = Vector3(0, 0, 0);

public:
    static const int ups_buffer_size = 50;
    float ups_buffer[ups_buffer_size];
    int ups_index = 0;
    double ups_last_time;
    float updateUPS();
    double getRotoffset(){return rotoffset;}
    unsigned int state;

    double time;
    double last_ts;
    double last_vx;

    double x, y;


    void navdataCallback(const ardrone_autonomy::Navdata::ConstPtr &msg);
    void magnetoCallback(const ardrone_autonomy::navdata_magneto::ConstPtr &msg);
// void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void initCallback(const std_msgs::Empty::ConstPtr &msg);
    void resetToPosition(double x, double y, double heading);
    Nav();
    void run(ros::NodeHandle *n);

    Vector3 getPosition();
    double getRotation();

    Vector3 getRawPosition();
    double getRawRotation();
    void resetRaw();

    void resetRawRotation();
};


#endif //PROJECT_NAV_H
