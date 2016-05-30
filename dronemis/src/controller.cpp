/*
 * https://github.com/parcon/arl_ardrone_examples
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

double current_time() {
    return ros::Time::now().toSec();
}

int main(int argc, char **argv) {
    double start_time;
    double takeoff_time = 3.0;
    double fly_time = 10.0;
    double land_time = 3.0;

    ros::init(argc, argv, "controller");

    ros::NodeHandle n;

    ros::Publisher pub_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher pub_land = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
    ros::Rate loop_rate(50);

    ros::Time::init();

    start_time = current_time();
    ROS_INFO("Start Time %f", start_time);
    while (ros::ok()) {

        ROS_INFO("takeoff");
        while (current_time() < (start_time + takeoff_time)) {

            std_msgs::Empty empty_msg;

            pub_takeoff.publish(empty_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }

        ROS_INFO("landing");
        while (current_time() < (start_time + takeoff_time + land_time)) {

            std_msgs::Empty empty_msg;

            pub_land.publish(empty_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }


    //  system("rostopic pub /ardrone/takeoff std_msgs/Empty");

    return 0;
}