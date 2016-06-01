/*
 * https://github.com/parcon/arl_ardrone_examples
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

#define LOOP_RATE (50)
double current_time() {
    return ros::Time::now().toSec();
}

int main(int argc, char **argv) {

    int takeoff_time = 3;
    double fly_time = 1.0;
    double land_time = 3.0;

    ros::init(argc, argv, "controller");

    ros::NodeHandle n;

    ros::Publisher pub_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher pub_land = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
    ros::Rate loop_rate(LOOP_RATE);

    int i = 0;
    int j = 0;
    printf("Enter a key to start: ");
    getchar();
    while (ros::ok()) {

        ROS_INFO("takeoff %d", (int)takeoff_time*LOOP_RATE);
        for(; i < takeoff_time*LOOP_RATE; i++){

            std_msgs::Empty empty_msg;
            pub_takeoff.publish(empty_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }

        for(; j < (takeoff_time+fly_time+land_time)*LOOP_RATE; j++){

            std_msgs::Empty empty_msg;
            pub_land.publish(empty_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    return 0;
}