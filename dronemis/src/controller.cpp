/*
 * https://github.com/parcon/arl_ardrone_examples
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"

#define LOOP_RATE (50)
unsigned int state;

int altitude;

double current_time() {
    return ros::Time::now().toSec();
}

void navdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
    state = msg->state;
    altitude = msg->altd;
}

int main(int argc, char **argv) {

    int takeoff_time = 3;
    double fly_time = 1.0;
    double land_time = 3.0;

    ros::init(argc, argv, "controller");

    ros::NodeHandle n;

    ros::Publisher pub_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher pub_land = n.advertise<std_msgs::Empty>("/ardrone/land", 1);

    ros::Subscriber sub = n.subscribe("ardrone/navdata", 1000, navdataCallback);


    ros::Rate loop_rate(LOOP_RATE);

    int i = 0;
    int j = 0;
    printf("Enter any key to start: ");
    getchar();

    ROS_INFO("takeoff %d", (int)takeoff_time*LOOP_RATE);
    for(; i < takeoff_time*LOOP_RATE; i++){

        std_msgs::Empty empty_msg;
        pub_takeoff.publish(empty_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    sub.shutdown();

    for(; j < (takeoff_time+fly_time+land_time)*LOOP_RATE; j++){

        std_msgs::Empty empty_msg;
        pub_land.publish(empty_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    printf("Tap to exit: ");
    getchar();

    return 0;
}