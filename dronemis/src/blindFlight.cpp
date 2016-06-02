//
// Created by mathias on 6/1/16.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

#define LOOP_RATE (50)
double current_time() {
    return ros::Time::now().toSec();
}

int main(int argc, char **argv) {

    int takeoff_time = 3;
    double fly_time = 1.0;
    double land_time = 3.0;
    double drift_time = 2.0;

    ros::init(argc, argv, "blindFlight");

    ros::NodeHandle n;

    ros::Publisher pub_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher pub_land = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
    ros::Publisher pub_control = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Rate loop_rate(LOOP_RATE);

    geometry_msgs::Twist cmd;

    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    double currentX = 0.0;
    double currentY = 0.0;
    double currentZ = 0.0;

    struct Waypoint{
        double x;
        double y;
        double z;
        Waypoint(double newX, double newY): x(newX), y(newY)
        {
        }
        Waypoint(){

        }
    };


    /*int routeLength = 8;
    Waypoint route[8];
    route[0] = Waypoint(0.0, 5.0);
    route[1] = Waypoint(2.0, 5.0);
    route[2] = Waypoint(2.0, 0.0);
    route[3] = Waypoint(4.0, 0.0);
    route[4] = Waypoint(4.0, 5.0);
    route[5] = Waypoint(6.0, 5.0);
    route[6] = Waypoint(6.0, 0.0);
    route[7] = Waypoint(0.0, 0.0);*/

    int routeLength = 4;
    Waypoint route[4];
    route[0] = Waypoint(0.0, 1.0);
    route[1] = Waypoint(1.0, 1.0);
    route[2] = Waypoint(1.0, 0.0);
    route[3] = Waypoint(0.0, 0.0);

    double timeToFly = 0.0;
    double baseSpeed = 0.5;

    int i = 0;
    int j = 0;
    while (ros::ok()) {

        ROS_INFO("takeoff %d", (int)takeoff_time*LOOP_RATE);
        for(; i < takeoff_time*LOOP_RATE; i++){

            std_msgs::Empty empty_msg;
            pub_takeoff.publish(empty_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
        // Land if the route has finished
        if(j == routeLength) {
            j = 0;
            for (; j < (takeoff_time + fly_time + land_time) * LOOP_RATE; j++) {

                std_msgs::Empty empty_msg;
                pub_land.publish(empty_msg);

                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        }


        double diffX = route[j].x - currentX;
        double diffY = route[j].y - currentY;

        if(diffX != 0) {
            timeToFly = std::abs(diffX) / baseSpeed;
            if(diffX < 0)
                cmd.linear.x = -baseSpeed;
            else
                cmd.linear.x = baseSpeed;
        } else{
            timeToFly = std::abs(diffY) / baseSpeed;
            if(diffY < 0)
                cmd.linear.y = -baseSpeed;
            else
                cmd.linear.y = baseSpeed;
        }
        
        for(int k = 0; k < timeToFly*LOOP_RATE; k++){
            pub_control.publish(cmd);

            ros::spinOnce();
            loop_rate.sleep();
        }

        currentX = route[j].x;
        currentY = route[j].y;

        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;

        for(int k = 0; k < 0.2*LOOP_RATE; k++){
            pub_control.publish(cmd);
            ros::spinOnce();
            loop_rate.sleep();
        }

        j++;
    }

    return 0;
}