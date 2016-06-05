/*
 * https://github.com/parcon/arl_ardrone_examples
 */

#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"

#define LOOP_RATE (50)
unsigned int state;

int altd;
int rX;
int rY;
int rZ;
int vX;
int vY;
int vZ;


double current_time() {
    return ros::Time::now().toSec();
}

void navdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{

    state = msg->state;
    altd = msg->altd;
    rX = msg->rotX;
    rY = msg->rotY;
    rZ = msg->rotZ;
    vX = msg->vx;
    vY = msg->vy;
    vZ = msg->vz;
    printf("s: %d\ta: %d\trX: %d\trY: %d\trZ: %d\tvX: %d\tvY: %d\tvZ: %d\n", state, altd, rX, rY, rZ, vX,vY,vZ);

}

int main(int argc, char **argv) {

    int takeoff_time = 3;
    double fly_time = 1.0;
    double land_time = 3.0;
    double aaz;

    ros::init(argc, argv, "controller");

    ros::NodeHandle n;

    ros::Publisher pub_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher pub_land = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
    ros::Publisher pub_reset = n.advertise<std_msgs::Empty>("/ardrone/reset", 1);

    ros::Publisher pub_control = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    ros::Subscriber sub = n.subscribe("ardrone/navdata", 5000, navdataCallback);


    ros::Rate loop_rate(LOOP_RATE);
    std_msgs::Empty empty_msg;

    while (ros::ok()) {

        int i = 0;
        int j = 0;

        printf("Enter any key to start: ");
        getchar();

        ROS_INFO("takeoff %d", (int) takeoff_time * LOOP_RATE);
        pub_takeoff.publish(empty_msg);

        while (state != 3 && state != 7) {
            ros::spinOnce();
            loop_rate.sleep();
        }

       // pub_control.publish(cmd);


        while (aaz<1.0 && (state == 3 || state == 7)) {
            aaz+=0.01;
            if(aaz> 1.0)
                aaz = 1.0;
            cmd.angular.z = aaz;
            pub_control.publish(cmd);
            ros::spinOnce();
            loop_rate.sleep();
            printf("AAZ: %f\n", aaz);
        }

        j=0;
        while (j<LOOP_RATE*100 && (state == 3 || state == 7)) {
            j+=LOOP_RATE;
            ros::spinOnce();
            loop_rate.sleep();
            printf("SLEEP: %d\n", j);
        }

        while (aaz>0.0 && (state == 3 || state == 7)) {
            aaz-=0.01;
            if (aaz <= 0.0)
                aaz = 0.0;
            cmd.angular.z = aaz;
            pub_control.publish(cmd);
            ros::spinOnce();
            loop_rate.sleep();
            printf("AAZ: %f\n", aaz);
        }

        j=0;
        while (j<LOOP_RATE*10 && (state == 3 || state == 7)) {
            j+=LOOP_RATE;
            ros::spinOnce();
            loop_rate.sleep();
            printf("SLEEP: %d\n", j);
        }

        aaz = 0;
        cmd.angular.z = aaz;
        pub_control.publish(cmd);
        ros::spinOnce();

        pub_land.publish(empty_msg);
        while (state != 2) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        printf("DERP\n");
        ros::spinOnce();
        loop_rate.sleep();
        printf("DERP\n");
        ros::spinOnce();
        loop_rate.sleep();
        printf("DERP\n");
    }


    return 0;
}