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


double current_time() {
    return ros::Time::now().toNSec();
}

void navdataCallback(const ardrone_autonomy::Navdata::ConstPtr &msg)
{

    state = msg->state;

  //  printf("s: %d\ta: %d\trot: %6.2f, %6.2f, %6.2f\tvel: %6.2f, %6.2f, %6.2f \tacc: %8.4f, %8.4f, %8.4f\n", state, altd, rX, rY, rZ, vX,vY,vZ, aX, aY, aZ);

}



void takeoff(){

}
void land(){

}
int main(int argc, char **argv) {

    ros::init(argc, argv, "controller");

    ros::NodeHandle n;



    std_msgs::Empty empty_msg;
    ros::Rate loop_rate(LOOP_RATE);


    ros::Publisher pub_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher pub_land = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
  //  ros::Publisher pub_reset = n.advertise<std_msgs::Empty>("/ardrone/reset", 1);
    ros::Publisher pub_reset_pos = n.advertise<std_msgs::Empty>("nav/init", 1);




    ros::Publisher pub_control = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.5;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    ros::Subscriber sub = n.subscribe("ardrone/navdata", 5000, navdataCallback);

//    bool direction = 0;

    while(ros::ok()){

        printf("Enter any key to start: ");
        getchar();

        // takeoff
        pub_takeoff.publish(empty_msg);
        for (int k = 0; k < LOOP_RATE*6; ++k) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        pub_reset_pos.publish(empty_msg);
        for (int k = 0; k < LOOP_RATE/10; ++k) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        cmd.linear.x = 2.0;
        pub_control.publish(cmd);
        for (int k = 0; k < LOOP_RATE * 5; ++k) {
            ros::spinOnce();
            loop_rate.sleep();
        }


        cmd.linear.x = 0.0;
        pub_control.publish(cmd);
        for (int k = 0; k < LOOP_RATE * 3; ++k) {
            ros::spinOnce();
            loop_rate.sleep();
        }



        pub_land.publish(empty_msg);
        for (int k = 0; k < LOOP_RATE*3; ++k) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        pub_reset_pos.publish(empty_msg);





/*






        // wait
        for (int k = 0; k < LOOP_RATE; ++k) {
            ros::spinOnce();
            loop_rate.sleep();
        }











        // land
        cmd.linear.x = 0.0;
        pub_control.publish(cmd);
        for (int k = 0; k < LOOP_RATE * 0.5; ++k) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        pub_land.publish(empty_msg);
        while (state != 2) {
            ros::spinOnce();
            loop_rate.sleep();
        }

















        // takeoff
        pub_takeoff.publish(empty_msg);
        while (state != 3 && state != 7) {
            ros::spinOnce();
            loop_rate.sleep();
        }



















        cmd.linear.x = -1.0;
        pub_control.publish(cmd);
        for (int k = 0; k < LOOP_RATE * 3; ++k) {
            ros::spinOnce();
            loop_rate.sleep();
        }






        // land
        cmd.linear.x = 0.0;
        pub_control.publish(cmd);
        for (int k = 0; k < LOOP_RATE * 0.5; ++k) {
            ros::spinOnce();
            loop_rate.sleep();
        }
        pub_land.publish(empty_msg);
        while (state != 2) {
            ros::spinOnce();
            loop_rate.sleep();
        }













        for (int l = 0; l < LOOP_RATE * 2; ++l) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        direction = !direction;
        */
    }
    return 0;
}