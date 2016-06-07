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

void navdataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{

    state = msg->state;

  //  printf("s: %d\ta: %d\trot: %6.2f, %6.2f, %6.2f\tvel: %6.2f, %6.2f, %6.2f \tacc: %8.4f, %8.4f, %8.4f\n", state, altd, rX, rY, rZ, vX,vY,vZ, aX, aY, aZ);

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
    cmd.linear.x = 0.5;
    cmd.linear.y = 0.5;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    ros::Subscriber sub = n.subscribe("ardrone/navdata", 5000, navdataCallback);


    ros::Rate loop_rate(LOOP_RATE);
    std_msgs::Empty empty_msg;

while(ros::ok()){
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
    for (int k = 0; k < LOOP_RATE * 3; ++k) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    pub_control.publish(cmd);
    for (int k = 0; k < LOOP_RATE * 3; ++k) {
        ros::spinOnce();
        loop_rate.sleep();
    }
  //  pub_reset.publish(empty_msg);
/*


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
*/

    pub_land.publish(empty_msg);
    while (state != 2) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    for (int l = 0; l < LOOP_RATE * 2; ++l) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
    return 0;
}