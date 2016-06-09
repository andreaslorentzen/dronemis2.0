
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "ardrone_autonomy/Navdata.h"
#include "navdata/Nav.h"


unsigned int state = 0;

int altd;
float rX;
float rY;
float rZ;
float vX;
float vY;
float vZ;
float aX;
float aY;
float aZ;

float x = 0.0;
float y = 0.0;

double last;

bool landed = 1;


double current_time();
void navdataCallback2(const ardrone_autonomy::Navdata::ConstPtr &msg);
void resetXY(const std_msgs::Empty::ConstPtr &msg);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");
    ROS_INFO("start");
    ros::NodeHandle n;

    Nav nav;
    //nav.run(n);

/*
    ros::NodeHandle n;

    ros::Subscriber sub_navdata = n.subscribe("ardrone/navdata", 5000, navdataCallback2);
    ros::Subscriber sub_reset = n.subscribe("position/reset", 5000, resetXY);


    ros::Rate loop_rate(1);

    last = current_time();


    ros::spin();
*/
    return 0;
}/*
double current_time() {
    return ros::Time::now().toNSec();
}
unsigned int oldState;
void navdataCallback2(const ardrone_autonomy::Navdata::ConstPtr &msg)
{
    if(landed)
        return;

    oldState = state;
    state = msg->state;
*//*    if(state != oldState){
        printf("New State: %d\n", state);
    }
*//*


    altd = msg->altd;
    rX = msg->rotX;
    rY = msg->rotY;
    rZ = msg->rotZ;
    vX = msg->vx;
    vY = msg->vy;
    vZ = msg->vz;
    aX = msg->ax;
    aY = msg->ay;
    aZ = msg->az;

    double curr = (current_time()-last)/1000000000;
    last = current_time();

    x += vX * curr + 0.5 * aX * curr*curr;
    y += vY * curr + 0.5 * aY * curr*curr;

    //x += vX * curr + 0.5 ;
    //y += vY * curr + 0.5 ;

//    printf("s: %d\ta: %d\trot: %6.2f, %6.2f, %6.2f\tvel: %6.2f, %6.2f, %6.2f \tacc: %8.4f, %8.4f, %8.4f\n", state, altd, rX, rY, rZ, vX,vY,vZ, aX, aY, aZ);
    printf("s: %d\t%6.2f, %6.2f\n", state, x, y);


}
void resetXY(const std_msgs::Empty::ConstPtr &msg){
    landed = !landed;

    x = 0.0;
    y = 0.0;


    printf("landed: %d",landed);

}*/