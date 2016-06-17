
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

    Vector3 pos = nav.getPosition();

    nav.resetToPosition(10, 10, 0);

    nav.getPosition();




    return 0;
}