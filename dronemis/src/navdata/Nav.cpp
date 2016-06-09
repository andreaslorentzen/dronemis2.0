//
// Created by andreas on 6/9/16.
//

#include "Nav.h"

void Nav::run(ros::NodeHandle n) {
    ros::Subscriber sub_navdata = n.subscribe<ardrone_autonomy::Navdata>("ardrone/navdata", 5000, &Nav::navdataCallback, this);
    ros::Subscriber sub_init = n.subscribe<std_msgs::Empty>("nav/init", 5000, &Nav::initCallback, this);

    last = current_time();

    ros::spin();
}
double Nav::current_time() {
    return ros::Time::now().toNSec();
}
void Nav::initCallback(const std_msgs::Empty::ConstPtr &msg){
    running = !running;

    position.x = 0.0;
    position.y = 0.0;

//    printf("running: %d",running);
}
void Nav::navdataCallback(const ardrone_autonomy::Navdata::ConstPtr &msg) {
    if(running)
        return;

    state = msg->state;


    float vX = msg->vx;
    float vY = msg->vy;
//    float vZ = msg->vz;
    float aX = msg->ax;
    float aY = msg->ay;
//    float aZ = msg->az;

    rotation.x = msg->rotX;
    rotation.y = msg->rotY;
    rotation.z = msg->rotZ;

    mag.x = msg->magX;
    mag.y = msg->magY;
    mag.z = msg->magZ;

    position.z = msg->altd;

    double curr = (current_time()-last)/1000000000;
    last = current_time();

    position.x += vX * curr + 0.5 * aX * curr*curr;
    position.y += vY * curr + 0.5 * aY * curr*curr;

    //x += vX * curr + 0.5 ;
    //y += vY * curr + 0.5 ;

//    printf("s: %d\ta: %d\trot: %6.2f, %6.2f, %6.2f\tvel: %6.2f, %6.2f, %6.2f \tacc: %8.4f, %8.4f, %8.4f\n", state, altd, rX, rY, rZ, vX,vY,vZ, aX, aY, aZ);
    printf("s: %d\t%6.2f, %6.2f\n", state, position.x, position.y);


}


Nav::Nav() {
    running = 1;
    position.x = 0.0;
    position.y = 0.0;
}


