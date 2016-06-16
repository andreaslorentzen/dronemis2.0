//
// Created by andreas on 6/9/16.
//

#include <fstream>
#include "Nav.h"

void Nav::run(ros::NodeHandle *n) {

    ros::Subscriber sub_navdata = n->subscribe<ardrone_autonomy::Navdata>("ardrone/navdata", 5000, &Nav::navdataCallback, this);
    ros::Subscriber sub_magneto = n->subscribe<ardrone_autonomy::navdata_magneto>("ardrone/navdata_magneto", 5000, &Nav::magnetoCallback, this);
    ros::Subscriber sub_imu = n->subscribe<sensor_msgs::Imu>("ardrone/imu", 5000, &Nav::imuCallback, this);

    ros::Subscriber sub_init = n->subscribe<std_msgs::Empty>("nav/init", 5000, &Nav::initCallback, this);

    while(ros::ok()){
        ros::spinOnce();
    }

}
double Nav::current_time() {
    return ros::Time::now().toNSec();
}
void Nav::initCallback(const std_msgs::Empty::ConstPtr &msg){
    running = !running;

    position.x = 0.0;
    position.y = 0.0;
    x=0;
    y=0;

//    printf("running: %d",running);
}
int counter = 0;
void Nav::navdataCallback(const ardrone_autonomy::Navdata::ConstPtr &msg) {

    state = msg->state;



    float vx = msg->vx;
    float vy = msg->vy;
//    float vZ = msg->vz;
    float ax = msg->ax;
    float ay = msg->ay;
//    float aZ = msg->az;

    /*
    position.z = msg->altd;

    double curr = 0.005;
    position.x += vx * curr + 0.5 * ax * curr*curr;
    position.y += vy * curr + 0.5 * ay * curr*curr;
*/
    std::string filename = "../workspaces/dronemis_ws/src/dronemis/src/navdata/log";
    filename.append(std::to_string(start_time));
    filename.append("-1");
    filename.append(".csv");

    std::ofstream file;
    file.open (filename, std::ios::app);
    file << state;
    file << ";";
    file << position.x;
    file << ";";
    file << position.y;
    file << ";";
    file << msg->ax;
    file << ";";
    file << msg->ay;
    file << ";";
    file << msg->vx;
    file << ";";
    file << msg->vy;
    file << "\n";
    file.close();


    //x += vX * curr + 0.5 ;
    //y += vY * curr + 0.5 ;

//    printf("s: %d\ta: %d\trot: %6.2f, %6.2f, %6.2f\tvel: %6.2f, %6.2f, %6.2f \tacc: %8.4f, %8.4f, %8.4f\n", state, altd, rX, rY, rZ, vX,vY,vZ, aX, aY, aZ);


}
void Nav::magnetoCallback(const ardrone_autonomy::navdata_magneto::ConstPtr &msg) {
    rotation = msg->heading_fusion_unwrapped;
}

void Nav::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {


    float vx = msg->angular_velocity.x;
    float vy = msg->angular_velocity.y;
//    float vZ = msg->vz;
    float ax = msg->linear_acceleration.x;
    float ay = msg->linear_acceleration.y;
//    float aZ = msg->az;

    position.z = 0.0;
/*
     = (current_time()-last)/1000000000;
    last = current_time();
*/



    double curr = 0.005;
    position.x += vx * curr + 0.5 * ax * curr*curr;
    position.y += vy * curr + 0.5 * ay * curr*curr;

    x += vx * curr + 0.5 * ax * curr*curr;
    y += vy * curr + 0.5 * ay * curr*curr;

    std::string filename = "../workspaces/dronemis_ws/src/dronemis/src/navdata/log";
    filename.append(std::to_string(start_time));
    filename.append("-2");
    filename.append(".csv");

    std::ofstream file;
    file.open (filename, std::ios::app);
    file << state;
    file << ";";
    file << position.x;
    file << ";";
    file << position.y;
    file << ";";
    file << ax;
    file << ";";
    file << ay;
    file << ";";
    file << vx;
    file << ";";
    file << vy;
    file << "\n";
    file.close();


}



Nav::Nav() {
    running = 0;
    position.x = 0.0;
    position.y = 0.0;
    x=0;
    y=0;
    start_time = (int) ros::Time::now().toSec();
}

