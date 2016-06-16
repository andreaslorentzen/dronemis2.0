//
// Created by andreas on 6/9/16.
//

#include <fstream>
#include "Nav.h"

void Nav::run(ros::NodeHandle *n) {

    ros::Subscriber sub_navdata = n->subscribe<ardrone_autonomy::Navdata>("ardrone/navdata", 5000,
                                                                          &Nav::navdataCallback, this);
    ros::Subscriber sub_magneto = n->subscribe<ardrone_autonomy::navdata_magneto>("ardrone/navdata_magneto", 5000,
                                                                                  &Nav::magnetoCallback, this);
    //ros::Subscriber sub_imu = n->subscribe<sensor_msgs::Imu>("ardrone/imu", 5000, &Nav::imuCallback, this);

    ros::Subscriber sub_init = n->subscribe<std_msgs::Empty>("nav/init", 5000, &Nav::initCallback, this);

    while (ros::ok()) {
        ros::spinOnce();
    }

}

double Nav::current_time() {
    return ros::Time::now().toNSec();
}

void Nav::initCallback(const std_msgs::Empty::ConstPtr &msg) {
    running = !running;

    position.x = 0.0;
    position.y = 0.0;
    x = 0;
    y = 0;

//    printf("running: %d",running);
}

int counter = 0;
int tickindex = 0;
float ticksum = 0;
int ticklist[100];
void Nav::navdataCallback(const ardrone_autonomy::Navdata::ConstPtr &msg) {

    state = msg->state;

    float ts = msg->tm;
    float vx = msg->vx;
    float vy = msg->vy;
//    float vZ = msg->vz;
    float ax = msg->ax;
    float ay = msg->ay;
//    float aZ = msg->az;
    if (last_ts == 0)
        last_ts = ts;
    float INTERVAL = (ts - last_ts)/1000000;
    float avx = (last_vx + vx)/2;
    last_vx = vx;

    ticksum-=ticklist[tickindex];
    ticksum+=INTERVAL;
    ticklist[tickindex]=INTERVAL;
    if (++tickindex==100)
        tickindex=0;
    float updatesPerSec = ticksum/100;


    last_ts = ts;
    position.z = msg->altd;

    position.x += avx * INTERVAL;//+ 0.5 * ax * INTERVAL * INTERVAL;
    position.y += vy * INTERVAL;//+ 0.5 * ay * INTERVAL * INTERVAL;
    x += vx * INTERVAL;
    y += vy * INTERVAL;
    std::string filename = "../workspaces/dronemis_ws/src/dronemis/src/navdata/log";
    filename.append(std::to_string(start_time));
    filename.append(".csv");


    ROS_INFO("I:\t%f\t(x,y):\t%d\t%d\tups:\t%d",INTERVAL, (int) position.x,(int) position.y, updatesPerSec);
/*
    std::ofstream file;
    file.open(filename, std::ios::app);
    file << ts;
    file << ";";
    file << state;
    file << ";";
    file << position.x;
    file << ";";
    file << position.y;
    file << ";";
    file << x;
    file << ";";
    file << y;
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

*/
    //x += vX * INTERVAL + 0.5 ;
    //y += vY * INTERVAL + 0.5 ;

//    printf("s: %d\ta: %d\trot: %6.2f, %6.2f, %6.2f\tvel: %6.2f, %6.2f, %6.2f \tacc: %8.4f, %8.4f, %8.4f\n", state, altd, rX, rY, rZ, vX,vY,vZ, aX, aY, aZ);


}

void Nav::magnetoCallback(const ardrone_autonomy::navdata_magneto::ConstPtr &msg) {
    rotation = msg->heading_fusion_unwrapped;
}

/*
    void Nav::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    }
*/



Nav::Nav() {
    last_ts = 0;
    running = 0;
    position.x = 0.0;
    position.y = 0.0;
    time = 0;
    last_vx = 0;
    x = 0;
    y = 0;
    start_time = (int) ros::Time::now().toSec();
}

