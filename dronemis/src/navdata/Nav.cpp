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

int counter_size = 10;
int counter = 0;

/*int tickindex = 0;
float ticksum = 0;
int ticklist[100];*/
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
    float interval = (ts - last_ts) / 1000000;
    float avx = (last_vx + vx) / 2;

    if (lastvX != 0.0) if ((lastvX < avx && lastaX < 0.0)) {
        lastvX = avx;
        lastaX = ax;
        ROS_INFO("Discarded!");
        return;
    }

    lastvX = avx;
    float ups = updateUPS();
    /*ticksum-=ticklist[tickindex];
    ticksum+=INTERVAL;
    ticklist[tickindex]=INTERVAL;
    if (++tickindex==100)
        tickindex=0;
    float updatesPerSec = ticksum/100;
*/


    last_ts = ts;
    position.z = msg->altd;

    position.x += avx * interval;//+ 0.5 * ax * INTERVAL * INTERVAL;
    position.y += vy * interval;//+ 0.5 * ay * INTERVAL * INTERVAL;
    x += vx * interval;
    y += vy * interval;
    std::string filename = "../workspaces/dronemis_ws/src/dronemis/src/navdata/log";
    filename.append(std::to_string(start_time));
    filename.append(".csv");


    if (++counter >= counter_size) {
        //ROS_INFO("I:\t%f\t(x,y):\t%d\t%d\tups:\t%8.1f", interval, (int) position.x, (int) position.y, ups);
        counter = 0;
    }
    std::ofstream file;
    file.open(filename, std::ios::app);
/*
    file << ts;
    file << ";";*/
    file << msg->tm;
    file << ";";
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


    //x += vX * INTERVAL + 0.5 ;
    //y += vY * INTERVAL + 0.5 ;

//    printf("s: %d\ta: %d\trot: %6.2f, %6.2f, %6.2f\tvel: %6.2f, %6.2f, %6.2f \tacc: %8.4f, %8.4f, %8.4f\n", state, altd, rX, rY, rZ, vX,vY,vZ, aX, aY, aZ);


}

float Nav::updateUPS() {
    double time = ros::Time::now().toSec();
    float interval = time - ups_last_time;
    ups_last_time = time;
// ROS_INFO("%f", interval);
    ups_buffer[ups_index] = interval;
    ups_index++;
    if (ups_index == ups_buffer_size)
        ups_index = 0;
    float acc = 0;
    for (int i = 0; i < ups_buffer_size; i++)
        acc += ups_buffer[i];
// acc /= 1000.0;

    return ups_buffer_size / acc;
}

void Nav::magnetoCallback(const ardrone_autonomy::navdata_magneto::ConstPtr &msg) {
    float original_rotation = msg->heading_fusion_unwrapped;
    rotation = original_rotation;

    while(rotation < 0)
        rotation += 360;

    while(rotation > 360)
        rotation -= 360;

    /*ROS_INFO("ORIGINAL = %f", original_rotation);
    ROS_INFO("rotation = %f", rotation);*/

}

void Nav::resetToPosition(double x, double y, double heading) {
    //TODO IMPLEMENT THIS
    position.x = 0.0;
    position.y = 0.0;
    x = 0;
    y = 0;
}

Vector3 Nav::transformCoordinates(Vector3 incomingVector) {
    Vector3 rotationMatrix[3];
    rotationMatrix[0] = Vector3(cos(rotation), -sin(rotation), 0);
    rotationMatrix[1] = Vector3(sin(rotation), cos(rotation), 0);
    rotationMatrix[2] = Vector3(0, 0, 1);

    Vector3 tempVector(incomingVector.x - position.x, incomingVector.y - position.y, incomingVector.z - position.z);

    Vector3 resultVector(rotationMatrix[0].x * tempVector.x + rotationMatrix[0].y * tempVector.y +
                         rotationMatrix[0].z * tempVector.z,
                         rotationMatrix[1].x * tempVector.x + rotationMatrix[1].y * tempVector.y +
                         rotationMatrix[1].z * tempVector.z,
                         rotationMatrix[2].x * tempVector.x + rotationMatrix[2].y * tempVector.y +
                         rotationMatrix[2].z * tempVector.z);


    resultVector.x += position.x;
    resultVector.y += position.y;
    resultVector.z += position.z;

    ROS_INFO("Incoming vector;");
    ROS_INFO("X = %F", incomingVector.x);
    ROS_INFO("Y = %F", incomingVector.y);
    ROS_INFO("Z = %F", incomingVector.z);

    ROS_INFO("Outgoing vector");
    ROS_INFO("X = %F", resultVector.x);
    ROS_INFO("Y = %F", resultVector.y);
    ROS_INFO("Z = %F", resultVector.z);

    return resultVector;
}


Nav::Nav() {
    last_ts = 0;
    running = 0;
    time = 0;
    last_vx = 0;
    x = 0;
    y = 0;
    start_time = (int) ros::Time::now().toSec();
    ups_last_time = ros::Time::now().toSec();

    ups_index = 0;
    ups_last_time = 0;
}

