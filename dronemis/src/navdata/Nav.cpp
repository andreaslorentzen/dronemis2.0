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
#ifdef DEBUG_NAV_COUT
        ROS_INFO("Discarded!");
#endif
        return;
    }
    lastvX = avx;
    float ups = updateUPS();
    last_ts = ts;
    position.z = msg->altd;

    position.x += avx * interval;//+ 0.5 * ax * INTERVAL * INTERVAL;
    position.y += vy * interval;//+ 0.5 * ay * INTERVAL * INTERVAL;
    x += vx * interval;
    y += vy * interval;
    if (++counter >= counter_size) {
        //ROS_INFO("I:\t%f\t(x,y):\t%d\t%d\tups:\t%8.1f", interval, (int) position.x, (int) position.y, ups);
        counter = 0;
    }
#ifdef DEBUG_NAV_LOG
    std::string filename = "../workspaces/dronemis_ws/src/dronemis/src/navdata/log";
    filename.append(std::to_string(start_time));
    filename.append(".csv");
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
#endif
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
    float drone_heading = msg->heading_fusion_unwrapped;


    while(drone_heading < 0)
        drone_heading += 360;

    while(drone_heading > 360)
        drone_heading -= 360;

    if (!rotinit){
        rotoffset = drone_heading;
        rotinit = true;
    }


    rotation = drone_heading - rotoffset;

#ifdef DEBUG_NAV_COUT
    ROS_INFO("ORIGINAL = %f", drone_heading);
    ROS_INFO("rotation = %f", rotation);
#endif
}
double Nav::getRotation() {
    double rot = QRoffset + rotation;
    if(rot >= 360)
        rot -= 360;
    return rot;
}

void Nav::resetToPosition(double x, double y, double heading) {
    QRx = x;
    QRy = y;
    QRheading = heading;
    QRoffset = heading - rotation;

    position.x = 0;
    position.y = 0;
}

Vector3 Nav::getPosition() {
    double temp_rotation = (QRheading-90)*(-1);

    if(temp_rotation >= 360)
        temp_rotation -= 360;

    temp_rotation = temp_rotation/180 * M_PI;

    Vector3 rotationMatrix[3];
    rotationMatrix[0] = Vector3(cos(temp_rotation), -sin(temp_rotation), 0);
    rotationMatrix[1] = Vector3(sin(temp_rotation), cos(temp_rotation), 0);
    rotationMatrix[2] = Vector3(0, 0, 1);

    Vector3 position_vector(position.x, position.y, 0);
    Vector3 qr_vector(QRx, QRy, 0);

    Vector3 resultVector(rotationMatrix[0].x * position_vector.x + rotationMatrix[0].y * position_vector.y + rotationMatrix[0].z * position_vector.z,
                         rotationMatrix[1].x * position_vector.x + rotationMatrix[1].y * position_vector.y + rotationMatrix[1].z * position_vector.z,
                         rotationMatrix[2].x * position_vector.x + rotationMatrix[2].y * position_vector.y + rotationMatrix[2].z * position_vector.z);

    resultVector.x += qr_vector.x;
    resultVector.y += qr_vector.y;
    resultVector.z = position.z;

#ifdef DEBUG_NAV_COUT
    ROS_INFO("Outgoing vector");
    ROS_INFO("X = %F", resultVector.x);
    ROS_INFO("Y = %F", resultVector.y);
    ROS_INFO("Z = %F", resultVector.z);
#endif

    return resultVector;
}


Nav::Nav() {
    rotinit = false;
    rotoffset = 0;
    last_ts = 0;
    running = 0;
    time = 0;
    last_vx = 0;
    x = 0;
    y = 0;
    start_time = (int) ros::Time::now().toSec();
    ups_last_time = ros::Time::now().toSec();

     QRx=0;
     QRy=0;
     QRheading=90;
     QRoffset=0;

    rotation = 0;

    ups_index = 0;
    ups_last_time = 0;
}

