//
// Created by mathias on 6/4/16.
//

#include "FlightController.h"

FlightController::FlightController(): loop_rate(0){
    x = 0;
    y = 0;
    z = 0;
    baseSpeed;
    LOOP_RATE = 0;
    takeoff_time = 3;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
}

FlightController::FlightController(int loopRate, ros::NodeHandle nh): loop_rate(loopRate) {
    x = 0;
    y = 0;
    z = 0;
    baseSpeed = 0.5;
    LOOP_RATE = loopRate;
    takeoff_time = 3;

    pub_land = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
    pub_takeoff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    pub_control = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);


    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
}

// Destructor
FlightController::~FlightController() {
    // TODO implement this to actually do something
}

void FlightController::goToWaypoint(Command newWaypoint) {
    double timeToFly;
    double diffX = newWaypoint.x - x;
    double diffY = newWaypoint.y - y;
    double absX = abs(diffX);
    double absY = abs(diffY);
    double deltaDiffs;

    ROS_INFO("diffX = %f", diffX);
    ROS_INFO("diffY = %f", diffY);

    if(diffX != 0 && diffY != 0){
        timeToFly = std::sqrt(std::pow(diffX, 2) + std::pow(diffY,2)) / baseSpeed;
        double glideSpeed = pow(baseSpeed, 2);
        if(absX == absY){
            double actualSpeed = sqrt(glideSpeed * 0.5);
            if(diffX < 0)
                cmd.linear.x = -actualSpeed;
            else
                cmd.linear.x = actualSpeed;

            if(diffY < 0)
                cmd.linear.y = -actualSpeed;
            else
                cmd.linear.y = actualSpeed;
        } else if (absX > absY){
            // De her to elseifs virker måske, men er ikke ligefrem sikker.
            deltaDiffs = absY / absX;
            if(diffY < 0)
                cmd.linear.y = -sqrt(glideSpeed*deltaDiffs);
            else
                cmd.linear.y = sqrt(glideSpeed*deltaDiffs);

            if(diffX < 0)
                cmd.linear.x = -sqrt(glideSpeed*(1.0-deltaDiffs));
            else
                cmd.linear.x = sqrt(glideSpeed*(1.0-deltaDiffs));

        } else if(absX < absY){
            deltaDiffs = absX / absY;

            if(diffX < 0)
                cmd.linear.x = sqrt(glideSpeed*deltaDiffs);
            else
                cmd.linear.x = -sqrt(glideSpeed*deltaDiffs);

            if(diffY < 0)
                cmd.linear.y = -sqrt(glideSpeed*(1.0-deltaDiffs));
            else
                cmd.linear.y = sqrt(glideSpeed*(1.0-deltaDiffs));
        }
    } else{
        if (diffX != 0) {
            timeToFly = std::abs(diffX) / baseSpeed;
            if (diffX < 0)
                cmd.linear.x = -baseSpeed;
            else
                cmd.linear.x = baseSpeed;
        } else {
            timeToFly = std::abs(diffY) / baseSpeed;
            if (diffY < 0)
                cmd.linear.y = -baseSpeed;
            else
                cmd.linear.y = baseSpeed;
        }
    }

    ROS_INFO("X = %F", cmd.linear.x);
    ROS_INFO("Y = %F", cmd.linear.y);

    for(int k = 0; k < timeToFly*LOOP_RATE; k++){
        pub_control.publish(cmd);

        ros::spinOnce();
        loop_rate.sleep();
    }

    x = newWaypoint.x;
    y = newWaypoint.y;

    hover(1);
}

void FlightController::turnDrone(double degrees) {
    cmd.angular.z = 0.5;

    for(int k = 0; k < ((degrees/22.5))*LOOP_RATE; k++){

        while(pub_control.getNumSubscribers() == 0);

        pub_control.publish(cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }

    hover(1);
}

void FlightController::hover(int time){
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    for(int k = 0; k < time*LOOP_RATE; k++){
        pub_control.publish(cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void FlightController::takeOff() {

    for(int i = 0; i < takeoff_time*LOOP_RATE; i++){
        std_msgs::Empty empty_msg;
        pub_takeoff.publish(empty_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void FlightController::land() {
    double fly_time = 1.0;
    double land_time = 3.0;

    for (int j = 0; j < (takeoff_time + fly_time + land_time) * LOOP_RATE; j++) {

        std_msgs::Empty empty_msg;
        pub_land.publish(empty_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

