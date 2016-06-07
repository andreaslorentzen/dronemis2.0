//
// Created by mathias on 6/4/16.
//

#include "FlightController.h"

FlightController::FlightController(){
    x = 0;
    y = 0;
    z = 0;
    baseSpeed = 0.5;
    LOOP_RATE = 0;
    takeoff_time = 3;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    straightFlight = false;
}

FlightController::FlightController(int loopRate, ros::NodeHandle nh) {
    x = 0;
    y = 0;
    z = 0;
    baseSpeed = 0.5;
    LOOP_RATE = loopRate;
    takeoff_time = 3;
    straightFlight = false;
    pub_land = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
    pub_takeoff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    pub_control = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_reset = nh.advertise<std_msgs::Empty>("/ardrone/reset", 1);


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
    double diffZ = newWaypoint.z - z;
    double absX = abs(diffX);
    double absY = abs(diffY);
    double deltaDiffs;

    ROS_INFO("diffX = %f", diffX);
    ROS_INFO("diffY = %f", diffY);

    if (!straightFlight) {
        if (diffX != 0 && diffY != 0) {
            timeToFly = std::sqrt(std::pow(diffX, 2) + std::pow(diffY, 2)) / baseSpeed;
            double glideSpeed = pow(baseSpeed, 2);
            if (absX == absY) {
                double actualSpeed = sqrt(glideSpeed * 0.5);
                if (diffX < 0)
                    cmd.linear.x = -actualSpeed;
                else
                    cmd.linear.x = actualSpeed;

                if (diffY < 0)
                    cmd.linear.y = -actualSpeed;
                else
                    cmd.linear.y = actualSpeed;
            } else if (absX > absY) {
                // De her to elseifs virker måske, men er ikke ligefrem sikker.
                deltaDiffs = absY / absX;

                if (diffY < 0)
                    cmd.linear.y = -sqrt(glideSpeed * deltaDiffs);
                else
                    cmd.linear.y = sqrt(glideSpeed * deltaDiffs);

                if (diffX < 0)
                    cmd.linear.x = -sqrt(glideSpeed * (1.0 - deltaDiffs));
                else
                    cmd.linear.x = sqrt(glideSpeed * (1.0 - deltaDiffs));

            } else if (absX < absY) {
                deltaDiffs = absX / absY;

                if (diffX < 0)
                    cmd.linear.x = sqrt(glideSpeed * deltaDiffs);
                else
                    cmd.linear.x = -sqrt(glideSpeed * deltaDiffs);

                if (diffY < 0)
                    cmd.linear.y = -sqrt(glideSpeed * (1.0 - deltaDiffs));
                else
                    cmd.linear.y = sqrt(glideSpeed * (1.0 - deltaDiffs));
            }
        } else {
            if (diffX != 0) {
                timeToFly = absX / baseSpeed;
                if (diffX < 0)
                    cmd.linear.x = -baseSpeed;
                else
                    cmd.linear.x = baseSpeed;
            } else {
                timeToFly = absX / baseSpeed;
                if (diffY < 0)
                    cmd.linear.y = -baseSpeed;
                else
                    cmd.linear.y = baseSpeed;
            }
        }

        ROS_INFO("X = %F", cmd.linear.x);
        ROS_INFO("Y = %F", cmd.linear.y);

        publishToControl(timeToFly);

        x = newWaypoint.x;
        y = newWaypoint.y;
    } else{
        if(diffX != 0.0){
            timeToFly = absX / baseSpeed;

            if(diffX < 0){
                cmd.linear.x = -baseSpeed;
            } else
                cmd.linear.x = baseSpeed;

            ROS_INFO("Time to fly = %F", timeToFly);
            ROS_INFO("X = %F", cmd.linear.x);

            publishToControl(timeToFly);

            x = newWaypoint.x;
        }

        if(diffY != 0.0){
            timeToFly = absY / baseSpeed;

            if(diffY < 0)
                cmd.linear.y = -baseSpeed;
            else
                cmd.linear.y = baseSpeed;

            ROS_INFO("Time to fly = %F", timeToFly);
            ROS_INFO("Y = %F", cmd.linear.x);

            publishToControl(timeToFly);

            y = newWaypoint.y;
        }
    }
    if (diffZ != 0.0){
        timeToFly = abs(diffZ) / baseSpeed;

        if(diffZ < 0)
            cmd.linear.z = -baseSpeed;
        else
            cmd.linear.z = baseSpeed;

        ROS_INFO("Time to fly = %F", timeToFly);
        ROS_INFO("Z = %F", cmd.linear.z);

        publishToControl(timeToFly);

        z = newWaypoint.z;
    }


}

void FlightController::publishToControl(double timeToFly){
    for(int k = 0; k < timeToFly*LOOP_RATE; k++){

        // Implement some waiting if none has subscribed
        // while(pub_control.getNumSubscribers() == 0);

        pub_control.publish(cmd);

        ros::spinOnce();
        ros::Rate(LOOP_RATE).sleep();
        //loop_rate.sleep();
    }

    // enable auto hover
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    for(int k = 0; k < 0.4*LOOP_RATE; k++){
        pub_control.publish(cmd);

        ros::spinOnce();
        ros::Rate(LOOP_RATE).sleep();
    }
}

void FlightController::turnDrone(double degrees) {
    cmd.angular.z = 0.5;
    ROS_INFO("Turning %F", degrees);
    publishToControl(((degrees/22.5)));
}

void FlightController::hover(int time){
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    publishToControl(time);
}

void FlightController::takeOff() {

    for(int i = 0; i < takeoff_time*LOOP_RATE; i++){
        std_msgs::Empty empty_msg;
        pub_takeoff.publish(empty_msg);

        ros::spinOnce();
        ros::Rate(LOOP_RATE).sleep();
    }
}

void FlightController::land() {
    double fly_time = 1.0;
    double land_time = 3.0;

    for (int j = 0; j < (takeoff_time + fly_time + land_time) * LOOP_RATE; j++) {

        std_msgs::Empty empty_msg;
        pub_land.publish(empty_msg);

        ros::spinOnce();
        ros::Rate(LOOP_RATE).sleep();
    }
}

void FlightController::reset(){
    std_msgs::Empty empty_msg;
    pub_reset.publish(empty_msg);
    ros::spinOnce();
}

void FlightController::setStraightFlight(bool newState) {
    straightFlight = newState;
}

