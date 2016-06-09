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
    baseSpeed = 0.2;
    LOOP_RATE = loopRate;
    takeoff_time = 3;
    straightFlight = false;
    pub_land = nh.advertise<std_msgs::Empty>("/ardrone/land", 1);
    pub_takeoff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    pub_control = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_reset = nh.advertise<std_msgs::Empty>("/ardrone/reset", 1);
    rotation = 0;

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

void FlightController::run(){

    Route myRoute;
    myRoute.initRoute(true);

    ros::Rate loop_rate(LOOP_RATE);
    setStraightFlight(true);

    while (ros::ok()) {

        takeOff();

        while (!myRoute.hasAllBeenVisited()) {
            Command currentCommand = myRoute.nextCommand();

            if (currentCommand.commandType == Command::goTo) {
                goToWaypoint(currentCommand);
            } else if (currentCommand.commandType == Command::hover) {
                hover(currentCommand.timeToHover);
            } else if (currentCommand.commandType == Command::turn) {
                turnDrone(currentCommand.degrees);
            }
        }

        land();

        break;
    }
}

void FlightController::goToWaypoint(Command newWaypoint) {
    double timeToFly;
    double diffX = newWaypoint.x - x;
    double diffY = newWaypoint.y - y;
    double diffZ = newWaypoint.z - z;
    double absX = abs(diffX);
    double absY = abs(diffY);
    double deltaDiffs;

    if (!straightFlight) {
        if (diffX != 0 && diffY != 0) {
            timeToFly = (std::sqrt(std::pow(diffX, 2) + std::pow(diffY, 2)) / baseSpeed)/2;
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
                timeToFly = (absX / baseSpeed)/2;
                if (diffX < 0)
                    cmd.linear.x = -baseSpeed;
                else
                    cmd.linear.x = baseSpeed;
            } else {
                timeToFly = (absX / baseSpeed)/2;
                if (diffY < 0)
                    cmd.linear.y = -baseSpeed;
                else
                    cmd.linear.y = baseSpeed;
            }
        }

        publishToControl(timeToFly);

        x = newWaypoint.x;
        y = newWaypoint.y;
    } else{
        if(diffX != 0.0){
            timeToFly = (absX / baseSpeed)/2;

            if(diffX < 0){
                cmd.linear.x = -baseSpeed;
            } else
                cmd.linear.x = baseSpeed;

            ROS_INFO("Time to fly = %F", timeToFly);
            ROS_INFO("X = %F", cmd.linear.x);

            publishToControl(timeToFly/2);

            x = newWaypoint.x;
        }

        if(diffY != 0.0){
            timeToFly = (absY / baseSpeed)/2;

            if(diffY < 0)
                cmd.linear.y = -baseSpeed;
            else
                cmd.linear.y = baseSpeed;

            ROS_INFO("Time to fly = %F", timeToFly);
            ROS_INFO("Y = %F", cmd.linear.x);

            publishToControl(timeToFly/2);

            y = newWaypoint.y;
        }
    }
    if (diffZ != 0.0){
        timeToFly = (abs(diffZ) / baseSpeed)/2;

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

    ROS_INFO("MOVING");
    pub_control.publish(cmd);

    for(int k = 0; k < timeToFly*LOOP_RATE; k++){

        // Implement some waiting if none has subscribed
        // while(pub_control.getNumSubscribers() == 0);

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

    pub_control.publish(cmd);
    for(int k = 0; k < LOOP_RATE; k++){
        ros::spinOnce();
        ros::Rate(LOOP_RATE).sleep();
    }
}

void FlightController::turnDrone(double degrees) {
    // Denne metode er langt fra præcis
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

    ROS_INFO("HOVER");
    publishToControl(time);

}

void FlightController::takeOff() {

    std_msgs::Empty empty_msg;
    pub_takeoff.publish(empty_msg);

    for(int i = 0; i < takeoff_time*LOOP_RATE; i++){
        ros::spinOnce();
        ros::Rate(LOOP_RATE).sleep();
    }

    hover(1);

    cmd.linear.z = 0.5;
    publishToControl(2);
}

void FlightController::land() {
    double fly_time = 1.0;
    double land_time = 3.0;


    std_msgs::Empty empty_msg;
    pub_land.publish(empty_msg);

    for (int j = 0; j < (takeoff_time + fly_time + land_time) * LOOP_RATE; j++) {
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

MyVector FlightController::transformCoordinates(MyVector incomingVector) {
    MyVector rotationMatrix[3];
    rotationMatrix[0] = MyVector(cos(rotation), -sin(rotation), 0);
    rotationMatrix[1] = MyVector(sin(rotation), cos(rotation), 0);
    rotationMatrix[2] = MyVector(0, 0, 1);

    MyVector tempVector(incomingVector.x-x, incomingVector.y-y, incomingVector.z-z);

    MyVector resultVector(rotationMatrix[0].x*tempVector.x+rotationMatrix[0].y*tempVector.y+rotationMatrix[0].z*tempVector.z,
                        rotationMatrix[1].x*tempVector.x+rotationMatrix[1].y*tempVector.y+rotationMatrix[1].z*tempVector.z,
                        rotationMatrix[2].x*tempVector.x+rotationMatrix[2].y*tempVector.y+rotationMatrix[2].z*tempVector.z);


    resultVector.x += x;
    resultVector.y += y;
    resultVector.z += z;

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


