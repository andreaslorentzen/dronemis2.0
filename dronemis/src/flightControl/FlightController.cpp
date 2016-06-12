//
// Created by mathias on 6/4/16.
//

#include "FlightController.h"

#define DEBUG 1

void* startNavdata(void *thread_args);
void* startCV(void *thread_args);
void* startController(void *thread_arg);

struct thread_data{
    Nav *navData;
    CV_Handler *cvHandler;
    FlightController *controller;
    ros::MultiThreadedSpinner spinner;
    ros::NodeHandle *n;
} myThreadData;

FlightController::FlightController(){
    baseSpeed = 0.1;
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

FlightController::FlightController(int loopRate, ros::NodeHandle *nh, ros::MultiThreadedSpinner spinner) {
    baseSpeed = 0.1;
    LOOP_RATE = loopRate;
    takeoff_time = 3;
    straightFlight = false;
    pub_land = nh->advertise<std_msgs::Empty>("/ardrone/land", 1);
    pub_takeoff = nh->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    pub_control = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_reset = nh->advertise<std_msgs::Empty>("/ardrone/reset", 1);

    precision = 50;
    maxSpeed = 0.5;

    cvHandler = new CV_Handler();

    myThreadData.cvHandler = cvHandler;
    navData = new Nav();
    myThreadData.navData = navData;
    myThreadData.spinner = spinner;
    myThreadData.n = nh;

    pthread_t threads[2];
    pthread_create(&threads[0], NULL, startCV, &myThreadData);
    pthread_create(&threads[1], NULL, startNavdata, &myThreadData);


    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
}

// Destructor
FlightController::~FlightController() {
    delete(cvHandler);
}

void FlightController::run(){
    Route myRoute;
    myRoute.initRoute(true);


    ros::Rate loop_rate(LOOP_RATE);
    setStraightFlight(true);

    while (ros::ok()) {

        takeOff();
        ROS_INFO("BEFORE TAKEOFF");
        while (!myRoute.hasAllBeenVisited()) {
            ROS_INFO("HI THERE");
            Command currentCommand = myRoute.nextCommand();

            if (currentCommand.commandType == Command::goTo) {
                goToWaypoint(currentCommand);
            } else if (currentCommand.commandType == Command::hover) {
                hover(currentCommand.timeToHover);
            } else if (currentCommand.commandType == Command::turn) {
               //   urnDrone(currentCommand.degrees);
            }
        }

        land();

        break;
    }



    return;
}

void FlightController::goToWaypoint(Command newWaypoint) {
    double dx = newWaypoint.x - navData->position.x;
    double dy = newWaypoint.y - navData->position.y;
    double dz = newWaypoint.z - navData->position.z;



    bool moved = false;

    while(abs(dz) > precision){
        cmd.linear.z = getSpeed(dz);

        pub_control.publish(cmd);
        dz = newWaypoint.z - navData->position.z;

#ifdef DEBUG
        ROS_INFO("dz = %F", dz);
#endif
        ros::Rate(LOOP_RATE).sleep();
        moved = true;
    }

    if(moved)
        hover(1);

    moved = false;
    if (!straightFlight) {
        // TODO decide if this should be implemented or not
    } else{
        while(abs(dx) > precision){
            cmd.linear.x = getSpeed(dx);

            pub_control.publish(cmd);
            dx = newWaypoint.x - navData->position.x;

        #ifdef DEBUG
            ROS_INFO("newpoint = %F", newWaypoint.x);
            ROS_INFO("navData = %F", navData->position.x);
            ROS_INFO("dx = %F", dx);
            ROS_INFO("Speed = %F", cmd.linear.x);
            ROS_INFO("rotation = %F", navData->rotation);
        #endif
            ros::Rate(LOOP_RATE).sleep();
            moved = true;
        }

        if(moved)
            hover(1);

        moved = false;
        while(abs(dy) > precision){

            cmd.linear.y = getSpeed(dy);

            pub_control.publish(cmd);
            dy = newWaypoint.y - navData->position.y;

        #ifdef DEBUG
            ROS_INFO("dy = %F", dy);
        #endif
            ros::Rate(LOOP_RATE).sleep();
            moved = true;
        }

        if(moved)
            hover(1);
    }
}

double FlightController::getSpeed(double distance) {
    double speed = baseSpeed*(distance/150);
    if(speed > maxSpeed)
        speed = maxSpeed;

    return speed;
}

void FlightController::turnTowardsPoint(Command waypoint) {

    double target_angle = atan2(waypoint.y, waypoint.x); // angle towards waypoint position
    double target_deg = target_angle * 180 / M_PI;

    double ori_deg;

    float offset = 0.5;
    do{
        ori_deg = navData->rotation;
        if(ori_deg < 0)
            ori_deg = 360 + ori_deg;

    } while(ori_deg < target_deg-offset or ori_deg > target_deg+offset);
}

double FlightController::getRotationalSpeed(double target_deg, double ori_deg){
    double dir; // direction
    double rot_speed; // calculated rotational speed




    double diff_deg = target_deg - ori_deg;

    if ( diff_deg < 0 )
        diff_deg = 360 + diff_deg;

    printf("target_deg:\t%6.2f deg\n", diff_deg);


    if (diff_deg < 180){
        dir = -1;
        printf("Turn left");
    } else{
        dir = 1;
        diff_deg = 360 -diff_deg;
        printf("Turn right");
    }

    rot_speed = (diff_deg*diff_deg)/200; // speed to rotate with

    if(rot_speed > 0.5)
        rot_speed = 0.5;

    rot_speed *= dir;

    return rot_speed;
}

void FlightController::hover(int time){
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    ROS_INFO("HOVER");

    for(int i = 0; i < 3; i++)
        pub_control.publish(cmd);
}

void FlightController::takeOff() {

    std_msgs::Empty empty_msg;
    pub_takeoff.publish(empty_msg);

    for(int i = 0; i < takeoff_time*LOOP_RATE; i++){
        ros::Rate(LOOP_RATE).sleep();
    }

    hover(1);
}

void FlightController::land() {
    std_msgs::Empty empty_msg;
    pub_land.publish(empty_msg);

    for (int j = 0; j < LOOP_RATE; j++) {
        ros::Rate(LOOP_RATE).sleep();
    }
}

void FlightController::reset(){
    std_msgs::Empty empty_msg;
    pub_reset.publish(empty_msg);
}

void FlightController::setStraightFlight(bool newState) {
    straightFlight = newState;
}

MyVector FlightController::transformCoordinates(MyVector incomingVector) {
   /* MyVector rotationMatrix[3];
    rotationMatrix[0] = MyVector(cos(rotation), -sin(rotation), 0);
    rotationMatrix[1] = MyVector(sin(rotation), cos(rotation), 0);
    rotationMatrix[2] = MyVector(0, 0, 1);

    MyVector tempVector(incomingVector.x-navData->position.x, incomingVector.y-navData->position.y, incomingVector.z-navData->position.z);

    MyVector resultVector(rotationMatrix[0].x*tempVector.x+rotationMatrix[0].y*tempVector.y+rotationMatrix[0].z*tempVector.z,
                        rotationMatrix[1].x*tempVector.x+rotationMatrix[1].y*tempVector.y+rotationMatrix[1].z*tempVector.z,
                        rotationMatrix[2].x*tempVector.x+rotationMatrix[2].y*tempVector.y+rotationMatrix[2].z*tempVector.z);


    resultVector.x += navData->position.x;
    resultVector.y += navData->position.y;
    resultVector.z += navData->position.z;

    ROS_INFO("Incoming vector;");
    ROS_INFO("X = %F", incomingVector.x);
    ROS_INFO("Y = %F", incomingVector.y);
    ROS_INFO("Z = %F", incomingVector.z);

    ROS_INFO("Outgoing vector");
    ROS_INFO("X = %F", resultVector.x);
    ROS_INFO("Y = %F", resultVector.y);
    ROS_INFO("Z = %F", resultVector.z);
*/
    return incomingVector;

}

void FlightController::startProgram() {
    if (!started) {
        ROS_INFO("STARTING!");
        pthread_t thread;
        myThreadData.controller = this;
        pthread_create(&thread, NULL, startController, &myThreadData);
        started = true;
    }
}

void FlightController::resetProgram(){
    ROS_INFO("MANUEL RESET!");
    reset();
}

void FlightController::abortProgram(){
    ROS_INFO("MANUEL ABORT!");
    land();
}

void* startNavdata(void *thread_arg){
    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;


    thread_data->navData->run(thread_data->n, thread_data->spinner);
    pthread_exit(NULL);
}

void* startCV(void *thread_arg) {
    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;

    thread_data->cvHandler->run();
    pthread_exit(NULL);
}

void* startController(void *thread_arg){
    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;

    thread_data->controller->run();
    pthread_exit(NULL);
}