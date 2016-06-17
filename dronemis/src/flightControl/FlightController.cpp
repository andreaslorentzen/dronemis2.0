//
// Created by mathias on 6/4/16.
//

#include "FlightController.h"

#define DEBUG 0

void* startNavdata(void *thread_args);
void* startCV(void *thread_args);
void* startController(void *thread_arg);

struct thread_data{
    Nav *navData;
    CV_Handler *cvHandler;
    FlightController *controller;
    ros::NodeHandle *n;
} myThreadData;

FlightController::FlightController(){

    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;

    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    straightFlight = false;

    LOOP_RATE = 50;


    ros::NodeHandle nh;

    if (nh.hasParam("TAKEOFF_TIME"))
        nh.getParam("TAKEOFF_TIME", TAKEOFF_TIME);
    else
        TAKEOFF_TIME = 6;

    if (nh.hasParam("TOLERANCE"))
        nh.getParam("TOLERANCE", TOLERANCE);
    else
        TOLERANCE = 100;

    if (nh.hasParam("TRANSIT_SPEED"))
        nh.getParam("TRANSIT_SPEED", TRANSIT_SPEED);
    else
        TRANSIT_SPEED = 0.05;

    if (nh.hasParam("CRUISE_SPEED"))
        nh.getParam("CRUISE_SPEED", CRUISE_SPEED);
    else
        CRUISE_SPEED = 0.01;

    if (nh.hasParam("CRUISE_LIMIT"))
        nh.getParam("CRUISE_LIMIT", CRUISE_LIMIT);
    else
        CRUISE_LIMIT = 300;

    if (nh.hasParam("CONTROL_SLEEP"))
        nh.getParam("CONTROL_SLEEP", CONTROL_SLEEP);
    else
        CONTROL_SLEEP = 10;

    control_loop = ros::Rate(CONTROL_SLEEP);

}

FlightController::FlightController(int loopRate, ros::NodeHandle *nh, Nav *nav): FlightController() {
    LOOP_RATE = loopRate;





    pub_land = nh->advertise<std_msgs::Empty>("/ardrone/land", 1);
    pub_takeoff = nh->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    pub_control = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_reset = nh->advertise<std_msgs::Empty>("/ardrone/reset", 1);

    cvHandler = new CV_Handler();
    qr = new QR(cvHandler);

    navData = nav;

    myThreadData.cvHandler = cvHandler;
    myThreadData.navData = navData;
    myThreadData.n = nh;

    pthread_t threads[2];
    pthread_create(&threads[0], NULL, startCV, &myThreadData);
    pthread_create(&threads[1], NULL, startNavdata, &myThreadData);



}

// Destructor
FlightController::~FlightController() {
    delete(cvHandler);
    delete(qr);
}

void FlightController::run(){
    Route myRoute;
    myRoute.initRoute(true);
    ros::Publisher pub_reset_pos = myThreadData.n->advertise<std_msgs::Empty>("nav/init", 1);

    std_msgs::Empty empty_msg;



    ros::Rate loop_rate(LOOP_RATE);
    setStraightFlight(true);

    bool firstIteration = true;

    DronePos dronePos;
    double startHeading;

    while (ros::ok()) {
        takeOff();

        startHeading = navData->rotation;

        bool turning = true;
        double amountTurned = 0;
        double turnStepSize = 30;
        dronePos = qr->checkQR();

        /*while(!dronePos.positionLocked){

            if(turning) {
                turnDegrees(turnStepSize);
                dronePos = qr->checkQR();
                amountTurned += turnStepSize;
                if(amountTurned >= 360)
                    turning = false;
            } else{
                Command tempCommand(navData->position.x + 500, navData->position.y);

                goToWaypoint(tempCommand);

                dronePos = qr->checkQR();
            }

            do {
                double targetHeading = navData->rotation - dronePos.angle;

                if (dronePos.relativeY > 150 && dronePos.relativeY < 225){
                    goToWaypoint(Command(navData->position.x, navData->position.y+(dronePos.relativeX)));
                    double currentHeading = navData->rotation;
                    if(currentHeading < 0)
                        currentHeading = 360 + currentHeading;
                    turnDegrees(targetHeading-currentHeading);
                }

                dronePos = qr->checkQR();

            } while((dronePos.numberOfQRs > 2 && !dronePos.positionLocked));


        }

        navData->resetToPosition(dronePos.x*10, dronePos.y*10, dronePos.heading);

        ROS_INFO("X = %d", dronePos.x);
        ROS_INFO("Y = %d", dronePos.y);
        ROS_INFO("heading = %d", dronePos.heading);

        land();
        return;*/

        hover(1);

        while (!myRoute.hasAllBeenVisited()) {
            Command currentCommand;
            /*if(firstIteration) {
                currentCommand = myRoute.findNearestWaypoint(navData->position.x, navData->position.y,
                                                             navData->position.z);
                firstIteration = false;
            }else {
                currentCommand = myRoute.nextCommand();
            }*/
            currentCommand = myRoute.nextCommand();

            if (currentCommand.commandType == Command::goTo) {
                goToWaypoint(currentCommand);
            } else if (currentCommand.commandType == Command::hover) {
                hover(currentCommand.timeToHover);
            } else if (currentCommand.commandType == Command::turn) {
                turnDegrees(currentCommand.degrees);
            }
        }

        land();

        break;
    }



    return;
}

void FlightController::goToWaypoint(Command newWaypoint) {

    MyVector d (newWaypoint.x - navData->position.x,
                newWaypoint.y - navData->position.y,
                //newWaypoint.z - navData->position.z);
                0);

    MyVector v_vec (0.0, 0.0, 0.0);


    /*
     * VERSION 1
     */

    while ((int) abs(d.x) > TOLERANCE){

        v_vec.x = getSpeed(d.x);

        if(v_vec.x != cmd.linear.x){
            printf("vx: %f\tdx: %f \n", v_vec.x, d.x);
            cmd.linear.x = v_vec.x;
            pub_control.publish(cmd);
        }

        control_loop.sleep();

        d.x = newWaypoint.x - navData->position.x;
        d.y = newWaypoint.y - navData->position.y;
        d.z = newWaypoint.z - navData->position.z;

    }

    hover(2);

    while ((int) abs(d.y) > TOLERANCE){

        v_vec.y = getSpeed(d.y);

        if(v_vec.y != cmd.linear.y){
            printf("vx: %f\tdx: %f \n", v_vec.y, d.y);
            cmd.linear.y = v_vec.y;
            pub_control.publish(cmd);
        }

        control_loop.sleep();

        d.x = newWaypoint.x - navData->position.x;
        d.y = newWaypoint.y - navData->position.y;
        d.z = newWaypoint.z - navData->position.z;

    }
    printf("Stopped at: %f\t%f \n", navData->position.x, navData->position.y);
    /*
     * VERSION 2
     */
    //while (((int) abs(d.x)) > TOLERANCE || ((int) abs(d.y)) > TOLERANCE || (int) abs(d.z) > TOLERANCE){
  /* while (((int) abs(d.x)) > TOLERANCE || ((int) abs(d.y)) > TOLERANCE ){

        v_vec = getVelocity(d);
        //    ROS_INFO("vx: %f\tvy: %f\n",v_vec.x,v_vec.y);
        //    ROS_INFO("dx: %f\tdy: %f\n",d.x,d.y);
      // if(v_vec.x != cmd.linear.x || v_vec.y != cmd.linear.y || v_vec.z != cmd.linear.z){
        if(v_vec.x != cmd.linear.x || v_vec.y != cmd.linear.y ){
            cmd.linear.x = v_vec.x;
            cmd.linear.y = v_vec.y;
            //cmd.linear.z = v_vec.z;
            ROS_INFO("SEND: vx: %f\tvy: %f\n",v_vec.x,v_vec.y);
            pub_control.publish(cmd);
        }
        control_loop.sleep();
        d.x = newWaypoint.x - navData->position.x;
        d.y = newWaypoint.y - navData->position.y;
        //d.z = newWaypoint.z - navData->position.z;
    }
*/
    hover(2);


//    cmd.linear.y = getSpeed(dy);
//    cmd.linear.z = getSpeed(dz);







/*
    while(abs(dz) > TOLERANCE){
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
*/


/*




    int timer = 0;
    int time_limit = 200;
    if (!straightFlight) {
        // TODO decide if this should be implemented or{ not
    } else{

        while(timer < time_limit){
            if ((int) abs(dx) < TOLERANCE){
                timer++;
                cmd.linear.x = 0.0;
            }
            else {
                timer = 0;

            }

            ROS_INFO("dx = %f  speed = %f\n", d.x, cmd.linear.x);
            ROS_INFO("timer = %d  distance = %d\n", timer, (int) abs(d.x));




            dx = newWaypoint.x - navData->position.x;

        #ifdef DEBUG
            ROS_INFO("newpoint = %F", newWaypoint.x);
            ROS_INFO("navData = %F", navData->position.x);
            ROS_INFO("dx = %F", dx);
            ROS_INFO("Speed = %F", cmd.linear.x);
            ROS_INFO("rotation = %F", navData->rotation);
        #endif
            ros::Rate(10).sleep();
            moved = true;
        }
        ROS_INFO("Position is: %f\n", navData->position.x);
*/
/*
        moved = false;
        while(abs(dy) > TOLERANCE){

            cmd.linear.y = getSpeed(dy);

            pub_control.publish(cmd);
            dy = newWaypoint.y - navData->position.y;

        #ifdef DEBUG
            ROS_INFO("dy = %F", dy);
        #endif
            ros::Rate(LOOP_RATE).sleep();
            moved = t150rue;
        }

        if(moved)
            hover(1);

    }
    */
}

double FlightController::getSpeed(double distance) {
    if(distance <= TOLERANCE)
        return 0.0;

    if (distance <= CRUISE_LIMIT)
        return CRUISE_SPEED;

    return TRANSIT_SPEED;
}
MyVector FlightController::getVelocity(MyVector d) {
    double f = 1;

    MyVector v_vec;

    v_vec.x = getSpeed(d.x);
    v_vec.y = getSpeed(d.y);
    v_vec.z = getSpeed(d.z);

    double v = v_vec.distance();
    if (v > TRANSIT_SPEED)
        f = TRANSIT_SPEED/v;

    else if(v > CRUISE_SPEED)
        f = CRUISE_SPEED/v;

    v_vec.x *= f;
    v_vec.y *= f;
    v_vec.z *= f;
    return v_vec;
}

void FlightController::turnDegrees(double degrees){
    double ori_deg = navData->rotation;
    double target_deg = ori_deg+degrees;;
    float offset = 5;

    if (target_deg > 360)
        target_deg = target_deg - 360;

    do {
        ori_deg = navData->rotation;


        cmd.angular.z = getRotationalSpeed( target_deg, ori_deg);
        pub_control.publish(cmd);
    } while(ori_deg < target_deg-offset or ori_deg > target_deg+offset);
ROS_INFO("ori_deg: %6.2f", ori_deg);
    hover(1);
}

void FlightController::turnTowardsPoint(Command waypoint) {

    double target_angle = atan2(waypoint.y, waypoint.x); // angle towards waypoint position
    double target_deg = target_angle * 180 / M_PI; // conversion to degrees

    double ori_deg;

    float offset = 0.5;
    do{

    } while(ori_deg < target_deg-offset or ori_deg > target_deg+offset);
}

double FlightController::getRotationalSpeed(double target_deg, double ori_deg){
    double dir; // direction
    double rot_speed; // calculated rotational speed

    double diff_deg = target_deg - ori_deg; // calculate difference

#ifdef DEBUG
    printf("target_deg: %6.2f     deg diff_deg: %6.2f deg    ori_deg: %6.2f deg", target_deg,diff_deg,ori_deg);
#endif
    if ((diff_deg < 180 && diff_deg > 0) || (diff_deg < -180)) {
        dir = -1;
        if (diff_deg < -180)
            diff_deg = 360 + diff_deg;
#ifdef DEBUG
        printf("Turn left\n");
#endif
    } else {
        dir = 1;
        if(diff_deg > 180)
            diff_deg = 360 -diff_deg;
#ifdef DEBUG
        printf("Turn right\n");
#endif
    }

    rot_speed = 0.1; // speed to rotate with

    if(diff_deg < 30)
        rot_speed = 0.1;

    rot_speed *= dir; // make sure to rotate the correct way
#ifdef DEBUG
    printf("Rotspeed = %f\n", rot_speed);
#endif

    return rot_speed;
}

void FlightController::hover(int time){
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

#ifdef DEBUG
    ROS_INFO("HOVER");
#endif

    pub_control.publish(cmd);
    pub_control.publish(cmd);
    pub_control.publish(cmd);
    for(int i = 0; i < time*LOOP_RATE; i++){
        ros::Rate(LOOP_RATE).sleep();
    }
}

void FlightController::takeOff() {

    std_msgs::Empty empty_msg;
    pub_takeoff.publish(empty_msg);

    for(int i = 0; i < TAKEOFF_TIME*LOOP_RATE; i++){
        ros::Rate(LOOP_RATE).sleep();
    }

    hover(2);
}

void FlightController::land() {
    std_msgs::Empty empty_msg;
    pub_land.publish(empty_msg);

    for (int j = 0; j < LOOP_RATE*6; j++) {
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
    started = false;
    reset();
}

void FlightController::abortProgram(){
    ROS_INFO("MANUEL ABORT!");
    started = false;
    land();
}

void* startNavdata(void *thread_arg){
    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;

    thread_data->navData->run(thread_data->n);

    pthread_exit(NULL);
}

void* startCV(void *thread_arg) {
    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;

    thread_data->cvHandler->run(thread_data->navData);
    pthread_exit(NULL);
}

void* startController(void *thread_arg){
    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;

    thread_data->controller->run();
    pthread_exit(NULL);
}