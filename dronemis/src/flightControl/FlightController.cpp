#include "FlightController.h"

void *startNavdata(void *thread_args);

void *startCV(void *thread_args);

void *startController(void *thread_arg);

void *runQR(void *thread_arg);

struct thread_data {
    Nav *navData;
    CV_Handler *cvHandler;
    FlightController *controller;
    ros::NodeHandle *n;
} myThreadData;

FlightController::FlightController() {

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

FlightController::FlightController(int loopRate, ros::NodeHandle *nh, Nav *nav) : FlightController() {
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
    delete (cvHandler);
    delete (qr);
}

void FlightController::run() {
    Route myRoute;
    myRoute.initRoute(true);

    ros::Rate loop_rate(LOOP_RATE);
    setStraightFlight(true);

    bool firstIteration = true;

    DronePos dronePos;

    takeOff();

        bool turning = true;
        double amountTurned = 0;
        double turnStepSize = 30;


        hoverDuration(5);

        cmd.linear.z = 0.5;
        pub_control.publish(cmd);
        while (navData->getPosition().z < 1200)
            ros::Rate(LOOP_RATE).sleep();

        hoverDuration(1);

        Vector3 pos;
        double starting_orientation = navData->getRawRotation();
        ROS_INFO("Start while");

/*
       while (!dronePossion.positionLocked) {
            ROS_INFO("in while");

            if (turning) {
                turnDegrees(turnStepSize);

                double orientation = navData->getRawRotation();

                amountTurned += angleDifference(starting_orientation,orientation);
                ROS_INFO("AMOUNTTURNED: %f",amountTurned);
                starting_orientation = orientation;
                if (amountTurned >= 360)
                    turning = false;
            } else {
                navData->resetRaw();
                Command tempWaypoint(500, 0);


                goToWaypoint(tempWaypoint);

                turning = true;
                amountTurned = 0;

            }


           */
/* do {
                double targetHeading = navData->getRotation() - dronePossion.angle;

                if (dronePossion.relativeY > 150 && dronePossion.relativeY < 225) {
                    pos = navData->getPosition();
                    goToWaypoint(Command(pos.x, pos.y + (dronePos.relativeX)));
                    double currentHeading = navData->getRotation();
                    if (currentHeading < 0)
                        currentHeading = 360 + currentHeading;
                    turnDegrees(targetHeading - currentHeading);
                }
            } while ((dronePossion.numberOfQRs > 2 && !dronePossion.positionLocked));*//*




        }
        ROS_INFO("end while");

        navData->resetToPosition(dronePossion.x * 10, dronePossion.y * 10, dronePossion.heading);

*/

    flyForward(0.8);
    hoverDuration(2);
    flyForward(0.8);
    hoverDuration(2);
    turnDegrees(90);
    flyForward(0.8);
    hoverDuration(2);

/*    double start_orientation = navData->getRawRotation();
//    while(true){
        rotateDrone(1);
        for (int i = 0; i < LOOP_RATE*0.25; ++i) {
            ros::Rate(LOOP_RATE).sleep();
        }
        if(abs(start_orientation - navData->getRawRotation()) > 90){
            hoverDuration(2);
         //   break;
        }
//    }
*/

//    flyForward(0.25);
    /*hoverDuration(2);
    flyForward(0.25);
    hoverDuration(2);
    flyForward(0.25);
    */

    hoverDuration(2);


/*
        lookingForQR = false;
#ifdef DEBUG
        ROS_INFO("X = %d", dronePossion.x);
        ROS_INFO("Y = %d", dronePossion.y);
        ROS_INFO("heading = %d", dronePossion.heading);

        land();
        return;
#endif

        hoverDuration(1);

        while (!myRoute.hasAllBeenVisited()) {
            Command currentCommand;
            */
/*if(firstIteration) {
                currentCommand = myRoute.findNearestWaypoint(navData->position.x, navData->position.y,
                                                             navData->position.z);
                firstIteration = false;
            }else {
                currentCommand = myRoute.nextCommand();
            }*//*

            currentCommand = myRoute.nextCommand();
            for (int i = 0; i < 100; i++)
                ROS_INFO("IT IS HERE!!!!!!!!!!!!!!");
            if (currentCommand.commandType == Command::goTo) {
                goToWaypoint(currentCommand);
            } else if (currentCommand.commandType == Command::hover) {
                hoverDuration(currentCommand.timeToHover);
            } else if (currentCommand.commandType == Command::turn) {

                turnDegrees(currentCommand.degrees);
            }
        }
*/
    land();


    return;
}

void FlightController::goToWaypoint(Command newWaypoint) {

    Vector3 pos = navData->getPosition();

    Vector3 d(newWaypoint.x - pos.x,
              newWaypoint.y - pos.y,
            //newWaypoint.z - navData->position.z);
              0);

    Vector3 v_vec(0.0, 0.0, 0.0);
    printf("goto: pos: %3.f\t%3.f, d:%3.f\t%3.f  \n", pos.x, pos.y, d.x, d.y);


    /*
     * VERSION 1
     */

    turnTowardsPoint(newWaypoint);

    while (d.x > TOLERANCE) {

        v_vec.x = getSpeed(d.distance());

        if (v_vec.x != cmd.linear.x) {
            printf("vx: %f\tdx: %f \n", v_vec.x, d.distance());
            cmd.linear.x = v_vec.x;
            pub_control.publish(cmd);
        }

        control_loop.sleep();

        pos = navData->getPosition();
        d.x = newWaypoint.x - pos.x;
        d.y = newWaypoint.y - pos.y;
        d.z = newWaypoint.z - pos.z;

    }

    hoverDuration(2);

    /*while ((int) abs(d.y) > TOLERANCE){

        v_vec.y = getSpeed(d.y);

        if(v_vec.y != cmd.linear.y){
            printf("vx: %f\tdx: %f \n", v_vec.y, d.y);
            cmd.linear.y = v_vec.y;
            pub_control.publish(cmd);
        }

        control_loop.sleep();

        pos = navData->getPosition();
        d.x = newWaypoint.x - pos.x;
        d.y = newWaypoint.y - pos.y;
        d.z = newWaypoint.z - pos.z;

    }*/
#ifdef DEBUG
    printf("Stopped at: %f\t%f \n", navData->getPosition().x, navData->getPosition().y);
#endif
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
    hoverDuration(2);


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
        hoverDuration(1);
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
            hoverDuration(1);

    }
    */
}

double FlightController::getSpeed(double distance) {
    if (abs(distance) <= TOLERANCE)
        return 0.0;


    if (abs(distance) <= CRUISE_LIMIT) if (distance < 0)
        return -CRUISE_SPEED;
    else
        return CRUISE_SPEED;

    if (distance < 0)
        return -TRANSIT_SPEED;
    else
        return TRANSIT_SPEED;
}

Vector3 FlightController::getVelocity(Vector3 d) {
    double f = 1;

    Vector3 v_vec;

    v_vec.x = getSpeed(d.x);
    v_vec.y = getSpeed(d.y);
    v_vec.z = getSpeed(d.z);

    double v = v_vec.distance();
    if (v > TRANSIT_SPEED)
        f = TRANSIT_SPEED / v;

    else if (v > CRUISE_SPEED)
        f = CRUISE_SPEED / v;

    v_vec.x *= f;
    v_vec.y *= f;
    v_vec.z *= f;
    return v_vec;
}


/*
void FlightController::turnDegrees(double degrees){
    double ori_deg = navData->getRotation();
    double target_deg = ori_deg+degrees;;
    int iterations;

    iterations = abs(((int)degrees/30))+1;

    ROS_INFO("iterations = %d", iterations);

    if(iterations ==1)
        iterations = 2;

    float offset = 1;



    for(int i = 1; i < iterations; i++){
        ROS_INFO("Now inside the loop: %d",i);
        if(i == iterations-1 && ((int)degrees % 30) != 0)
            if(degrees > 0)
                target_deg = ori_deg + ((int)degrees%30);
            else
                target_deg = ori_deg - ((int)degrees%30);
        else
            if(degrees > 0)
                target_deg = ori_deg + 30;
            else
                target_deg = ori_deg -30;

        if (target_deg > 360)
            target_deg = target_deg - 360;

        if (target_deg < 0)
            target_deg = target_deg + 360;

        do {
            ori_deg = navData->getRotation();
            cmd.angular.z = getRotationalSpeed(target_deg, ori_deg);
            pub_control.publish(cmd);
        } while (ori_deg < target_deg - offset or ori_deg > target_deg + offset);
        hover(1);
    }
#ifdef DEBUG
    //ROS_INFO("ori_deg: %6.2f", ori_deg);
#endif
    hoverDuration(1);
}
*/

double FlightController::formatAngle(double angle) {
    double result = angle;
    if (result < 0)
        result += 360;
    else if (result > 360)
        result -= 360;
    // ROS_INFO("FORMAT ANGLE: %6.2f\t RESULT ANGLE: %6.2f", angle, result);
    return result;
}

double FlightController::angleDifference(double a1, double a2) {
    double angle = a1 - a2;
    if (angle < -180) {
            angle = 360 + angle;
    } else if (angle > 180)
            angle = 360 - angle;
    if (angle < 0)
        angle *= -1;
    return angle;
}

int FlightController::angleDirection(double a1, double a2) {
    int direction = 1;
    double difference = a2 - a1; // calculate difference
    if ((difference < 180 && difference > 0) || (difference < -180))
        direction = -1;
    return direction;
}

double FlightController::scaleValueTo(double value, double target) {
    return (target / value) * value;
}

/*int last_ts = (int) (ros::Time::now().toNSec() / 1000000);
int time = (int) (ros::Time::now().toNSec() / 1000000);
time_counter += (time - last_ts);
last_ts = time;*/
void FlightController::turnDegrees(double degrees) {
    double orientation = navData->getRotation();
    double target = formatAngle(orientation + degrees);
    double difference = angleDifference(orientation, target);
    int direction = angleDirection(orientation, target);
    int last_direction = direction;
    ROS_INFO("Degrees: %3.1f\torientation: %3.1f\ttarget: %3.1f\tdifference: %3.1f\tdirection: %d\t",degrees, orientation,target, difference, direction  );
    int last_ts = (int) (ros::Time::now().toNSec() / 1000000);
    int time_counter = 0;


    rotateDrone(direction * 1.0);
    int debug_counter = 0;
    while (true) {

        int time = (int) (ros::Time::now().toNSec() / 1000000);
        time_counter += (time - last_ts);
        last_ts = time;
        debug_counter++;
        orientation = navData->getRotation();
        direction = angleDirection(orientation, target);
        difference = angleDifference(orientation, target);

        //if(debug_counter % 100 == 0)
           // ROS_INFO("LOOP orientation: %3.1f\tdifference: %3.1f\tdirection: %d\ttime_counter: %d\ttarget: %3.0f",orientation,difference,direction, time_counter, target);
        if (time_counter > 100) {
            hoverDuration(1);
            time_counter = 0;
            last_ts = (int) (ros::Time::now().toNSec() / 1000000);
            rotateDrone(direction * 1.0);
        }

        if( difference < (time_counter*time_counter)/20000+4){
            //hoverDuration(3);
            break;
        }


    }

    ROS_INFO("VICTORY! orientation: %6.1f\ttarget: %6.1f", orientation, target);
    hoverDuration(3);
}


void FlightController::rotateDrone(double speed) {
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = speed;
    pub_control.publish(cmd);

}

void FlightController::hoverDrone() {
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    pub_control.publish(cmd);
}


/*
void FlightController::turnDegrees(double degrees) {
    double orientation = navData->getRotation();
    double target = formatAngle(orientation + degrees);
    double difference = angleDifference(orientation, target);
    double precision = 3;
    double slice_max = 30;
    double slice;
    int direction;

    ROS_INFO("ori: %3.1f\ttar: %3.1f\tdiff: %3.1f\tdeg: %3.1f", orientation, target, difference, degrees);
    int slice_counter = 0; // FOR TESTING PURPOSE
    while (difference > precision) {
        slice_counter++;
        if (difference > slice_max)
            slice = slice_max;
        else
            slice = difference;

        double slice_target = formatAngle(orientation - slice * angleDirection(orientation, target));
        double slice_difference = angleDifference(orientation, slice_target);
        double stop_prediction;
        direction = angleDirection(orientation, slice_target);


        if (slice > slice_max)
            stop_prediction = 0;
        else
            stop_prediction = 1 + (slice_max - (slice_max - slice)) / 3;
        ROS_INFO(
                "BEFORE INNER:\tcounter: %d\tori: %3.0f\ttar: %3.0f\tdiff: %3.0f\ts_tar: %3.0f\ts_diff: %3.0f\tslice: %3.0f\tdir: %d",
                slice_counter, orientation, target, difference, slice_target, slice_difference, slice, direction);
        while (slice_difference > stop_prediction) {
            cmd.angular.z = 0.5 * direction;
            pub_control.publish(cmd);
            if (difference < 10) {
                ROS_INFO("SLEEP INNER");
                cmd.angular.z = 0;
                pub_control.publish(cmd);
            }
            orientation = navData->getRotation();
            slice_difference = angleDifference(orientation, slice_target);
            ROS_INFO("INNER:\tcounter: %d\tori: %3.0f\ttar: %3.0f\tdiff: %3.0f\ts_tar: %3.0f\ts_diff: %3.0f\tslice: %3.0f\tdir: %d",
                     slice_counter, orientation, target, difference, slice_target, slice_difference, slice, direction);
        }
        ROS_INFO(
                "AFTER INNER:\tcounter: %d\tori: %3.0f\ttar: %3.0f\tdiff: %3.0f\ts_tar: %3.0f\ts_diff: %3.0f\tslice: %3.0f\tdir: %d",
                slice_counter, orientation, target, difference, slice_target, slice_difference, slice, direction);
        ROS_INFO("--------------------------");
        hover(3);
        orientation = navData->getRotation();
        difference = formatAngle(orientation - target);
    }
    ROS_INFO("VICTORY!!!!: ori: %3.0f\ttar: %3.0f\tdiff: %3.0f", orientation, target, difference);
    hoverDuration(5);
}*/

void FlightController::turnTowardsPoint(Command waypoint) {


    Vector3 pos = navData->getPosition();

    ROS_INFO("X = %f ", pos.x);
    ROS_INFO("Y = %f ", pos.y);

    double target_angle = atan2(waypoint.y - pos.y, waypoint.x - pos.x); // angle towards waypoint position
    double target_deg = target_angle / 180 * M_PI; // conversion to degrees

    ROS_INFO("Target angle  = %f", target_angle);
    ROS_INFO("Target degrees = %f", target_deg);

    if (target_deg < 0)
        target_deg += 360;

    if (target_deg > 360)
        target_deg -= 360;

    double ori_deg = navData->getRotation();


    ROS_INFO("Original degrees = %f", ori_deg);
    turnDegrees(target_deg - ori_deg);
}

double FlightController::getRotationalSpeed(double target_deg, double ori_deg) {
    double dir; // direction
    double rot_speed; // calculated rotational speed

    double diff_deg = target_deg - ori_deg; // calculate difference

#ifdef DEBUG
    //printf("target_deg: %6.2f     deg diff_deg: %6.2f deg    ori_deg: %6.2f deg", target_deg,diff_deg,ori_deg);
#endif
    if ((diff_deg < 180 && diff_deg > 0) || (diff_deg < -180)) {
        dir = -1;
        if (diff_deg < -180)
            diff_deg = 360 + diff_deg;
#ifdef DEBUG
        //printf("Turn left\n");
#endif
    } else {
        dir = 1;
        if (diff_deg > 180)
            diff_deg = 360 - diff_deg;
#ifdef DEBUG
        //printf("Turn right\n");
#endif
    }

    rot_speed = 0.1; // speed to rotate with

    if (diff_deg < 30)
        rot_speed = 0.1;

    rot_speed *= dir; // make sure to rotate the correct way
#ifdef DEBUG
    //printf("Rotspeed = %f\n", rot_speed);
#endif

    return rot_speed;
}

void FlightController::hoverDuration(int time) {

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
    for (int i = 0; i < time * LOOP_RATE; i++) {
        ros::Rate(LOOP_RATE).sleep();
    }
}

void FlightController::takeOff() {

    std_msgs::Empty empty_msg;
    pub_takeoff.publish(empty_msg);

    for (int i = 0; i < TAKEOFF_TIME * LOOP_RATE; i++) {
        ros::Rate(LOOP_RATE).sleep();
    }

    hoverDuration(2);
}

void FlightController::land() {
    std_msgs::Empty empty_msg;
    pub_land.publish(empty_msg);

    for (int j = 0; j < LOOP_RATE * 6; j++) {
        ros::Rate(LOOP_RATE).sleep();
    }
}

void FlightController::reset() {
    std_msgs::Empty empty_msg;
    pub_reset.publish(empty_msg);
}

void FlightController::setStraightFlight(bool newState) {
    straightFlight = newState;
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

void FlightController::resetProgram() {
    DronePos dronepos = qr->checkQR();
    ROS_INFO("found : %d", dronepos.numberOfQRs);
    cout << "Relative position (x,y) = " << dronepos.relativeX << "," << dronepos.relativeY << endl;
    cout << "Angle and positionLock" << dronepos.angle << " and " << dronepos.positionLocked << endl;
    //ROS_INFO("MANUEL RESET!");
    //started = false;
    //reset();
}

void FlightController::abortProgram() {
    ROS_INFO("MANUEL ABORT!");
    started = false;
    land();
}

geometry_msgs::Twist FlightController::getEmptyCmd() {
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
}

void FlightController::flyForward(double time) {
    geometry_msgs::Twist cmd = getEmptyCmd();
    cmd.linear.x = 1;
    pub_control.publish(cmd);
    for (int i = 0; i < LOOP_RATE*time; ++i) {
        ros::Rate(LOOP_RATE).sleep();
    }
}


void *startNavdata(void *thread_arg) {
    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;

    thread_data->navData->run(thread_data->n);

    pthread_exit(NULL);
}

void *startCV(void *thread_arg) {
    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;

    thread_data->cvHandler->run(thread_data->navData);
    pthread_exit(NULL);
}

void *startController(void *thread_arg) {
    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;

    pthread_t thread;
    pthread_create(&thread, NULL, runQR, thread_data);


    thread_data->controller->run();

    pthread_exit(NULL);
}

void *runQR(void *thread_arg) {
    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;
#ifdef DEBUG
    ROS_INFO("Inside the thread");
#endif
    while (thread_data->controller->lookingForQR) {
        thread_data->controller->dronePossion = thread_data->controller->getQr()->checkQR();
        ros::Rate(25).sleep();
    }
    pthread_exit(NULL);
}