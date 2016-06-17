//
// Created by mathias on 6/4/16.
//

#ifndef PROJECT_FLIGHTCONTROLLER_H
#define PROJECT_FLIGHTCONTROLLER_H

#include <ros/ros.h>
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "Command.h"
#include "Route.h"
#include "../navdata/Nav.h"
#include "../OpenCv/CV_Handler.h"
#include "../OpenCv/QR.h"
#include "../Vector3.h"

class FlightController{
public:
    FlightController();
    FlightController(int loopRate, ros::NodeHandle *nh, Nav *nav);
    ~FlightController();
    void goToWaypoint(Command newWaypoint);
    void turnTowardsPoint(Command waypoint);
    void hover(int time);
    void takeOff();
    void land();
    void reset();
    void setStraightFlight(bool newState);
    void run();
    void startProgram(void);
    void resetProgram(void);
    void abortProgram(void);
    bool lookingForQR = true;
    DronePos dronePossion;
    QR* getQr(){return qr;};
private:
    bool started;
    int LOOP_RATE;
    int TAKEOFF_TIME;
    bool straightFlight;
    ros::Publisher pub_takeoff;
    ros::Publisher pub_land;
    ros::Publisher pub_control;
    ros::Publisher pub_reset;
    Nav *navData;
    CV_Handler *cvHandler;
    QR *qr;
    geometry_msgs::Twist cmd;
    Vector3 transformCoordinates(Vector3 incomingVector);
    double getRotationalSpeed(double target_deg, double ori_deg);

    int TOLERANCE;
    double TRANSIT_SPEED;
    double CRUISE_SPEED;
    double CRUISE_LIMIT;
    int CONTROL_SLEEP;

    ros::Rate control_loop = ros::Rate(0);

    double getSpeed(double distance);
    Vector3 getVelocity(Vector3 d);
    void turnDegrees(double degrees);
};

#endif //PROJECT_FLIGHTCONTROLLER_H
