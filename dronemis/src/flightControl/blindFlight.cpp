//
// Created by mathias on 6/1/16.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "Route.h"
#include "FlightController.h"

#define LOOP_RATE (50)

int main(int argc, char **argv) {

    ros::init(argc, argv, "blindFlight");

    ros::NodeHandle n;

    Route myRoute;
    myRoute.initRoute(true);

    FlightController controller(LOOP_RATE, n);
    controller.setStraightFlight(true);

    int i = 0;
    while (ros::ok()) {

        controller.takeOff();

        while(!myRoute.hasAllBeenVisited()){
            Command currentCommand = myRoute.nextCommand();

            if(currentCommand.commandType == Command::goTo){
                controller.goToWaypoint(currentCommand);
            } else if(currentCommand.commandType == Command::hover){
                controller.hover(currentCommand.timeToHover);
            } else if(currentCommand.commandType == Command::turn){
                controller.turnDrone(currentCommand.turn);
            }
        }

        controller.land();

        return 0;
    }

    return 0;
}