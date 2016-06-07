//
// Created by mathias on 6/2/16.
//

#include "Route.h"


Route::Route() {
    currentWaypoint = -1;
}

void Route::initRoute(bool useFile) {
    if(useFile){
        //TODO read in the file
        ifstream flightPlan;
        char input[100];
        flightPlan.open("flightPlan.txt");
        if(flightPlan.is_open())
            while(!flightPlan.eof()){
                flightPlan >> input;
                ROS_INFO("Hi there");
            }

        flightPlan.close();
    } else{
        waypoints.push_back(Waypoint(1.0, 0.0));
        waypoints.push_back(Waypoint(1.0, 1.0));
        waypoints.push_back(Waypoint(0.0, 1.0));
        waypoints.push_back(Waypoint(0.0, 0.0));
    }
}

Waypoint Route::findNearestWaypoint(double x, double y) {
    double nearestDistance = 0.0;
    int nearestWaypointNumber = 0;


    for(unsigned int i = 0; i < waypoints.size(); i++){
        double diffX = waypoints[i].x - x;
        double diffY = waypoints[i].y - y;
        double distance = std::sqrt(std::pow(diffX, 2) + std::pow(diffY, 2));

        if (distance < nearestDistance)
            nearestWaypointNumber = i;
    }

    currentWaypoint = nearestWaypointNumber;
    return waypoints[nearestWaypointNumber];
}

Waypoint Route::nextWaypoint() {

    if(currentWaypoint == ((int)waypoints.size()))
        currentWaypoint = 0;
    else
        currentWaypoint++;

    ROS_INFO("check check : %d", currentWaypoint);

    waypoints[currentWaypoint].visited = true;

    return waypoints[currentWaypoint];
}

bool Route::hasAllBeenVisited() {
    for(unsigned int i = 0; i < waypoints.size(); i++){
        if (!waypoints[i].visited)
            return false;
    }
    return true;
}