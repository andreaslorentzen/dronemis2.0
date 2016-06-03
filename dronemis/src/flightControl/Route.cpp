//
// Created by mathias on 6/2/16.
//

#include "Route.h"


Route::Route() {
    routeLength = 4;

    waypoints.push_back(Waypoint(1.0, 0.0));
    waypoints.push_back(Waypoint(1.0, 1.0));
    waypoints.push_back(Waypoint(0.0, 1.0));
    waypoints.push_back(Waypoint(0.0, 0.0));

    currentWaypoint = -1;
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
    if(currentWaypoint == routeLength-1)
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