//
// Created by mathias on 6/2/16.
//

#ifndef PROJECT_ROUTE_H
#define PROJECT_ROUTE_H

#include <vector>
#include <cmath>
#include "ros/ros.h"

using namespace std;

struct Waypoint{
    double x;
    double y;
    double z;
    bool visited;
    Waypoint(double newX, double newY): x(newX), y(newY)
    {
        visited = false;
    }
    Waypoint(){

    }
};

class Route{

public:
    Route();
    Waypoint findNearestWaypoint(double x, double y);
    Waypoint nextWaypoint();
    bool hasAllBeenVisited();
private:
    vector<Waypoint> waypoints;
    int routeLength;
    int currentWaypoint;
};

#endif //PROJECT_ROUTE_H
