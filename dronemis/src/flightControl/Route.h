//
// Created by mathias on 6/2/16.
//

#ifndef PROJECT_ROUTE_H
#define PROJECT_ROUTE_H

#include <vector>
#include <cmath>
#include "ros/ros.h"
#include <iostream>
#include <fstream>

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
    Waypoint(double newX, double newY, double newZ): x(newX), y(newY), z(newZ)
    {
        visited = false;
    }
    Waypoint(){
        visited = false;
    }
};

class Route{

public:
    Route();
    void initRoute(bool useFile);
    Waypoint findNearestWaypoint(double x, double y);
    Waypoint nextWaypoint();
    bool hasAllBeenVisited();
private:
    vector<Waypoint> waypoints;
    int routeLength;
    int currentWaypoint;
};

#endif //PROJECT_ROUTE_H
