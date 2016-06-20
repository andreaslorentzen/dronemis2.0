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
#include <string>
#include "Command.h"

using namespace std;

class Route{

public:
    Route();
    ~Route();
    void initRoute(bool useFile);
    Command findNearestWaypoint(double x, double y);
    Command nextCommand();
    bool hasAllBeenVisited();
private:
    vector<Command> commands;
    int currentCommand;
};

#endif //PROJECT_ROUTE_H
