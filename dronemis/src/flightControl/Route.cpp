//
// Created by mathias on 6/2/16.
//

#include "Route.h"


Route::Route() {
    currentCommand = -1;
}

//Destructor
Route::~Route() {
    // TODO implement this to actually do something
}

void Route::initRoute(bool useFile) {
    if(useFile){
        ifstream flightPlan;
        flightPlan.open("../workspaces/dronemis_ws/src/dronemis/src/flightControl/flightPlan.txt");
        char input[1000];

        if(flightPlan.is_open()) {
            #ifdef DEBUG
            ROS_INFO("is open");
            #endif
            while (!flightPlan.eof()) {
                flightPlan >> input;
                #ifdef DEBUG
                    ROS_INFO("the input is: %s", input);
                #endif
                if(strcmp(input, "goto") == 0){
                    flightPlan >> input;
                    double tempX = atof(input);
                    flightPlan >> input;
                    double tempY = atof(input);
                    flightPlan >> input;
                    double tempZ = atof(input);
                    commands.push_back(Command(tempX, tempY, tempZ));
                } else if(strcmp(input, "turn") == 0) {

                    flightPlan >> input;
#ifdef DEBUG
                    ROS_INFO("input = %s", input);
#endif
                    double degrees =  atof(input);
                    commands.push_back(Command(degrees));
                } else if(strcmp(input, "hoverDuration") == 0) {
                    flightPlan >> input;
                    int time = atof(input);
                    commands.push_back(time);
                }
            }
        }
        flightPlan.close();
#ifdef DEBUG
        ROS_INFO("READ IN WAYPOINTS");
        for(unsigned int i = 0; i < commands.size(); i++){
            ROS_INFO("Command %d", i);
        }
#endif
    } else{
        commands.push_back(Command(1.0, 0.0));
        commands.push_back(Command(1.0, 1.0));
        commands.push_back(Command(0.0, 1.0));
        commands.push_back(Command(0.0, 0.0));
    }
}

Command Route::findNearestWaypoint(double x, double y) {
    // THIS METHOD IS CURRENTLY NOT WORKING
    double nearestDistance = 100000;
    int nearestWaypointNumber = 0;


    for(unsigned int i = 0; i < commands.size(); i++){
        if (commands[i].commandType == commands[i].goTo) {
            double diffX = commands[i].x - x;
            double diffY = commands[i].y - y;
            double distance = diffX + diffY;

            if (distance < nearestDistance)
                nearestWaypointNumber = i;
        }
    }

    currentCommand = nearestWaypointNumber;
    commands[nearestWaypointNumber].visited = true;

    return commands[nearestWaypointNumber];
}

Command Route::nextCommand() {

    if(currentCommand == ((int)commands.size()))
        currentCommand = 0;
    else
        currentCommand++;

    ROS_INFO("check check : %d", currentCommand);

    commands[currentCommand].visited = true;

    return commands[currentCommand];
}

bool Route::hasAllBeenVisited() {
    for(unsigned int i = 0; i < commands.size(); i++){
        if (!commands[i].visited)
            return false;
    }
    return true;
}