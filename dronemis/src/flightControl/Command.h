//
// Created by mathias on 6/4/16.
//

#ifndef PROJECT_WAYPOINT_H
#define PROJECT_WAYPOINT_H

struct Command{
    enum commandTypes {goTo = 1, hover = 2, turn = 3} commandType;

    double x;
    double y;
    double z;
    double degrees;
    int timeToHover;

    bool visited;
    Command(double newX, double newY): x(newX), y(newY)
    {
        visited = false;
        commandType = goTo;
    }
    Command(double newX, double newY, double newZ): x(newX), y(newY), z(newZ){
        visited = false;
        commandType = goTo;
    }
    Command(double turnDegrees): degrees(turnDegrees){
        visited = false;
        commandType = turn;
    }
    Command(int time): timeToHover(time){
        visited = false;
        commandType = hover;
    }
    Command(){
        visited = false;
    }
};

#endif //PROJECT_WAYPOINT_H
