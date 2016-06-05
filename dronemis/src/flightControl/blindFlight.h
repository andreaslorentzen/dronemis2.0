//
// Created by hippomormor on 6/4/16.
//

#ifndef PROJECT_BLINDFLIGHT_H
#define PROJECT_BLINDFLIGHT_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "Route.h"
#include "../GUI/ControlPanel/controlpanel.h"
#include <QApplication>

#define NUM_THREADS 2
#define LOOP_RATE (50)

class blindFlight {

public:
    struct thread_data {
        int argc;
        char **argv;
    };

    static void abortProgram(void);
    static void resetProgram(void);
    static void startProgram(void);
};

#endif //PROJECT_BLINDFLIGHT_H
