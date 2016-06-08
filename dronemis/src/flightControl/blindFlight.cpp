//
// Created by mathias on 6/1/16.
//

#include "blindFlight.h"
#include "../OpenCv/CV_Handler.h"

void *controlThread(void *thread_arg);
void *buttonThread(void *thread_arg);
void *cvThread(void *thread_arg);
struct blindFlight::thread_data td[NUM_THREADS];
bool started = false;
FlightController *controller;
CV_Handler *cvHandler;

int main(int argc, char **argv) {


    td[1].argc = argc;
    td[1].argv = argv;

    ros::init(argc, argv, "blindFlight");
    ros::NodeHandle n;

    controller = new FlightController(LOOP_RATE, n);

    pthread_t threads[2];
    pthread_create(&threads[1], NULL, buttonThread, &td[1]);
    pthread_create(&threads[1], NULL, cvThread, &td[2]);


    ros::spin();

    pthread_exit(NULL);
}


void *controlThread(void *thread_arg) {

    Route myRoute;
    myRoute.initRoute(true);

    ros::Rate loop_rate(LOOP_RATE);
    controller->setStraightFlight(true);

    while (ros::ok()) {

        controller->takeOff();

        while (!myRoute.hasAllBeenVisited()) {
            Command currentCommand = myRoute.nextCommand();

            if (currentCommand.commandType == Command::goTo) {
                controller->goToWaypoint(currentCommand);
            } else if (currentCommand.commandType == Command::hover) {
                controller->hover(currentCommand.timeToHover);
            } else if (currentCommand.commandType == Command::turn) {
                controller->turnDrone(currentCommand.degrees);
            }
        }

        controller->land();

        started = false;
        pthread_exit(NULL);

    }

    started = false;
    pthread_exit(NULL);
}

void *buttonThread(void *thread_arg) {

    struct blindFlight::thread_data *thread_data;
    thread_data = (struct blindFlight::thread_data *) thread_arg;

    // Creating Control panel
    QApplication a(thread_data->argc, thread_data->argv);
    ControlPanel w;
    w.show();
    a.exec();

    pthread_exit(NULL);
}

void *cvThread(void *thread_arg) {
    cvHandler = new CV_Handler();
    cvHandler->run();

    pthread_exit(NULL);
}

void blindFlight::abortProgram(void) {
    ROS_INFO("MANUEL ABORT!");
    controller->reset();
}

void blindFlight::resetProgram(void) {
    ROS_INFO("MANUEL RESET!");
    controller->reset();
}

void blindFlight::startProgram(void) {
    if (!started) {
        ROS_INFO("STARTING!");
        pthread_t thread;
        pthread_create(&thread, NULL, controlThread, &td[0]);
        started = true;
    }
}