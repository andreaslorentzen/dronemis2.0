//
// Created by mathias on 6/9/16.
//

#include <QApplication>
#include "OpenCv/CV_Handler.h"
#include "flightControl/FlightController.h"
#include "GUI/ControlPanel/controlpanel.h"

#define NUM_THREADS 4
#define LOOP_RATE (50)

void *controlThread(void *thread_arg);
void *buttonThread(void *thread_arg);
void *cvThread(void *thread_arg);
void *navdataThread(void *thread_Arg);

bool started = false;
FlightController *controller;
CV_Handler *cvHandler;
Nav *navdata;

struct thread_data {
    int argc;
    char **argv;
};

struct thread_data td[NUM_THREADS];

int main(int argc, char **argv){
    td[0].argc = argc;
    td[0].argv = argv;

    ros::init(argc, argv, "blindFlight");
    ros::NodeHandle n;

    controller = new FlightController(LOOP_RATE, n);
    cvHandler = new CV_Handler();
    navdata = new Nav(n);

    pthread_t threads[NUM_THREADS];
    pthread_create(&threads[0], NULL, buttonThread, &td[0]);
    pthread_create(&threads[1], NULL, cvThread, &td[1]);
    pthread_create(&threads[2], NULL, navdataThread, &td[2]);

    ros::spin();

    pthread_exit(NULL);
}

void *controlThread(void *thread_arg){
    controller->run(navdata, cvHandler);
    started = false;
    pthread_exit(NULL);
}

void *navdataThread(void *thread_arg){
    navdata->run();
    pthread_exit(NULL);
}

void *buttonThread(void *thread_arg) {

    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;

    // Creating Control panel
    QApplication a(thread_data->argc, thread_data->argv);
    ControlPanel w;
    w.show();
    a.exec();

    pthread_exit(NULL);
}

void *cvThread(void *thread_arg) {
    cvHandler->run();

    pthread_exit(NULL);
}

void blindFlight::abortProgram(void) {
    ROS_INFO("MANUEL ABORT!");
    controller->land();
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