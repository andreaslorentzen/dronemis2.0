//
// Created by mathias on 6/9/16.
//

#include <QApplication>
#include "OpenCv/CV_Handler.h"
#include "flightControl/FlightController.h"
#include "GUI/ControlPanel/controlpanel.h"
#include "ros/callback_queue.h"

#define NUM_THREADS 4
#define LOOP_RATE (50)

void *controlThread(void *thread_arg);
void *buttonThread(void *thread_arg);
void *cvThread(void *thread_arg);
void *navdataThread(void *thread_Arg);

bool started = false;
FlightController *controller;

struct thread_data {
    int argc;
    char **argv;
};

struct thread_data td[NUM_THREADS];

int main(int argc, char **argv){
    td[0].argc = argc;
    td[0].argv = argv;

    ros::init(argc, argv, "blindFlight");
    ros::NodeHandle *n = new ros::NodeHandle();
    ros::MultiThreadedSpinner spinner;

    controller = new FlightController(LOOP_RATE, n, spinner);

    pthread_t thread;
    pthread_create(&thread, NULL, buttonThread, &td[0]);

    pthread_exit(NULL);
}

void *buttonThread(void *thread_arg) {

    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;

    // Creating Control panel
    QApplication a(thread_data->argc, thread_data->argv);
    ControlPanel w;
    w.setController(controller);
    w.show();
    a.exec();

    pthread_exit(NULL);
}
