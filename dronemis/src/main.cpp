//
// Created by mathias on 6/9/16.
//

#include <QtCore>
#include <QApplication>
#include "OpenCv/CV_Handler.h"
#include "flightControl/FlightController.h"
#include "GUI/ControlPanel/controlpanel.h"
#include "ros/callback_queue.h"

#define NUM_THREADS 4
#define LOOP_RATE (50)


void *buttonThread(void *thread_arg);
FlightController *controller;
Nav *nav;

struct thread_data {
    int argc;
    char **argv;
    ros::NodeHandle *n;
    Nav *nav;
};

struct thread_data td[NUM_THREADS];

int main(int argc, char **argv){
    td[0].argc = argc;
    td[0].argv = argv;


    ros::init(argc, argv, "blindFlight");
    ros::NodeHandle *n = new ros::NodeHandle();
    nav = new Nav();
    controller = new FlightController(LOOP_RATE, n, nav);

    td[0].n = n;
    td[0].nav = nav;

    pthread_t thread;
    pthread_create(&thread, NULL, buttonThread, &td[0]);

    ros::spin();

    pthread_exit(NULL);

}

void *buttonThread(void *thread_arg) {

    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;

    // Creating Control panel
    QApplication a(thread_data->argc, thread_data->argv);
    ControlPanel w;
    w.setValues(thread_data->nav, controller, thread_data->n, 300);
    w.show();
    a.exec();

    ros::Rate r(10);
    while(thread_data->n->ok()) {
        ros::spinOnce();
        r.sleep();
    }

    pthread_exit(NULL);
}
