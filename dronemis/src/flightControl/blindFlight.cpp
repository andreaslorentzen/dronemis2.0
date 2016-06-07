//
// Created by mathias on 6/1/16.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "Route.h"
#include "FlightController.h"

#define NUM_THREADS 2
#define LOOP_RATE (50)

struct thread_data{
    ros::Publisher pub_land;
    ros::Publisher pub_takeoff;
    ros::Publisher pub_control;
    FlightController controller;
    Route myRoute;
};

void *controlThread(void *thread_arg);
void *abortThread(void *thread_arg);

int main(int argc, char **argv) {

    ros::init(argc, argv, "blindFlight");

    struct thread_data td[NUM_THREADS];


    ros::NodeHandle n;

    FlightController controller(LOOP_RATE, n);
    td[0].myRoute = Route();
    td[0].myRoute.initRoute(true);

    td[0].controller = controller;
    td[1].controller = controller;

    pthread_t threads[NUM_THREADS];
    pthread_create(&threads[0], NULL, controlThread, &td[0]);
    pthread_create(&threads[1], NULL, abortThread, &td[1]);

    ros::spin();

    pthread_exit(NULL);
}


void *controlThread(void *thread_arg) {
    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;

    thread_data->controller.setStraightFlight(true);


    while (ros::ok()) {

        printf("Enter a key to start: ");
        getchar();

        thread_data->controller.takeOff();

        while (!thread_data->myRoute.hasAllBeenVisited()) {
            Command currentCommand = thread_data->myRoute.nextCommand();

            if (currentCommand.commandType == Command::goTo) {
                thread_data->controller.goToWaypoint(currentCommand);
            } else if (currentCommand.commandType == Command::hover) {
                thread_data->controller.hover(currentCommand.timeToHover);
            } else if (currentCommand.commandType == Command::turn) {
                thread_data->controller.turnDrone(currentCommand.degrees);
            }
        }

        thread_data->controller.land();

        pthread_exit(NULL);

    }

    pthread_exit(NULL);
}

void *abortThread(void *thread_arg) {
    struct thread_data *thread_data;
    thread_data = (struct thread_data*) thread_arg;

    // System call to make terminal send all keystrokes directly to stdin
    system("/bin/stty raw");

    while (1) {
        // Abort if 'Esc' is pressed
        if  (getchar() == 27) {
            ROS_INFO("MANUEL ABORT!");
            thread_data->controller.land();
            break;
        }   usleep(10);
    }

    // System call to set terminal behaviour to normal
    system("/bin/stty cooked");

    pthread_exit(NULL);
}