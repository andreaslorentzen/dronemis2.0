//
// Created by mathias on 6/1/16.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "Route.h"
#include "FlightController.h"

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
    struct thread_data td[NUM_THREADS];

    ros::init(argc, argv, "blindFlight");

    ros::NodeHandle n;

    td[0].controller = FlightController(LOOP_RATE, n);
    td[0].myRoute = Route();
    td[1].pub_land = pub_land;

    pthread_t threads[NUM_THREADS];
    pthread_create(&threads[0], NULL, controlThread, &td[0]);
    pthread_create(&threads[1], NULL, abortThread, &td[1]);

    ros::spin();

    pthread_exit(NULL);
}


void *controlThread(void *thread_arg) {
    int takeoff_time = 3;
    double fly_time = 1.0;
    double land_time = 3.0;
    struct thread_data *thread_data;
    thread_data = (struct thread_data *) thread_arg;

    thread_data->controller.setStraightFlight(true);

    int i = 0;
    printf("Enter a key to start: ");
    getchar();
    while (ros::ok()) {

        thread_data->controller.takeOff();

        while (!thread_data->myRoute.hasAllBeenVisited()) {
            Command currentCommand = thread_data->myRoute.nextCommand();

            if (currentCommand.commandType == Command::goTo) {
                controller.goToWaypoint(currentCommand);
            } else if (currentCommand.commandType == Command::hover) {
                controller.hover(currentCommand.timeToHover);
            } else if (currentCommand.commandType == Command::turn) {
                controller.turnDrone(currentCommand.degrees);
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
            std_msgs::Empty empty_msg;
            thread_data->pub_land.publish(empty_msg);
            break;
        }   usleep(10);
    }

    // System call to set terminal behaviour to normal
    system("/bin/stty cooked");

    pthread_exit(NULL);
}