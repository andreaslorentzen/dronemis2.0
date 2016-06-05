//
// Created by mathias on 6/1/16.
//

#include "blindFlight.h"

ros::Publisher pub_land;
ros::Publisher pub_takeoff;
ros::Publisher pub_control;

void *controlThread(void *thread_arg);
void *buttonThread(void *thread_arg);
void run(void);
struct blindFlight::thread_data td[NUM_THREADS];
bool started = false;

int main(int argc, char **argv) {

    td[1].argc = argc;
    td[1].argv = argv;

    ros::init(argc, argv, "blindFlight");
    ros::NodeHandle n;
    pub_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    pub_land = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
    pub_control = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    pthread_t threads[NUM_THREADS];
    pthread_create(&threads[1], NULL, buttonThread, &td[1]);

    ros::spin();

    pthread_exit(NULL);
}


void run(void) {
    if (!started) {
        ROS_INFO("STARTING!");
        pthread_t threads[NUM_THREADS];
        pthread_create(&threads[0], NULL, controlThread, &td[0]);
        started = true;
    }
}


void *controlThread(void *thread_arg) {
    int takeoff_time = 3;
    double fly_time = 1.0;
    double land_time = 3.0;

    ros::Rate loop_rate(LOOP_RATE);

    geometry_msgs::Twist cmd;

    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    double currentX = 0.0;
    double currentY = 0.0;


    double timeToFly;
    double baseSpeed = 0.5;

    Route myRoute;

    int i = 0;

    while (ros::ok()) {

        ROS_INFO("takeoff %d", (int) takeoff_time * LOOP_RATE);
        // take off
        for (; i < takeoff_time * LOOP_RATE; i++) {
            std_msgs::Empty empty_msg;
            pub_takeoff.publish(empty_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }

        // Land if the route has finished
        if (myRoute.hasAllBeenVisited()) {
            for (int j = 0; j < (takeoff_time + fly_time + land_time) * LOOP_RATE; j++) {

                std_msgs::Empty empty_msg;
                pub_land.publish(empty_msg);

                ros::spinOnce();
                loop_rate.sleep();
            }
            // return to stop the program
            pthread_exit(NULL);
        }

        Waypoint currentWaypoint = myRoute.nextWaypoint();

        ROS_INFO("waypoint = %f", currentWaypoint.x);
        ROS_INFO("waypoint = %f", currentWaypoint.y);

        double diffX = currentWaypoint.x - currentX;
        double diffY = currentWaypoint.y - currentY;
        double absX = abs(diffX);
        double absY = abs(diffY);

        double deltaDiffs;

        ROS_INFO("diffX = %f", diffX);
        ROS_INFO("diffY = %f", diffY);

        if (diffX != 0 && diffY != 0) {
            timeToFly = std::sqrt(std::pow(diffX, 2) + std::pow(diffY, 2)) / baseSpeed;
            double glideSpeed = pow(baseSpeed, 2);
            if (absX == absY) {
                double actualSpeed = sqrt(glideSpeed * 0.5);
                if (diffX < 0)
                    cmd.linear.x = -actualSpeed;
                else
                    cmd.linear.x = actualSpeed;

                if (diffY < 0)
                    cmd.linear.y = -actualSpeed;
                else
                    cmd.linear.y = actualSpeed;
            } else if (absX > absY) {
                // De her to elseifs virker måske, men er ikke ligefrem sikker.
                deltaDiffs = absY / absX;
                if (diffY < 0)
                    cmd.linear.y = -sqrt(glideSpeed * deltaDiffs);
                else
                    cmd.linear.y = sqrt(glideSpeed * deltaDiffs);

                if (diffX < 0)
                    cmd.linear.x = -sqrt(
                            glideSpeed * (1.0 - deltaDiffs));                              //Mathias de er ens?
                else
                    cmd.linear.x = -sqrt(
                            glideSpeed * (1.0 - deltaDiffs));                              //Mathias de er ens?

            } else if (absX < absY) {
                deltaDiffs = absX / absY;

                if (diffX < 0)
                    cmd.linear.x = sqrt(glideSpeed * deltaDiffs);
                else
                    cmd.linear.x = -sqrt(glideSpeed * deltaDiffs);

                if (diffY < 0)
                    cmd.linear.y = -sqrt(glideSpeed * (1.0 - deltaDiffs));
                else
                    cmd.linear.y = sqrt(glideSpeed * (1.0 - deltaDiffs));
            }
        } else {
            if (diffX != 0) {
                timeToFly = std::abs(diffX) / baseSpeed;
                if (diffX < 0)
                    cmd.linear.x = -baseSpeed;
                else
                    cmd.linear.x = baseSpeed;
            } else {
                timeToFly = std::abs(diffY) / baseSpeed;
                if (diffY < 0)
                    cmd.linear.y = -baseSpeed;
                else
                    cmd.linear.y = baseSpeed;
            }
        }
        for (int k = 0; k < timeToFly * LOOP_RATE; k++) {
            pub_control.publish(cmd);

            ros::spinOnce();
            loop_rate.sleep();
        }

        currentX = currentWaypoint.x;
        currentY = currentWaypoint.y;

        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;

        for (int k = 0; k < 0.4 * LOOP_RATE; k++) {
            pub_control.publish(cmd);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

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

void blindFlight::abortProgram(void) {
    ROS_INFO("MANUEL ABORT!");
    std_msgs::Empty empty_msg;
    pub_land.publish(empty_msg);
}

void blindFlight::resetProgram(void) {
    ROS_INFO("MANUEL RESET!");
    std_msgs::Empty empty_msg;
    pub_land.publish(empty_msg);
}

void blindFlight::startProgram(void) {
    run();
}