// ROS
#include <ros/ros.h>

// C++
#include <string>

// project libraries
#include "tracker.h"

using namespace std;

const int SPIN_RATE = 50;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tracker");
    Tracker tracker;

    ros::start();

    // Main program rate.
    ros::Rate looper(SPIN_RATE);
    while(ros::ok()) {
        ros::spinOnce();
        tracker.update();
        looper.sleep();
    }

    ros::shutdown();

    return 0;
}
