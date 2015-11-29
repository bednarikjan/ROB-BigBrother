// ROS
#include <ros/ros.h>

// C++
#include <string>

// project libraries
#include "video_recorder.h"

using namespace std;

const int SPIN_RATE = 100;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "video_recorder");

    VideoRecorder vidRec;

    ros::start();

    // Main program rate.
    ros::Rate looper(SPIN_RATE);
    while(ros::ok()) {
        ros::spinOnce();
        vidRec.update();
        looper.sleep();
    }

    ros::shutdown();

    return 0;
}
