#ifndef VIDEO_RECORDER_H
#define VIDEO_RECORDER_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// OpenCV libraries
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class VideoRecorder {
public:
    static const std::string IMAGE_RAW_TOPIC;    //!< Name of the topic conveying camera stream.

public:
    VideoRecorder();
    ~VideoRecorder();

    void update();

private:
    ros::NodeHandle* nh_;
    ros::Publisher onImgPub_;

    std::string windowName_;
    std::string fileName_;
    cv::VideoCapture cap_;
    double fps_;
    bool playBack_;

private:
    void init();
};

#endif // #ifndef VIDEO_RECORDER_H
