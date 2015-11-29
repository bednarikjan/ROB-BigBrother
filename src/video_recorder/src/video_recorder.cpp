// Project libraries
#include "video_recorder.h"

// ROS libraries
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// C++
#include <cmath>

const std::string VideoRecorder::IMAGE_RAW_TOPIC  = "/camera/image_raw";

VideoRecorder::VideoRecorder() : windowName_("video"), playBack_(false)
{
    nh_ = new ros::NodeHandle();
    onImgPub_ = nh_->advertise<sensor_msgs::Image>(IMAGE_RAW_TOPIC, 10);

    if (!ros::param::get("~input", fileName_)) {
        ROS_FATAL_STREAM("Could not get parameter input");
        exit(1);
    }

    init();
}

VideoRecorder::~VideoRecorder()
{
    ;
}

void VideoRecorder::update()
{
    if(playBack_) {
        cv::Mat frame;

        if(!cap_.read(frame)) {
            ROS_ERROR_STREAM("Cannot read frames.");
        }

        cv::imshow(windowName_, frame);


        // Publish the image message
        cv_bridge::CvImage cv_img;
        cv_img.encoding = "bgr8";
        cv_img.image = frame;
        sensor_msgs::Image msgImg = *(cv_img.toImageMsg());
        onImgPub_.publish(msgImg);

        cv::waitKey(fps_);
    }
}

void VideoRecorder::init()
{
    cap_.open(fileName_);

    if ( !cap_.isOpened() ) {
        ROS_ERROR_STREAM("Cannot open the video file");
        exit(1);
    }

    fps_ = cap_.get(CV_CAP_PROP_FPS);

    ROS_INFO_STREAM("FPS = " << fps_);

    if(isnan(fps_) || fps_ == 0) {
        fps_ = 30;
    }

    cv::namedWindow(windowName_, CV_WINDOW_AUTOSIZE);

    playBack_ = true;
}
