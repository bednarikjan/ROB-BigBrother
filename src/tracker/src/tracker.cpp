// Project libraries
#include "tracker.h"

// OpenCV libraries
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ROS libraries
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

const std::string Tracker::IMAGE_RAW_TOPIC  = "/camera/image_raw";
const std::string Tracker::KEY_DOWN_TOPIC   = "/keyboard/keydown";
const std::string Tracker::KEY_UP_TOPIC     = "/keyboard/keyup";

Tracker::Tracker()
{
    nh_ = new ros::NodeHandle();
    onImgSub_           = nh_->subscribe(IMAGE_RAW_TOPIC, 5, &Tracker::onImage,     this);
    keyDownSub_         = nh_->subscribe(KEY_DOWN_TOPIC,  5, &Tracker::onKeyDown,   this);
    keyUpSub_           = nh_->subscribe(KEY_UP_TOPIC,    5, &Tracker::onKeyUp,     this);
}

Tracker::~Tracker()
{
    ;
}

void Tracker::update()
{
    ;
}

void Tracker::onImage(const sensor_msgs::Image::ConstPtr img)
{
    cv_bridge::CvImagePtr cv_img;

    try {
        cv_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame;
    cv_img->image.copyTo(frame);

    cv::imshow("img", frame);
    cv::waitKey(1);
}

void Tracker::onKeyDown(const keyboard::Key::ConstPtr key)
{
    ;
}

void Tracker::onKeyUp(const keyboard::Key::ConstPtr key)
{
    ;
}
