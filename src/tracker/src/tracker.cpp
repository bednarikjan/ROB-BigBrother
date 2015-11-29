// Project libraries
#include "tracker.h"

// OpenCV libraries
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// OpenTLD
#include "ImAcq.h"
#include "TLDUtil.h"

const std::string Tracker::IMAGE_RAW_TOPIC  = "/camera/image_raw";
const std::string Tracker::KEY_DOWN_TOPIC   = "/keyboard/keydown";
const std::string Tracker::KEY_UP_TOPIC     = "/keyboard/keyup";
const std::string Tracker::WINDOW_NAME      = "tracker";

cv::Rect Tracker::bb_ = cv::Rect(-1, -1, -1, -1);
bool Tracker::drag_ = false;
cv::Point Tracker::p1_;
cv::Mat Tracker::initImg_;

Tracker::Tracker() : receivedNewFrame_(false), trackerInitialized_(false), firstFrame_(true)
{
    nh_ = new ros::NodeHandle();
    onImgSub_           = nh_->subscribe(IMAGE_RAW_TOPIC, 5, &Tracker::onImage,     this);
    keyDownSub_         = nh_->subscribe(KEY_DOWN_TOPIC,  5, &Tracker::onKeyDown,   this);
    keyUpSub_           = nh_->subscribe(KEY_UP_TOPIC,    5, &Tracker::onKeyUp,     this);

//    if(!ros::param::get("width", frameWidth_)) {
//        ROS_ERROR_STREAM("Cannot find parameter 'width'");
//        exit(1);
//    }

//    if(!ros::param::get("height", frameHeight_)) {
//        ROS_ERROR_STREAM("Cannot find parameter 'width'");
//        exit(1);
//    }

    // OpenTLD
    tld_= new tld::TLD();
//    tld_->detectorCascade->imgWidth = frameWidth_;
//    tld_->detectorCascade->imgHeight = frameHeight_;
//    tld_->detectorCascade->imgWidthStep = 3;    // ?
}

Tracker::~Tracker()
{
    delete tld_;
}

void Tracker::update()
{
    if(receivedNewFrame_){
        receivedNewFrame_ = false;

        if(trackerInitialized_) {
//            tld_->processImage(frame_->image);
            tld_->processImage(frame_);

            if(tld_->currBB != NULL) {
//                cv::rectangle(frame_->image, *(tld_->currBB), cv::Scalar(0, 255, 0));
                cv::rectangle(frame_, *(tld_->currBB), cv::Scalar(0, 255, 0));
            }
        }

//        cv::imshow(WINDOW_NAME, frame_->image);
        cv::imshow(WINDOW_NAME, frame_);

        int c = (cv::waitKey(1) & 255);
        switch(c) {
            case 255:
                break;

            // space key
            case 32:
//                if(getBBFromUser(frame_->image)) {
//                    trackerInitialized_ = true;

//                    cv::Mat gray(frame_->image.rows, frame_->image.cols, CV_8UC1);
//                    cv::cvtColor(frame_->image, gray, CV_BGR2GRAY);

////                    ROS_INFO_STREAM("selected roi: [" << bb_.x << ", " << bb_.y << ", " <<
////                                    bb_.x + bb_.width << ", " << bb_.y + bb_.height << "]");

//                    tld_->selectObject(gray, &bb_);
//                }

                if(getBBFromUser(frame_)) {
                    trackerInitialized_ = true;

                    cv::Mat gray(frame_.rows, frame_.cols, CV_8UC1);
                    cv::cvtColor(frame_, gray, CV_BGR2GRAY);

    //                    ROS_INFO_STREAM("selected roi: [" << bb_.x << ", " << bb_.y << ", " <<
    //                                    bb_.x + bb_.width << ", " << bb_.y + bb_.height << "]");

                    tld_->selectObject(gray, &bb_);
                }


                ROS_INFO_STREAM("selected roi: [" << bb_.x << ", " << bb_.y << ", " <<
                                bb_.x + bb_.width << ", " << bb_.y + bb_.height << "]");

                break;

            default:
                ROS_INFO_STREAM("key pressed: " << c << " ");
                break;
        }
    }
}

void Tracker::onImage(const sensor_msgs::Image::ConstPtr img)
{
    try {
        cv::Mat f = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8)->image;
        cv::resize(f, frame_, cv::Size(640, 480));
        receivedNewFrame_ = true;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(firstFrame_) {
        firstFrame_ = false;

//        cv::Mat gray(frame_->image.rows, frame_->image.cols, CV_8UC1);
//        cv::cvtColor(frame_->image, gray, CV_BGR2GRAY);
        cv::Mat gray(frame_.rows, frame_.cols, CV_8UC1);
        cv::cvtColor(frame_, gray, CV_BGR2GRAY);

        tld_->detectorCascade->imgWidth = gray.cols;
        tld_->detectorCascade->imgHeight = gray.rows;
        tld_->detectorCascade->imgWidthStep = gray.step;

        ROS_INFO_STREAM("image width, height, step: " << gray.cols << ", " <<
                        gray.rows << ", " << gray.step);
    }
}

void Tracker::onKeyDown(const keyboard::Key::ConstPtr key)
{
    switch (key->code) {
        case keyboard::Key::KEY_SPACE:
        break;


    }

}

void Tracker::onKeyUp(const keyboard::Key::ConstPtr key)
{
    ;
}


bool Tracker::getBBFromUser(cv::Mat& frame)
{
    Tracker::bb_ = cv::Rect(-1, -1, -1, -1);
    bool correctBB = false;

    Tracker::initImg_ = frame;

    cv::setMouseCallback(WINDOW_NAME, mouseHandler, NULL);

    cv::putText(Tracker::initImg_, "Draw a bounding box and press Enter", cv::Point(0, 60),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255));
    cv::imshow(WINDOW_NAME, Tracker::initImg_);

    while(!correctBB)
    {
            char key = (cv::waitKey(0) & 255);

            if(tolower(key) == 'q')
            {
                return false;
            }

            if(((key == '\n') || (key == '\r')) && (bb_.x != -1) && (bb_.y != -1))
            {
                correctBB = true;
            }
    }

    if(Tracker::bb_.width < 0)
    {
        Tracker::bb_.x += Tracker::bb_.width;
        Tracker::bb_.width = abs(Tracker::bb_.width);
    }

    if(Tracker::bb_.height < 0)
    {
        Tracker::bb_.y += Tracker::bb_.height;
        Tracker::bb_.height = abs(Tracker::bb_.height);
    }

    cv::setMouseCallback(WINDOW_NAME, NULL, NULL);

    return true;
}


void Tracker::mouseHandler(int event, int x, int y, int flags, void *param)
{
    /* user press left button */
    if(event == CV_EVENT_LBUTTONDOWN && !drag_)
    {
        p1_ = cv::Point(x, y);
        drag_ = true;
    }

    /* user drags the mouse */
    if(event == CV_EVENT_MOUSEMOVE && drag_)
    {
        cv::Mat img;
        initImg_.copyTo(img);

        cv::rectangle(img, cv::Rect(p1_, cv::Point(x, y)), cv::Scalar(0, 0, 255));
        cv::imshow(WINDOW_NAME, img);
    }

    /* user release left button */
    if(event == CV_EVENT_LBUTTONUP && drag_)
    {
        bb_ = cv::Rect(p1_.x, p1_.y, x - p1_.x, y - p1_.y);
        drag_ = false;
    }
}
