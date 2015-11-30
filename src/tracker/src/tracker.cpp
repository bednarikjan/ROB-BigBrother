// Project libraries
#include "tracker.h"

// OpenCV libraries
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// OpenTLD
#include "ImAcq.h"
#include "TLDUtil.h"

const std::string Tracker::IMAGE_RAW_TOPIC  = "camera/image_raw";
const std::string Tracker::KEY_DOWN_TOPIC   = "keyboard/keydown";
const std::string Tracker::KEY_UP_TOPIC     = "keyboard/keyup";
const std::string Tracker::JOY_TOPIC        = "joy";
const std::string Tracker::MANIP_MOVE_TOPIC = "manip_move";
const std::string Tracker::WINDOW_NAME      = "tracker";

const int Tracker::RADIUS_CENTER    = 20;
const int Tracker::RADIUS_INERTIA   = 50;

cv::Rect Tracker::bb_ = cv::Rect(-1, -1, -1, -1);
bool Tracker::drag_ = false;
cv::Point Tracker::p1_;
cv::Mat Tracker::initImg_;

Tracker::Tracker() : state_(INITIALIZATION), receivedNewFrame_(false), trackerInitialized_(false),
    firstFrame_(true), cameraInMotion_(false), maxAzimuthSpeed_(250), maxElevationSpeed_(250),
    dwBB_(0), dhBB_(0)
{
    nh_ = new ros::NodeHandle();

    // Load parameters from ROS parameter server.
    if (!ros::param::get("/master_ns", master_ns_)) {
        ROS_FATAL_STREAM("Could not get parameter overview_unit_ns");
        exit(1);
    }

    onImgSub_           = nh_->subscribe(IMAGE_RAW_TOPIC, 5, &Tracker::onImage,     this);
    keyDownSub_         = nh_->subscribe(std::string("/") + master_ns_ + std::string("/") + KEY_DOWN_TOPIC,  5, &Tracker::onKeyDown,   this);
    keyUpSub_           = nh_->subscribe(std::string("/") + master_ns_ + std::string("/") + KEY_UP_TOPIC,    5, &Tracker::onKeyUp,     this);
    joySub_             = nh_->subscribe(std::string("/") + master_ns_ + std::string("/") + JOY_TOPIC,       5, &Tracker::onJoy,       this);
    manipMovePub_       = nh_->advertise<msgs::ManipulatorMove>(MANIP_MOVE_TOPIC, 5);

    // OpenTLD
    tld_= new tld::TLD();
}

Tracker::~Tracker()
{
    delete tld_;
}

void Tracker::update()
{
    if(!firstFrame_) {
        // Let the user select a bounding box
        if(state_ == INITIALIZATION) {
            cv::Mat frameToShow;
            frame_.copyTo(frameToShow);

            if(dwBB_) ch_.updateWidth(dwBB_);
            if(dhBB_) ch_.updateHeight(dhBB_);

            ch_.draw(frameToShow);

            cv::imshow(WINDOW_NAME, frameToShow);
            cv::waitKey(1);
        }

        // Track
        else if(state_ == TRACKING) {
            if(receivedNewFrame_) {
                receivedNewFrame_ = false;

                cv::Mat frameShow;
                frame_.copyTo(frameShow);

                tld_->processImage(frame_);

                if(tld_->currBB != NULL) {
                    cv::Rect  objBB = *(tld_->currBB);
                    cv::Point objCenter(objBB.x + objBB.width / 2, objBB.y + objBB.height / 2);
                    cv::rectangle(frameShow, objBB, cv::Scalar(0, 255, 0));

                    // Move the manipualtor so that the object would get to the center.
                    double obj2center = euclideanDistance(frameCenter_, objCenter);
                    if( cameraInMotion_ && obj2center > (double)RADIUS_CENTER ||
                       !cameraInMotion_ && obj2center > (double)RADIUS_INERTIA) {

                        cameraInMotion_ = true;
                        move(((double)(frameCenter_.x - objCenter.x) / frameCenter_.x) * maxAzimuthSpeed_,
                             ((double)(frameCenter_.y - objCenter.y) / frameCenter_.y) * maxElevationSpeed_);

                    } else if(cameraInMotion_) {
                        // TODO stop manipulator after some timeout
                        cameraInMotion_ = false;
                        move(0.0, 0.0);
                    }
                } else if(cameraInMotion_) {
                    cameraInMotion_ = false;
                    move(0.0, 0.0);
                }

                cv::circle(frameShow, frameCenter_, RADIUS_CENTER, cv::Scalar(0, 0, 255));
                cv::circle(frameShow, frameCenter_, RADIUS_INERTIA, cv::Scalar(255, 0, 255));

                cv::imshow(WINDOW_NAME, frameShow);
                cv::waitKey(1);
            }
        }

        // Other prospecitve program mode(s)
        else {
            ;
        }
    }
}

void Tracker::move(double azi, double ele)
{
    msgs::ManipulatorMove mvMsg;
    mvMsg.type = "continuous";
    mvMsg.azimuth   = azi;
    mvMsg.elevation = ele;
    manipMovePub_.publish(mvMsg);
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

        frameWidth_     = gray.cols;
        frameHeight_    = gray.rows;
        frameCenter_    = cv::Point(frameWidth_ / 2, frameHeight_ / 2);

        tld_->detectorCascade->imgWidth = frameWidth_;
        tld_->detectorCascade->imgHeight = frameHeight_;
        tld_->detectorCascade->imgWidthStep = gray.step;

        ROS_INFO_STREAM("image width, height, step: " << gray.cols << ", " <<
                        gray.rows << ", " << gray.step);

        ch_.x(frameCenter_.x);
        ch_.y(frameCenter_.y);
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

void Tracker::onJoy(const sensor_msgs::Joy::ConstPtr& joy)
{
    static bool buttonDown = false;

    if(!buttonDown) {
        // Buttons
        if(joy->buttons[9]) {
            buttonDown = true;
            switch(state_) {
                case INITIALIZATION:
                    initTracker(ch_.boundingBox());
                    changeState(TRACKING);
                    break;

                case TRACKING:
                    changeState(INITIALIZATION);
                    break;
            }
        }

        // Joysticks
        switch(state_){
            case INITIALIZATION:
                // Update boudning box size
                dwBB_ = (int)(-joy->axes[2] * 5.0);
                dhBB_ = (int)( joy->axes[3] * 5.0);
                break;

            case TRACKING:
                break;
        }
    }
    else
    {
        buttonDown = false;
    }
}

bool Tracker::initTracker()
{
    bool success = false;

    if((success = getBBFromUser(frame_)) == true) {
        cv::Mat gray(frame_.rows, frame_.cols, CV_8UC1);
        cv::cvtColor(frame_, gray, CV_BGR2GRAY);
        tld_->selectObject(gray, &bb_);
    }

    ROS_INFO_STREAM("selected roi: [" << bb_.x << ", " << bb_.y << ", " <<
                    bb_.x + bb_.width << ", " << bb_.y + bb_.height << "]");

    return success;
}

void Tracker::initTracker(cv::Rect bb)
{
    cv::Mat gray(frame_.rows, frame_.cols, CV_8UC1);
    cv::cvtColor(frame_, gray, CV_BGR2GRAY);
    tld_->selectObject(gray, &bb);

    ROS_INFO_STREAM("selected roi: [" << bb.x << ", " << bb.y << ", " <<
                    bb.x + bb.width << ", " << bb.y + bb.height << "]");
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

double Tracker::euclideanDistance(cv::Point p, cv::Point q)
{
    int dx = p.x - q.x;
    int dy = p.y - q.y;

    return std::sqrt(dx * dx + dy * dy);
}
