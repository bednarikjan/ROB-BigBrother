#ifndef TRACKER_H
#define TRACKER_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <keyboard/Key.h>
#include <tf/transform_listener.h>

// ROS libraries
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenTLD
#include "TLD.h"

class Tracker {
public:
    static const std::string IMAGE_RAW_TOPIC;    //!< Name of the topic conveying camera stream.
    static const std::string KEY_DOWN_TOPIC;
    static const std::string KEY_UP_TOPIC;
    static const std::string WINDOW_NAME;

    //Initial bounding box
    static cv::Rect bb_;
    static bool drag_;
    static cv::Point p1_;
    static cv::Mat initImg_;

public:
    Tracker();
    ~Tracker();

    //! \brief update
    //!
    void update();

    //!
    //! \brief mouseHandler Mouse callback
    //! \param event
    //! \param x
    //! \param y
    //! \param flags
    //! \param param
    //!
    static void mouseHandler(int event, int x, int y, int flags, void *param);

private:
    ros::NodeHandle* nh_;
    ros::Subscriber onImgSub_;
    ros::Subscriber keyDownSub_;
    ros::Subscriber keyUpSub_;
    tf::TransformListener tfListener_;  //!< Listener finds transforms for azimuth and elevation joints.

//    cv_bridge::CvImagePtr frame_;
    cv::Mat frame_;
    bool receivedNewFrame_;

    int frameWidth_;
    int frameHeight_;
    bool firstFrame_;

    // OpenTLD
    tld::TLD *tld_;
    bool trackerInitialized_;

private:
    //! \brief onImage Callback function for incoming images
    //! \param img
    //!
    void onImage(const sensor_msgs::Image::ConstPtr img);

    //! \brief onKeyDown Keyboard key down callback
    //! \param key
    //!
    void onKeyDown(const keyboard::Key::ConstPtr key);

    //! \brief onKeyUp Keyboard key up callback
    //! \param key
    //!
    void onKeyUp(const keyboard::Key::ConstPtr key);

    //! \brief getBBFromUser Gets the bounding box drawn by the user.
    //! It is used to initializae the tracker.
    //! \return
    //!
    bool getBBFromUser(cv::Mat &frame);

};

#endif // #ifndef TRACKER_H
