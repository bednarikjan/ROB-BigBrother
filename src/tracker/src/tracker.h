#ifndef TRACKER_H
#define TRACKER_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <keyboard/Key.h>
#include <tf/transform_listener.h>

class Tracker {
public:
    static const std::string IMAGE_RAW_TOPIC;    //!< Name of the topic conveying camera stream.
    static const std::string KEY_DOWN_TOPIC;
    static const std::string KEY_UP_TOPIC;

public:
    Tracker();
    ~Tracker();

    //! \brief update
    //!
    void update();

private:
    ros::NodeHandle* nh_;
    ros::Subscriber onImgSub_;
    ros::Subscriber keyDownSub_;
    ros::Subscriber keyUpSub_;
    tf::TransformListener tfListener_;  //!< Listener finds transforms for azimuth and elevation joints.

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
};

#endif // #ifndef TRACKER_H
