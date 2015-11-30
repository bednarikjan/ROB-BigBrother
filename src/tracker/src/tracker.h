#ifndef TRACKER_H
#define TRACKER_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <keyboard/Key.h>
#include <tf/transform_listener.h>

// ROS libraries
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Project libraries
#include <msgs/ManipulatorMove.h>
#include "crosshair.h"

// OpenTLD
#include "TLD.h"

class Tracker {
public:
    static const std::string IMAGE_RAW_TOPIC;    //!< Name of the topic conveying camera stream.
    static const std::string JOY_TOPIC;
    static const std::string KEY_DOWN_TOPIC;
    static const std::string KEY_UP_TOPIC;
    static const std::string MANIP_MOVE_TOPIC;
    static const std::string WINDOW_NAME;

    static const int RADIUS_CENTER;
    static const int RADIUS_INERTIA;

    enum eState {
        INITIALIZATION,
        TRACKING,
    };

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
    ros::Subscriber joySub_;            //!< Joystick event subscription.
    ros::Publisher manipMovePub_;       //!< Publisher of the messages controlling the motion of the manipulator
    tf::TransformListener tfListener_;  //!< Listener finds transforms for azimuth and elevation joints.

    std::string master_ns_;

    eState state_;

//    cv_bridge::CvImagePtr frame_;
    cv::Mat frame_;
    bool receivedNewFrame_;

    int frameWidth_;
    int frameHeight_;
    cv::Point frameCenter_;
    bool firstFrame_;

    // OpenTLD
    tld::TLD *tld_;
    bool trackerInitialized_;   

    bool cameraInMotion_;
    double maxAzimuthSpeed_;
    double maxElevationSpeed_;

    // Crosshair and bounding box
    Crosshair ch_;
    int dwBB_;
    int dhBB_;

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

    //! \brief onJoy Joystick callback
    //! \param joy
    //!
    void onJoy(const sensor_msgs::Joy::ConstPtr& joy);

    //! \brief getBBFromUser Gets the bounding box drawn by the user.
    //! It is used to initializae the tracker.
    //! \return
    //!
    bool getBBFromUser(cv::Mat &frame);

    //! \brief initTracker
    //! \return
    //!
    bool initTracker();   

    //! \brief initTracker
    //! \return
    //!
    void initTracker(cv::Rect bb);

    //! \brief euclideanDistance Computes the Euclidean distance between two points
    //! \param from
    //! \param to
    //! \return
    //!
    double euclideanDistance(cv::Point p, cv::Point q);

    //! \brief changeState Set program state.
    //! \param state
    //!
    void changeState(eState state) { state_ = state; }

    //! \brief move Sne dthe ManipulatorMove message to the manipualtor with the
    //! given speeds 'azi' and 'ele' (given by mrads).
    //! \param azi
    //! \param ele
    //!
    void move(double azi, double ele);

//    void moveCamera();
};

#endif // #ifndef TRACKER_H
