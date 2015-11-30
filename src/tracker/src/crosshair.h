#ifndef CROSSHAIR_H
#define CROSSHAIR_H

// C++
#include <cmath>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class Crosshair {
public:

    //! \brief Crosshair Contructor
    //!
    Crosshair(int x = 0, int y = 0, int lineLnegth = 15, int spaceLength = 5,
              int vHeight = 15, double vAngle = (M_PI / 6.0),
              cv::Scalar colorCH = cv::Scalar(0, 0, 255),
              int bbLineWidth = 3, cv::Scalar colorBB = cv::Scalar(255, 0, 0));

    //! \brief Crosshair Destructor
    //!
    ~Crosshair();

    //! \brief x x position getter.
    //! \return
    //!
    int x() { return x_; }

    //! \brief y y position getter.
    //! \return
    //!
    int y() { return y_; }

    //! \brief x x position setter
    //! \param xPos
    //!
    void x(int xPos) { x_ = xPos; init(); }

    //! \brief y y position seter
    //! \param yPos
    //!
    void y(int yPos) { y_ = yPos; init(); }

    //! \brief width Bounding box width getter.
    //! \return
    //!
    int width() { return bbW_; }

    //! \brief width Bounding box heoght getter.
    //! \return
    //!
    int height() { return bbH_; }

    //! \brief boundingBox Bounding box getter.
    //! \return
    //!
    cv::Rect boundingBox() { return cv::Rect(bbTL_, bbBR_); }

    //! \brief draw Draws a crosshair on the frame 'frame'
    //! \param frame The input frame to draw the crosshair on
    //!
    void draw(cv::Mat& frame);

    //! \brief move Move crosshait to relative position.
    //! \param x Relative x offset.
    //! \param y Relative y offset.
    //!
    void move(int x, int y);

    //! \brief updateWidth Updates the width of the bounding box by 'dw' pixels
    //! \param dw
    //!
    void updateWidth(int dw);

    //! \brief updateWidth Updates the height of the bounding box by 'dh' pixels
    //! \param dh
    //!
    void updateHeight(int dh);

    //! \brief init Computes the line points if the crosshair
    //!
    void init();

private:
    int x_;
    int y_;
    cv::Scalar colorCH_;
    cv::Scalar colorBB_;

    int lineLen_;
    int spaceLen_;
    int vHeight_;
    double vAngle_;
    double vTanAngle_;

    cv::Point2i l1, l2;
    cv::Point2i t1, t2;
    cv::Point2i r1, r2;
    cv::Point2i b1, b2;
    cv::Point2i vl1, vl2;
    cv::Point2i vr1, vr2;

    // Bounding box
    int bbW_;               //!< Bounding box width.
    int bbH_;              //!< Bounding box height.
    cv::Point2i bbTL_;      //!< Top left point of the bounding box.
    cv::Point2i bbBR_;      //!< Bottom right point of the bounding box.
    int bbLineWidth_;
};

#endif // #ifndef CROSSHAIR_H
