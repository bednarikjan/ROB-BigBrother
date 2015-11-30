// Project libraries
#include "crosshair.h"

Crosshair::Crosshair(int x, int y, int lineLnegth, int spaceLength, int vHeight, double vAngle, cv::Scalar colorCH, int bbLineWidth, cv::Scalar colorBB) :
    x_(x), y_(y), lineLen_(lineLnegth), spaceLen_(spaceLength),
    vHeight_(vHeight), vAngle_(vAngle), colorCH_(colorCH),
    bbW_(30), bbH_(30), bbLineWidth_(bbLineWidth), colorBB_(colorBB)
{
    vTanAngle_ = std::tan(vAngle_);

    // compute the line points of the crosshair and bounding box
    init();
}

Crosshair::~Crosshair()
{
    ;
}

void Crosshair::move(int x, int y)
{
    x_ += x;
    y_ += y;

    cv::Point2i update(x, y);

    // corsshair
    l1  += update;  l2 += update;
    t1  += update;  t2 += update;
    r1  += update;  r2 += update;
    b1  += update;  b2 += update;
    vl1 += update; vl2 += update;
    vr1 += update; vr2 += update;

    // bounding box
    bbTL_ += update; bbBR_ += update;
}

void Crosshair::draw(cv::Mat& frame)
{
    // crosshair
    cv::line(frame, l1, l2,   colorCH_, 1, 8);
    cv::line(frame, r1, r2,   colorCH_, 1, 8);
    cv::line(frame, t1, t2,   colorCH_, 1, 8);
    cv::line(frame, b1, b2,   colorCH_, 1, 8);
    cv::line(frame, vl1, vl2, colorCH_, 1, 8);
    cv::line(frame, vr1, vr2, colorCH_, 1, 8);

    // bounding box
    cv::rectangle(frame, bbTL_, bbBR_, colorBB_, bbLineWidth_);
}

void Crosshair::updateWidth(int dw)
{
    bbW_ = std::max(0, bbW_ + 2 * dw);
    init();
}

void Crosshair::updateHeight(int dh)
{
    bbH_ = std::max(0, bbH_ + 2 * dh);
    init();
}

void Crosshair::init()
{
    // crosshair
    // left
    l1 = cv::Point2i(x_ - spaceLen_, y_);
    l2 = cv::Point2i(x_ - spaceLen_ - lineLen_, y_);

    // right
    r1 = cv::Point2i(x_ + spaceLen_, y_);
    r2 = cv::Point2i(x_ + spaceLen_ + lineLen_, y_);

    // top
    t1 = cv::Point2i(x_, y_ - spaceLen_);
    t2 = cv::Point2i(x_, y_ - spaceLen_ - lineLen_);

    // bottom
    b1 = cv::Point2i(x_, y_ + spaceLen_);
    b2 = cv::Point2i(x_, y_ + spaceLen_ + lineLen_);

    // bottom upside-down V shape
    vl1 = cv::Point2i(x_ - (int)(vTanAngle_ * vHeight_), y_ + vHeight_);
    vl2 = cv::Point2i(x_, y_);
    vr1 = cv::Point2i(x_ + (int)(vTanAngle_ * vHeight_), y_ + vHeight_);
    vr2 = cv::Point2i(x_, y_);

    // bounding box
    bbTL_ = cv::Point2i(x_ - bbW_ / 2, y_ - bbH_ / 2);
    bbBR_ = cv::Point2i(x_ + bbW_ / 2, y_ + bbH_ / 2);
}
