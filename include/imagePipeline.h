#pragma once

#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <boxes.h>
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"
#include "LaserData.hpp"

class ImagePipeline
{
private:
    cv::Mat img;
    bool isValid;
    image_transport::Subscriber sub;

public:
    ImagePipeline(ros::NodeHandle &n);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    int getTemplateID(Boxes &boxes, LaserData laserData);
};
