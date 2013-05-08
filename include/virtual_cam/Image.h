#ifndef VIRTUAL_CAM_IMAGE_H_
#define VIRTUAL_CAM_IMAGE_H_

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <fstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>

// Boost
#include <boost/foreach.hpp>

#include <cv_bridge/cv_bridge.h>

class Image {

public:

    static Image loadFromFile(const std::string& filename);

    virtual ~Image();

    void saveToFile(const std::string& filename);

    cv::Mat getDepthImage() const;

    cv::Mat getRGBImage() const;

    cv::Mat getMaskImage() const;

    void setDepthImage(const cv::Mat& image);

    void setRGBImage(const cv::Mat& image);

    void setMaskImage(const cv::Mat& image);

protected:

    Image();

    sensor_msgs::Image rgb;
    sensor_msgs::Image depth;
    sensor_msgs::Image mask;
    sensor_msgs::CameraInfo cam_info;

};

#endif
