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

    Image();

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

    sensor_msgs::Image rgb_msg_;
    sensor_msgs::Image depth_msg_;
    sensor_msgs::Image mask_msg_;

    cv::Mat rgb_image_;
    cv::Mat depth_image_;
    cv::Mat mask_image_;

    sensor_msgs::CameraInfo cam_info;

};

#endif
