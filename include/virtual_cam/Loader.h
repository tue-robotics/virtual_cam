#ifndef _VIRTUAL_CAM_LOADER_H_
#define _VIRTUAL_CAM_LOADER_H_

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <fstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>

// Boost
#include <boost/foreach.hpp>

#include <cv_bridge/cv_bridge.h>

class ImageLoader {

public:

    ImageLoader();

    ImageLoader(const std::string& rgb_topic, const std::string& depth_topic, const std::string& info_topic, const std::string& frame_id);

    virtual ~ImageLoader();

    void load(const std::string& filename);

    cv::Mat getDepthImage() const;

    cv::Mat getRGBImage() const;

    void publish() const;

protected:

    ros::NodeHandle* nh_;

    std::string frame_id_;

    mutable sensor_msgs::Image rgb;
    mutable sensor_msgs::Image depth;
    mutable sensor_msgs::CameraInfo cam_info;

    ros::Publisher pub_rgb;
    ros::Publisher pub_depth;
    ros::Publisher pub_cam_info;

};

#endif
