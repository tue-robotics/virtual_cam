#include <ros/ros.h>

// OpenCv
//#include "cv.h"
//#include "highgui.h"
//#include "image_geometry/pinhole_camera_model.h"

// For transforming ROS/OpenCV images
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

// Messages
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "stereo_msgs/DisparityImage.h"

// Synchronize topics
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <fstream>

#include <rosbag/bag.h>


using namespace std;

// Typedef for synchronizer policy
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image> CamSyncPolicy;

string filename;

bool stored = false;

void ImageCallback(const sensor_msgs::ImageConstPtr& image_msg,
        const sensor_msgs::CameraInfoConstPtr& cam_info_msg,
        const sensor_msgs::ImageConstPtr& depth_image_msg) {

    rosbag::Bag bag_out;
    bag_out.open(filename, rosbag::bagmode::Write);
    bag_out.write("rgb", ros::Time::now(), *image_msg);
    bag_out.write("depth", ros::Time::now(), *depth_image_msg);
    bag_out.write("camera_info", ros::Time::now(), *cam_info_msg);

    stored = true;
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"NODE_NAME");
    ros::NodeHandle nh("~");

    if (argc != 2) {
        cout << "Usage: save FILENAME.yaml" << endl;
        return 1;
    }

    filename = argv[1];

    message_filters::Subscriber<sensor_msgs::Image> sub_img(nh, "/camera/rgb/image_rect_color", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_cam_info(nh, "/camera/rgb/camera_info", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_disp_img(nh, "/camera/depth_registered/image", 1);

    // register the subscribers using approximate synchronizer
    message_filters::Synchronizer<CamSyncPolicy> sync(CamSyncPolicy(25), sub_img, sub_cam_info, sub_disp_img);
    sync.registerCallback(boost::bind(&ImageCallback, _1, _2, _3));

    ros::Rate r(10);
    while(ros::ok() && !stored) {
        ros::spinOnce();
        r.sleep();
    }

    if (stored) {
        cout << "Success" << endl;
    }

}
