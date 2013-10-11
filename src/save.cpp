#include <ros/ros.h>

// OpenCv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "image_geometry/pinhole_camera_model.h"

// For transforming ROS/OpenCV images
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

// Messages
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "stereo_msgs/DisparityImage.h"

// Synchronize topics
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// tf
#include <tf/transform_listener.h>

#include <fstream>

#include <rosbag/bag.h>

#include "virtual_cam/cheese.h"


using namespace std;

// Typedef for synchronizer policy
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image> CamSyncPolicy;

string filename;
bool stored = false;
std::string cam_frame_id;
rosbag::Bag bag_out;


//Stuff for the cheese service
ros::ServiceServer snapShotService;
sensor_msgs::ImageConstPtr rgb_image;


/// Take a picture service, saves as png
bool takeAPicturePNG(virtual_cam::cheeseRequest &req, virtual_cam::cheeseResponse& resp)
{
    string fn = req.fileName;
    cv::Mat picture = cv_bridge::toCvCopy(rgb_image)->image;

    cv::imwrite(fn, picture);
    resp.success = true;
    return true;
}


void ImageCallbackCheese(const sensor_msgs::ImageConstPtr image_msg,
        const sensor_msgs::CameraInfoConstPtr cam_info_msg,
        const sensor_msgs::ImageConstPtr depth_image_msg)
{
    rgb_image = image_msg;
}


void ImageCallback(const sensor_msgs::ImageConstPtr image_msg,
        const sensor_msgs::CameraInfoConstPtr cam_info_msg,
        const sensor_msgs::ImageConstPtr depth_image_msg) {

    bag_out.open(filename, rosbag::bagmode::Write);
    bag_out.write("rgb", ros::Time::now(), *image_msg);
    bag_out.write("depth", ros::Time::now(), *depth_image_msg);
    bag_out.write("camera_info", ros::Time::now(), *cam_info_msg);

    cam_frame_id = image_msg->header.frame_id;

    stored = true;
}

void showUsage() {
    cout << "Usage:" << endl
         << "    save SERVICE RGB_TOPIC DEPTH_TOPIC CAM_INFO_TOPIC" << endl
         << "or" << endl
         << "    save FILENAME RGB_TOPIC DEPTH_TOPIC CAM_INFO_TOPIC [ -tf PARENT_FRAME ]" << endl;
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"virtual_cam_saver");
    ros::NodeHandle nh("~");

    bool service_mode = (argc > 1 && std::string(argv[1]) == "SERVICE");

    if (argc < 5) {
        showUsage();
        return 1;
    }

    std::string rgb_topic = argv[2];
    std::string depth_topic = argv[3];
    std::string cam_info_topic = argv[4];

    std::string tf_parent_frame;
    if (argc == 7) {
        if (std::string(argv[5]) == "-tf") {
            tf_parent_frame = argv[6];
        } else {
            showUsage();
            return 1;
        }
    }

    message_filters::Subscriber<sensor_msgs::Image> sub_img(nh, rgb_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_disp_img(nh, depth_topic, 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_cam_info(nh, cam_info_topic, 1);

    // register the subscribers using approximate synchronizer
    message_filters::Synchronizer<CamSyncPolicy> sync(CamSyncPolicy(25), sub_img, sub_cam_info, sub_disp_img);

    if (service_mode) {
        ROS_INFO("Started in service mode...");
        sync.registerCallback(boost::bind(&ImageCallbackCheese, _1, _2, _3));
        snapShotService = nh.advertiseService<virtual_cam::cheeseRequest, virtual_cam::cheeseResponse>("cheese", boost::bind(&takeAPicturePNG, _1, _2));
        ros::Rate r(2);
        while(ros::ok()) {
            ros::spinOnce();
            r.sleep();
        }
    } else {

        ROS_INFO("Listening to topics ...");
        filename = argv[1];

        sync.registerCallback(boost::bind(&ImageCallback, _1, _2, _3));
        ros::Rate r(10);
        while(ros::ok() && !stored) {
            ros::spinOnce();
            r.sleep();
        }

        if (stored) {
            if (tf_parent_frame != "") {
                tf::TransformListener tf_listener;
                if (tf_listener.waitForTransform(tf_parent_frame, cam_frame_id, ros::Time(), ros::Duration(3))) {
                    tf::StampedTransform transform;
                    tf_listener.lookupTransform(tf_parent_frame, cam_frame_id, ros::Time(), transform);

                    geometry_msgs::TransformStamped transform_msg;
                    tf::transformStampedTFToMsg(transform, transform_msg);
                    bag_out.write("tf", ros::Time::now(), transform_msg);

                } else {
                    cerr << "No transform available between " << tf_parent_frame << " and " << cam_frame_id << std::endl;
                }
            }

            cout << "Success" << endl;
        }
    }

}
