#include "virtual_cam/Loader.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc,argv,"virtual_cam_loader");
    ros::NodeHandle nh;

    if (argc != 7) {
        cout << "Usage: load FILENAME RGB_TOPIC DEPTH_TOPIC RGB_CAM_INFO_TOPIC DEPTH_CAM_INFO_TOPIC CAMERA_FRAME" << endl;
        return 1;
    }

    ImageLoader loader(argv[2], argv[3], argv[4], argv[6]);
    loader.setDepthCameraTopic(argv[5]);
    loader.load(argv[1]);

    ros::Rate r(10);
    while(ros::ok()) {
        loader.publish();
        r.sleep();
    }

}
