#include "virtual_cam/Loader.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc,argv,"virtual_cam_loader");
    ros::NodeHandle nh;

    if (argc != 2) {
        cout << "Usage: load FILENAME" << endl;
        return 1;
    }

    ImageLoader loader("/camera/rgb/image_rect_color", "/camera/depth_registered/image", "/camera/rgb/camera_info", "/openni_rgb_optical_frame");
    loader.load(argv[1]);

    ros::Rate r(10);
    while(ros::ok()) {
        loader.publish();
        r.sleep();
    }

}
