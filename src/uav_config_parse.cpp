#include "uav_config/read_config_drone.h"
#include "iostream"

int main(int argc, char **argv){

    // ConfigParser kun0 = ConfigParser("/home/nvidia/mulcam_ws/src/uav_config/extrinsic_intrinsic/body_four_camera.yaml"); //nvidia
    ConfigParser kun0 = ConfigParser("/home/hezijia/catkin_ws/src/uav_config/extrinsic_intrinsic/body_four_camera.yaml"); //laptop
    // kun0.print_all();

    return 0;
}