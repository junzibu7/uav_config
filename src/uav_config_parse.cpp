#include "uav_config/read_config_drone.h"
#include "iostream"
int main(int argc, char **argv){

    ConfigParser kun0 = ConfigParser("/home/zph/vins_ws/src/uav_config/extrinsic_intrinsic/kun0_Body_Cam_Imu_Marker_Landmarks_Extrinsics_Config.yaml");
    kun0.print_all();

    return 0;
}