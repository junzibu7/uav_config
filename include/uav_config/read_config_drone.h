#ifndef READ_CONFIG_DRONE_H_
#define READ_CONFIG_DRONE_H_


#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <vector>
#include <string>
#include <cstring>
#include <algorithm>

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Eigen>

//! wzy add this two function to avoid bug >>>>>>>>>>
// YAML::BadConversion::~BadConversion()
// {
// }
    
// void  YAML::detail::node_data::convert_to_map(const shared_memory_holder& pMemory)
// {
// }
//! wzy add this two function to avoid bug <<<<<<<<<<

static bool GetFileNames(const std::string &path, std::vector<std::string> &filenames) {
  DIR *pDir;
  struct dirent *ptr;
  if (!(pDir = opendir(path.c_str()))) {
    std::cerr << "Current folder doesn't exist!" << std::endl;
    return false;
  }
  while ((ptr = readdir(pDir)) != nullptr) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
      filenames.push_back(path + "/" + ptr->d_name);
    }
  }
  closedir(pDir);
  std::sort(filenames.begin(), filenames.end());
  return true;
}

static bool FileExists(const std::string& file) {
  struct stat file_status{};
  if (stat(file.c_str(), &file_status) == 0 &&
      (file_status.st_mode & S_IFREG)) {
    return true;
  }
  return false;
}

static void ConcatenateFolderAndFileNameBase(
        const std::string& folder, const std::string& file_name,
        std::string* path) {
  *path = folder;
  if (path->back() != '/') {
    *path += '/';
  }
  *path = *path + file_name;
}

static std::string ConcatenateFolderAndFileName(
        const std::string& folder, const std::string& file_name) {
  std::string path;
  ConcatenateFolderAndFileNameBase(folder, file_name, &path);
  return path;
}

struct ColorCamera{
   std::string name;
   int image_width{};
   int image_height{};
   double fx{};
   double fy{};
   double cx{};
   double cy{};
   std::string distortion_model;
   double D[5];
};

struct InfracameraA{
    int image_width{};
    int image_height{};
    double fx{};
    double fy{};
    double cx{};
    double cy{};
    std::string distortion_model;
    double D[5];
    Eigen::Matrix4d T_camColor_camIR1;
};

struct InfracameraB{
    int image_width{};
    int image_height{};
    double fx{};
    double fy{};
    double cx{};
    double cy{};
    std::string distortion_model;
    double D[5];
    Eigen::Matrix4d T_camIR1_camIR2;
};

struct InfraCameras{
    std::string name;
    InfracameraA cam1;
    InfracameraB cam2;
};

struct RealsenseCamera{
    std::string serial_no;
    ColorCamera color_camera;
    InfraCameras ir_camera;
};

struct Imu{
    std::string imu_name;
};

struct Marker{
    std::string marker_type;
    Eigen::Matrix4d T_base_marker;
};

struct IRLandmark{
    std::string layout_name;
    int number;
    Eigen::Matrix3Xd layout;
    Eigen::Matrix4d T_drone_IRLandmark;
};

struct ConfigParser{
    std::string uav_name;
    RealsenseCamera cameraA;
    RealsenseCamera cameraB;    
    RealsenseCamera cameraC;
    RealsenseCamera cameraD;
    Eigen::Matrix4d T_imu_t265;
    Eigen::Matrix4d T_cam_image;
    Imu imu;
    Marker marker;
    IRLandmark ir_landmark;
    Eigen::Matrix3d Vicon_correction;

    std::vector<double> v_temp;

    ConfigParser(const std::string& config_file){
        // std::cout << "Config file is " << config_file << std::endl;
        if(!FileExists(config_file)){
            // std::cerr << "Config file " << config_file << " doesn't exist." << std::endl;
            return;
        }
        YAML::Node file_node = YAML::LoadFile(config_file);

        uav_name = file_node["uav_name"].as<std::string>();
        // std::cout << "Start parse cameraA >>>> " << std::endl;
        ///================= cameraA parse =================///
        YAML::Node cameraA_node = file_node["cameraA"];
        cameraA.serial_no = cameraA_node["serial_no"].as<std::string>();
        ///>>>>>>>>>>>>>>> color camera
        YAML::Node color_cameraA_node = cameraA_node["color_camera"];
        cameraA.color_camera.name = color_cameraA_node["name"].as<std::string>();
        cameraA.color_camera.image_width = color_cameraA_node["image_width"].as<int>();
        cameraA.color_camera.image_height = color_cameraA_node["image_height"].as<int>();
        cameraA.color_camera.fx = color_cameraA_node["fx"].as<double>();
        cameraA.color_camera.fy = color_cameraA_node["fy"].as<double>();
        cameraA.color_camera.cx = color_cameraA_node["cx"].as<double>();
        cameraA.color_camera.cy = color_cameraA_node["cy"].as<double>();
        cameraA.color_camera.distortion_model = color_cameraA_node["distortion_model"].as<std::string>();
        for (int i = 0; i < color_cameraA_node["D"].size(); ++i) {
          cameraA.color_camera.D[i] = color_cameraA_node["D"][i].as<double>();
        }
        ///<<<<<<<<<<<<<<< color camera


        ///>>>>>>>>>>>>>>> ir camera
        YAML::Node ir_cameraA_node = cameraA_node["ir_camera"];
        cameraA.ir_camera.name = ir_cameraA_node["name"].as<std::string>();
        ///>>>>>>>>>>>>>>> IR 1 camera
        YAML::Node IR_1_cameraA_node = ir_cameraA_node["IR_1"];
        cameraA.ir_camera.cam1.image_width = IR_1_cameraA_node["image_width"].as<int>();
        cameraA.ir_camera.cam1.image_height = IR_1_cameraA_node["image_height"].as<int>();
        cameraA.ir_camera.cam1.fx = IR_1_cameraA_node["fx"].as<double>();
        cameraA.ir_camera.cam1.fy = IR_1_cameraA_node["fy"].as<double>();
        cameraA.ir_camera.cam1.cx = IR_1_cameraA_node["cx"].as<double>();
        cameraA.ir_camera.cam1.cy = IR_1_cameraA_node["cy"].as<double>();
        cameraA.ir_camera.cam1.distortion_model = IR_1_cameraA_node["distortion_model"].as<std::string>();
        for (int i = 0; i < IR_1_cameraA_node["D"].size(); ++i) {
          cameraA.ir_camera.cam1.D[i] = IR_1_cameraA_node["D"][i].as<double>();
        }
        v_temp.clear();
        for (int i = 0; i < IR_1_cameraA_node["T_camColor_camIR1"].size(); ++i) {
            v_temp.emplace_back(IR_1_cameraA_node["T_camColor_camIR1"][i].as<double>());
        }
        convert_Eigen_4d(cameraA.ir_camera.cam1.T_camColor_camIR1, v_temp);
        ///<<<<<<<<<<<<<<< IR 1 camera
        ///>>>>>>>>>>>>>>> IR 2 camera
        YAML::Node IR_2_cameraA_node = ir_cameraA_node["IR_2"];
        cameraA.ir_camera.cam2.image_width = IR_2_cameraA_node["image_width"].as<int>();
        cameraA.ir_camera.cam2.image_height = IR_2_cameraA_node["image_height"].as<int>();
        cameraA.ir_camera.cam2.fx = IR_2_cameraA_node["fx"].as<double>();
        cameraA.ir_camera.cam2.fy = IR_2_cameraA_node["fy"].as<double>();
        cameraA.ir_camera.cam2.cx = IR_2_cameraA_node["cx"].as<double>();
        cameraA.ir_camera.cam2.cy = IR_2_cameraA_node["cy"].as<double>();
        cameraA.ir_camera.cam2.distortion_model = IR_2_cameraA_node["distortion_model"].as<std::string>();
        for (int i = 0; i < IR_2_cameraA_node["D"].size(); ++i) {
          cameraA.ir_camera.cam2.D[i] = IR_2_cameraA_node["D"][i].as<double>();
        }
        v_temp.clear();
        for (int i = 0; i < IR_2_cameraA_node["T_camIR1_camIR2"].size(); ++i) {
            v_temp.emplace_back(IR_2_cameraA_node["T_camIR1_camIR2"][i].as<double>());
        }
        convert_Eigen_4d(cameraA.ir_camera.cam2.T_camIR1_camIR2, v_temp);
        ///<<<<<<<<<<<<<<< IR 2 camera
        ///<<<<<<<<<<<<<<< ir camera
        // std::cout << "Finish parse cameraA <<<< " << std::endl;

        // std::cout << "Start parse cameraB >>>> " << std::endl;
        ///================= cameraB parse =================///
        YAML::Node cameraB_node = file_node["cameraB"];
        cameraB.serial_no = cameraB_node["serial_no"].as<std::string>();
        ///>>>>>>>>>>>>>>> color camera
        YAML::Node color_cameraB_node = cameraB_node["color_camera"];
        cameraB.color_camera.name = color_cameraB_node["name"].as<std::string>();
        cameraB.color_camera.image_width = color_cameraB_node["image_width"].as<int>();
        cameraB.color_camera.image_height = color_cameraB_node["image_height"].as<int>();
        cameraB.color_camera.fx = color_cameraB_node["fx"].as<double>();
        cameraB.color_camera.fy = color_cameraB_node["fy"].as<double>();
        cameraB.color_camera.cx = color_cameraB_node["cx"].as<double>();
        cameraB.color_camera.cy = color_cameraB_node["cy"].as<double>();
        cameraB.color_camera.distortion_model = color_cameraB_node["distortion_model"].as<std::string>();
        for (int i = 0; i < color_cameraB_node["D"].size(); ++i) {
          cameraB.color_camera.D[i] = color_cameraB_node["D"][i].as<double>();
        }
        ///<<<<<<<<<<<<<<< color camera


        ///>>>>>>>>>>>>>>> ir camera
        YAML::Node ir_cameraB_node = cameraB_node["ir_camera"];
        cameraB.ir_camera.name = ir_cameraB_node["name"].as<std::string>();
        ///>>>>>>>>>>>>>>> IR 1 camera
        YAML::Node IR_1_cameraB_node = ir_cameraB_node["IR_1"];
        cameraB.ir_camera.cam1.image_width = IR_1_cameraB_node["image_width"].as<int>();
        cameraB.ir_camera.cam1.image_height = IR_1_cameraB_node["image_height"].as<int>();
        cameraB.ir_camera.cam1.fx = IR_1_cameraB_node["fx"].as<double>();
        cameraB.ir_camera.cam1.fy = IR_1_cameraB_node["fy"].as<double>();
        cameraB.ir_camera.cam1.cx = IR_1_cameraB_node["cx"].as<double>();
        cameraB.ir_camera.cam1.cy = IR_1_cameraB_node["cy"].as<double>();
        cameraB.ir_camera.cam1.distortion_model = IR_1_cameraB_node["distortion_model"].as<std::string>();
        for (int i = 0; i < IR_1_cameraB_node["D"].size(); ++i) {
          cameraB.ir_camera.cam1.D[i] = IR_1_cameraB_node["D"][i].as<double>();
        }
        v_temp.clear();
        for (int i = 0; i < IR_1_cameraB_node["T_camColor_camIR1"].size(); ++i) {
            v_temp.emplace_back(IR_1_cameraB_node["T_camColor_camIR1"][i].as<double>());
        }
        convert_Eigen_4d(cameraB.ir_camera.cam1.T_camColor_camIR1, v_temp);
        ///<<<<<<<<<<<<<<< IR 1 camera
        ///>>>>>>>>>>>>>>> IR 2 camera
        YAML::Node IR_2_cameraB_node = ir_cameraB_node["IR_2"];
        cameraB.ir_camera.cam2.image_width = IR_2_cameraB_node["image_width"].as<int>();
        cameraB.ir_camera.cam2.image_height = IR_2_cameraB_node["image_height"].as<int>();
        cameraB.ir_camera.cam2.fx = IR_2_cameraB_node["fx"].as<double>();
        cameraB.ir_camera.cam2.fy = IR_2_cameraB_node["fy"].as<double>();
        cameraB.ir_camera.cam2.cx = IR_2_cameraB_node["cx"].as<double>();
        cameraB.ir_camera.cam2.cy = IR_2_cameraB_node["cy"].as<double>();
        cameraB.ir_camera.cam2.distortion_model = IR_2_cameraB_node["distortion_model"].as<std::string>();
        for (int i = 0; i < IR_2_cameraB_node["D"].size(); ++i) {
          cameraB.ir_camera.cam2.D[i] = IR_2_cameraB_node["D"][i].as<double>();
        }
        v_temp.clear();
        for (int i = 0; i < IR_2_cameraB_node["T_camIR1_camIR2"].size(); ++i) {
            v_temp.emplace_back(IR_2_cameraB_node["T_camIR1_camIR2"][i].as<double>());
        }
        convert_Eigen_4d(cameraB.ir_camera.cam2.T_camIR1_camIR2, v_temp);
        ///<<<<<<<<<<<<<<< IR 2 camera
        ///<<<<<<<<<<<<<<< ir camera
        // std::cout << "Finish parse cameraB <<<< " << std::endl;

        // std::cout << "Start parse cameraC >>>> " << std::endl;
        ///================= cameraC parse =================///
        YAML::Node cameraC_node = file_node["cameraC"];
        cameraC.serial_no = cameraC_node["serial_no"].as<std::string>();
        ///>>>>>>>>>>>>>>> color camera
        YAML::Node color_cameraC_node = cameraC_node["color_camera"];
        cameraC.color_camera.name = color_cameraC_node["name"].as<std::string>();
        cameraC.color_camera.image_width = color_cameraC_node["image_width"].as<int>();
        cameraC.color_camera.image_height = color_cameraC_node["image_height"].as<int>();
        cameraC.color_camera.fx = color_cameraC_node["fx"].as<double>();
        cameraC.color_camera.fy = color_cameraC_node["fy"].as<double>();
        cameraC.color_camera.cx = color_cameraC_node["cx"].as<double>();
        cameraC.color_camera.cy = color_cameraC_node["cy"].as<double>();
        cameraC.color_camera.distortion_model = color_cameraC_node["distortion_model"].as<std::string>();
        for (int i = 0; i < color_cameraC_node["D"].size(); ++i) {
          cameraC.color_camera.D[i] = color_cameraC_node["D"][i].as<double>();
        }
        ///<<<<<<<<<<<<<<< color camera


        ///>>>>>>>>>>>>>>> ir camera
        YAML::Node ir_cameraC_node = cameraC_node["ir_camera"];
        cameraC.ir_camera.name = ir_cameraC_node["name"].as<std::string>();
        ///>>>>>>>>>>>>>>> IR 1 camera
        YAML::Node IR_1_cameraC_node = ir_cameraC_node["IR_1"];
        cameraC.ir_camera.cam1.image_width = IR_1_cameraC_node["image_width"].as<int>();
        cameraC.ir_camera.cam1.image_height = IR_1_cameraC_node["image_height"].as<int>();
        cameraC.ir_camera.cam1.fx = IR_1_cameraC_node["fx"].as<double>();
        cameraC.ir_camera.cam1.fy = IR_1_cameraC_node["fy"].as<double>();
        cameraC.ir_camera.cam1.cx = IR_1_cameraC_node["cx"].as<double>();
        cameraC.ir_camera.cam1.cy = IR_1_cameraC_node["cy"].as<double>();
        cameraC.ir_camera.cam1.distortion_model = IR_1_cameraC_node["distortion_model"].as<std::string>();
        for (int i = 0; i < IR_1_cameraC_node["D"].size(); ++i) {
          cameraC.ir_camera.cam1.D[i] = IR_1_cameraC_node["D"][i].as<double>();
        }
        v_temp.clear();
        for (int i = 0; i < IR_1_cameraC_node["T_camColor_camIR1"].size(); ++i) {
            v_temp.emplace_back(IR_1_cameraC_node["T_camColor_camIR1"][i].as<double>());
        }
        convert_Eigen_4d(cameraC.ir_camera.cam1.T_camColor_camIR1, v_temp);
        ///<<<<<<<<<<<<<<< IR 1 camera
        ///>>>>>>>>>>>>>>> IR 2 camera
        YAML::Node IR_2_cameraC_node = ir_cameraC_node["IR_2"];
        cameraC.ir_camera.cam2.image_width = IR_2_cameraC_node["image_width"].as<int>();
        cameraC.ir_camera.cam2.image_height = IR_2_cameraC_node["image_height"].as<int>();
        cameraC.ir_camera.cam2.fx = IR_2_cameraC_node["fx"].as<double>();
        cameraC.ir_camera.cam2.fy = IR_2_cameraC_node["fy"].as<double>();
        cameraC.ir_camera.cam2.cx = IR_2_cameraC_node["cx"].as<double>();
        cameraC.ir_camera.cam2.cy = IR_2_cameraC_node["cy"].as<double>();
        cameraC.ir_camera.cam2.distortion_model = IR_2_cameraC_node["distortion_model"].as<std::string>();
        for (int i = 0; i < IR_2_cameraC_node["D"].size(); ++i) {
          cameraC.ir_camera.cam2.D[i] = IR_2_cameraC_node["D"][i].as<double>();
        }
        v_temp.clear();
        for (int i = 0; i < IR_2_cameraC_node["T_camIR1_camIR2"].size(); ++i) {
            v_temp.emplace_back(IR_2_cameraC_node["T_camIR1_camIR2"][i].as<double>());
        }
        convert_Eigen_4d(cameraC.ir_camera.cam2.T_camIR1_camIR2, v_temp);
        ///<<<<<<<<<<<<<<< IR 2 camera
        ///<<<<<<<<<<<<<<< ir camera
        // std::cout << "Finish parse cameraC <<<< " << std::endl;


        // std::cout << "Start parse cameraD >>>> " << std::endl;
        ///================= cameraD parse =================///
        YAML::Node cameraD_node = file_node["cameraD"];
        cameraD.serial_no = cameraD_node["serial_no"].as<std::string>();
        ///>>>>>>>>>>>>>>> color camera
        YAML::Node color_cameraD_node = cameraD_node["color_camera"];
        cameraD.color_camera.name = color_cameraD_node["name"].as<std::string>();
        cameraD.color_camera.image_width = color_cameraD_node["image_width"].as<int>();
        cameraD.color_camera.image_height = color_cameraD_node["image_height"].as<int>();
        cameraD.color_camera.fx = color_cameraD_node["fx"].as<double>();
        cameraD.color_camera.fy = color_cameraD_node["fy"].as<double>();
        cameraD.color_camera.cx = color_cameraD_node["cx"].as<double>();
        cameraD.color_camera.cy = color_cameraD_node["cy"].as<double>();
        cameraD.color_camera.distortion_model = color_cameraD_node["distortion_model"].as<std::string>();
        for (int i = 0; i < color_cameraD_node["D"].size(); ++i) {
          cameraD.color_camera.D[i] = color_cameraD_node["D"][i].as<double>();
        }
        ///<<<<<<<<<<<<<<< color camera


        ///>>>>>>>>>>>>>>> ir camera
        YAML::Node ir_cameraD_node = cameraD_node["ir_camera"];
        cameraD.ir_camera.name = ir_cameraD_node["name"].as<std::string>();
        ///>>>>>>>>>>>>>>> IR 1 camera
        YAML::Node IR_1_cameraD_node = ir_cameraD_node["IR_1"];
        cameraD.ir_camera.cam1.image_width = IR_1_cameraD_node["image_width"].as<int>();
        cameraD.ir_camera.cam1.image_height = IR_1_cameraD_node["image_height"].as<int>();
        cameraD.ir_camera.cam1.fx = IR_1_cameraD_node["fx"].as<double>();
        cameraD.ir_camera.cam1.fy = IR_1_cameraD_node["fy"].as<double>();
        cameraD.ir_camera.cam1.cx = IR_1_cameraD_node["cx"].as<double>();
        cameraD.ir_camera.cam1.cy = IR_1_cameraD_node["cy"].as<double>();
        cameraD.ir_camera.cam1.distortion_model = IR_1_cameraD_node["distortion_model"].as<std::string>();
        for (int i = 0; i < IR_1_cameraD_node["D"].size(); ++i) {
          cameraD.ir_camera.cam1.D[i] = IR_1_cameraD_node["D"][i].as<double>();
        }
        v_temp.clear();
        for (int i = 0; i < IR_1_cameraD_node["T_camColor_camIR1"].size(); ++i) {
            v_temp.emplace_back(IR_1_cameraD_node["T_camColor_camIR1"][i].as<double>());
        }
        convert_Eigen_4d(cameraD.ir_camera.cam1.T_camColor_camIR1, v_temp);
        ///<<<<<<<<<<<<<<< IR 1 camera
        ///>>>>>>>>>>>>>>> IR 2 camera
        YAML::Node IR_2_cameraD_node = ir_cameraD_node["IR_2"];
        cameraD.ir_camera.cam2.image_width = IR_2_cameraD_node["image_width"].as<int>();
        cameraD.ir_camera.cam2.image_height = IR_2_cameraD_node["image_height"].as<int>();
        cameraD.ir_camera.cam2.fx = IR_2_cameraD_node["fx"].as<double>();
        cameraD.ir_camera.cam2.fy = IR_2_cameraD_node["fy"].as<double>();
        cameraD.ir_camera.cam2.cx = IR_2_cameraD_node["cx"].as<double>();
        cameraD.ir_camera.cam2.cy = IR_2_cameraD_node["cy"].as<double>();
        cameraD.ir_camera.cam2.distortion_model = IR_2_cameraD_node["distortion_model"].as<std::string>();
        for (int i = 0; i < IR_2_cameraD_node["D"].size(); ++i) {
          cameraD.ir_camera.cam2.D[i] = IR_2_cameraD_node["D"][i].as<double>();
        }
        v_temp.clear();
        for (int i = 0; i < IR_2_cameraD_node["T_camIR1_camIR2"].size(); ++i) {
            v_temp.emplace_back(IR_2_cameraD_node["T_camIR1_camIR2"][i].as<double>());
        }
        convert_Eigen_4d(cameraD.ir_camera.cam2.T_camIR1_camIR2, v_temp);
        ///<<<<<<<<<<<<<<< IR 2 camera
        ///<<<<<<<<<<<<<<< ir camera
        // std::cout << "Finish parse cameraD <<<< " << std::endl;

        // std::cout << "Start parse T_cam_image >>>> " << std::endl;
        ///================= T_cam_image =================///
        v_temp.clear();
        for (int i = 0; i < file_node["T_cam_image"].size(); ++i) {
            v_temp.emplace_back(file_node["T_cam_image"][i].as<double>());
        }
        convert_Eigen_4d(T_cam_image, v_temp);
        ///================= T_cam_image =================///
        // std::cout << "Finish parse T_cam_image <<<< " << std::endl;

        // std::cout << "Start parse T_imu_t265 >>>> " << std::endl;
        ///================= T_imu_t265 =================///
        v_temp.clear();
        for (int i = 0; i < file_node["T_imu_t265"].size(); ++i) {
            v_temp.emplace_back(file_node["T_imu_t265"][i].as<double>());
        }
        convert_Eigen_4d(T_imu_t265, v_temp);
        ///================= T_imu_t265 =================///
        // std::cout << "Finish parse T_imu_t265 <<<< " << std::endl;

        // std::cout << "Start parse Vicon_correction >>>> " << std::endl;
        ///================= Vicon_correction =================///
        v_temp.clear();
        for (int i = 0; i < file_node["Vicon_correction"].size(); ++i) {
            v_temp.emplace_back(file_node["Vicon_correction"][i].as<double>());
        }
        convert_Eigen_3d(Vicon_correction, v_temp);
        ///================= Vicon_correction =================///
        // std::cout << "Finish parse Vicon_correction <<<< " << std::endl;


        // std::cout << "Start parse imu >>>> " << std::endl;
        ///================= imu parse =================///
        YAML::Node imu_node = file_node["imu"];
        imu.imu_name = imu_node["imu_name"].as<std::string>();
        // std::cout << "Finish parse imu <<<< " << std::endl;

        // std::cout << "Start parse marker >>>> " << std::endl;
        ///================= marker parse =================///
        YAML::Node marker_node = file_node["Marker"];
        marker.marker_type = marker_node["marker_type"].as<std::string>();
        v_temp.clear();
        for (int i = 0; i < marker_node["T_base_marker"].size(); ++i) {
            v_temp.emplace_back(marker_node["T_base_marker"][i].as<double>());
        }
        convert_Eigen_4d(marker.T_base_marker, v_temp);
        // std::cout << "Finish parse marker <<<< " << std::endl;

        // std::cout << "Start parse IRLandmarker >>>> " << std::endl;
        ///================= IRlandmarker parse =================///
        YAML::Node IRLandmarker_node = file_node["IRLandmark"];
        ir_landmark.layout_name = IRLandmarker_node["layout_name"].as<std::string>();
        // std::cout << "Start layout >>>> " << std::endl;
        v_temp.clear();
        for (int i = 0; i < IRLandmarker_node["layout"].size(); ++i) {
            v_temp.emplace_back(IRLandmarker_node["layout"][i].as<double>());
        }
        ir_landmark.number = IRLandmarker_node["number"].as<int>();
        //转为3行xN列的Eigen::Matrix3Xd
        ir_landmark.layout.resize(3, ir_landmark.number);
        convert_3xN(ir_landmark.layout, v_temp);
        // std::cout << "Start T_marker_IRLandmark >>>> " << std::endl;
        v_temp.clear();
        for (int i = 0; i < IRLandmarker_node["T_drone_IRLandmark"].size(); ++i) {
            v_temp.emplace_back(IRLandmarker_node["T_drone_IRLandmark"][i].as<double>());
        }
        convert_Eigen_4d(ir_landmark.T_drone_IRLandmark, v_temp);
        // std::cout << "Finish parse IRLandmarker <<<< " << std::endl;
    }


    void convert_Eigen_4d(Eigen::Matrix4d &m, std::vector<double> &vec){
        int k = 0;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m(i,j) = vec[k];
                k++;
            }
        }
    }

    void convert_Eigen_3d(Eigen::Matrix3d &m, std::vector<double> &vec){
        int k = 0;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m(i,j) = vec[k];
                k++;
            }
        }
    }

    void convert_3xN(Eigen::Matrix3Xd &m, std::vector<double> &vec){
        int k = 0;
        for (int i = 0; i < m.cols(); ++i) {
            for (int j = 0; j < 3; ++j) {
                m(j,i) = vec[k];
                k++;
            }
        }
    }

    // void print_all(){
    //     print_config_cameraA();
    //     print_config_cameraB();
    //     print_config_cameraC();
    //     print_config_cameraD();
    //     print_config_cam2image();
    //     print_config_imu();
    //     print_config_marker();
    //     print_config_IRLandmark();
    // }

    // void print_config_cameraA(){
    //     std::cout << "///============== cameraA =================///" << std::endl;
    //     std::cout << "serial_no: " << cameraA.serial_no.c_str() << std::endl;
    //     std::cout << "///>>>>>>>>>>>>>> color camera >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "fx: " << cameraA.color_camera.fx << "\tfy: " << cameraA.color_camera.fy <<
    //     "\tcx: " << cameraA.color_camera.cx << "\tcy: " << cameraA.color_camera.cy << std::endl;
    //     std::cout << std::endl;

    //     std::cout << "///>>>>>>>>>>>>>> ir camera >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "///>>>>>>>>>>>>>> ir camera 1 >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "fx: " << cameraA.ir_camera.cam1.fx << "\tfy: " << cameraA.ir_camera.cam1.fy <<
    //               "\tcx: " << cameraA.ir_camera.cam1.cx << "\tcy: " << cameraA.ir_camera.cam1.cy << std::endl;
    //     std::cout << "T_camColor_camIR1: " << std::endl << cameraA.ir_camera.cam1.T_camColor_camIR1.matrix() << std::endl;
    //     std::cout << std::endl;

    //     std::cout << "///>>>>>>>>>>>>>> ir camera 2 >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "fx: " << cameraA.ir_camera.cam2.fx << "\tfy: " << cameraA.ir_camera.cam2.fy <<
    //               "\tcx: " << cameraA.ir_camera.cam2.cx << "\tcy: " << cameraA.ir_camera.cam2.cy << std::endl;
    //     std::cout << "T_camIR1_camIR2: " << std::endl << cameraA.ir_camera.cam2.T_camIR1_camIR2.matrix() << std::endl;
    //     std::cout << std::endl;
    // }

    // void print_config_cameraB(){
    //     std::cout << "///============== cameraB =================///" << std::endl;
    //     std::cout << "serial_no: " << cameraB.serial_no.c_str() << std::endl;
    //     std::cout << "///>>>>>>>>>>>>>> color camera >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "fx: " << cameraB.color_camera.fx << "\tfy: " << cameraB.color_camera.fy <<
    //     "\tcx: " << cameraB.color_camera.cx << "\tcy: " << cameraB.color_camera.cy << std::endl;
    //     std::cout << std::endl;

    //     std::cout << "///>>>>>>>>>>>>>> ir camera >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "///>>>>>>>>>>>>>> ir camera 1 >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "fx: " << cameraB.ir_camera.cam1.fx << "\tfy: " << cameraB.ir_camera.cam1.fy <<
    //               "\tcx: " << cameraB.ir_camera.cam1.cx << "\tcy: " << cameraB.ir_camera.cam1.cy << std::endl;
    //     std::cout << "T_camColor_camIR1: " << std::endl << cameraB.ir_camera.cam1.T_camColor_camIR1.matrix() << std::endl;
    //     std::cout << std::endl;

    //     std::cout << "///>>>>>>>>>>>>>> ir camera 2 >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "fx: " << cameraB.ir_camera.cam2.fx << "\tfy: " << cameraB.ir_camera.cam2.fy <<
    //               "\tcx: " << cameraB.ir_camera.cam2.cx << "\tcy: " << cameraB.ir_camera.cam2.cy << std::endl;
    //     std::cout << "T_camIR1_camIR2: " << std::endl << cameraB.ir_camera.cam2.T_camIR1_camIR2.matrix() << std::endl;
    //     std::cout << std::endl;
    // }

    // void print_config_cameraC(){
    //     std::cout << "///============== cameraC =================///" << std::endl;
    //     std::cout << "serial_no: " << cameraC.serial_no.c_str() << std::endl;
    //     std::cout << "///>>>>>>>>>>>>>> color camera >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "fx: " << cameraC.color_camera.fx << "\tfy: " << cameraC.color_camera.fy <<
    //     "\tcx: " << cameraC.color_camera.cx << "\tcy: " << cameraC.color_camera.cy << std::endl;
    //     std::cout << std::endl;

    //     std::cout << "///>>>>>>>>>>>>>> ir camera >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "///>>>>>>>>>>>>>> ir camera 1 >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "fx: " << cameraC.ir_camera.cam1.fx << "\tfy: " << cameraC.ir_camera.cam1.fy <<
    //               "\tcx: " << cameraC.ir_camera.cam1.cx << "\tcy: " << cameraC.ir_camera.cam1.cy << std::endl;
    //     std::cout << "T_camColor_camIR1: " << std::endl << cameraC.ir_camera.cam1.T_camColor_camIR1.matrix() << std::endl;
    //     std::cout << std::endl;

    //     std::cout << "///>>>>>>>>>>>>>> ir camera 2 >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "fx: " << cameraC.ir_camera.cam2.fx << "\tfy: " << cameraC.ir_camera.cam2.fy <<
    //               "\tcx: " << cameraC.ir_camera.cam2.cx << "\tcy: " << cameraC.ir_camera.cam2.cy << std::endl;
    //     std::cout << "T_camIR1_camIR2: " << std::endl << cameraC.ir_camera.cam2.T_camIR1_camIR2.matrix() << std::endl;
    //     std::cout << std::endl;
    // }

    // void print_config_cameraD(){
    //     std::cout << "///============== cameraD =================///" << std::endl;
    //     std::cout << "serial_no: " << cameraD.serial_no.c_str() << std::endl;
    //     std::cout << "///>>>>>>>>>>>>>> color camera >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "fx: " << cameraD.color_camera.fx << "\tfy: " << cameraD.color_camera.fy <<
    //     "\tcx: " << cameraD.color_camera.cx << "\tcy: " << cameraD.color_camera.cy << std::endl;
    //     std::cout << std::endl;

    //     std::cout << "///>>>>>>>>>>>>>> ir camera >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "///>>>>>>>>>>>>>> ir camera 1 >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "fx: " << cameraD.ir_camera.cam1.fx << "\tfy: " << cameraD.ir_camera.cam1.fy <<
    //               "\tcx: " << cameraD.ir_camera.cam1.cx << "\tcy: " << cameraD.ir_camera.cam1.cy << std::endl;
    //     std::cout << "T_camColor_camIR1: " << std::endl << cameraD.ir_camera.cam1.T_camColor_camIR1.matrix() << std::endl;
    //     std::cout << std::endl;

    //     std::cout << "///>>>>>>>>>>>>>> ir camera 2 >>>>>>>>>>>>>>>>>///" << std::endl;
    //     std::cout << "fx: " << cameraD.ir_camera.cam2.fx << "\tfy: " << cameraD.ir_camera.cam2.fy <<
    //               "\tcx: " << cameraD.ir_camera.cam2.cx << "\tcy: " << cameraD.ir_camera.cam2.cy << std::endl;
    //     std::cout << "T_camIR1_camIR2: " << std::endl << cameraD.ir_camera.cam2.T_camIR1_camIR2.matrix() << std::endl;
    //     std::cout << std::endl;
    // }

    // void print_config_cam2image(){
    //     std::cout << "///============== T_cam_image =================///" << std::endl;
    //     std::cout << "T_cam_image: " << std::endl << T_cam_image.matrix() << std::endl;
    //     std::cout << std::endl;
    // }

    // void print_config_imu(){
    //     std::cout << "///============== imu =================///" << std::endl;
    //     std::cout << "imu_name: " << std::endl << imu.imu_name << std::endl;
    //     std::cout << std::endl;
    // }

    // void print_config_marker(){
    //     std::cout << "///============== marker =================///" << std::endl;
    //     std::cout << "T_base_marker: " << std::endl << marker.T_base_marker.matrix() << std::endl;
    //     std::cout << std::endl;
    // }

    // void print_config_IRLandmark(){
    //     std::cout << "///============== IRLandmark =================///" << std::endl;
    //     std::cout << "layout_name: \t" << ir_landmark.layout_name << std::endl;
    //     std::cout << "layout: \n" << ir_landmark.layout.matrix() << std::endl;
    //     std::cout << "T_drone_IRLandmark: \n" << ir_landmark.T_drone_IRLandmark.matrix() << std::endl;
    //     std::cout << std::endl;
    // }


};

#endif 