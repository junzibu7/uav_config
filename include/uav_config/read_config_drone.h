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
   Eigen::Matrix4d T_imu_camColor;
};

struct InfraCamera1{
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

struct InfraCamera2{
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
    InfraCamera1 cam1;
    InfraCamera2 cam2;
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
    Eigen::Matrix4d T_imu_marker;
};

struct IRLandmark{
    std::string layout_name;
    int number;
    Eigen::Matrix3Xd layout;
    Eigen::Matrix4d T_imu_IRLandmark;
};

struct ConfigParser{
    std::string uav_name;
    RealsenseCamera front_camera;
    RealsenseCamera side_camera;
    Eigen::Matrix4d T_imu_t265;
    Eigen::Matrix4d T_cam_image;
    Imu imu;
    Marker marker;
    IRLandmark ir_landmark;

    ConfigParser(const std::string& config_file){
        std::cout << "Config file is " << config_file << std::endl;
        if(!FileExists(config_file)){
            std::cerr << "Config file " << config_file << " doesn't exist." << std::endl;
            return;
        }
        YAML::Node file_node = YAML::LoadFile(config_file);

        uav_name = file_node["uav_name"].as<std::string>();
        std::cout << "Start parse front camera >>>> " << std::endl;
        ///================= front camera parse =================///
        YAML::Node front_camera_node = file_node["front_camera"];
        front_camera.serial_no = front_camera_node["serial_no"].as<std::string>();
        ///>>>>>>>>>>>>>>> color camera
        YAML::Node color_camera_node = front_camera_node["color_camera"];
        front_camera.color_camera.name = color_camera_node["name"].as<std::string>();
        front_camera.color_camera.image_width = color_camera_node["image_width"].as<int>();
        front_camera.color_camera.image_height = color_camera_node["image_height"].as<int>();
        front_camera.color_camera.fx = color_camera_node["fx"].as<double>();
        front_camera.color_camera.fy = color_camera_node["fy"].as<double>();
        front_camera.color_camera.cx = color_camera_node["cx"].as<double>();
        front_camera.color_camera.cy = color_camera_node["cy"].as<double>();
        front_camera.color_camera.distortion_model = color_camera_node["distortion_model"].as<std::string>();
        for (int i = 0; i < color_camera_node["D"].size(); ++i) {
          front_camera.color_camera.D[i] = color_camera_node["D"][i].as<double>();
        }
        std::vector<double> v_temp;
        for (int i = 0; i < color_camera_node["T_imu_camColor"].size(); ++i) {
            v_temp.emplace_back(color_camera_node["T_imu_camColor"][i].as<double>());
        }
        convert_Eiegn_4d(front_camera.color_camera.T_imu_camColor, v_temp);
        ///<<<<<<<<<<<<<<< color camera


        ///>>>>>>>>>>>>>>> ir camera
        YAML::Node ir_camera_node = front_camera_node["ir_camera"];
        front_camera.ir_camera.name = ir_camera_node["name"].as<std::string>();
        ///>>>>>>>>>>>>>>> IR 1 camera
        YAML::Node IR_1_node = ir_camera_node["IR_1"];
        front_camera.ir_camera.cam1.image_width = IR_1_node["image_width"].as<int>();
        front_camera.ir_camera.cam1.image_height = IR_1_node["image_height"].as<int>();
        front_camera.ir_camera.cam1.fx = IR_1_node["fx"].as<double>();
        front_camera.ir_camera.cam1.fy = IR_1_node["fy"].as<double>();
        front_camera.ir_camera.cam1.cx = IR_1_node["cx"].as<double>();
        front_camera.ir_camera.cam1.cy = IR_1_node["cy"].as<double>();
        front_camera.ir_camera.cam1.distortion_model = IR_1_node["distortion_model"].as<std::string>();
        for (int i = 0; i < IR_1_node["D"].size(); ++i) {
          front_camera.ir_camera.cam1.D[i] = IR_1_node["D"][i].as<double>();
        }
        v_temp.clear();
        for (int i = 0; i < IR_1_node["T_camColor_camIR1"].size(); ++i) {
            v_temp.emplace_back(IR_1_node["T_camColor_camIR1"][i].as<double>());
        }
        convert_Eiegn_4d(front_camera.ir_camera.cam1.T_camColor_camIR1, v_temp);
        ///<<<<<<<<<<<<<<< IR 1 camera
        ///>>>>>>>>>>>>>>> IR 2 camera
        YAML::Node IR_2_node = ir_camera_node["IR_2"];
        front_camera.ir_camera.cam2.image_width = IR_2_node["image_width"].as<int>();
        front_camera.ir_camera.cam2.image_height = IR_2_node["image_height"].as<int>();
        front_camera.ir_camera.cam2.fx = IR_2_node["fx"].as<double>();
        front_camera.ir_camera.cam2.fy = IR_2_node["fy"].as<double>();
        front_camera.ir_camera.cam2.cx = IR_2_node["cx"].as<double>();
        front_camera.ir_camera.cam2.cy = IR_2_node["cy"].as<double>();
        front_camera.ir_camera.cam2.distortion_model = IR_2_node["distortion_model"].as<std::string>();
        for (int i = 0; i < IR_2_node["D"].size(); ++i) {
          front_camera.ir_camera.cam2.D[i] = IR_2_node["D"][i].as<double>();
        }
        v_temp.clear();
        for (int i = 0; i < IR_2_node["T_camIR1_camIR2"].size(); ++i) {
            v_temp.emplace_back(IR_2_node["T_camIR1_camIR2"][i].as<double>());
        }
        convert_Eiegn_4d(front_camera.ir_camera.cam2.T_camIR1_camIR2, v_temp);
        ///<<<<<<<<<<<<<<< IR 2 camera
        ///<<<<<<<<<<<<<<< ir camera
        std::cout << "Finish parse front camera <<<< " << std::endl;

        std::cout << "Start parse side camera >>>> " << std::endl;
        ///================= side camera parse =================///
        YAML::Node side_camera_node = file_node["side_camera"];
        side_camera.serial_no = side_camera_node["serial_no"].as<std::string>();
        ///>>>>>>>>>>>>>>> side color camera
        YAML::Node side_color_camera_node = side_camera_node["color_camera"];
        side_camera.color_camera.name = side_color_camera_node["name"].as<std::string>();
        side_camera.color_camera.image_width = side_color_camera_node["image_width"].as<int>();
        side_camera.color_camera.image_height = side_color_camera_node["image_height"].as<int>();
        side_camera.color_camera.fx = side_color_camera_node["fx"].as<double>();
        side_camera.color_camera.fy = side_color_camera_node["fy"].as<double>();
        side_camera.color_camera.cx = side_color_camera_node["cx"].as<double>();
        side_camera.color_camera.cy = side_color_camera_node["cy"].as<double>();
        side_camera.color_camera.distortion_model = side_color_camera_node["distortion_model"].as<std::string>();
        for (int i = 0; i < side_color_camera_node["D"].size(); ++i) {
          side_camera.color_camera.D[i] = side_color_camera_node["D"][i].as<double>();
        }
        v_temp.clear();
        for (int i = 0; i < side_color_camera_node["T_imu_camColor"].size(); ++i) {
            v_temp.emplace_back(side_color_camera_node["T_imu_camColor"][i].as<double>());
        }
        convert_Eiegn_4d(side_camera.color_camera.T_imu_camColor, v_temp);
        ///<<<<<<<<<<<<<<< side color camera
        ///>>>>>>>>>>>>>>> side ir camera
        YAML::Node side_ir_camera_node = side_camera_node["ir_camera"];
        side_camera.ir_camera.name = side_ir_camera_node["name"].as<std::string>();
        ///>>>>>>>>>>>>>>> IR 1 camera
        YAML::Node side_IR_1_node = side_ir_camera_node["IR_1"];
        side_camera.ir_camera.cam1.image_width = side_IR_1_node["image_width"].as<int>();
        side_camera.ir_camera.cam1.image_height = side_IR_1_node["image_height"].as<int>();
        side_camera.ir_camera.cam1.fx = side_IR_1_node["fx"].as<double>();
        side_camera.ir_camera.cam1.fy = side_IR_1_node["fy"].as<double>();
        side_camera.ir_camera.cam1.cx = side_IR_1_node["cx"].as<double>();
        side_camera.ir_camera.cam1.cy = side_IR_1_node["cy"].as<double>();
        side_camera.ir_camera.cam1.distortion_model = side_IR_1_node["distortion_model"].as<std::string>();
        for (int i = 0; i < side_IR_1_node["D"].size(); ++i) {
          side_camera.ir_camera.cam1.D[i] = side_IR_1_node["D"][i].as<double>();
        }
        v_temp.clear();
        for (int i = 0; i < side_IR_1_node["T_camColor_camIR1"].size(); ++i) {
            v_temp.emplace_back(side_IR_1_node["T_camColor_camIR1"][i].as<double>());
        }
        convert_Eiegn_4d(side_camera.ir_camera.cam1.T_camColor_camIR1, v_temp);
        ///<<<<<<<<<<<<<<< IR 1 camera
        ///>>>>>>>>>>>>>>> IR 2 camera
        YAML::Node side_IR_2_node = side_ir_camera_node["IR_2"];
        side_camera.ir_camera.cam2.image_width = side_IR_2_node["image_width"].as<int>();
        side_camera.ir_camera.cam2.image_height = side_IR_2_node["image_height"].as<int>();
        side_camera.ir_camera.cam2.fx = side_IR_2_node["fx"].as<double>();
        side_camera.ir_camera.cam2.fy = side_IR_2_node["fy"].as<double>();
        side_camera.ir_camera.cam2.cx = side_IR_2_node["cx"].as<double>();
        side_camera.ir_camera.cam2.cy = side_IR_2_node["cy"].as<double>();
        side_camera.ir_camera.cam2.distortion_model = side_IR_2_node["distortion_model"].as<std::string>();
        for (int i = 0; i < side_IR_2_node["D"].size(); ++i) {
          side_camera.ir_camera.cam2.D[i] = side_IR_2_node["D"][i].as<double>();
        }
        v_temp.clear();
        for (int i = 0; i < side_IR_2_node["T_camIR1_camIR2"].size(); ++i) {
            v_temp.emplace_back(side_IR_2_node["T_camIR1_camIR2"][i].as<double>());
        }
        convert_Eiegn_4d(side_camera.ir_camera.cam2.T_camIR1_camIR2, v_temp);
        ///<<<<<<<<<<<<<<< IR 2 camera
        ///<<<<<<<<<<<<<<< side ir camera
        std::cout << "Finish parse side camera <<<< " << std::endl;

        std::cout << "Start parse T_cam_image >>>> " << std::endl;
        ///================= T_cam_image =================///
        v_temp.clear();
        for (int i = 0; i < file_node["T_cam_image"].size(); ++i) {
            v_temp.emplace_back(file_node["T_cam_image"][i].as<double>());
        }
        convert_Eiegn_4d(T_cam_image, v_temp);
        ///================= T_cam_image =================///
        std::cout << "Finish parse T_cam_image <<<< " << std::endl;

        std::cout << "Start parse T_imu_t265 >>>> " << std::endl;
        ///================= T_imu_t265 =================///
        v_temp.clear();
        for (int i = 0; i < file_node["T_imu_t265"].size(); ++i) {
            v_temp.emplace_back(file_node["T_imu_t265"][i].as<double>());
        }
        convert_Eiegn_4d(T_imu_t265, v_temp);
        ///================= T_imu_t265 =================///
        std::cout << "Finish parse T_imu_t265 <<<< " << std::endl;


        std::cout << "Start parse imu >>>> " << std::endl;
        ///================= imu parse =================///
        YAML::Node imu_node = file_node["imu"];
        imu.imu_name = imu_node["imu_name"].as<std::string>();
        std::cout << "Finish parse imu <<<< " << std::endl;

        std::cout << "Start parse marker >>>> " << std::endl;
        ///================= marker parse =================///
        YAML::Node marker_node = file_node["Marker"];
        marker.marker_type = marker_node["marker_type"].as<std::string>();
        v_temp.clear();
        for (int i = 0; i < marker_node["T_imu_marker"].size(); ++i) {
            v_temp.emplace_back(marker_node["T_imu_marker"][i].as<double>());
        }
        convert_Eiegn_4d(marker.T_imu_marker, v_temp);
        std::cout << "Finish parse marker <<<< " << std::endl;

        std::cout << "Start parse IRLandmarker >>>> " << std::endl;
        ///================= IRlandmarker parse =================///
        YAML::Node IRLandmarker_node = file_node["IRLandmark"];
        ir_landmark.layout_name = IRLandmarker_node["layout_name"].as<std::string>();
        std::cout << "Start layout >>>> " << std::endl;
        v_temp.clear();
        for (int i = 0; i < IRLandmarker_node["layout"].size(); ++i) {
            v_temp.emplace_back(IRLandmarker_node["layout"][i].as<double>());
        }
        ir_landmark.number = IRLandmarker_node["number"].as<int>();
        //转为3行xN列的Eigen::Matrix3Xd
        ir_landmark.layout.resize(3, ir_landmark.number);
        convert_3xN(ir_landmark.layout, v_temp);
        std::cout << "Start T_imu_IRLandmark >>>> " << std::endl;
        v_temp.clear();
        for (int i = 0; i < IRLandmarker_node["T_imu_IRLandmark"].size(); ++i) {
            v_temp.emplace_back(IRLandmarker_node["T_imu_IRLandmark"][i].as<double>());
        }
        convert_Eiegn_4d(ir_landmark.T_imu_IRLandmark, v_temp);
        std::cout << "Finish parse IRLandmarker <<<< " << std::endl;
    }


    void convert_Eiegn_4d(Eigen::Matrix4d &m, std::vector<double> &vec){
        int k = 0;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
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

    void print_all(){
        print_config_front_camera();
        print_config_side_camera();
        print_config_cam2image();
        print_config_imu();
        print_config_marker();
        print_config_IRLandmark();
    }

    void print_config_front_camera(){
        std::cout << "///============== front camera =================///" << std::endl;
        std::cout << "serial_no: " << front_camera.serial_no.c_str() << std::endl;
        std::cout << "///>>>>>>>>>>>>>> color camera >>>>>>>>>>>>>>>>>///" << std::endl;
        std::cout << "fx: " << front_camera.color_camera.fx << "\tfy: " << front_camera.color_camera.fy <<
        "\tcx: " << front_camera.color_camera.cx << "\tcy: " << front_camera.color_camera.cy << std::endl;
        std::cout << "T_imu_camColor: " << std::endl << front_camera.color_camera.T_imu_camColor.matrix() << std::endl;
        std::cout << std::endl;

        std::cout << "///>>>>>>>>>>>>>> ir camera >>>>>>>>>>>>>>>>>///" << std::endl;
        std::cout << "///>>>>>>>>>>>>>> ir camera 1 >>>>>>>>>>>>>>>>>///" << std::endl;
        std::cout << "fx: " << front_camera.ir_camera.cam1.fx << "\tfy: " << front_camera.ir_camera.cam1.fy <<
                  "\tcx: " << front_camera.ir_camera.cam1.cx << "\tcy: " << front_camera.ir_camera.cam1.cy << std::endl;
        std::cout << "T_camColor_camIR1: " << std::endl << front_camera.ir_camera.cam1.T_camColor_camIR1.matrix() << std::endl;
        std::cout << std::endl;

        std::cout << "///>>>>>>>>>>>>>> ir camera 2 >>>>>>>>>>>>>>>>>///" << std::endl;
        std::cout << "fx: " << front_camera.ir_camera.cam2.fx << "\tfy: " << front_camera.ir_camera.cam2.fy <<
                  "\tcx: " << front_camera.ir_camera.cam2.cx << "\tcy: " << front_camera.ir_camera.cam2.cy << std::endl;
        std::cout << "T_camIR1_camIR2: " << std::endl << front_camera.ir_camera.cam2.T_camIR1_camIR2.matrix() << std::endl;
        std::cout << std::endl;
    }

    void print_config_side_camera(){
        std::cout << "///============== side camera =================///" << std::endl;
        std::cout << "serial_no: " << side_camera.serial_no.c_str() << std::endl;
        std::cout << "///>>>>>>>>>>>>>> color camera >>>>>>>>>>>>>>>>>///" << std::endl;
        std::cout << "fx: " << side_camera.color_camera.fx << "\tfy: " << side_camera.color_camera.fy <<
                  "\tcx: " << side_camera.color_camera.cx << "\tcy: " << side_camera.color_camera.cy << std::endl;
        std::cout << "T_imu_camColor: " << std::endl << side_camera.color_camera.T_imu_camColor.matrix() << std::endl;
        std::cout << std::endl;

        std::cout << "///>>>>>>>>>>>>>> ir camera >>>>>>>>>>>>>>>>>///" << std::endl;
        std::cout << "///>>>>>>>>>>>>>> ir camera 1 >>>>>>>>>>>>>>>>>///" << std::endl;
        std::cout << "fx: " << side_camera.ir_camera.cam1.fx << "\tfy: " << side_camera.ir_camera.cam1.fy <<
                  "\tcx: " << side_camera.ir_camera.cam1.cx << "\tcy: " << side_camera.ir_camera.cam1.cy << std::endl;
        std::cout << "T_camColor_camIR1: " << std::endl << side_camera.ir_camera.cam1.T_camColor_camIR1.matrix() << std::endl;
        std::cout << std::endl;

        std::cout << "///>>>>>>>>>>>>>> ir camera 2 >>>>>>>>>>>>>>>>>///" << std::endl;
        std::cout << "fx: " << side_camera.ir_camera.cam2.fx << "\tfy: " << side_camera.ir_camera.cam2.fy <<
                  "\tcx: " << side_camera.ir_camera.cam2.cx << "\tcy: " << side_camera.ir_camera.cam2.cy << std::endl;
        std::cout << "T_camIR1_camIR2: " << std::endl << side_camera.ir_camera.cam2.T_camIR1_camIR2.matrix() << std::endl;
        std::cout << std::endl;
    }

    void print_config_cam2image(){
        std::cout << "///============== T_cam_image =================///" << std::endl;
        std::cout << "T_cam_image: " << std::endl << T_cam_image.matrix() << std::endl;
        std::cout << std::endl;
    }

    void print_config_imu(){
        std::cout << "///============== imu =================///" << std::endl;
        std::cout << "imu_name: " << std::endl << imu.imu_name << std::endl;
        std::cout << std::endl;
    }

    void print_config_marker(){
        std::cout << "///============== marker =================///" << std::endl;
        std::cout << "T_imu_marker: " << std::endl << marker.T_imu_marker.matrix() << std::endl;
        std::cout << std::endl;
    }

    void print_config_IRLandmark(){
        std::cout << "///============== IRLandmark =================///" << std::endl;
        std::cout << "layout_name: \t" << ir_landmark.layout_name << std::endl;
        std::cout << "layout: \n" << ir_landmark.layout.matrix() << std::endl;
        std::cout << "T_imu_IRLandmark: \n" << ir_landmark.T_imu_IRLandmark.matrix() << std::endl;
        std::cout << std::endl;
    }


};

#endif  // READ_CONFIG_DRONE_H_
