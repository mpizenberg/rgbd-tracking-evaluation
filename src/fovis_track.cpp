#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <fovis/fovis.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace fs = boost::filesystem;

// This file is automatically formatted with clang-format.

const std::string usage = "Usage: ./fovis_track associations_file";

// Function checking that input arguments are valid.
fs::path check_arguments(int argc, char **argv) {

  // Check that the number of arguments is correct.
  if (argc != 2) {
    std::cerr << usage << std::endl;
    std::exit(-1);
  }

  // Check that the associations file exists.
  const fs::path associations_file = fs::path(argv[1]);
  if (!fs::is_regular_file(associations_file)) {
    std::cerr << associations_file << " is not a valid file." << std::endl;
    std::exit(-1);
  }

  return associations_file;
}

struct Assoc {
  double rgb_timestamp;
  fs::path rgb_file_path;
  double depth_timestamp;
  fs::path depth_file_path;
};

// Build a vector containing timestamps and full paths of rgb and depth images.
std::vector<Assoc> extract_associations(fs::path associations_file_path) {

  std::vector<Assoc> associations;
  const fs::path dataset_dir = associations_file_path.parent_path();
  std::ifstream assoc_file(associations_file_path.string());

  // Extract associations at each line of the file.
  for (std::string line; std::getline(assoc_file, line);) {
    std::istringstream iss(line);
    double rgb_timestamp, depth_timestamp;
    fs::path rgb_file_path, depth_file_path;
    if (iss >> rgb_timestamp >> rgb_file_path >> depth_timestamp >>
        depth_file_path) {
      Assoc assoc = {rgb_timestamp, dataset_dir / rgb_file_path,
                     depth_timestamp, dataset_dir / depth_file_path};
      associations.push_back(assoc);
    }
  }

  return associations;
}

// Generate ICL-NUIM camera intrinsics parameters.
fovis::CameraIntrinsicsParameters create_camera() {
  fovis::CameraIntrinsicsParameters camParams;
  camParams.width = 640;
  camParams.height = 480;

  // fr1
  camParams.fx = 517.306408;
  camParams.fy = 516.469215;
  camParams.cx = 318.643040;
  camParams.cy = 255.313989;
  camParams.k1 = 0.262383;
  camParams.k2 = -0.953104;
  camParams.p1 = -0.005358;
  camParams.p2 = 0.002628;
  camParams.k3 = 1.163314;

  // // icl-nuim
  // camParams.fx = 481.2;
  // camParams.fy = -480.0;
  // camParams.cx = 319.5;
  // camParams.cy = 239.5;
  // camParams.k1 = 0.0;
  // camParams.k2 = 0.0;
  // camParams.p1 = 0.0;
  // camParams.p2 = 0.0;
  // camParams.k3 = 0.0;

  return camParams;
}

// Retrieve the gray image from file.
std::vector<uint8_t> gray_img_from_file(std::string rgb_file) {
  cv::Mat rgb_image = cv::imread(rgb_file);
  cv::Mat img;
  cv::cvtColor(rgb_image, img, cv::COLOR_BGR2GRAY);
  std::vector<uint8_t> vec_u8;
  vec_u8.assign(img.datastart, img.dataend);
  return vec_u8;
}

// Retrieve depth map from file.
std::vector<float> depth_map_from_file(std::string depth_file) {
  cv::Mat depth_u16 = cv::imread(depth_file, -1);
  cv::Mat depth;
  depth_u16.convertTo(depth, CV_32FC1, 1.f / 5000.f);
  std::vector<float> vec_float;
  vec_float.assign((float *)depth.datastart, (float *)depth.dataend);
  for (int i = 0; i < vec_float.size(); i++) {
    if (vec_float[i] == 0.0) {
      vec_float[i] = NAN;
    }
  }
  return vec_float;
}

// Write a rigid body motion in the TUM format to stdout.
void write_pose(Eigen::Isometry3d pose) {
  Eigen::Vector3d t = pose.translation();
  Eigen::Quaterniond q(pose.rotation());
  std::cout << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " "
            << q.y() << " " << q.z() << " " << q.w() << std::endl;
}

int main(int argc, char **argv) {

  // Verify that the arguments are correct.
  fs::path associations_file_path = check_arguments(argc, argv);

  // Build a vector containing timestamps and full paths of images.
  std::vector<Assoc> associations =
      extract_associations(associations_file_path);

  // Initialize intrinsics camera parameters.
  fovis::CameraIntrinsicsParameters camera_intrinsics = create_camera();
  fovis::Rectification rectification(camera_intrinsics);

  // If we wanted to play around with the different VO parameters, we could set
  // them here in the "options" variable.
  fovis::VisualOdometryOptions options =
      fovis::VisualOdometry::getDefaultOptions();

  // Setup the visual odometry.
  fovis::VisualOdometry odometry(&rectification, options);

  // Track every frame.
  std::cout << std::fixed;
  for (Assoc assoc : associations) {

    // Load the gray image.
    std::vector<uint8_t> gray_data =
        gray_img_from_file(assoc.rgb_file_path.string());

    // Load the depth image.
    std::vector<float> depth_data =
        depth_map_from_file(assoc.depth_file_path.string());
    int depth_width = camera_intrinsics.width;
    int depth_height = camera_intrinsics.height;
    fovis::DepthImage depth_image(camera_intrinsics, depth_width, depth_height);
    depth_image.setDepthImage(&depth_data[0]);

    // Process the frame.
    odometry.processFrame(&gray_data[0], &depth_image);
    std::cout << assoc.rgb_timestamp << " ";
    write_pose(odometry.getPose());
  }

  return 0;
}
