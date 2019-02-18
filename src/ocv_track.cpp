// Copyright (C) 2019 Matthieu Pizenberg
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>

namespace fs = boost::filesystem;
namespace rgbd = cv::rgbd;

// This file is automatically formatted with clang-format.

const std::string usage =
    "Usage: ./Tum [Rgbd|ICP|RgbdICP] [fr1|fr2|fr3|icl] associations_file";

// Generate camera intrinsics parameters.
// BEWARE that fr1 and fr2 sequences are not undistorted
// so the performance on these might be worse.
cv::Mat create_camera(std::string camera_id) {
  cv::Mat camera_intrinsics = cv::Mat::eye(3, 3, CV_32FC1);

  if (camera_id == "fr1") {
    camera_intrinsics.at<float>(0, 0) = 517.306408; // fx
    camera_intrinsics.at<float>(1, 1) = 516.469215; // fy
    camera_intrinsics.at<float>(0, 2) = 318.643040; // cx
    camera_intrinsics.at<float>(1, 2) = 255.313989; // cy
  } else if (camera_id == "fr2") {
    camera_intrinsics.at<float>(0, 0) = 520.908620; // fx
    camera_intrinsics.at<float>(1, 1) = 521.007327; // fy
    camera_intrinsics.at<float>(0, 2) = 325.141442; // cx
    camera_intrinsics.at<float>(1, 2) = 249.701764; // cy
  } else if (camera_id == "fr3") {
    // Color images of Freiburg 3 sequences have already been undistorted.
    camera_intrinsics.at<float>(0, 0) = 535.433105; // fx
    camera_intrinsics.at<float>(1, 1) = 539.212524; // fy
    camera_intrinsics.at<float>(0, 2) = 320.106653; // cx
    camera_intrinsics.at<float>(1, 2) = 247.632132; // cy
  } else if (camera_id == "icl") {
    camera_intrinsics.at<float>(0, 0) = 481.20; // fx
    camera_intrinsics.at<float>(1, 1) = -480.0; // fy
    camera_intrinsics.at<float>(0, 2) = 319.50; // cx
    camera_intrinsics.at<float>(1, 2) = 239.50; // cy
  } else {
    std::cerr << "Unknown camera id: " << camera_id << std::endl;
    std::cerr << usage << std::endl;
    std::exit(-1);
  }

  return camera_intrinsics;
}

struct Checked {
  cv::Ptr<rgbd::Odometry> odometry;
  cv::Mat camera_intrinsics;
  fs::path associations_file_path;
};

// Function checking that input arguments are valid.
Checked check_arguments(int argc, char **argv) {

  // Check that the number of arguments is correct.
  if (argc != 4) {
    std::cerr << usage << std::endl;
    std::exit(-1);
  }

  // Check that the method is a valid string method name.
  const std::string method = argv[1];
  cv::Ptr<rgbd::Odometry> odometry =
      rgbd::Odometry::create(method + "Odometry");
  if (odometry.empty()) {
    std::cerr << "The method name must be one of [Rgbd|ICP|RgbdICP]."
              << std::endl
              << usage << std::endl;
    std::exit(-1);
  }

  // Check that the camera id is correct
  cv::Mat intrinsics = create_camera(argv[2]);

  // Check that the associations file exists.
  const fs::path associations_file = fs::path(argv[3]);
  if (!fs::is_regular_file(associations_file)) {
    std::cerr << associations_file << " is not a valid file." << std::endl;
    std::exit(-1);
  }

  return {odometry, intrinsics, associations_file};
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
  // Associations must follow the format:
  // depth_timestamp depth_file_path rgb_timestamp rgb_file_path
  for (std::string line; std::getline(assoc_file, line);) {
    std::istringstream iss(line);
    double rgb_timestamp, depth_timestamp;
    fs::path rgb_file_path, depth_file_path;
    if (iss >> depth_timestamp >> depth_file_path >> rgb_timestamp >>
        rgb_file_path) {
      Assoc assoc = {rgb_timestamp, dataset_dir / rgb_file_path,
                     depth_timestamp, dataset_dir / depth_file_path};
      associations.push_back(assoc);
    }
  }

  return associations;
}

// Create an OpenCV "OdometryFrame".
cv::Ptr<rgbd::OdometryFrame> create_odometry_frame(std::string rgb_file,
                                                   std::string depth_file) {
  // Read RGB (u8,u8,u8) and depth (u16) images.
  cv::Mat rgb_image = cv::imread(rgb_file);
  cv::Mat depth_u16 = cv::imread(depth_file, cv::IMREAD_ANYDEPTH);
  assert(depth_u16.type() == CV_16UC1);

  // Convert RGB to gray image.
  cv::Mat img;
  cv::cvtColor(rgb_image, img, cv::COLOR_BGR2GRAY);

  // Retrieve the mask of valid depths (different from 0).
  cv::Mat mask = depth_u16 > 0;

  // Scale depth image by 1/5000.
  cv::Mat depth;
  depth_u16.convertTo(depth, CV_32FC1, 1.f / 5000.f);

  return rgbd::OdometryFrame::create(img, depth, mask);
}

// Write a rigid body motion matrix in the TUM format to stdout.
void write_motion(cv::Mat motion) {

  // Retrieve the translation part.
  cv::Mat t = motion(cv::Rect(3, 0, 1, 3));

  // Retrieve the quaternion for rotation.
  cv::Mat rotation_vec;
  cv::Rodrigues(motion(cv::Rect(0, 0, 3, 3)), rotation_vec);
  double theta = cv::norm(rotation_vec);
  double q_w = std::cos(0.5 * theta);
  cv::Mat q_xyz;
  // If theta is big enough, q_xyz is computed as normal.
  // Else we have, at first order, theta = 2 * sin(theta/2).
  if (theta > 1e-6) {
    double sin_theta_2 = std::sin(0.5 * theta);
    q_xyz = sin_theta_2 * rotation_vec / theta;
  } else {
    q_xyz = 0.5 * rotation_vec;
  }

  // timestamp tx ty tz qx qy qz qw
  std::cout << t.at<double>(0) << " " << t.at<double>(1) << " "
            << t.at<double>(2) << " " << q_xyz.at<double>(0) << " "
            << q_xyz.at<double>(1) << " " << q_xyz.at<double>(2) << " " << q_w
            << std::endl;
}

int main(int argc, char **argv) {

  // Verify that the arguments are correct.
  Checked checked = check_arguments(argc, argv);

  // Build a vector containing timestamps and full paths of images.
  std::vector<Assoc> associations =
      extract_associations(checked.associations_file_path);

  // Initialize intrinsics camera parameters.
  checked.odometry->setCameraMatrix(checked.camera_intrinsics);

  // Declare loop variables.
  cv::Ptr<rgbd::OdometryFrame> src_frame;
  cv::Mat camera_motion;
  std::vector<cv::Mat> successive_motions;
  std::vector<cv::Mat> trajectory;
  bool first_frame = true;

  // Track every frame.
  std::cout << std::fixed;
  for (Assoc assoc : associations) {
    // Create an OpenCV "OdometryFrame".
    cv::Ptr<rgbd::OdometryFrame> dest_frame = create_odometry_frame(
        assoc.rgb_file_path.string(), assoc.depth_file_path.string());

    // If this is the first frame, we don't track anything.
    if (first_frame) {
      first_frame = false;
      src_frame = dest_frame;
      camera_motion = cv::Mat::eye(4, 4, CV_64FC1);
      continue;
    }

    // Track the frame.
    // Here we choose the old frame as src_frame,
    // so we have to inverse the motion found to retrieve camera coordinates.
    // If we want to avoid inversing, we could inverse the role of
    // src_frame and dest_frame, but I guess the algorithm would use the depth
    // info from dest_frame, which is different (a bit like tracking in
    // reverse).
    cv::Mat motion;
    if (!checked.odometry->compute(src_frame, dest_frame, motion)) {
      motion = cv::Mat::eye(4, 4, CV_64FC1);
    }
    successive_motions.push_back(motion);
    camera_motion = camera_motion * motion.inv();
    trajectory.push_back(camera_motion.clone());

    // Write the trajectory in the TUM format to stdout.
    std::cout << assoc.rgb_timestamp << " ";
    write_motion(camera_motion);

    // Reset the src_frame for next loop as the current dest_frame.
    src_frame = dest_frame;
  }

  return 0;
}
