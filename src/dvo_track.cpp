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

#include <Eigen/Geometry>
#include <boost/filesystem.hpp>
#include <dvo/core/intrinsic_matrix.h>
#include <dvo/core/surface_pyramid.h>
#include <dvo/dense_tracking.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace fs = boost::filesystem;

// This file is automatically formatted with clang-format.

const std::string usage =
    "Usage: ./dvo_track [fr1|fr2|fr3|icl] associations_file";

// Generate camera intrinsics parameters.
dvo::core::IntrinsicMatrix create_camera(std::string camera_id) {
  float fx, fy, cx, cy;
  if (camera_id == "fr1") {
    fx = 517.306408;
    fy = 516.469215;
    cx = 318.643040;
    cy = 255.313989;
  } else if (camera_id == "fr2") {
    fx = 520.908620;
    fy = 521.007327;
    cx = 325.141442;
    cy = 249.701764;
  } else if (camera_id == "fr3") {
    fx = 535.433105;
    fy = 539.212524;
    cx = 320.106653;
    cy = 247.632132;
  } else if (camera_id == "icl") {
    fx = 481.2;
    fy = -480.0;
    cx = 319.5;
    cy = 239.5;
  } else {
    std::cerr << "Unknown camera id: " << camera_id << std::endl;
    std::cerr << usage << std::endl;
    std::exit(-1);
  }
  return dvo::core::IntrinsicMatrix::create(fx, fy, cx, cy);
}

struct Checked {
  dvo::core::IntrinsicMatrix intrinsics;
  fs::path associations_file_path;
};

// Function checking that input arguments are valid.
Checked check_arguments(int argc, char **argv) {

  // Check that the number of arguments is correct.
  if (argc != 3) {
    std::cerr << usage << std::endl;
    std::exit(-1);
  }

  // Check that the camera id is correct
  dvo::core::IntrinsicMatrix intrinsics = create_camera(argv[1]);

  // Check that the associations file exists.
  const fs::path associations_file = fs::path(argv[2]);
  if (!fs::is_regular_file(associations_file)) {
    std::cerr << associations_file << " is not a valid file." << std::endl;
    std::exit(-1);
  }

  return {intrinsics, associations_file};
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

// Write a rigid body motion in the TUM format to stdout.
void write_pose(Eigen::Isometry3d pose) {
  Eigen::Vector3d t = pose.translation();
  Eigen::Quaterniond q(pose.rotation());
  std::cout << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " "
            << q.y() << " " << q.z() << " " << q.w() << std::endl;
}

int main(int argc, char **argv) {

  // Verify that the arguments are correct.
  Checked checked = check_arguments(argc, argv);

  // Build a vector containing timestamps and full paths of images.
  std::vector<Assoc> associations =
      extract_associations(checked.associations_file_path);

  // Init dvo::DenseTracker.
  dvo::DenseTracker dense_tracker(checked.intrinsics);

  // Track every frame.
  cv::Ptr<dvo::core::RgbdImagePyramid> previous_frame, current_frame;
  Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
  std::cout << std::fixed;
  bool first_frame = true;
  for (Assoc &assoc : associations) {

    // Load the depth image.
    cv::Mat depth_u16 =
        cv::imread(assoc.depth_file_path.string(), cv::IMREAD_ANYDEPTH);
    cv::Mat depth_float;
    dvo::core::SurfacePyramid::convertRawDepthImageSse(depth_u16, depth_float,
                                                       1.0f / 5000.0f);

    // Load the gray image.
    cv::Mat rgb_image = cv::imread(assoc.rgb_file_path.string());
    cv::Mat img, img_float;
    cv::cvtColor(rgb_image, img, cv::COLOR_BGR2GRAY);
    img.convertTo(img_float, CV_32F);

    // Build Rgbd image pyramid for this frame.
    current_frame = cv::Ptr<dvo::core::RgbdImagePyramid>(
        new dvo::core::RgbdImagePyramid(img_float, depth_float));
    rgb_image.convertTo(current_frame->level(0).rgb, CV_32FC3);

    if (first_frame) {
      first_frame = false;
      std::swap(previous_frame, current_frame);
      continue;
    }

    // Run dense tracker.
    Eigen::Affine3d relative_pose_affine;
    dense_tracker.match(*previous_frame, *current_frame, relative_pose_affine);
    Eigen::Isometry3d relative_pose(Eigen::Isometry3d::Identity());
    relative_pose.rotate(relative_pose_affine.rotation());
    relative_pose.translation() = relative_pose_affine.translation();
    pose = pose * relative_pose;

    // Print pose to stdout.
    std::cout << assoc.rgb_timestamp << " ";
    write_pose(pose);

    // Make current frame the previous one for next iteration.
    std::swap(previous_frame, current_frame);
  }

  return 0;
}
