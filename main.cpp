#include <iostream>
#include <tuple>
#include <functional>
#include <array>
#include <set>
#include <unordered_map>
#include <vector>
#include <fstream>

#include "Eigen/Dense"
#include "Eigen/SparseCore"
#include "Eigen/SparseCholesky"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"
#include "Eigen/src/SparseCore/SparseMatrix.h"

#include "json.hpp"

#include "optimization.h"

using namespace optimization;
using json = nlohmann::json;

/**
 * Test case attempting to align two vectors for each provided pose.
 */
Eigen::VectorXd AlignVectors(std::vector<Pose> pose_set) {
  Eigen::VectorXd cost(pose_set.size() * 6);

  for (size_t i = 0; i < pose_set.size(); ++i) {
    Pose S = pose_set[i];

    Eigen::Vector3d u1;
    u1 << 0, 1, 0;
    Eigen::Vector3d v1;
    v1 << 1, 0, 0;

    Eigen::Vector3d u2;
    u2 << 1, 0, 0;
    Eigen::Vector3d v2;
    v2 << 0, -1, 0;

    cost.segment(i * 6, 3) = S.R * v1 - u1 + S.T;
    cost.segment(i * 6 + 3, 3) = S.R * v2 - u2 + S.T;
  }

  return cost;
}

struct ApriltagMeasurement {
  /**
   * Store corners with the data structures:
   *
   *   [x_1]
   *   [y_1]
   *   [ ⋮ ]
   *   [y_4]
   */
  Eigen::Matrix<double, 8, 1> corners;
  int tag_id;
};

/**
 * @brief Computes the reprojection of a given point in ℝ³ onto a camera with pose in SE(3).
 *
 * @param K Camera instrinic matrix.
 * @param camera Camera pose in SE(3).
 * @param point Point in ℝ³.
 * @return Pixel reprojection, [x, y]ᵀ, of the provided point onto the camera image.
 */
Eigen::Vector2d ReprojectPoint(const Eigen::Matrix3d& K, 
                               const Pose& camera, 
                               const Eigen::Vector3d& point) {
    /**
     * Pinhole camera model:
     *
     *   s * p_c = K[R | T]p_w
     *
     * Where:
     *
     *   - p_w = [x  y  z  1]ᵀ is the homogenous world point.
     *   - p_c = [u  v  1]ᵀ is the corresponding homogenous image point.
     *   - K is the matrix of intrinsic camera paramters.
     *   - s is a scale factor for the image point.
     *   - R and T are the 3D rotation and 3D translation of the camera. 
     *
     * Taken directly from https://en.wikipedia.org/wiki/Perspective-n-Point.
     */
  Eigen::Matrix<double, 3, 4> transform;
  transform.leftCols(3) = camera.R;
  transform.rightCols(1) = camera.T;

  // std::cout << "Transform\n" << transform << std::endl;

  Eigen::Matrix<double, 4, 1> world_point;
  world_point.segment(0, 3) = point;
  world_point(3, 0) = 1;

  // std::cout << "world point\n" << world_point << std::endl;

  Eigen::Vector3d temp = K * transform * world_point;
  // std::cout << "temp\n" << temp << std::endl;
  Eigen::Vector3d p_c = temp / temp(2);

  // std::cout << "p_c\n" << p_c << std::endl;

  return p_c.segment(0, 2);
}

// /**
//    * Return corners with the data structures:
//    *
//    *   [x_1]
//    *   [y_1]
//    *   [ ⋮ ]
//    *   [y_4]
//    */
// Eigen::Matrix<int, 8, 1> ReprojectApriltag(Pose camera, Pose tag) {
//   // return ;
// }

// std::vector<Pose> SolveTags(std::vector<std::vector<ApriltagMeasurement>> frames) {
//   std::set<int> tag_ids;

//   for (auto &frame : frames) {
//     for (auto &measurement : frame) {
//       tag_ids.insert(measurement.tag_id);
//     }
//   }

//   /**
//    * Form map of tag ids to their relative positions in the set.
//    */
//   std::unordered_map<int, int> tag_map;
//   int index = 0;
//   for (const int& element : tag_ids) {
//       tag_map[element] = index++;
//   }

//   /**
//    * Form list of measurements with the data structure: 
//    *
//    *   <camera pose index, tag pose index, x_corners, y_corners>
//    */
//   std::vector<std::tuple<int, int, Eigen::VectorXd>> measurements;

//   for (int i = 0; i < frames.size(); ++i) {
//     for (auto &measurement : frames.at(i)) {
//       measurements.emplace_back(i + tag_ids.size(), tag_map[measurement.tag_id], measurement.corners);
//     }
//   }

//   auto loss = [measurements](std::vector<Pose> poses) -> Eigen::VectorXd {
//     // Eigen::VectorXd error{measurements.size() * 8};

//     // for (int i = 0; i < measurements.size(); ++i) {
//     //   auto measurement = measurements.at(i);

//     //   Pose camera_pose = poses[std::get<0>(measurement)];
//     //   Pose tag_pose = poses[std::get<1>(measurement)];

//     //   Eigen::VectorXd corners = std::get<2>(measurement);
//     //   Eigen::VectorXd corner_reprojection = ReprojectApriltag(camera_pose, tag_pose);

//     //   error.segment(i * 8, 8) = corners - corner_reprojection;
//     // }
//   };

//   return Optimize(loss, tag_ids.size() + frames.size());
// }

// struct Point {
//   double x;
//   double y;
// };

// struct Shape {
//     int id;
//     std::vector<Point> corners;
// };

// void from_json(const json& j, Point& p) {
//     j.at("x").get_to(p.x);
//     j.at("y").get_to(p.y);
// }

// void from_json(const json& j, Shape& s) {
//     j.at("id").get_to(s.id);
//     j.at("corners").get_to(s.corners);
// }

 
// int main() {
//   // Eigen::Matrix<double, 3, 3> K;
//   // K <<  5.9938E+02,  0.0000E+00,  4.7950E+02,
//   //       0.0000E+00,  5.9917E+02,  3.5950E+02,
//   //       0.0000E+00,  0.0000E+00,  1.0000E+00;

//   // // int N = 5;

//   // // std::vector<Pose> S = Optimize(AlignVectors, N);

//   // // // std::cout << S.R << std::endl;

//   // // // Print the Euler angles
//   // // for (int i = 0; i < 5; ++i) {
//   // //   std::cout << "Euler angles (ZYX order):\n" << S[i].R.eulerAngles(2, 1, 0) << std::endl;
//   // //   std::cout << "Translation:\n" << S[i].T << std::endl;
//   // // }

//   // auto vec = [](double x, double y, double z) {
//   //   Eigen::Vector3d evec;
//   //   evec << x, y, z;
//   //   return evec;
//   // };

//   // Eigen::Vector3d axis_angle(1.0, 0.0, 0.0); // Example vector, axis = (1,0,0), angle = 1 radian
//   // double angle = axis_angle.norm(); // The magnitude of the vector
//   // Eigen::Vector3d axis = axis_angle.normalized(); // The direction of the vector

//   // Eigen::AngleAxisd angleAxis(angle, axis);

//   // Eigen::Vector3d T;
//   // T << 1, 5, 3;

//   // Pose ref_camera = Pose{angleAxis.toRotationMatrix(), T};

//   // std::vector<Eigen::Vector3d> points;
//   // points.push_back(vec(5, 3, 2));
//   // points.push_back(vec(1, 4, 9));
//   // points.push_back(vec(5, 9, 2));
//   // points.push_back(vec(-4, 1, 2));

//   // auto loss = [ref_camera, points, K](std::vector<Pose> poses) -> Eigen::VectorXd {
//   //   Eigen::VectorXd ref_pixels(points.size() * 2);
//   //   Eigen::VectorXd measure_pixels(points.size() * 2);

//   //   Pose S = poses.at(0);

//   //   for (size_t i = 0; i < points.size(); ++i) {
//   //     ref_pixels.segment(i * 2, 2) = ReprojectPoint(K, ref_camera, points.at(i));
//   //     measure_pixels.segment(i * 2, 2) = ReprojectPoint(K, S, points.at(i));
//   //   }

//   //   return ref_pixels - measure_pixels;
//   // };

//   // Pose S = Optimize(loss, 1).at(0);

//   // std::cout << "Translation:\n" << S.T << std::endl;
//   // // std::cout << "Rotation:\n" << S.R.eu << std::endl;

//   return 0;
// }

int main() {
  std::vector<std::vector<ApriltagMeasurement>> frames;
  std::ifstream file("field_tags_2024.jsonl");

  if (!file.is_open()) {
      throw std::runtime_error("Unable to open file");
  }

  std::string line;
  while (std::getline(file, line)) {
    json j = json::parse(line);

    std::vector<ApriltagMeasurement> measurements;

    for (const auto& item : j) {
        ApriltagMeasurement measurement;
        measurement.tag_id = item["id"].get<int>();
        measurement.corners = Eigen::Matrix<double, 8, 1>();

        int index = 0;
        for (const auto& corner : item["corners"]) {
          measurement.corners(2 * index) = corner["x"].get<double>();
          measurement.corners(2 * index + 1) = corner["y"].get<double>();
          index += 1;
        }
        measurements.push_back(measurement);
    }
    frames.push_back(measurements);
  }

  file.close();

  // // Print parsed data
  // for (const auto& frame : frames) {
  //   std::cout << "Frame" << std::endl;
  //   for (const auto& measurement : frame) {
  //     std::cout << "Object ID: " << measurement.tag_id << std::endl;
  //     std::cout << "Corners:\n" << measurement.corners << std::endl;
  //   }
  // }

  // Object ID: 16
  // Corners:
  // 835.869
  // 265.512
  // 849.627
  // 262.44
  // 849.588
  // 241.223
  // 836.485
  // 244.896
  // Object ID: 6
  // Corners:
  // 798.166
  // 310.764
  // 808.583
  // 310.438
  // 808.774
  // 299.035
  // 798.521
  // 300.587

    auto vec = [](double x, double y, double z) {
    Eigen::Vector3d evec;
    evec << x, y, z;
    return evec;
  };

  Eigen::Matrix<double, 3, 3> K;
  K <<  5.9938E+02,  0.0000E+00,  4.7950E+02,
        0.0000E+00,  5.9917E+02,  3.5950E+02,
        0.0000E+00,  0.0000E+00,  1.0000E+00;

  ApriltagMeasurement tag_6;
  tag_6.tag_id = 6;
  tag_6.corners = Eigen::Matrix<double, 8, 1>();
  tag_6.corners << 798.166,
                    310.764,
                    808.583,
                    310.438,
                    808.774,
                    299.035,
                    798.521,
                    300.587;
  
  Pose tag_6_pose;
  tag_6_pose.T = Eigen::Vector3d();
  tag_6_pose.T << 1.8415, 8.2042, 1.355852;
  Eigen::Quaternion<double> Q;
  Q.coeffs() << -0.7071067811865475, 0, 0, 0.7071067811865475;
  tag_6_pose.R = Q.toRotationMatrix();

  std::cout << "R:\n" << tag_6_pose.R << std::endl;

  auto ReprojectApriltag = [vec, K](const Pose& camera, const Pose& tag) -> Eigen::Matrix<double, 8, 1> {
    std::array<Eigen::Vector3d, 4> corners{tag.R * vec(0, -6.5 / 2, -6.5 / 2) + tag.T, 
                                           tag.R * vec(0, 6.5 / 2, -6.5 / 2) + tag.T,
                                           tag.R * vec(0, 6.5 / 2, 6.5 / 2) + tag.T,
                                           tag.R * vec(0, -6.5 / 2, 6.5 / 2) + tag.T};
    
    Eigen::Matrix<double, 8, 1> pixels;
    
    for (size_t i = 0; i < 4; ++i) {
      // std::cout << "cam:\n" << camera.R << std::endl;
      // std::cout << "corn:\n" << corners.at(i) << std::endl;
      pixels.segment(2 * i, 2) = ReprojectPoint(K, camera, corners.at(i));
      // std::cout << "pixel:\n" << pixels.segment(2 * i, 2) << std::endl;
    }

    return pixels;
  };

  auto loss = [tag_6_pose, tag_6, ReprojectApriltag](std::vector<Pose> poses) {
    auto S = poses[0];

    std::cout << "Cost: \n" << ReprojectApriltag(S, tag_6_pose) - tag_6.corners << std::endl;

    return ReprojectApriltag(S, tag_6_pose) - tag_6.corners;
  };

  // Eigen::Vector3d zv = Eigen::Vector3d::Zero();
  // Eigen::Matrix3d zr = Eigen::Matrix3d::Identity();
  // Pose c = Pose{zr, zv};

  // std::cout << "Reprojection:\n" << ReprojectApriltag(c, tag_6_pose);

  Pose S = Optimize(loss, 1)[0];

  std::cout << "Translation:\n" << S.T << std::endl;

  return 0;
}