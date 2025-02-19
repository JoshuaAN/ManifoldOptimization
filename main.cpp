#include <iostream>
#include <tuple>
#include <functional>
#include <array>
#include <set>
#include <unordered_map>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/SparseCore"
#include "Eigen/SparseCholesky"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"
#include "Eigen/src/SparseCore/SparseMatrix.h"

#include "optimization.h"

using namespace optimization;

/**
 * Test case attempting to align two vectors for each provided pose.
 */
Eigen::VectorXd AlignVectors(std::vector<Pose> pose_set) {
  Eigen::VectorXd cost(pose_set.size() * 6);

  for (int i = 0; i < pose_set.size(); ++i) {
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
  Eigen::Matrix<int, 8, 1> corners;
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

  Eigen::Matrix<double, 4, 1> world_point;
  world_point.segment(0, 3) = point;
  world_point(3, 0) = 1;

  Eigen::Vector3d temp = K * transform * world_point;
  Eigen::Vector3d p_c = temp / temp(2);

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
 
int main() {
  int N = 5;

  std::vector<Pose> S = Optimize(AlignVectors, N);

  // std::cout << S.R << std::endl;

  // Print the Euler angles
  for (int i = 0; i < 5; ++i) {
    std::cout << "Euler angles (ZYX order):\n" << S[i].R.eulerAngles(2, 1, 0) << std::endl;
    std::cout << "Translation:\n" << S[i].T << std::endl;
  }

  return 0;
}