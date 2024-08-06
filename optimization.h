#include "Eigen/Dense"
#include "Eigen/SparseCore"
#include "Eigen/SparseCholesky"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"
#include "Eigen/src/SparseCore/SparseMatrix.h"

#pragma once

namespace optimization {

struct Pose {
  Eigen::Matrix3d R;
  Eigen::Vector3d T;
};

double Quadratic(double A, double B, double C);

Eigen::SparseMatrix<double> NumericalJacobian(std::function<Eigen::VectorXd(Eigen::VectorXd)> f, 
                                              Eigen::VectorXd x, 
                                              double step);

/**
 * @brief Computes the skew symmetric matrix, ŵ, of the given vector w = [wˣ, wʸ, wᶻ]ᵀ.
 *
 *       [ 0  -wᶻ  wʸ]
 *   ŵ = [ wᶻ  0  -wˣ]
 *       [-wʸ  wˣ  0 ]
 *
 * @param w Input vector w = [wˣ, wʸ, wᶻ]ᵀ.
 * @return Skew symmetric matrix ŵ.
 */
Eigen::Matrix3d SkewSymmetric(Eigen::Vector3d w);

/**
 * @brief Computes the matrix exponential of the skew symmetric matrix of the provided vector.
 *
 * We use the 
 */
Eigen::Matrix3d SkewSymmetricExponential(Eigen::Vector3d w);

/**
 * @brief Finds a set of poses of size n such that the provided cost function is minimized.
 *
 *   https://www.seas.upenn.edu/~cjtaylor/PUBLICATIONS/pdfs/TaylorTR94b.pdf
 *
 * @param cost Cost function mapping a set of poses to a cost vector.
 * @param n Size of pose set that must be passed to the cost function.
 */
std::vector<Pose> Optimize(std::function<Eigen::VectorXd(std::vector<Pose>)> cost, size_t n);

}
