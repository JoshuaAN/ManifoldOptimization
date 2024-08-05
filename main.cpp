#include <iostream>
#include <tuple>
#include <functional>

#include "Eigen/Dense"
#include "Eigen/SparseCore"
#include "Eigen/SparseCholesky"

struct Pose {
  Eigen::Matrix3d R;
  Eigen::Vector3d T;
};

Eigen::VectorXd AlignVectors(Pose S) {
  Eigen::Vector3d u1;
  u1 << 0, 1, 0;
  Eigen::Vector3d v1;
  v1 << 1, 0, 0;

  Eigen::Vector3d u2;
  u2 << 1, 0, 0;
  Eigen::Vector3d v2;
  v2 << 0, -1, 0;

  Eigen::VectorXd cost(6);

  Eigen::VectorXd cost1 = S.R * v1 - u1 + S.T;
  Eigen::VectorXd cost2 = S.R * v2 - u2 + S.T;

  cost(0) = cost1(0);
  cost(1) = cost1(1);
  cost(2) = cost1(2);
  cost(3) = cost2(0);
  cost(4) = cost2(1);
  cost(5) = cost2(2);

  return cost;
}

double Quadratic(double A, double B, double C) {
  double discriminant = B * B - 4 * A * C;
  assert(discriminant >= 0);

  return (-B + sqrt(discriminant)) / (2 * A);
}

Eigen::SparseMatrix<double> SparseIdentity(int n) {
  Eigen::SparseMatrix<double> A(n, n);
  std::vector<Eigen::Triplet<double>> triplets;

  for (int i = 0; i < n; ++i) {
    triplets.emplace_back(i, i, 1);
  }

  A.setFromTriplets(triplets.begin(), triplets.end());
  
  return A;
}

Eigen::SparseMatrix<double> NumericalJacobian(std::function<Eigen::VectorXd(Eigen::VectorXd)> f, 
                                              Eigen::VectorXd x, 
                                              double step) {
  int rows = f(x).rows();
  int cols = x.rows();

  Eigen::SparseMatrix<double> J(rows, cols);

  for (int col = 0; col < cols; ++col) {
    /**
     * Five-point stencil https://en.wikipedia.org/wiki/Five-point_stencil
     */
    Eigen::VectorXd h = Eigen::VectorXd::Zero(x.rows());
    h(col) = step;

    Eigen::VectorXd fprime = (-f(x + 2 * h) + 8 * f(x + h) - 8 * f(x - h) + f(x + 2 * h)) / (12 * step);
    for (int row = 0; row < rows; ++row) {
      // if (fprime(row) != 0) {
        /**
         * TODO: improve efficiency by building matrix with set of triplets.
         */
        J.coeffRef(row, col) = fprime(row);
      // }
    }
  }

  return J;
}

Eigen::Matrix3d SkewSymmetric(Eigen::Vector3d w) {
  Eigen::Matrix3d w_hat;
  w_hat << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return w_hat;
}

Eigen::Matrix3d SkewSymmetricExponential(Eigen::Vector3d w) {
  /**
   * Computes the matrix exponential of the skew symmetric matrix of w.
   *
   * https://math.stackexchange.com/questions/879351/matrix-exponential-of-a-skew-symmetric-matrix-without-series-expansion
   */
  double x = w.norm();

  if (x == 0) {
    return Eigen::Matrix3d::Identity();
  }

  Eigen::Matrix3d C = SkewSymmetric(w);

  return Eigen::Matrix3d::Identity() + sin(x) / x * C + (1 - cos(x)) / (x * x) * C * C;
}
 
Pose Optimize(std::function<Eigen::VectorXd(Pose)> cost) {
  /**
   * Trust region step acceptance parameter η
   */
  double eta = 0.1;

  /**
   * Trust region size Δ
   */
  double delta = 1;

  /**
   * Pose in SE(3) which we will form the local approximation around.
   */
  Eigen::Matrix3d R0;
  Eigen::Vector3d T0;
  R0 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  T0 << 0, 0, 0;
  Pose S0 = Pose{R0, T0};

  while (cost(S0).norm() > 1e-6) {
    /**
     * Form a local paramterization mapping ℝ6 to SE(3) around S₀
     */
    auto parameterization = [S0](Eigen::Matrix<double, 6, 1> x) -> Pose {
      Eigen::Vector3d w = x.head(3);
      Eigen::Vector3d t = x.tail(3);

      return Pose{S0.R * SkewSymmetricExponential(w), S0.T + t};
    };

    /**
     * Function composition of cost function and paramterization.
     */
    auto cost_parameterization = [parameterization, cost](Eigen::Matrix<double, 6, 1> x) -> Eigen::VectorXd {
      return cost(parameterization(x));
    };

    /**
     * Local paramterization is centered around x = 0 ∈ ⁶ℝ
     */
    Eigen::VectorXd x = Eigen::Matrix<double, 6, 1>::Zero();

    Eigen::SparseMatrix<double> J = NumericalJacobian(cost_parameterization, 
                                                      x, 
                                                      1e-6);

    /**
     * Guass-Newton approximates the Hessian and gradient with:
     *
     *   H = Jᵀ(x)J(x)
     *   g = Jᵀ(x)f(x)
     */
    Eigen::SparseMatrix<double> H = J.transpose() * J;
    Eigen::VectorXd g = J.transpose() * cost_parameterization(x);

    
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver{H};

    /**
     * If LLᵀ factorization failed, regularize matrix and try again until success.
     */
    double beta = 1e-3;
    while (solver.info() != Eigen::Success) {
      solver.compute(J.transpose() * J + SparseIdentity(J.rows()) * beta);
      beta *= 10;
    }

    /**
     * Newton's method:
     *
     *  Δx = −H⁻¹g
     */
    Eigen::VectorXd p_b = solver.solve(-g);

    /**
     * Dogleg trust region method to ensure convergence.
     */
    Eigen::VectorXd p_u = -g.dot(g) / (g.dot(H * g)) * g;

    Eigen::VectorXd p;

    bool step_limit = false; // If step lies on the edge of the trust region.

    if (p_b.norm() <= delta) {
      p = p_b;
    } else if (p_u.norm() >= delta) {
      step_limit = true;
      p = p_u / p_u.norm() * delta;
    } else {
      step_limit = true;
      double tau = Quadratic((p_b - p_u).squaredNorm(), 
                              2 * (p_b - p_u).dot(p_u), 
                              p_u.squaredNorm() - delta * delta);
      p = (p_b - p_u) * tau + p_u;
    }

    double actual_reduction = cost_parameterization(p).norm() - cost_parameterization(x).norm();
    double pred_reduction = (J * p + cost_parameterization(x)).norm() - (cost_parameterization(x)).norm();
    double rho = actual_reduction / pred_reduction;

    if (rho < 0.25) {
      delta *= 0.25;
    } else {
      if (rho > 0.75 && step_limit) {
        delta *= 2;
      }
    }

    if (rho > eta) {
      /**
      * Update S0 from step
      */
      S0 = parameterization(p);
    }
  }

  return S0;
}
 
int main() {
  Pose S = Optimize(AlignVectors);

  std::cout << S.R << std::endl;
  std::cout << S.T << std::endl;

  // Print the Euler angles
  std::cout << "Euler angles (ZYX order):\n" << S.R.eulerAngles(2, 1, 0) << std::endl;

  return 0;
}