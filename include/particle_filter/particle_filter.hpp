#pragma once
#ifndef __INCLUDE_PARTICLE_FILTER__
#define __INCLUDE_PARTICLE_FILTER__
#include <Eigen/Dense>
#include <algorithm>
#include <iterator>
#include <memory>
#include <vector>

#include <random>
#include <iostream>

namespace particle_filter {

class MultivariateNormalDistribution {
 public:
  MultivariateNormalDistribution() : generator_(device_()) {}
  ~MultivariateNormalDistribution() {}

  void pdf(const Eigen::Matrix3d &covariance, const size_t size,
           Eigen::MatrixXd &target) {
    // eigenvalue decomposition to obtain sigmas and rotation matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
    if (eigensolver.info() != Eigen::Success)
      throw std::runtime_error("eigensolver error");
    Eigen::Array3d sigmas = eigensolver.eigenvalues().array().sqrt();
    Eigen::Matrix3d rotation_matrix = eigensolver.eigenvectors();

    // normal distribution within orthogonal axis
    Eigen::RowVectorXd random_scalars[3];
    single_axis_pdf(0, sigmas[0], size, random_scalars[0]);
    single_axis_pdf(0, sigmas[1], size, random_scalars[1]);
    single_axis_pdf(0, sigmas[2], size, random_scalars[2]);
    target = Eigen::MatrixXd(3, size);
    target << random_scalars[0],
              random_scalars[1],
              random_scalars[2];

    // transform
    target = rotation_matrix * target;
  }

 private:
  std::random_device device_;
  std::mt19937 generator_;

  void single_axis_pdf(const double &mu, const double &sigma, const size_t size,
                       Eigen::RowVectorXd &target) {
    std::normal_distribution<double> rand_trans_dist(mu, sigma);
    target = Eigen::RowVectorXd::Zero(1, size).unaryExpr(
        [&](float dummy) { return rand_trans_dist(generator_); });
    std::cout << target << std::endl;
  }

};


}  // namespace particle_filter

#endif  // __INCLUDE_PARTICLE_FILTER__
