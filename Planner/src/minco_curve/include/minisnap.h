#ifndef MINISNAP_H
#define MINISNAP_H

#include "utils/poly_traj_utils.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <vector>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

namespace minisnap {
    using namespace std;
    using Eigen::VectorXd;
    using Eigen::MatrixXd;
    using Eigen::Vector3d;
    using Eigen::Map;
    using poly_traj::CoefficientMat;
    using poly_traj::Trajectory;

    template<typename T>
    using VecE = std::vector<T, Eigen::aligned_allocator<T>>;

    VectorXd timeAllocation(const std::vector<Eigen::Vector3d>& Path,
                            double max_vel, double max_acc,
                            double start_vel, double end_vel);

    poly_traj::Trajectory convertMinijerkToTraj5(const Eigen::MatrixXd& coeffs_all,
                                                 const Eigen::VectorXd& ts);

    Eigen::Vector3d getPosPoly(MatrixXd polyCoeff, int k, double t);

    MatrixXd make_diff_matrix(int order, int N);

    std::vector<Eigen::Vector3d> minisnap_solver(
        const std::vector<Eigen::Vector3d>& centers,
        const VecE<Eigen::MatrixX4d>& hpolys,
        const std::vector<Eigen::Vector3d>& path,
        int N,
        double lambda_center = 2.5);

    std::pair<Vector3d, Vector3d> solveOneSegment(const Vector3d& A, const Vector3d& B,
                                                  const Eigen::MatrixX4d& hpoly,
                                                  const Vector3d* prev_P2);

    std::vector<std::array<Vector3d, 4>> buildSafeBezierPath(
        const std::vector<Vector3d>& waypoints,
        const VecE<Eigen::MatrixX4d>& hpolys);

    poly_traj::Trajectory convertMinijerkToTraj3(
        const std::vector<std::array<Vector3d, 4>>& bezier_control_points,
        const Eigen::VectorXd& ts);
}

#endif