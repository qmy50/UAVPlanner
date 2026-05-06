#include "minisnap.h"
#include <bits/stdc++.h>   // 保留原有依赖
#include <ros/ros.h>

namespace minisnap {

    // VectorXd timeAllocation(const std::vector<Eigen::Vector3d>& Path,
    //                         double max_vel, double max_acc) {
    //     int n_seg = Path.size() - 1;
    //     double total_dist = 0.0f;
    //     VectorXd time(n_seg);
        
    //     double t_acc = max_vel / max_acc;                     // 加速时间
    //     double d_acc = 0.5 * max_acc * t_acc * t_acc;         // 加速段距离
    //     double d_thresh = 2.0 * d_acc;                        // 临界距离
        
    //     for (int i = 0; i < n_seg; ++i) {
    //         double dist = (Path[i+1] - Path[i]).norm();
    //         total_dist += dist;
    //         if (dist < 1e-6) {
    //             time(i) = 0.0;
    //             continue;
    //         }
            
    //         if (dist <= d_thresh) {
    //             double v_peak = std::sqrt(max_acc * dist);
    //             time(i) = 2.0 * v_peak / max_acc;
    //         } else {
    //             double t_const = (dist - d_thresh) / max_vel;
    //             time(i) = 2.0 * t_acc + t_const;
    //         }
    //     }
    //     ROS_WARN("max speed is: %f",max_vel);
    //     ROS_WARN("total dist is: %f",total_dist);
    //     return time;
    // }
    VectorXd timeAllocation(const std::vector<Eigen::Vector3d>& Path,
                        double max_vel, double max_acc,
                        double start_vel, double end_vel) {
    int n_seg = Path.size() - 1;
    VectorXd time(n_seg);
    
    // 1. 计算各段长度及累积弧长
    std::vector<double> seg_len(n_seg);
    std::vector<double> cum_len(n_seg + 1, 0.0);
    double total_dist = 0.0;
    for (int i = 0; i < n_seg; ++i) {
        double dist = (Path[i+1] - Path[i]).norm();
        seg_len[i] = dist;
        total_dist += dist;
        cum_len[i+1] = total_dist;
    }
    
    // 如果总长度接近零，所有段时间为零
    if (total_dist < 1e-6) {
        time.setZero();
        return time;
    }
    
    // 参数有效性检查
    if (max_acc <= 1e-6 || max_vel <= 1e-6) {
        // 采用匀速模型（按长度比例分配时间）
        double total_time = total_dist / max_vel;
        for (int i = 0; i < n_seg; ++i) {
            time(i) = (seg_len[i] / total_dist) * total_time;
        }
        return time;
    }
    
    // 2. 梯形/三角形速度曲线规划
    double v_max_actual = max_vel;
    // 计算加速、减速所需距离（使用 std::max 防止负值）
    double d_acc = std::max(0.0, (v_max_actual * v_max_actual - start_vel * start_vel) / (2.0 * max_acc));
    double d_dec = std::max(0.0, (v_max_actual * v_max_actual - end_vel * end_vel) / (2.0 * max_acc));
    double d_const = total_dist - d_acc - d_dec;
    
    if (d_const < 0.0) {
        // 无法达到 max_vel，降低峰值速度
        // 解方程：total_dist = (v_peak^2 - start_vel^2)/(2a) + (v_peak^2 - end_vel^2)/(2a)
        double v_peak_sq = max_acc * total_dist + (start_vel * start_vel + end_vel * end_vel) / 2.0;
        if (v_peak_sq < 0.0) v_peak_sq = 0.0;
        v_max_actual = std::sqrt(v_peak_sq);
        // 重新计算距离
        d_acc = std::max(0.0, (v_max_actual * v_max_actual - start_vel * start_vel) / (2.0 * max_acc));
        d_dec = std::max(0.0, (v_max_actual * v_max_actual - end_vel * end_vel) / (2.0 * max_acc));
        d_const = 0.0;
        // 数值修正：确保 d_acc + d_dec 不超过总距离
        if (d_acc + d_dec > total_dist) {
            double scale = total_dist / (d_acc + d_dec);
            d_acc *= scale;
            d_dec *= scale;
        }
    }
    
    // 计算各阶段时间
    double t_acc = (v_max_actual - start_vel) / max_acc;
    double t_dec = (v_max_actual - end_vel) / max_acc;
    double t_const = d_const / v_max_actual;
    if (t_const < 0.0) t_const = 0.0;
    
    // 3. 弧长 → 时间映射函数（带数值保护）
    auto time_at_s = [&](double s) -> double {
        if (s <= 0.0) return 0.0;
        if (s >= total_dist) return t_acc + t_const + t_dec;
        
        // 加速段
        if (s <= d_acc) {
            // 解二次方程：0.5 * max_acc * t^2 + start_vel * t - s = 0
            double a = 0.5 * max_acc;
            double b = start_vel;
            double c = -s;
            double discriminant = b * b - 4.0 * a * c;
            if (discriminant < 0.0) discriminant = 0.0;
            double t = (-b + std::sqrt(discriminant)) / (2.0 * a);
            return std::max(0.0, t);
        }
        // 匀速段
        else if (s <= d_acc + d_const) {
            return t_acc + (s - d_acc) / v_max_actual;
        }
        // 减速段
        else {
            double s_dec = s - (d_acc + d_const);
            // 减速方程：s_dec = v_max_actual * t - 0.5 * max_acc * t^2
            // => 0.5 * max_acc * t^2 - v_max_actual * t + s_dec = 0
            double a = 0.5 * max_acc;
            double b = -v_max_actual;
            double c = s_dec;
            double discriminant = b * b - 4.0 * a * c;
            if (discriminant < 0.0) discriminant = 0.0;
            double t = (-b - std::sqrt(discriminant)) / (2.0 * a);
            return t_acc + t_const + std::max(0.0, t);
        }
    };
    
    // 4. 计算每段时间
    for (int i = 0; i < n_seg; ++i) {
        double s_start = cum_len[i];
        double s_end   = cum_len[i+1];
        double t_start = time_at_s(s_start);
        double t_end   = time_at_s(s_end);
        time(i) = std::max(0.0, t_end - t_start);
        if (seg_len[i] < 1e-6) time(i) = 0.0;
    }
    
    // 调试信息
    double total_time = time.sum();
    ROS_WARN("timeAllocation: start_vel=%.2f, end_vel=%.2f, v_max_actual=%.2f, total_dist=%.2f, total_time=%.2f",
             start_vel, end_vel, v_max_actual, total_dist, total_time);
    if (std::isnan(total_time)) {
        ROS_ERROR("timeAllocation produced NaN total_time! Using fallback.");
        // 保底匀速分配
        double total_time_fallback = total_dist / max_vel;
        for (int i = 0; i < n_seg; ++i) {
            time(i) = (seg_len[i] / total_dist) * total_time_fallback;
        }
    }
    
    return time;
}

    poly_traj::Trajectory convertMinijerkToTraj5(const Eigen::MatrixXd& coeffs_all,
                                                 const Eigen::VectorXd& ts) {
        int n_seg = coeffs_all.rows();
        int n_coeff = coeffs_all.cols() / 3;
        if (n_coeff != 6) {
            std::cerr << "Error: convertMinisnapToTraj5 expects 6 coefficients per dimension, got " << n_coeff << std::endl;
            return poly_traj::Trajectory();
        }
        std::vector<double> durs(n_seg);
        std::vector<poly_traj::CoefficientMat> cMats(n_seg);
        for (int i = 0; i < n_seg; ++i) {
            durs[i] = ts(i);
            poly_traj::CoefficientMat mat;
            Eigen::VectorXd cx = coeffs_all.row(i).segment(0, n_coeff);
            Eigen::VectorXd cy = coeffs_all.row(i).segment(n_coeff, n_coeff);
            Eigen::VectorXd cz = coeffs_all.row(i).segment(2*n_coeff, n_coeff);
            for (int k = 0; k <= 5; ++k) {
                mat(0, 5 - k) = cx(k);
                mat(1, 5 - k) = cy(k);
                mat(2, 5 - k) = cz(k);
            }
            cMats[i] = mat;
        }
        return poly_traj::Trajectory(durs, cMats);
    }

    Eigen::Vector3d getPosPoly(MatrixXd polyCoeff, int k, double t) {
        Eigen::Vector3d ret;
        int _poly_num1D = 6;
        for (int dim = 0; dim < 3; dim++) {
            VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
            VectorXd time = VectorXd::Zero(_poly_num1D);
            for (int j = 0; j < _poly_num1D; j++) {
                if (j == 0)
                    time(j) = 1.0;
                else
                    time(j) = pow(t, j);
            }
            ret(dim) = coeff.dot(time);
        }
        return ret;
    }

    MatrixXd make_diff_matrix(int order, int N) {
        if (order >= N) return MatrixXd::Zero(0, N);
        MatrixXd S = MatrixXd::Zero(N - order, N);
        std::vector<int> coeff = {1};
        for (int k = 0; k < order; ++k) {
            std::vector<int> nc(coeff.size() + 1);
            for (size_t i = 0; i < coeff.size(); ++i) {
                nc[i] += coeff[i];
                nc[i + 1] -= coeff[i];
            }
            coeff.swap(nc);
        }
        for (int i = 0; i < N - order; ++i) {
            for (size_t j = 0; j < coeff.size(); ++j)
                S(i, i + j) = coeff[j];
        }
        return S;
    }

    std::vector<Eigen::Vector3d> minisnap_solver(
        const std::vector<Eigen::Vector3d>& centers,
        const VecE<Eigen::MatrixX4d>& hpolys,
        const std::vector<Eigen::Vector3d>& path,
        int N,
        double lambda_center)
    {
        int dim = 3;
        int varN = N * dim;

        MatrixXd S;
        if (N >= 5)
            S = make_diff_matrix(4, N);
        else if (N >= 3)
            S = make_diff_matrix(2, N);
        else
            S = make_diff_matrix(1, N);
        MatrixXd STS;
        if (S.rows() > 0)
            STS = S.transpose() * S;
        else
            STS = MatrixXd::Zero(N, N);

        MatrixXd Qt = MatrixXd::Zero(varN, varN);
        VectorXd q = VectorXd::Zero(varN);
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                double v = STS(i, j);
                for (int d = 0; d < dim; ++d) {
                    int idx_i = i * dim + d;
                    int idx_j = j * dim + d;
                    Qt(idx_i, idx_j) += v;
                }
            }
            for (int d = 0; d < dim; ++d) {
                int idx = i * dim + d;
                Qt(idx, idx) += lambda_center;
                q(idx) += -2.0 * lambda_center * centers[i][d];
            }
        }
        MatrixXd P = 2.0 * Qt;

        std::vector<Eigen::Triplet<double>> tripletList;
        std::vector<double> lbVec, ubVec;
        int rowIdx = 0;

        auto add_eq = [&](int idx, double val) {
            tripletList.emplace_back(rowIdx, idx, 1.0);
            lbVec.push_back(val);
            ubVec.push_back(val);
            rowIdx++;
        };
        add_eq(0*dim+0, path[0].x());
        add_eq(0*dim+1, path[0].y());
        add_eq(0*dim+2, path[0].z());
        add_eq((N-1)*dim+0, path.back().x());
        add_eq((N-1)*dim+1, path.back().y());
        add_eq((N-1)*dim+2, path.back().z());

        int nPoly = hpolys.size();
        for (int i = 1; i < N-1; ++i) {
            int idx = i - 1;
            if (idx >= nPoly) break;
            const auto &hpoly = hpolys[idx];
            for (int k = 0; k < hpoly.rows(); ++k) {
                double a = hpoly(k,0), b = hpoly(k,1), c = hpoly(k,2), d = hpoly(k,3);
                tripletList.emplace_back(rowIdx, i*dim+0, a);
                tripletList.emplace_back(rowIdx, i*dim+1, b);
                tripletList.emplace_back(rowIdx, i*dim+2, c);
                lbVec.push_back(-1e20);
                ubVec.push_back(d + 1e-6);
                rowIdx++;
            }
        }

        int m = rowIdx;
        Eigen::SparseMatrix<double> A_sparse(m, varN);
        A_sparse.setFromTriplets(tripletList.begin(), tripletList.end());
        Eigen::VectorXd lb = Eigen::VectorXd::Map(lbVec.data(), m);
        Eigen::VectorXd ub = Eigen::VectorXd::Map(ubVec.data(), m);

        OsqpEigen::Solver solver;
        solver.data()->setNumberOfVariables(varN);
        solver.data()->setNumberOfConstraints(m);
        Eigen::SparseMatrix<double> P_sparse = P.sparseView();
        solver.data()->setHessianMatrix(P_sparse);
        solver.data()->setGradient(q);
        solver.data()->setLinearConstraintsMatrix(A_sparse);
        solver.data()->setLowerBound(lb);
        solver.data()->setUpperBound(ub);
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        if (!solver.initSolver() || solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            std::cerr << "OSQP solve failed!" << std::endl;
            return {};
        }
        Eigen::VectorXd sol = solver.getSolution();
        std::vector<Eigen::Vector3d> traj(N);
        for (int i = 0; i < N; ++i) {
            traj[i] = Eigen::Vector3d(sol(i * dim + 0), sol(i * dim + 1), sol(i * dim + 2));
        }
        return traj;
    }

    std::pair<Vector3d, Vector3d> solveOneSegment(const Vector3d& A, const Vector3d& B,
                                                  const Eigen::MatrixX4d& hpoly,
                                                  const Vector3d* prev_P2) {
        // 占位实现，实际应替换为真实逻辑
        return { A + (B-A)/3.0, A + 2*(B-A)/3.0 };
    }

    std::vector<std::array<Vector3d, 4>> buildSafeBezierPath(
        const std::vector<Vector3d>& waypoints,
        const VecE<Eigen::MatrixX4d>& hpolys)
    {
        int M = waypoints.size() - 1;
        std::vector<std::array<Vector3d, 4>> segments(M);
        Vector3d prev_P2;
        for (int i = 0; i < M; ++i) {
            const Vector3d& A = waypoints[i];
            const Vector3d& B = waypoints[i+1];
            const Eigen::MatrixX4d& hp = hpolys[i];
            if (i == 0) {
                auto [P1, P2] = solveOneSegment(A, B, hp, nullptr);
                segments[i] = {A, P1, P2, B};
                prev_P2 = P2;
            } else {
                auto [P1, P2] = solveOneSegment(A, B, hp, &prev_P2);
                segments[i] = {A, P1, P2, B};
                prev_P2 = P2;
            }
        }
        return segments;
    }

    poly_traj::Trajectory convertMinijerkToTraj3(
        const std::vector<std::array<Vector3d, 4>>& bezier_control_points,
        const Eigen::VectorXd& ts)
    {
        int n_seg = bezier_control_points.size();
        Eigen::Matrix<double,4,4> M_bezier2poly;
        M_bezier2poly << 1,  0,  0, 0,
                        -3,  3,  0, 0,
                        3, -6,  3, 0,
                        -1,  3, -3, 1;

        std::vector<double> durs(n_seg);
        std::vector<poly_traj::CoefficientMat> cMats(n_seg);

        for (int i = 0; i < n_seg; ++i) {
            double T = ts(i);
            durs[i] = T;

            const auto& seg = bezier_control_points[i];
            Eigen::Vector4d Px, Py, Pz;
            for (int k = 0; k < 4; ++k) {
                Px(k) = seg[k].x();
                Py(k) = seg[k].y();
                Pz(k) = seg[k].z();
            }

            Eigen::Vector4d coeff_x_lift = M_bezier2poly * Px;
            Eigen::Vector4d coeff_y_lift = M_bezier2poly * Py;
            Eigen::Vector4d coeff_z_lift = M_bezier2poly * Pz;

            double invT = 1.0 / T;
            double invT2 = invT * invT;
            double invT3 = invT2 * invT;

            poly_traj::CoefficientMat mat;
            mat.setZero();

            mat(0, 2) = coeff_x_lift(3) * invT3;
            mat(0, 3) = coeff_x_lift(2) * invT2;
            mat(0, 4) = coeff_x_lift(1) * invT;
            mat(0, 5) = coeff_x_lift(0);

            mat(1, 2) = coeff_y_lift(3) * invT3;
            mat(1, 3) = coeff_y_lift(2) * invT2;
            mat(1, 4) = coeff_y_lift(1) * invT;
            mat(1, 5) = coeff_y_lift(0);

            mat(2, 2) = coeff_z_lift(3) * invT3;
            mat(2, 3) = coeff_z_lift(2) * invT2;
            mat(2, 4) = coeff_z_lift(1) * invT;
            mat(2, 5) = coeff_z_lift(0);

            cMats[i] = mat;
        }

        return poly_traj::Trajectory(durs, cMats);
    }

} // namespace minisnap