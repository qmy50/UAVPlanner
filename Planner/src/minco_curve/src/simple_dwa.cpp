#include "simple_dwa.h"

void DWAController::setupDWA(ros::NodeHandle &nh, const GridMap::Ptr &map){
    node_ = nh;
    grid_map_ = map;
    has_map_ = true;
    nh.param("dwa/v_max", v_max, 1.2);
    nh.param("dwa/v_min", v_min, -0.2);
    nh.param("dwa/w_max", w_max, 0.8);
    nh.param("dwa/w_min", w_min, -0.8);
    nh.param("dwa/v_acc", v_acc, 1.0);
    nh.param("dwa/w_acc", w_acc, 1.5);
    nh.param("dwa/v_res", v_res, 0.05);
    nh.param("dwa/w_res", w_res, 0.05);
    nh.param("dwa/dt", dt, 0.05);
    nh.param("dwa/predict_time", predict_time, 0.6);
    nh.param("dwa/safe_radius", safe_radius, 0.20);
    nh.param("dwa/startup_safe_radius", startup_safe_radius, 0.50);
    nh.param("dwa/startup_flag", startup_flag, true);
    nh.param("dwa/alpha", alpha, 0.5);
    nh.param("dwa/beta", beta, 0.4);
    nh.param("dwa/gamma", gamma, 0.8);
    nh.param("dwa/deadlock_threshold", deadlock_threshold, 20);
    nh.param("dwa/lookahead_dist", lookahead_dist_, 1.5);
    deadlock_count = 0;
}

DWAController::Traj DWAController::predictTrajectory(const std::vector<double>& pose, double v, double w) {
    std::vector<Eigen::Vector3d> traj;
    double x = pose[0], y = pose[1], yaw = pose[2], init_z = pose[3];
    double time = 0.0;
    while (time <= predict_time) {
        x += v * cos(yaw) * dt;
        y += v * sin(yaw) * dt;
        yaw += w * dt;
        traj.push_back({x, y, init_z});
        time += dt;
    }

    return {traj,yaw};
}

double DWAController::evaluateTrajectory(const Traj& _traj,const Eigen::Vector3d& target, double v, double safe_radius, std::vector<Eigen::Vector3d> predicted_obs) {
    if(!has_map_){
        ROS_WARN("We do not have the map !");
        return -std::numeric_limits<double>::infinity();
    }

    const auto& end_pose = _traj.traj.back();
    // 朝向得分
    double dx = target[0] - end_pose[0];
    double dy = target[1] - end_pose[1];
    double target_angle = atan2(dy, dx);
    double angle_diff = atan2(sin(target_angle - _traj.final_yaw), cos(target_angle - _traj.final_yaw));
    double heading_score = (M_PI - fabs(angle_diff)) / M_PI;

    // 静态障碍物硬约束
    for(const auto&traj_seg : _traj.traj){
        if(grid_map_->getInflateOccupancy(traj_seg) == 1){
            ROS_WARN("Traj get into the static obs !");
            return -std::numeric_limits<double>::infinity();
        };
    }

    // 动态障碍物惩罚
    double dynamic_penalty = 0.0;

    if (!predicted_obs.empty()) {
        double min_future_dist = std::numeric_limits<double>::max();
        for (const auto&traj_seg : _traj.traj) {
            for (const auto& ghost : predicted_obs) {
                double dist = hypot(traj_seg[0] - ghost[0], traj_seg[1] - ghost[1]);
                if (dist < min_future_dist) min_future_dist = dist;
            }
        }
        if (min_future_dist < safe_radius && fabs(v) > 0.05) {
            return -std::numeric_limits<double>::infinity();
        } else if (min_future_dist < 2.0) {
            dynamic_penalty = (2.0 - min_future_dist) / 2.0 * 4.0;
        }
    }

    double vel_score = std::max(0.0, v) / v_max;
    double total_score = alpha * heading_score + gamma * vel_score - dynamic_penalty;
    return total_score;
}

    DWAController::Velocity DWAController::calculateBestVelocity(const std::vector<double>& pose, double v_c, double w_c,
                                const Eigen::Vector3d& target,
                                double dynamic_safe_radius) {
    // 速度窗口
    double v_min_win = std::max(v_min, v_c - v_acc * dt);
    double v_max_win = std::min(v_max, v_c + v_acc * dt);
    double w_min_win = std::max(w_min, w_c - w_acc * dt);
    double w_max_win = std::min(w_max, w_c + w_acc * dt);

    // 平滑限速（根据朝向偏差）
    double dx = target[0] - pose[0];
    double dy = target[1] - pose[1];
    double target_angle = atan2(dy, dx);
    double angle_diff = fabs(atan2(sin(target_angle - pose[2]), cos(target_angle - pose[2])));
    if (angle_diff > 0.5) {
        double ratio = (angle_diff - 0.5) / (M_PI - 0.5);
        double limit_v = 0.6 - ratio * 0.45;
        if (limit_v < v_max_win) v_max_win = limit_v;
    }

    double best_v = 0.0, best_w = 0.0;
    double max_score = -std::numeric_limits<double>::infinity();
    double current_safe_radius = startup_flag ? startup_safe_radius : dynamic_safe_radius;

    double predict_time = 1.0;
    int steps = 15;
    std::vector<Eigen::Vector3d> predicted_obs = grid_map_->getPredictedObs(predict_time,steps);

    for (double v = v_min_win; v <= v_max_win + 0.001; v += v_res) {
        for (double w = w_min_win; w <= w_max_win + 0.001; w += w_res) {
            auto traj = predictTrajectory(pose, v, w);
            ROS_WARN("WE get the traj");
            double score = evaluateTrajectory(traj, target, v, current_safe_radius,predicted_obs);
            ROS_WARN("The score is %f",score);
            if (score > max_score) {
                max_score = score;
                best_v = v;
                best_w = w;
            }
        }
    }

    // 死锁处理
    if (fabs(best_v) < 0.05 && fabs(best_w) > 0.6) {
        deadlock_count++;
    } else {
        deadlock_count = 0;
    }

    if (max_score == -std::numeric_limits<double>::infinity() || deadlock_count >= deadlock_threshold) {
        ROS_WARN("[DWA] Deadlock! Reversing...");
        ROS_WARN("MAX_SCORE %f",max_score);
        deadlock_count = 0;
        return {-0.15, 0.0};
    }

    if (best_v > 0.1) startup_flag = false;
    return {best_v, best_w};
}