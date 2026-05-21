#ifndef _GRID_MAP_NEW_H
#define _GRID_MAP_NEW_H

#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <onboard_detector/utils.h>
#include <onboard_detector/GetDynamicObstacles.h>

#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;

// voxel hashing
template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

struct MappingParameters {
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  Eigen::Vector3i map_voxel_num_;
  Eigen::Vector3d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  string frame_id_;
  int pose_type_;

  double cx_, cy_, fx_, fy_;
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  bool use_depth_filter_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;
  double min_ray_length_, max_ray_length_;

  int local_map_margin_;
  double visualization_truncate_height_, virtual_ceil_height_, ground_height_;
  bool show_occ_time_;
  double unknown_flag_;
};

// 前置声明
struct MappingData;

class GridMap {
public:
  GridMap();
  ~GridMap();

  enum { POSE_STAMPED = 1, ODOMETRY = 2, INVALID_IDX = -10000 };

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  int toAddress(const Eigen::Vector3i& id);
  int toAddress(int& x, int& y, int& z);
  bool isInMap(const Eigen::Vector3d& pos);
  bool isInMap(const Eigen::Vector3i& idx);

  void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  void setOccupied(Eigen::Vector3d pos);
  int getOccupancy(Eigen::Vector3d pos);
  int getOccupancy(Eigen::Vector3i id);
  int getInflateOccupancy(Eigen::Vector3d pos);
  int getInflateOccupancy(Eigen::Vector3d pos ,const int& drone_id);

  void boundIndex(Eigen::Vector3i& id);
  bool isUnknown(const Eigen::Vector3i& id);
  bool isUnknown(const Eigen::Vector3d& pos);
  bool isKnownFree(const Eigen::Vector3i& id);
  bool isKnownOccupied(const Eigen::Vector3i& id);

  void initMap(ros::NodeHandle& nh);
  void publishMap();
  void publishMapInflate(bool all_info = false);
  void publishUnknown();

  bool hasDepthObservation();
  bool odomValid();
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  double getResolution();
  Eigen::Vector3d getOrigin();
  int getVoxelNum();

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  std::unique_ptr<MappingData> md_;

  // 回调函数
  void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                         const geometry_msgs::PoseStampedConstPtr& pose);
  void depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);

  void updateOccupancyCallback(const ros::TimerEvent& /*event*/);
  void visCallback(const ros::TimerEvent& /*event*/);

  void projectDepthImage();
  void raycastProcess();
  void clearAndInflateLocalMap();

  void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);

  // 同步策略
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  ros::NodeHandle node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;

  ros::Subscriber indep_cloud_sub_, indep_odom_sub_;
  ros::Publisher map_pub_, map_inf_pub_;
  ros::Publisher unknown_pub_;
  ros::Timer occ_timer_, vis_timer_;

  //动态障碍物相关
  ros::ServiceClient dynamic_obs_client_;
  onboard_detector::GetDynamicObstacles srv_;
  std::vector<onboardDetector::box3D> dynamic_obstacles_;
  std::mutex dynamic_obs_mutex_;
  ros::Timer dynamic_obs_timer_;
  ros::Publisher predict_obs_pub_;
  ros::Time last_dynamic_obs_time_;
  double dynamic_obs_timeout_;   
  ros::Publisher dynamic_obs_flag_pub_;
  bool use_dwa_;

  void dynamicObstacleTimerCallback(const ros::TimerEvent&);
  bool isPointInDynamicObstacle(const Eigen::Vector3d& pt);
  void clearDynamicObstaclesFromMap();
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;


  // swarm避障相关
  std::unordered_map<int, std::unordered_set<int>> temp_obs_by_id_; // key: drone_id, value: 该无人机产生的体素地址集合
  mutable std::mutex temp_obs_mutex_;
  double temp_obs_radius_;
  ros::Publisher temp_obs_pub_;
    
public:
  void addTemporaryObstacles(int drone_id, const Eigen::MatrixXd &points, double radius);
  void clearTemporaryObstacles(int drone_id);
  void clearAllTemporaryObstacles();
  std::unordered_set<int> getAllTempObsAddresses() const;
  void publishTempObstacles(const ros::Publisher& pub, const std::string& frame_id, const ros::Time& stamp);
  Eigen::Vector3i addressToIndex(int addr) const;

  std::vector<Eigen::Vector3d> getPredictedObs(double predict_time , int step);


};

#endif