#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <math.h>
#include <random>

using namespace std;
using namespace Eigen;

ros::Publisher _all_map_pub;

int _obs_num, _cir_num;
double _x_size, _y_size, _z_size, _init_x, _init_y, _resolution, _sense_rate;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _w_c_l, _w_c_h;

bool _has_map  = false;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
vector<int>     pointIdxSearch;
vector<float>   pointSquaredDistance;      

void RandomMapGenerate()
{
    // 清空旧地图
    cloudMap.points.clear();

    // 定义两个圆柱的参数
    struct Cylinder {
        double cx, cy;   // 中心坐标 (x, y)
        double radius;   // 半径 (m)
        double height;   // 高度 (m)
    };
    vector<Cylinder> cylinders = {
        {0.0, -0.6, 0.62, 6.0},
        {0.3,  1.2, 0.52, 6.0}
    };

    for (const auto& cyl : cylinders) {
        // 计算圆柱在 XY 平面上的包围盒范围（按分辨率对齐）
        double min_x = cyl.cx - cyl.radius;
        double max_x = cyl.cx + cyl.radius;
        double min_y = cyl.cy - cyl.radius;
        double max_y = cyl.cy + cyl.radius;
        double min_z = 0.0;
        double max_z = cyl.height;

        // 按分辨率步长遍历网格点
        for (double x = min_x; x <= max_x + 1e-6; x += _resolution) {
            // 对齐到分辨率网格中心
            double grid_x = floor(x / _resolution) * _resolution + _resolution / 2.0;
            for (double y = min_y; y <= max_y + 1e-6; y += _resolution) {
                double grid_y = floor(y / _resolution) * _resolution + _resolution / 2.0;
                // 检查是否在圆柱的半径内
                double dx = grid_x - cyl.cx;
                double dy = grid_y - cyl.cy;
                if (dx*dx + dy*dy > cyl.radius * cyl.radius)
                    continue;

                // 在高度方向上按分辨率步长生成点
                for (double z = min_z; z <= max_z + 1e-6; z += _resolution * 0.5) {
                    pcl::PointXYZ pt;
                    pt.x = grid_x;
                    pt.y = grid_y;
                    pt.z = z + 0.001;   // 避免完全落在边界上（原代码风格）
                    cloudMap.points.push_back(pt);
                }
            }
        }
    }

    // 如果生成了点，则重建 kd-tree
    if (!cloudMap.points.empty()) {
        cloudMap.width = cloudMap.points.size();
        cloudMap.height = 1;
        cloudMap.is_dense = true;
        kdtreeMap.setInputCloud(cloudMap.makeShared());
    }

    _has_map = true;

    pcl::toROSMsg(cloudMap, globalMap_pcd);
    globalMap_pcd.header.frame_id = "world";
}

void pubSensedPoints()
{     
   if( !_has_map ) return;

   _all_map_pub.publish(globalMap_pcd);
}

int main (int argc, char** argv) 
{        
   ros::init (argc, argv, "random_complex_scene");
   ros::NodeHandle n( "~" );

   _all_map_pub   = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);                      

   n.param("init_state_x", _init_x,       0.0);
   n.param("init_state_y", _init_y,       0.0);

   n.param("map/x_size",  _x_size, 50.0);
   n.param("map/y_size",  _y_size, 50.0);
   n.param("map/z_size",  _z_size, 5.0 );

   n.param("map/obs_num",    _obs_num,  30);
   n.param("map/circle_num", _cir_num,  30);
   n.param("map/resolution", _resolution, 0.2);

   n.param("ObstacleShape/lower_rad", _w_l,   0.3);
   n.param("ObstacleShape/upper_rad", _w_h,   0.8);
   n.param("ObstacleShape/lower_hei", _h_l,   3.0);
   n.param("ObstacleShape/upper_hei", _h_h,   7.0);

   n.param("CircleShape/lower_circle_rad", _w_c_l, 0.3);
   n.param("CircleShape/upper_circle_rad", _w_c_h, 0.8);

   n.param("sensing/rate", _sense_rate, 1.0);

   _x_l = - _x_size / 2.0;
   _x_h = + _x_size / 2.0;

   _y_l = - _y_size / 2.0;
   _y_h = + _y_size / 2.0;

   RandomMapGenerate();
   ros::Rate loop_rate(_sense_rate);
   while (ros::ok())
   {
      pubSensedPoints();
      ros::spinOnce();
      loop_rate.sleep();
   }
}