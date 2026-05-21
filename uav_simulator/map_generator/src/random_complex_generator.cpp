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
   random_device rd;
   default_random_engine eng(rd());

   uniform_real_distribution<double> rand_x = uniform_real_distribution<double>(_x_l, _x_h );
   uniform_real_distribution<double> rand_y = uniform_real_distribution<double>(_y_l, _y_h );
   uniform_real_distribution<double> rand_w = uniform_real_distribution<double>(_w_l, _w_h);
   uniform_real_distribution<double> rand_h = uniform_real_distribution<double>(_h_l, _h_h);

   pcl::PointXYZ pt_random;

   for(int i = 0; i < _obs_num; i ++)
   {
      double x, y, w, h;
      x    = rand_x(eng);
      y    = rand_y(eng);
      w    = rand_w(eng);

      // if(sqrt( pow(x - _init_x, 2) + pow(y - _init_y, 2) ) < 0.8 )
      //    continue;
      pcl::PointXYZ searchPoint(x, y, (_h_l + _h_h)/2.0);
      pointIdxSearch.clear();
      pointSquaredDistance.clear();

      bool need_skip = false;

      if(cloudMap.points.size() > 0)
      {
         if ( kdtreeMap.nearestKSearch(searchPoint, 1, pointIdxSearch, pointSquaredDistance) > 0 )
         {
            double min_distance = w + 0.6;
            if( sqrt(pointSquaredDistance[0]) < min_distance ){
               need_skip = true;
            }
         }
      }

      if(need_skip) continue;

      x = floor(x/_resolution) * _resolution + _resolution / 2.0;
      y = floor(y/_resolution) * _resolution + _resolution / 2.0;

      int widNum = ceil(w/_resolution);
      for(int r = -widNum/2.0; r < widNum/2.0; r ++ )
      {
         for(int s = -widNum/2.0; s < widNum/2.0; s ++ )
         {
            h    = rand_h(eng);
            int heiNum = 2.0 * ceil(h/_resolution);
            for(int t = 0; t < heiNum; t ++ ){
               pt_random.x = x + (r+0.0) * _resolution + 0.001;
               pt_random.y = y + (s+0.0) * _resolution + 0.001;
               pt_random.z = (t+0.0) * _resolution * 0.5 + 0.001;
               cloudMap.points.push_back( pt_random );
            }
         }
      }

      if(cloudMap.points.size() > 0)
      {
          kdtreeMap.setInputCloud(cloudMap.makeShared());
      }
   }

   cloudMap.width = cloudMap.points.size();
   cloudMap.height = 1;
   cloudMap.is_dense = true;

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