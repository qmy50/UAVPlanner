#include <ros/ros.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_geometry/polyhedron.h>
#include <decomp_geometry/geometric_utils.h>

#include <vector>
#include <iostream>

using namespace std;

// 生成一个立方体多面体
Polyhedron3D createCubePolyhedron(double x_min, double x_max, 
                                   double y_min, double y_max,
                                   double z_min, double z_max) {
    Polyhedron3D poly;
    
    // 添加6个平面（立方体的6个面）
    // 法向量指向外部
    
    // 前面 (x = x_max)
    poly.add(Hyperplane3D(Vec3f(x_max, 0, 0), Vec3f(1, 0, 0)));
    // 后面 (x = x_min)
    poly.add(Hyperplane3D(Vec3f(x_min, 0, 0), Vec3f(-1, 0, 0)));
    // 右面 (y = y_max)
    poly.add(Hyperplane3D(Vec3f(0, y_max, 0), Vec3f(0, 1, 0)));
    // 左面 (y = y_min)
    poly.add(Hyperplane3D(Vec3f(0, y_min, 0), Vec3f(0, -1, 0)));
    // 上面 (z = z_max)
    poly.add(Hyperplane3D(Vec3f(0, 0, z_max), Vec3f(0, 0, 1)));
    // 下面 (z = z_min)
    poly.add(Hyperplane3D(Vec3f(0, 0, z_min), Vec3f(0, 0, -1)));
    
    return poly;
}

// 生成一个棱柱多面体（三角柱）
Polyhedron3D createPrismPolyhedron() {
    Polyhedron3D poly;
    
    // 底面三角形顶点
    Vec3f p0(0, 0, 0);
    Vec3f p1(1, 0, 0);
    Vec3f p2(0.5, 0.866, 0);
    
    // 顶面三角形顶点
    Vec3f p3(0, 0, 1);
    Vec3f p4(1, 0, 1);
    Vec3f p5(0.5, 0.866, 1);
    
    // 计算每个面的法向量
    // 底面 (z = 0)
    poly.add(Hyperplane3D(Vec3f(0, 0, 0), Vec3f(0, 0, -1)));
    
    // 顶面 (z = 1)
    poly.add(Hyperplane3D(Vec3f(0, 0, 1), Vec3f(0, 0, 1)));
    
    // 三个侧面
    // 侧面1: (p0, p1, p4, p3)
    Vec3f n1 = (p1 - p0).cross(p3 - p0);
    n1.normalize();
    poly.add(Hyperplane3D(p0, n1));
    
    // 侧面2: (p1, p2, p5, p4)
    Vec3f n2 = (p2 - p1).cross(p4 - p1);
    n2.normalize();
    poly.add(Hyperplane3D(p1, n2));
    
    // 侧面3: (p2, p0, p3, p5)
    Vec3f n3 = (p0 - p2).cross(p5 - p2);
    n3.normalize();
    poly.add(Hyperplane3D(p2, n3));
    
    return poly;
}

// 平移多面体
Polyhedron3D translatePolyhedron(const Polyhedron3D& poly, const Vec3f& offset) {
    Polyhedron3D result;
    for (const auto& hp : poly.hyperplanes()) {
        result.add(Hyperplane3D(hp.p_ + offset, hp.n_));
    }
    return result;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_polyhedron_visualization");
    ros::NodeHandle nh;
    
    ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("/test_polyhedron", 1);
    
    ros::Rate rate(0.5);  // 0.5Hz，每2秒发布一次
    
    ROS_INFO("Test polyhedron visualization started. Publishing to /test_polyhedron");
    ROS_INFO("Please add PolyhedronArray display in RViz and set topic to /test_polyhedron");
    
    while (ros::ok()) {
        // 创建多面体数组
        decomp_ros_msgs::PolyhedronArray poly_array;
        poly_array.header.frame_id = "map";
        poly_array.header.stamp = ros::Time::now();
        
        // ========== 测试1: 立方体 ==========
        Polyhedron3D cube = createCubePolyhedron(-2, 2, -1.5, 1.5, -1, 1);
        poly_array.polyhedrons.push_back(DecompROS::polyhedron_to_ros(cube));
        ROS_INFO("Added cube: x∈[-2,2], y∈[-1.5,1.5], z∈[-1,1]");
        
        // ========== 测试2: 棱柱（平移后添加） ==========
        Polyhedron3D prism = createPrismPolyhedron();
        prism = translatePolyhedron(prism, Vec3f(3, 0, 0));  // 平移避免重叠
        poly_array.polyhedrons.push_back(DecompROS::polyhedron_to_ros(prism));
        ROS_INFO("Added prism at x≈3");
        
        // ========== 测试3: 更小的立方体 ==========
        Polyhedron3D small_cube = createCubePolyhedron(-1, 1, -1, 1, -1, 1);
        small_cube = translatePolyhedron(small_cube, Vec3f(-3, 0, 0));
        poly_array.polyhedrons.push_back(DecompROS::polyhedron_to_ros(small_cube));
        ROS_INFO("Added small cube at x≈-3");
        
        // 发布
        poly_pub.publish(poly_array);
        ROS_INFO("Published %zu polyhedrons", poly_array.polyhedrons.size());
        
        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}