cmake_minimum_required(VERSION 3.10)
project(minco_curve)

set(CMAKE_EXPORT_COMPILE_COMMANDS True)
set(CMAKE_BUILD_TYPE Release)

# Eigen
find_package(Eigen3 REQUIRED)
link_directories(${catkin_LIBRARY_DIRS})

# catkin 组件（移除 ompl）
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    tf
    tf2_ros
    decomp_ros_utils
    decomp_ros_msgs
    decomp_util
    plan_env
    path_searching 
    onboard_detector
)

# PCL
find_package(PCL REQUIRED)
find_package(OsqpEigen REQUIRED)

# 包含目录
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES PATH_SEARCHER_LIB
    CATKIN_DEPENDS nav_msgs roscpp tf decomp_ros_utils decomp_ros_msgs decomp_util
)

# 创建可执行文件
add_executable(fake_planner_node 
    src/fake_planner_node.cpp
    src/fake_planner_fsm.cpp
    src/plan_manager.cpp
    src/minisnap.cpp
    src/stc_gen_3D.cpp
    src/traj_optimizer_3D.cpp
    src/simple_dwa.cpp)

# 链接库（仅链接实际需要的）
target_link_libraries(fake_planner_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    OsqpEigen::OsqpEigen
)

add_executable(traj_server_node src/traj_server.cpp)

# 链接库（仅链接实际需要的）
target_link_libraries(traj_server_node
    ${catkin_LIBRARIES}
)

add_executable(fake_odom_node src/fake_odom.cpp)

# 链接库（仅链接实际需要的）
target_link_libraries(fake_odom_node
    ${catkin_LIBRARIES}
)


add_executable(test_node 
    src/test.cpp
    src/minisnap.cpp
    src/stc_gen_3D.cpp
    src/traj_optimizer_3D.cpp)

# 链接库（仅链接实际需要的）
target_link_libraries(test_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    OsqpEigen::OsqpEigen
)