# UAV Global Planner Based on STC and MINCO

# 基于安全走廊与MINCO的四旋翼动态避障路径规划器

## 一. 工作流程：

1.采用三维 A* 算法生成初始全局参考路径；

2.基于 DecompROS 库构建环境安全走廊，将其作为轨迹优化的硬约束

3.使用离散式 MiniSnap 方法结合 OSQP 求解器生成平滑航点，并将其作为初始值输入 MINCO 优化器，输出最终的平滑多项式轨迹

4.全程执行全局路径规划，仅利用深度相机当前视场内的障碍物信息进行持续在线更新，实现动态避障

5.集成 EGO-Planner 中的 SO3 控制器完成高精度轨迹跟踪控制

该方案适用于未知环境下的无人机自主导航、动态避障与平滑轨迹跟踪任务。

## 二. Quick Start：

在rviz中进行纯数字仿真

克隆项目到本地
```
git clone https://github.com/qmy50/UAVPlanner.git
```
或者
```
git clone https://gitee.com/qiu-mengyao114/UAVPlanner.git
```
编译
```
catkin_make
```
```
source ./devel/setup.bash
```

运行rviz仿真
```
roslaunch minco_curve test_my_fsm.launch
```
<img width="400" height="225" alt="2026年05月07日 屏幕视频 16时31分57秒(1)" src="https://github.com/user-attachments/assets/ecdaf5c6-0460-4635-9a3d-63906787f3a3" />


仿真相关参数可以在test_my_fsm.launch, simulator.xml以及advance_param.xml文件中设置。 

支持在rviz中使用3D navgoal plugin手动设置航点，或使用launch文件中给定航点两种模式。

建议设置最大飞行速度在2m/s内。


## 三. 使用PX4进行在环仿真：

<img width="2425" height="1325" alt="a5bea344c9540a3221604caa8a0a6833" src="https://github.com/user-attachments/assets/5b70684a-d02f-4cea-8692-3245c28cfc32" />


这里需要配置好开源无人机仿真平台XTdrone。

开源链接为： https://gitee.com/robin_shaun/XTDrone

使用文档为： https://www.yuque.com/xtdrone/manual_cn

配置好后，运行
```
roslaunch px4 indoor1.launch
```

按照文档开启无人机offboard模式,起飞并悬停，推出键盘控制节点

开启使用gazebo位姿真值节点
```
cd ~/XTDrone/sensing/pose_ground_truth/
python get_local_pose.py iris 1
```
开启视角转换节点
```
cd ~/XTDrone/motion_planning/3d
python ego_transfer.py iris 0
```

开启规划器
```
roslaunch minco_curve run_in_XTdrone.launch
```

此时可以在rviz中使用3D navgoal设置目标点并在gazebo中看到无人机向目标点飞行

## 四. 使用MCP服务器控制无人机：

可以结合该规划器，通过配置mcp server 来使用大语言模型进行无人机控制。

关于ros的mcp server使用详见项目： https://github.com/robotmcp/ros-mcp-server

可以参考Planner/script/mcp_config.json进行mcp服务器配置，注意需要根据本地路径对文件进行修改

在安装好rosbridge后，运行开启rosbridge
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

并开启rviz仿真

```
roslaunch minco_curve test_my_fsm.launch
```
随后可以向大语言模型输入“控制无人机想最前方飞行10m”等命令来控制无人机前往目标点


## 四. Reference：
[1]. 规划器整体建构及轨迹跟踪与动态环境更新，a*搜索等模块参考/使用 ego planner 

链接为：https://github.com/ZJU-FAST-Lab/ego-planner

[2]. 三维安全走廊生成采用开源库DecompROS，链接为：[https://github.com/Photin1a/FlowCore](https://github.com/sikang/DecompROS)

[3]. 将STC与minco相结合参考了二维规划器FlowCore，链接为：https://github.com/Photin1a/FlowCore

[4]. 加入安全走廊硬约束的离散minisnap QP求解，参考链接为：https://github.com/Gerrylgr/trajectory_optimization

[5]. PX4仿真部分使用XTdroen，链接为： https://gitee.com/robin_shaun/XTDrone
