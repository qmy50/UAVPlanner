# UAV Global Planner Based on STC and MINCO

# 基于安全走廊与MINCO的四旋翼动态避障路径规划器

## 一. 工作流程：

1.采用三维 A* 算法生成初始全局参考路径；

2.基于 DecompROS 库构建环境安全走廊，将其作为轨迹优化的硬约束

3.使用离散式 MiniSnap 方法结合 OSQP 求解器生成平滑航点，并将其作为初始值输入 MINCO 优化器，输出最终的平滑多项式轨迹

4.全程执行全局路径规划，仅利用深度相机当前视场内的障碍物信息进行持续在线更新，实现动态避障

5.集成 EGO-Planner 中的 SO3 控制器完成高精度轨迹跟踪控制

该方案适用于未知环境下的无人机自主导航、动态避障与平滑轨迹跟踪任务。

## 二. 运行：


```
git clone https://github.com/qmy50/UAVPlanner.git
或者
git clone https://gitee.com/qiu-mengyao114/UAVPlanner.git

catkin_make

source ./devel/setup.bash

roslaunch minco_curve test_my_fsm.launch

```
