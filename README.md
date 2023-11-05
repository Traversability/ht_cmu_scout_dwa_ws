> cmu 的局部规划方法 部署到了松灵scout车 （只是简单部署，参数没调）

# 使用

- 下载代码

  ```
  git@github.com:Traversability/ht_cmu_scout_dwa_ws.git
  ```

- 安装gazebo插件，如果没安装的话

  ```
  sudo apt-get install ros-{ros版本}-hector-gazebo-plugins
  ```

- 初始化工作空间 编译

  ```
  cd ht_cmu_scout_dwa_ws/src
  catkin_init_workspace
  cd ..
  catkin build #建议catkin build，catkin_make可能编译失败
  ```

- 下载CMU的仿真模型

  ```
  ./src/vehicle_simulator/mesh/download_environments.sh
  ```

- 编译

- 启动机器人和仿真地形

  ```
  source devel/setup.bash
  roslaunch scout_gazebo_sim scout_v2_playpen.launch
  ```

- 启动定位程序

  ```
  source devel/setup.bash
  roslaunch scout_gazebo_sim localization_example.launch
  ```

- 启动局部规划器

  ```
  source devel/setup.bash
  roslaunch vehicle_simulator system_scout.launch
  ```

- rviz 中选择目标点，局部规划执行

  ![](README.assets/2023-11-04%2011-42-04%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

# 参考资料

- CMU自主导航和探索框架官网：https://www.cmu-exploration.com/
- CMU局部规划论文
  - Maximum_Likelihood_Path_Planning_for_Fast_Aerial_Maneuvers_and_Collision_Avoidance
  - Falco Fast likelihood‐based collision avoidance with extension to human‐guided navigation