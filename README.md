# 增强版激光雷达惯性里程计与建图系统
此项目参考fastlio2，scantext,gstm

这是一个基于 FAST-LIO2 的深度增强版本，专为 Livox 系列激光雷达（如 Mid-360, Avia）设计与优化。

相比原版，它不仅仅是一个“里程计”，更是一个完整的 SLAM 系统。您可以把它想象成一部电影制作团队：**前端负责拍摄素材（建图），后端负责剪辑拼接（回环优化），最后还能用来放映（重定位）。**

---

##  核心功能

本项目包含四大核心模块，为您提供从建图到应用的全流程解决方案：

- **前端建图 (LIO Mapping)**
    - 实时运行 FAST-LIO2 里程计，构建高精度地图。
    - **增强功能**：支持在线回环检测（Scan Context），支持将关键帧和描述子保存至硬盘（为离线优化做准备）。
    - **特色**：支持动态物体剔除（将移动的人/车从地图中剔除，使地图更干净、更静态）。

- **多阶段联合优化 (Multi-session Mapping)**
    - **离线“拼图”功能**。
    - **应用场景**：如果您昨天录了一段路，今天又录了一段路。这个功能可以自动检测两段路重合的地方，把它们拼成一张完整的、无漂移的全局大地图。

- **在线重定位 (Online Relocalization)**
    - **已知地图定位**。
    - **应用场景**：手里已经有一张做好的地图，机器人开机后，不再新建地图，而是通过雷达数据快速匹配，知道自己在地图的哪个角落。

- **动态消除 (Object-level Updating)**
    - **局部修补功能**。
    - **应用场景**：地图大部分没变，只有某个角落多了一辆车或少了一堵墙。这个功能可以只更新那一小块区域，而不是重跑整个地图，极大地提高了效率。

---

##  环境配置

> **警告：依赖版本非常关键，请严格遵守！**

- **操作系统**：Ubuntu 20.04 (ROS Noetic)
- **PCL & Eigen**：使用系统默认即可。
- **GTSAM**：**必须使用 4.0.3 稳定版**。
    - **不要使用** GitHub 上的最新 develop/4.2a 版本，否则会报 abstract class 编译错误。

### 安装 GTSAM 4.0.3
```bash
wget https://github.com/borglab/gtsam/archive/refs/tags/4.0.3.zip
unzip 4.0.3.zip && cd gtsam-4.0.3/build
cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j4 && sudo make install
```

Livox 驱动：安装 sdk2和livox_ros_driver2。
先安装编译好sdk2，然后将livox_ros_driver2下载到src里，和本项目代码同级。

编译流程
```bash
mkdir -p vilota_lidar_ws/src
cd ～/vilota_lidar_ws/src
git clone <本仓库地址>
git clone <livox_ros_driver2的地址>
```


关键：必须带上 ROS1 参数，否则 livox 驱动会报错
```bash
catkin_make -DROS_EDITION=ROS1
```

# 运行指南
场景一：单次建图 + 在线回环 (最常用)
目标：跑一个 Bag 包，实时消除漂移，得到完美的地图。
修改配置 (config/hap_livox.yaml):
yaml

```bash
loopClosureEnableFlag: true  # 开启回环
savePCD: true                # 保存结果
rootDir: "/home/li/data/result/"  # 设置保存路径
```

启动程序：
```bash
source devel/setup.bash
roslaunch fast_lio_sam mapping_hap_livox.launch
```

播放数据：
```
rosbag play your_data.bag
```
结束：播放完后按 Ctrl+C，等待地图保存完成。

场景二：多阶段联合优化 (Multi-session)
目标：把两段不同时间录制的 Bag (Session 01, Session 02) 拼在一起。
第一步：准备数据 (前端录制)
修改 config/hap_livox.yaml，关闭在线回环 (loopClosureEnableFlag: false)，开启 savePCD: true。
设置 rootDir: "/home/li/data/session_01/"，跑第一个包，保存。
设置 rootDir: "/home/li/data/session_02/"，跑第二个包，保存。

第二步：离线优化 (后端拼接)
修改 config/multi_session.yaml:
```bash
sessions_dir: "/home/li/data/"      # 数据总目录
central_sess_name: "session_01"     # 基准地图
query_sess_name: "session_02"       # 待拼接地图
save_directory: "merged_result/"    # 结果保存名
```

启动优化：
```
roslaunch fast_lio_sam multi_session.launch
```

查看结果：结果保存在 merged_result 文件夹中。使用 pcl_viewer 查看 aft_map.pcd。
场景三：在线重定位
目标：机器人加载这一张建好的地图，进行导航定位。
修改配置 (config/online_relocalization.yaml):
```
priorDir: "/home/li/data/session_01/"  # 指定先验地图路径
```

启动重定位节点：
```
roslaunch fast_lio_sam online_relocalization.launch
```

启动建图节点 (作为里程计输入)：
注意：此时建图节点的 savePCD 建议关闭，减少负载。
```
roslaunch fast_lio_sam mapping_hap_livox.launch
```
初始化：在 RViz 中使用 2D Pose Estimate 按钮给一个大致初始位置。

结果评估 (Evaluation)
运行多阶段优化或回环后，系统会生成轨迹文件 (LOG/pos_log.txt)。推荐使用 EVO 工具对比优化前后的轨迹效果。
安装 EVO
```
pip install evo --upgrade --no-binary evo
```

对比轨迹
```
～/.local/bin/evo_traj tum <(awk '{print  $ 1, $ 2, $ 3, $ 4, $ 5, $ 6, $ 7, $ 8}' LOG/pos_log.txt) --ref=ground_truth.txt -p --plot_mode=xyz
```
(注：请根据实际文件路径调整命令)
