# 增强版激光雷达惯性里程计与建图系统

此项目参考 fastlio2，scantext, gstm

这是一个基于 FAST-LIO2 的深度增强版本，专为 Livox 系列激光雷达（如 Mid-360, Avia）设计与优化。

**前端负责建图，后端负责剪辑拼接，前后端都可以开启回环优化，最后还能用来重定位。**

## 核心功能

本项目包含四大核心模块，提供从建图到应用的全流程解决方案：

### 前端建图 (LIO Mapping)
- 实时运行 FAST-LIO2 里程计，构建高精度地图。
- **增强功能**：支持在线回环检测（Scan Context），支持将关键帧和描述子保存至硬盘（为离线优化做准备）。
- **特色**：支持动态物体剔除（将移动的人/车从地图中剔除，使地图更干净、更静态）。

### 多阶段联合优化 (Multi-session Mapping)
- **离线"拼图"功能**。
- **应用场景**：如果您昨天录了一段路，今天又录了一段路。这个功能可以自动检测两段路重合的地方，把它们拼成一张完整的、无漂移的全局大地图。

### 在线重定位 (Online Relocalization)
- **已知地图定位**。
- **应用场景**：手里已经有一张做好的地图，机器人开机后，不再新建地图，而是通过雷达数据快速匹配，知道自己在地图的哪个角落。

### 动态消除 (Object-level Updating)
- **局部修补功能**。
- **应用场景**：地图大部分没变，只会消除动态物体或某个小物体

## 环境配置

- **操作系统**：Ubuntu 20.04 (ROS Noetic)
- **PCL & Eigen**：使用系统默认即可，编译时会有警告，但是只要编译成功就不影响
- **GTSAM**：**必须使用 4.0.3 稳定版**。
  - **不要使用** GitHub 上的最新 develop/4.2a 版本，否则会报 abstract class 编译错误。

### 1. 安装 GTSAM 4.0.3
```bash
cd ~
wget https://github.com/borglab/gtsam/archive/refs/tags/4.0.3.zip
unzip 4.0.3.zip && cd gtsam-4.0.3/build
cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j4 && sudo make install
```

### 2. 安装依赖
```bash
sudo apt update
sudo apt install libpcl-dev libeigen3-dev

sudo apt update
sudo apt install libgeographic-dev

sudo apt update
sudo apt install ros-noetic-darknet-ros-msgs
```

### 3. 安装 livox sdk2
```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 4. 准备工作空间
```bash
cd ~
mkdir -p vilota_lidar_ws/src
cd ~/vilota_lidar_ws/src
git clone https://github.com/RiseBun/lidar.git
```

```bash
cd ~/vilota_lidar_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

```bash
cd ~/vilota_lidar_ws/src/livox_ros_driver2
cp package_ROS1.xml package.xml
```

### 5. 编译
```bash
cd ~/vilota_lidar_ws
catkin_make -DROS_EDITION=ROS1
```

> 注：若电脑内存不够，可使用 `-j1`，由于版本问题终端会有警告，只要不报错就 ok

## 运行指南

### 场景一：单次建图 + 在线回环 (最常用)
**目标**：跑一个 Bag 包，实时消除漂移，得到完美的地图。

1. **修改配置** (`config/hap_livox.yaml`):
   ```yaml
   loopClosureEnableFlag: true  # 开启回环
   savePCD: true                # 保存结果
   rootDir: "/home/li/data/result/"  # 设置保存路径
   ```

2. **启动程序**：
   ```bash
   cd ~/vilota_lidar_ws
   source devel/setup.bash
   roslaunch fast_lio_sam mapping_hap_livox.launch
   ```

3. **播放数据**：
   ```bash
   rosbag play your_data.bag
   ```

4. **结束**：播放完后按 Ctrl+C，等待地图保存完成，注意保存路径。

### 场景二：多阶段联合优化 (Multi-session)
**目标**：把两段不同时间录制的 Bag (Session 01, Session 02) 拼在一起。

#### 第一步：准备数据 (前端录制)
- 修改 `config/hap_livox.yaml`，关闭在线回环 (`loopClosureEnableFlag: false`)，开启 `savePCD: true`。
- 设置 `rootDir: "/home/li/data/session_01/"`，跑第一个包，保存。
- 设置 `rootDir: "/home/li/data/session_02/"`，跑第二个包，保存。

#### 第二步：离线优化 (后端拼接)
1. **修改配置** (`config/multi_session.yaml`):
   ```yaml
   sessions_dir: "/home/li/data/"      # 数据总目录
   central_sess_name: "session_01"     # 基准地图
   query_sess_name: "session_02"       # 待拼接地图
   save_directory: "merged_result/"    # 结果保存名
   ```

2. **启动优化**：
   ```bash
   roslaunch fast_lio_sam multi_session.launch
   ```

3. **查看结果**：结果保存在 `merged_result` 文件夹中。使用 `pcl_viewer` 查看 `aft_map.pcd`。

### 场景三：在线重定位
**目标**：机器人加载这一张建好的地图，进行导航定位。

1. **修改配置** (`config/online_relocalization.yaml`):
   ```yaml
   priorDir: "/home/li/data/session_01/"  # 指定先验地图路径
   ```

2. **启动重定位节点**：
   ```bash
   roslaunch fast_lio_sam online_relocalization.launch
   ```

3. **启动建图节点** (作为里程计输入)：
   > 注意：此时建图节点的 `savePCD` 建议关闭，减少负载。
   ```bash
   roslaunch fast_lio_sam mapping_hap_livox.launch
   ```

4. **初始化**：在 RViz 中使用 2D Pose Estimate 按钮给一个大致初始位置。

## 结果评估 (Evaluation)

运行多阶段优化或回环后，系统会生成轨迹文件 (`LOG/pos_log.txt`)。推荐使用 EVO 工具对比优化前后的轨迹效果。

### 安装 EVO
```bash
pip install evo --upgrade --no-binary evo
```

### 对比轨迹
```bash
~/.local/bin/evo_traj tum <(awk '{print $1, $2, $3, $4, $5, $6, $7, $8}' LOG/pos_log.txt) --ref=ground_truth.txt -p --plot_mode=xyz
```

> 注：请根据实际文件路径调整命令