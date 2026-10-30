# 本机 ROS1 / ROS2 / bridge 环境

当前部署入口、完整启动方式、在线/离线边界与未提交 Git 分析，见 [DEPLOYMENT.md](DEPLOYMENT.md)（2026-10-30）。本文件继续保留环境细节和历史实验记录。

最新的阈值选择、EMA 受控对照和 RViz 网络输出说明见 [TUNING.md](TUNING.md)。

宿主机工程根目录为 `/mnt/nvme1n1/whd`，三个容器内保持相同绝对路径。镜像提供系统和依赖，FAST-LIO-SAM、EMCP、UAM2TE 和 bridge 源码都来自宿主机。Python 修改直接生效；C++、消息定义和编译配置修改后需要重编译相应工作空间。

| 容器 / 镜像 | 基础环境 | 用途 |
| --- | --- | --- |
| `ros1-noetic` / `robotics/noetic-cuda:20261028` | `emcupy:blackwell-20261026`，Ubuntu 20.04 / Noetic；从 `rellis-builder-runtime:20261027` 引入 GTSAM SDK | rosbag、FAST-LIO-SAM、UAM2TE、EMCP；CUDA 12.8 nvcc、Python 3.10 / Torch 2.7.1 cu128 / CuPy 13.6 |
| `ros2-jazzy` / `robotics/jazzy-cuda:20261028` | `osrf/ros:jazzy-desktop-full`，Ubuntu 24.04；CUDA 来自 `nvidia/cuda:12.8.1-cudnn-devel-ubuntu24.04` | Nav2；Python 3.12 / Torch 2.10 cu128 / CuPy 13.6 |
| `ros1-ros2-bridge` / `robotics/bridge-jazzy:20261028` | Jazzy Desktop Full + 上游方案的 Noble Noetic PPA | ROS1/ROS2 消息转换，无 GPU 依赖 |

ROS1 的系统 Python 3.8 用于 `rospy` / `cv_bridge`；EMCP C++ 嵌入独立 Python 3.10。当前 UAM2TE 模型使用宿主机 `.venv` 的 Python 3.12，通过本机 HTTP 与 ROS 图像节点交换结果，可以直接在 Noetic 容器内运行。无需继续依赖原来的 `torch` 容器。

UAM2TE 已把 PointPillars 导入移到实际创建该点云编码器的分支，避免纯图像推理加载宿主机上为新 glibc 编译的旧扩展。需要 PointPillars 的模型仍须在目标系统中重编译其扩展；不能直接复用 Ubuntu 24.04 的 `.so` 到 Noetic / Focal。

Jazzy 镜像从宿主机 `uam2te/.venv/lib/python3.12/site-packages` 获取已有 GPU 依赖快照，输入版本记录在 `gpu-dependencies.lock`，随后固定 NumPy 1.26.4、SciPy 1.13.1、OpenCV 4.10 与 CuPy 13.6，兼容 ROS 的 NumPy ABI。项目代码和 checkpoint 不复制进镜像。Noetic 两个基础镜像为本机已有镜像，换机器时需迁移或重建它们。

## 构建与进入

宿主机执行：

```bash
cd /mnt/nvme1n1/whd/ros-jazzy-ros1-bridge-builder
git submodule update --init --recursive
bash runtime/prepare_bridge_source.sh
# 如果宿主机还没有该包，先获取相机矫正源码：
test -d ../calibrator || git clone https://github.com/Hyper-Devil/calibrator.git ../calibrator
docker compose -f runtime/compose.yaml build
docker compose -f runtime/compose.yaml up -d
docker exec ros1-noetic bash /mnt/nvme1n1/whd/ros-jazzy-ros1-bridge-builder/runtime/build_noetic.sh
docker exec ros1-ros2-bridge bash /mnt/nvme1n1/whd/ros-jazzy-ros1-bridge-builder/runtime/build_bridge.sh
```

后续只需 `up -d` 和按需重编译。`bash runtime/enter.sh noetic`、`jazzy` 或 `bridge` 打开已 source 对应环境的终端。三个容器默认空闲，不自动启动 ROS Master、导航或 bridge。

| 工作空间 | 宿主机来源 |
| --- | --- |
| `.runtime/noetic` | `stepp_adapter/third_party/FAST_LIO_SAM/FAST_LIO_SAM`、`emcp/src/elevation_mapping_cupy`、`uam2te/scripts/ros_workspace/src/ros_interface`、`traversability_dataset_builder`、`calibrator` 的符号链接 |
| `.runtime/messages-ros1` / `messages-ros2` | 本仓库 `custom_msgs/` 子模块，当前 revision `d41b98c28667165483eb7d8ed725f0e8bdf9b296` |
| `.runtime/bridge` | 本仓库下 `ros1_bridge/`，固定 revision `8f1b7e903e1fb4462218db659bb4981228c12206` |

FAST-LIO-SAM 的 Livox 消息依赖暂时链接到宿主机现有 `rellis_builder_runs/20261027/ws/src/livox_ros_driver2`。此包用于编译，不要求回放 Livox 数据。`.runtime` 内各环境编译产物互不复用；bridge 的 ROS1 消息是在 Ubuntu 24.04 编译，不能作为 Focal 的 catkin overlay。

## 回放启动顺序

下列命令每组在独立终端执行。宿主机先进入本仓库，执行 `bash runtime/enter.sh noetic` 后可直接运行 Noetic 命令。

1. **ROS1 Master**：`roscore -p 11671`。另一个 Noetic 终端执行 `rosparam set /use_sim_time true`。
2. **ROS1 推理 worker**：Noetic 终端执行以下命令，等待 ready。

   ```bash
   bash /mnt/nvme1n1/whd/ros-jazzy-ros1-bridge-builder/runtime/run_model.sh \
     --config configs/experiments/speed_response/tasks_20261020/curve_app_s_s42_20261020.yaml \
     --checkpoint checkpoints/speed_curve_matrix_20261020/curve_app_s_s42_20261020/20261021-015701_curve_app_s_s42_20261020_best-epoch-001-valid_curve_normalized_mae-0.0578.ckpt \
     --reference-speed 0.8
   ```

3. **ROS1 SLAM 和标定**：例如 RELLIS 执行 `roslaunch fast_lio_sam mapping_rellis3d.launch use_sim_time:=true rviz:=false`，并另载该序列的相机内外参。BISTU 使用 `mapping_rs_bistu.launch`，自动启动已加入工作空间的 `calibrator` 包。
4. **ROS1 推理适配和地图融合**：按平台设置图像/CameraInfo 话题和 TF，例如 RELLIS：

   ```bash
   roslaunch uam2te_msgs navigation.launch \
     reference_speed:=0.8 use_sim_time:=true \
     image_topic:=/pylon_camera_node/image_raw \
     camera_info_topic:=/pylon_camera_node/camera_info \
     map_frame:=camera_init base_frame:=base_link
   ```

   CameraInfo 话题必须对应实际标定发布器。更多建图、CVaR 和保存参数见宿主机 `uam2te/scripts/ros_workspace/src/ros_interface/readme.md`。
5. **Bridge**：宿主机执行 `docker exec -it ros1-ros2-bridge bash /mnt/nvme1n1/whd/ros-jazzy-ros1-bridge-builder/runtime/run_bridge.sh`。
6. **ROS2 Nav2**：在宿主机执行 `bash runtime/enter.sh jazzy`，使用机器人的 Nav2 launch 和参数，设置 `use_sim_time:=true`。Nav2 已安装；机器人 footprint、控制器、传感器源和地图坐标系需要机器人配置。当前阶段没有用某个移动 bag 验证完整 Nav2 导航。
7. **ROS1 bag**：最后在 Noetic 终端 `rosbag play --clock --pause /absolute/path/recording.bag --topics ...`，按实际 bag 指定 LiDAR、IMU、图像和 CameraInfo，检查后按空格播放。使用 FAST-LIO-SAM 提供 TF 时，不要同时回放 bag 内冲突的 `/tf` / `/tf_static`。不要循环播放而继续累积同一份 EMA 历史。

模型 worker 与建图 launch 的 `reference_speed` 必须一致。RELLIS 当前 SLAM 的全局坐标为 `camera_init`，Nav2 应保持一致，或明确增加 `map` 到 `camera_init` 的变换；不要假设它自动发布了 `map -> odom`。

## 桥接边界与性能

三个容器均使用 host network 和 host IPC。ROS Master 为 `127.0.0.1:11671`，DDS domain 为 71，发现范围设为本机，避免混入现有 ROS 任务。当前设置用于同机部署；接入局域网传感器 / 远程 ROS 节点时，需统一调整 ROS_IP、Master 和 DDS 发现范围。

`topics.yaml` 是 `parameter_bridge` 的话题白名单，默认包含 `/clock`、`/tf`、`/tf_static`、`/Odometry`、`/similarity_costmap` 和 `/cmd_vel`。此版本为白名单内话题建立双向桥，不能把 YAML 理解为强制单向路由。避免同一话题在两端存在两个独立数据源。

图像、原始点云、EMCP GridMap 默认留在 ROS1。需要显示或算法消费时再显式增加跨桥话题。bridge 会执行类型转换和序列化；host IPC 允许 DDS 使用共享内存，不能消除 ROS1/ROS2 转换开销。`tf_static` 和地图使用 transient-local，传感器基准使用 best-effort / depth 1，降低排队延迟。

`/similarity_costmap` 为连续风险 `0..100`、未知 `-1` 的 OccupancyGrid。Nav2 StaticLayer 若要保留连续风险，应配置 `map_topic: /similarity_costmap`、`map_subscribe_transient_local: true`、`trinary_costmap: false`，并保留未知区语义；机器人 footprint、膨胀和障碍物层需另外配置。该风险图不是完整的碰撞检测配置。

## UDP / DDS 参数

2026-10-28 检查到宿主机 `rmem_max/wmem_max` 为 212992 字节，`ipfrag_high_thresh` 为 4 MiB、`ipfrag_time` 为 30 秒。`/proc/net/snmp` 中的 RcvbufErrors 为历史累计值，判断当前丢包应比较测试前后增量。

`fastdds.xml` 请求 16 MiB 收发 socket 缓冲；`network.conf` 建议把宿主机缓冲上限提高到 64 MiB，分片重组上限 128 MiB，超时 3 秒。数值作为测试起点。只改容器 XML 而不提高宿主机内核上限，socket 请求仍可能被限制。host network 下这组 sysctl 在宿主机统一设置。

```bash
python3 runtime/tune_network.py
# 需要宿主机 root；脚本先保存原值，然后临时修改四项参数。
sudo python3 runtime/tune_network.py --apply --output .runtime/network-backup.json
# 恢复原值：
sudo python3 runtime/tune_network.py --restore .runtime/network-backup.json
```

当前没有自动修改宿主机 sysctl。确认收益后可由管理员将 `network.conf` 安装到 `/etc/sysctl.d/` 持久化。参考 [ROS2 官方 DDS tuning 文档](https://github.com/ros2/ros2_documentation/blob/jazzy/source/How-To-Guides/DDS-tuning.rst)。

## 验证

GPU 实际运算检查（在对应容器已 source GPU 环境后）：`python /mnt/nvme1n1/whd/ros-jazzy-ros1-bridge-builder/runtime/check_gpu.py`，覆盖 Torch 矩阵运算、CuPy JIT 内核和 nvcc。

通信基准在宿主机运行，测试期间要求端口 11671 没有其他 Master / bridge 使用：

```bash
python3 runtime/benchmark.py --output .runtime/benchmark-local --duration 10
python3 runtime/benchmark.py --udp-only --output .runtime/benchmark-udp-baseline --duration 10
python3 runtime/benchmark.py --check-bridge --output .runtime/bridge-smoke
```

每次测试 2.76 MB Image × 30 Hz 和 1.6 MB PointCloud2 × 10 Hz，分别测试两个方向，记录收发数、延迟、带宽和容器 CPU。默认保留 Fast DDS 的本机传输选择；`--udp-only` 用于观察 UDP 传输和缓冲瓶颈。测试只清理自己启动的进程，容器保持运行。合成数据测试不代表推理帧率或移动场景建图精度。

`--check-bridge` 验证晚加入 ROS2 订阅者收到地图和静态 TF、ROS2 `/cmd_vel` 传至 ROS1，以及 GridMap 的 `info.header -> header` 映射和 ChannelInfo 自定义消息转换。基准中的 `loss_percent` 包含发现阶段尚未连接时丢失的消息；`first` 和 `missing_within_stream` 可区分启动缺帧与接收期间缺帧。CPU 是整个采样窗口的容器平均值，包含收尾等待；不支持累计 CPU 计数时记为 null。

2026-10-28 本机验证记录位于 `.runtime/validation.json`，镜像版本记录在 `images.lock.json`：两个 ROS 容器均通过 Blackwell Torch / CuPy 内核检查；FAST-LIO-SAM 和 EMCP 编译通过；上述跨桥接口全部通过；新 Noetic 内真实 checkpoint + RELLIS 静态图像/点云生成 1786 个有效全局栅格、最多 5 次观测，UAM2TE 相关测试 65 项通过。

10 秒合成数据基准的默认传输接收速率约为图像 29.75 / 29.80 Hz、点云 9.80 / 9.60 Hz，P95 延迟分别约 13.92 / 20.67 ms、8.30 / 7.03 ms（ROS1→2 / ROS2→1）。包含启动阶段的总缺帧率为图像 1.33% / 7.33%、点云 3% / 5%，其中 ROS2→1 图像首次收到序号 20，接收期间另缺 2 帧。UDP-only 对照结果保存在 `.runtime/benchmark-udp-baseline/result.json`，整段检查期间宿主机 UDP RcvbufErrors / SndbufErrors 增量为 0。这些结果不能归因于尚未执行的内核调优，也不能证明持续运行无丢帧。

## 2026-10-28：BISTU bag / CVaR 双窗口实验

本次 bag 为 `/mnt/sda/datasets/whd/bunker/USED_ROSBAG_25/251004/2025-10-04-16-56-15.bag`，约 78.36 秒。使用 APP score seed42 指定 checkpoint，参考速度 0.8 m/s、EMA gamma 0.2，图像经 calibrator 去畸变并裁为 1920×1440，再输入模型。最终确定 support 门槛 0.15、0.5 m/格、五档阈值 `[0.20,0.35,0.50,0.65]`；实验控制器默认按此配置和 0.2 倍速启动。新导航入口默认订阅 `image_rect` 和 `image_rect/camera_info`。

宿主机执行以下命令启动整套实验；端口 11671 / 8765 必须空闲：

```bash
cd /mnt/nvme1n1/whd/ros-jazzy-ros1-bridge-builder
python3 runtime/cvar_session.py start --display :1 \
  --bag /mnt/sda/datasets/whd/bunker/USED_ROSBAG_25/251004/2025-10-04-16-56-15.bag \
  --config /mnt/nvme1n1/whd/uam2te/configs/experiments/speed_response/tasks_20261020/curve_app_s_s42_20261020.yaml \
  --checkpoint /mnt/nvme1n1/whd/uam2te/checkpoints/speed_curve_matrix_20261020/curve_app_s_s42_20261020/20261021-015701_curve_app_s_s42_20261020_best-epoch-001-valid_curve_normalized_mae-0.0578.ckpt
```

脚本在 `ros1-noetic` 内启动独立 Master、模型 worker、FAST-LIO-SAM、calibrator、EMCP、两个代价图导出器，以及 DISPLAY=:1 上的两个 RViz 窗口。此实验直接使用 ROS1 可视化，不需要启动 Nav2 或 bridge。回放仅包含图像、RS LiDAR 和 IMU，默认速率 0.2；不循环回放。

| RViz / 话题 | 内容 |
| --- | --- |
| `CVaR_alpha_095.rviz` / `/similarity_costmap` | alpha=0.95，全局五档风险代价 |
| `CVaR_alpha_005.rviz` / `/similarity_costmap_alpha_005` | alpha=0.05，相同融合统计、不同 CVaR 系数 |
| `/fast_lio_sam/mapping/map_global_optimized` | 保存服务导出的完整 SLAM 点云，可在 RViz 勾选 `SLAM global cloud` |
| `/path` | SLAM 轨迹 |
| `/hikrobot_camera_L/image_rect` | 输入模型的矫正图，可勾选 `Rectified camera` |

当前两张图均为 0.5 m/格、代价 `[0,30,50,70,100]`、未知 -1；白色代价低，黑色代价高。滚轮缩放，按中键拖动地图。两个窗口可以分别调整，但出图比较时应保持相同观察范围。

若回放结束后才勾选完整点云显示，再执行一次下方的 `save`，SLAM 保存服务会重新发布完整点云。

```bash
python3 runtime/cvar_session.py status
python3 runtime/cvar_session.py pause
python3 runtime/cvar_session.py resume
python3 runtime/cvar_session.py save
python3 runtime/cvar_session.py stop
```

回放结束约 10 秒后自动保存，再停止 bag 和旧的周期静态 TF 发布器；观察节点用原标定变换与最后的 SLAM 位姿发布 latched static TF，保留完整结束场景。该最终静止位姿仅用于结束后查看。再次回放必须先 stop、再 start，新会话会清空 EMA 并重新建立动态 TF。不要单独重启 bag 向同一统计重复融合。

当前会话目录记录在 `.runtime/cvar_active.json`，最终参数对应 `.runtime/cvar_20261028_174155/`。历史首轮 `.runtime/cvar_20261028_155304/` 使用 0.1 m 连续地图：910 帧模型输出、779 条里程计、54107 个有效全局格，单格最多 126 次观测。已验证两种 alpha 的 mean / variance / count 完全一致，52690 格在 alpha=0.95 下代价严格更高。首轮忙时适配器丢弃了部分图像，没有处理 bag 中全部 1568 帧；最终 0.2 倍速会话已完整处理 1568 帧。

输出包含 `statistics_alpha_095.npz`、`statistics_alpha_005.npz`、两张代价图 PNG、`prediction_metadata.json`、`odometry.csv`、20 秒/50 秒附近相机帧、`slam/GlobalMap.pcd`、`slam/filterGlobalMap.pcd` 和优化关键帧轨迹。`result.json` 保存验证结果。

### 附件旧图的 0.5 m / 五档设置

旧版本 `d85119d` 的 `elevation_mapping_cupy/launch/bunker.launch` 和 `bunker_parameters.yaml` 均使用 0.5 m/格；`generate_costmap.py` 使用下表。PDF 的五个主要灰度值 255、202、166、131、77 也与该版本的五档代价及 RViz Map alpha=0.7 吻合。PDF 本身没有米制比例尺，0.5 m 的判断来自对应旧代码配置。

| CVaR 可通行分数 | 输出代价 |
| --- | --- |
| <0.3 | 100 |
| [0.3, 0.5) | 70 |
| [0.5, 0.7) | 50 |
| [0.7, 0.9) | 30 |
| >=0.9 | 0 |

历史代码把极低分数当作未知并跳过，这个缺陷不应恢复；当前以观测 count 区分未知。0.1 m 连续地图若仅为展示而重采样 / 分档，不等于从 0.5 m 栅格重新融合 EMA。严格比较空间分辨率时，应重建相应分辨率的融合统计，并明确粗格聚合规则。

### 0.5 m / 五档重新回放

当前实验已默认 `--resolution 0.5 --cost-encoding five_level`；先执行 stop 再 start，会从零重新融合。`--play-rate` 默认 0.2，可显式指定其他回放速率。分档只用于导出的代价，EMA mean / variance / count 仍保存原始浮点统计；未知按 count=0 判断。

2026-10-28 第二轮结果位于 `.runtime/cvar_20261028_161339/`：0.5 m 融合和导出分辨率、400×400 全局图、2455 个有效格、最大观测次数 150。两个 alpha 的统计一致，1122 格在 alpha=0.95 下代价更高。实际已观测值为 30/50/70/100，本次没有达到 score>=0.9 的 0 代价格；-1 仍表示未知。RViz 使用旧版 Map alpha=0.7 和白色背景。

可用宿主机模型环境生成对齐的比较图，渲染过程读取保存统计，不重新融合：

```bash
../uam2te/.venv/bin/python runtime/render_cvar_maps.py .runtime/cvar_20261028_161339
```

输出为会话目录中的 `cvar_comparison.png` 和 `cvar_comparison.pdf`。

### support 过滤对照

在完整 start 命令中增加 `--support-min 0.15`，保留 `--ema-gamma 0.2 --resolution 0.5 --cost-encoding five_level --cost-thresholds 0.2 0.35 0.5 0.65 --play-rate 0.2 --record-ema`。
用户完成 0.1/0.15/0.3 对照后确定 0.15；本实验控制器和 `cvar_experiment.launch` 默认 0.15，
显式传入 `--support-min 0.0` 可接受全部模型有效输出。通用 `uam2te_msgs/navigation.launch` 默认仍为 0.0。
修改门槛前先停止旧会话，从空地图重新回放。

适配器保持 `/uam2te/conservative_score` 的原始 q，另用 `/uam2te/mapping_score` 向 EMCP 发布建图输入。
support<所选门槛的像素为 NaN，现有融合器会跳过，不更新 mean/variance/count。已知格保留历史，
从未得到通过门槛观测的格子保持未知 -1；通过门槛后仍使用 q=score×support。
此门槛只影响模型风险地图；LiDAR/IMU 的 SLAM 和高程几何继续更新。
RViz 新增 `Mapping support >= 0.15` 掩码，白色表示通过门槛。阈值记录在 session 与 navigation metadata 中。

完整回放结束后，可重放日志得到最新世界状态，再与无过滤基线比较：

```bash
../uam2te/.venv/bin/python runtime/compare_ema.py .runtime/cvar_20261028_172749 --gammas 0.2
../uam2te/.venv/bin/python runtime/compare_support_filter.py \
  .runtime/cvar_20261028_164643/ema_sweep_020_030_040/gamma_200 \
  .runtime/cvar_20261028_172749/ema_sweep/gamma_200 \
  --extra-filtered .runtime/cvar_20261028_171812/ema_sweep/gamma_200 \
  --output .runtime/cvar_20261028_172749/support_filter_comparison
```

比较图与 JSON 同时报告有效覆盖面积和最高代价面积。两次独立回放可能有不同的投影采样或 SLAM 轨迹，
因此该结果属于系统回放对照，而非逐个投影样本配对的因果实验。

### 最终参数的 300 dpi 图像

成图保存在 `/mnt/nvme1n1/whd/cvar_results/20261028_final/`。单图 3000×1950、双图合版 3000×3750 像素，
PNG 为 300 dpi，并提供 PDF。图中叠加同轮 779 个在线 SLAM 位姿、Start/End 标注和 10 m 比例尺。
原始 0.5 m 栅格与风险值保持不变；无轨迹版本也一并导出。参数与文件来源见 `figure_manifest.json`。

```bash
../uam2te/.venv/bin/python runtime/export_cvar_figure.py .runtime/cvar_20261028_174155 \
  --output /mnt/nvme1n1/whd/cvar_results/20261028_final --dpi 300
```

脚本默认读取与此前对照图相同的完整世界格统计。再次导出请指定新的空目录；
如需匹配 RViz 的局部周期显示缓存，可传 `--statistics rviz`。

### 2025-06-18-15-17-18：alpha=0.95 成图

bag 为 `/mnt/sda/datasets/whd/bunker/USED_ROSBAG_25/250618/2025-06-18-15-17-18.bag`，
时长 91.08 秒、865 帧图像。已核对原始图像 2048×1536 及对应数据集的标定，沿用 BISTU 矫正流程。
最终参数不变：support>=0.15、gamma=0.2、0.5 m、五档阈值 0.20/0.35/0.50/0.65、参考速度 0.8 m/s。

运行结果位于 `.runtime/cvar_20261028_181026/`：完整处理 865 帧，保存 906 个 SLAM 位姿；
674 次实际投影事件、65865 次接受观测，完整世界格 1532 个，周期显示缓存 1519 个。
日志重放逐次核对 GPU posterior，最大绝对误差 7.45e-9。

仅导出 alpha=0.95 的两版图，均为 3000×1950 像素、300 dpi，并提供 PDF。
`--plain-map-only` 会移除无标注版的全部文字、轨迹、图例和比例尺；标注版保留实际轨迹、Start/End、alpha、比例尺和图例。

```bash
../uam2te/.venv/bin/python runtime/export_cvar_figure.py .runtime/cvar_20261028_181026 \
  --output /mnt/nvme1n1/whd/cvar_results/2025-06-18-15-17-18_alpha095 \
  --dpi 300 --alpha 0.95 --plain-map-only
```

该目录保存两版 PNG/PDF、轨迹 CSV、参数与来源记录；打包文件为
`/mnt/nvme1n1/whd/cvar_results/2025-06-18-15-17-18_alpha095_300dpi.zip`。
之前 2025-10-04 bag 的成图完整保留。

### RELLIS-3D 00003：20200213_trail_1_r

此序列使用 Ouster OS1-64 与 Ouster IMU、Pylon Bayer 相机。入口为 `cvar_rellis_experiment.launch`，
加载 `fast_lio_sam/mapping_rellis3d.launch` 和本序列的 `calibration.json` / `camera.launch`。
已使用官方标定包及 bag 的 CameraInfo、静态 TF 交叉核验序列 00003 的内外参。

`rellis_camera.py` 通过 cv_bridge 去马赛克，在 1920×1200 原生分辨率上矫正，保持 K，输出 D=0。
输出图像 `/uam2te/rellis/image_rect`，CameraInfo 位于该话题的 `/camera_info`。
该 Python 节点由宿主机挂载的 `cvar_runtime` 包提供，`env.sh noetic` 将 runtime 目录加入 ROS_PACKAGE_PATH，
无需复制项目或重建容器。与 BISTU 使用相同模型、参考速度、support 门槛、EMA 和五档映射。

EMCP 点云输入必须带有正确的移动传感器原点，本配置使用 `/cloud_registered_body`。
当前 EMCP 封装以 PointCloud2.header.frame_id 查传感器位姿；直接使用全局 `/cloud_registered` 会把原点固定在 camera_init，
在该序列约 14 m 的爬升中造成高度筛选错误。该错误试运行已标记 RUN_REJECTED，未用于成图。
有效重跑会话为 `.runtime/cvar_20261028_195707/`，上坡及末段均已检查。

```bash
python3 runtime/cvar_session.py start --platform rellis \
  --bag /mnt/sda/datasets/OpenAccess/RELLIS-3D/Full-stack-Merged-data/20200213_trail_1_r.bag \
  --calibration-dir /mnt/nvme1n1/whd/ros-jazzy-ros1-bridge-builder/.runtime/rellis_00003_cvar_preflight/calibration \
  --config /mnt/nvme1n1/whd/uam2te/configs/experiments/speed_response/tasks_20261020/curve_app_s_s42_20261020.yaml \
  --checkpoint /mnt/nvme1n1/whd/uam2te/checkpoints/speed_curve_matrix_20261020/curve_app_s_s42_20261020/20261021-015701_curve_app_s_s42_20261020_best-epoch-001-valid_curve_normalized_mae-0.0578.ckpt \
  --map-length 400 --origin-x -50 --origin-y -200 --play-rate 0.4 --record-ema
```

只回放 Pylon Image/CameraInfo、Ouster points/imu，不回放原 bag 的 TF、轮速里程计或其他 IMU。
全局范围 x=[-50,350] m、y=[-200,200] m；0.5 m 栅格不变，覆盖约 240 m 的轨迹。
2185 帧中处理了 2183 帧（99.91%），记录 2181 个 SLAM 位姿、2022 次有效投影事件，接受观测 115628 次。
完整世界格 5793 个，显示缓存 5783 个；重放 GPU posterior 的最大绝对误差为 7.45e-9。

```bash
../uam2te/.venv/bin/python runtime/compare_ema.py .runtime/cvar_20261028_195707 --gammas 0.2
../uam2te/.venv/bin/python runtime/export_cvar_figure.py .runtime/cvar_20261028_195707 \
  --output /mnt/nvme1n1/whd/cvar_results/rellis3d_00003_trail_1_r_alpha095 \
  --alpha 0.95 --plain-map-only --dpi 300 --width-inches 15 --auto-aspect
```

两版 PNG 为 4500×1016 像素、300 dpi，另提供 PDF。长幅版式将 alpha 标注放在左上方，避免遮挡终点。
无标注版无文字、轨迹、图例和比例尺。原始标定、参数、轨迹、覆盖检查与成图保存在输出目录，
打包文件为 `/mnt/nvme1n1/whd/cvar_results/rellis3d_00003_trail_1_r_alpha095_300dpi.zip`。
输出目录要求为空，重复出图时指定新目录。

### RELLIS-3D 00001：替换旧图顶部的代价地图

序列映射为 `00001 -> 20200213_trail_1.bag`，`00003 -> 20200213_trail_1_r.bag`。
00001 与 00003 是同一路线的相反行驶方向，按对应序列分别提取、核验标定。
00001 的预检目录为 `.runtime/rellis_00001_cvar_preflight/`，有效运行目录为
`.runtime/cvar_20261028_215325/`。参考速度 0.8 m/s，alpha=0.95；其他最终参数不变。

2321 帧全部推理，2316 个在线 SLAM 位姿，2049 次投影事件、113198 次接受观测，
完整世界格 6001 个；逐次核对 GPU EMA posterior，最大绝对误差 7.45e-9。
轨迹最长约 250 m、最高爬升 17.71 m，末段覆盖已检查。

```bash
../uam2te/.venv/bin/python runtime/compare_ema.py .runtime/cvar_20261028_215325 --gammas 0.2
../uam2te/.venv/bin/python runtime/export_cvar_figure.py .runtime/cvar_20261028_215325 \
  --output /mnt/nvme1n1/whd/cvar_results/rellis3d_00001_trail_1_alpha095 \
  --alpha 0.95 --plain-map-only --plain-full-bleed --width-inches 15 --auto-aspect \
  --bounds -3.01435026143281 261.5 -23.48245475690332 39.48245475690332
```

`--plain-full-bleed` 必须配合 `--plain-map-only`，使无标注图的地图占满画布；
按世界坐标范围决定物理长宽比，背景沿用 Unknown 色，不做空间平滑或非等比缩放。
本例的 4.20098:1 比例来自用户旧图顶部画布；无标注 PNG 为 4500×1071，
轨迹版为 4500×1317，均为 300 dpi，并提供 PDF。
结果目录包含统计 NPZ、序列标定、参数、轨迹及核验记录；打包文件为
`/mnt/nvme1n1/whd/cvar_results/rellis3d_00001_trail_1_alpha095_300dpi.zip`。
重复导出须指定新的空目录。

### 同一 bag 的三速度对照

`cvar_session.py start --reference-speeds 0.4 0.9 1.4` 使用 BISTU 三通道入口
`cvar_speed_experiment.launch`。模型每张图只前向一次，再查询三个速度；0.9 m/s 沿用模型的 logit 插值。
三个速度是模型参考速度，bag 中的实际运动不会改变。各速度分别按自己的 support>=0.15 过滤，
以 `32FC3` 一起送入 EMCP；一次投影得到的世界格和 UV 由三个独立 EMA 状态共享。
点云使用带移动原点的 `/cloud_registered_body`。默认单速度与 RELLIS 入口保持原来的参数接口。

该模式自动启用 EMA 日志。实时标准 similarity 话题、alpha 导出和网络诊断显示第一个速度，
另外两个速度的完整世界状态在同一个融合实例中独立保存，回放结束后一起导出：

```bash
../uam2te/.venv/bin/python runtime/materialize_speed_maps.py .runtime/cvar_20261028_204409 \
  --output /mnt/nvme1n1/whd/cvar_results/2025-06-25-16-43-45_speed_comparison
```

导出前逐图检查三个速度的候选世界格与 UV 完全一致，逐次检查 CPU 重放与 GPU EMA posterior，
并验证三个速度没有共用统计存储。输出保留每速度的统计 NPZ、同一轨迹 CSV、参数与来源 JSON，
提供同范围的单图无标注/轨迹 PNG、PDF，以及三列对比图，均为 300 dpi。
`comparison_manifest.json` 同时记录各速度覆盖和共同覆盖区域的代价变化；support 门槛相同不代表过滤掩码相同。
`export_cvar_figure.py --bounds xmin xmax ymin ymax` 可强制所有单图共享世界范围。
原有单通道 `compare_ema.py` 不用于三通道日志。

### 2025-06-25-16-50-28：三速度结果

bag 为 `/mnt/sda/datasets/whd/bunker/UNUSED_ROSBAG_25/2025-06-25-16-50-28.bag`，
时长 45.993 秒，437 帧图像全部推理；运行目录 `.runtime/cvar_20261028_221421/`。
使用 0.4/0.9/1.4 m/s，同一模型前向、SLAM 和三通道投影；最终 support、EMA、alpha、五档映射不变。
世界存储范围为 [-200,200] m × [-200,200] m，分辨率 0.5 m；仍只显示实际覆盖范围。
bag 未携带 CameraInfo，沿用现有同日 BISTU 内外参，实际配置快照与来源说明保存在 `calibration_used/`。

455 个在线位姿，264 次共享投影，逐次核对 GPU posterior，最大绝对误差 7.45e-9。
三速度有效格分别为 629/629/646，共同覆盖 629 格；共同区域内代价 0 占比分别为
20.19% / 12.40% / 5.88%，完整五档占比及变化见 `comparison_manifest.json`。
结果位于 `/mnt/nvme1n1/whd/cvar_results/2025-06-25-16-50-28_speed_comparison/`，
合版 7200×2460，单图 3000×1950，均为 300 dpi PNG 并提供 PDF、统计与参数；同名 `_300dpi.zip` 为打包文件。
RViz 的 `/speed_comparison/v_0400/costmap`、`v_0900/costmap`、`v_1400/costmap`
均与对应完整世界统计逐格核对，轨迹为 `/speed_comparison/trajectory`。

16:52:08 的初始回放已按路径更正停止，`.runtime/cvar_20261028_221047/RUN_SUPERSEDED.json`
记录替代关系；该部分数据未用于本次出图。

### 完整 SLAM、只融合 20–30 s 的观测

`cvar_session.py start --mapping-window 20 30` 保持 rosbag 从头到尾回放和完整 SLAM、推理，
仅对 `/uam2te/mapping_score` 加采集时间门控。窗口是相对原 bag 开始记录时间的 [20,30) 秒；
使用图像 header stamp，不使用回调到达时刻、墙钟或当前 /clock，迟到消息仍按采集时间判定。
窗口前 EMA 为空，窗口后不再发布融合观测；所有雷达与 IMU 仍供 SLAM 和地形几何使用。
三个参考速度与 support、EMA、CVaR、分辨率、五档映射均不变。

2025-06-25-16-50-28 的窗口运行目录为 `.runtime/cvar_20261028_224313/`。
437 帧全部推理，455 个完整 SLAM 位姿；源图像有 95 帧属于窗口，
发布与独立接收的 95 个时间戳逐一匹配；窗口前 191 帧、窗口后 151 帧均未进入融合。
59 次共享投影，逐次 GPU posterior 核验最大误差 7.45e-9。
三个速度有效格为 290/290/293，均从空统计开始累计。

```bash
# 完整启动命令见结果目录 index.json；关键新增参数为：
python3 runtime/cvar_session.py start ... --reference-speeds 0.4 0.9 1.4 --mapping-window 20 30
../uam2te/.venv/bin/python runtime/materialize_speed_maps.py .runtime/cvar_20261028_224313 \
  --output /mnt/nvme1n1/whd/cvar_results/2025-06-25-16-50-28_t020_030_speed_comparison
```

`navigation_inference.py` 实现发布门控并输出 `mapping_window_audit.json`；
`watch_cvar_experiment.py` 独立接收、检查窗口前空图及窗口外仍有推理。
`expected_image_window.json` 从原 bag 独立提取目标时间戳。
导出脚本只叠加窗口内的 100 个原始 SLAM 位姿（约 20.038–29.938 s），
Start/End 对应窗口轨迹首尾；完整 455 位姿另存 `odometry_full.csv`，不做端点外推或轨迹平滑。
边界、延迟消息、三通道及 support 保持测试可在 Noetic 中运行：

```bash
/usr/bin/python3 /mnt/nvme1n1/whd/uam2te/tests/test_navigation_mapping_window.py -v
```

输出目录和同名 `_300dpi.zip` 包含单图、三列 PNG/PDF、统计、完整/窗口轨迹及门控审计。
合版 7200×2460、单图 3000×1950，均为 300 dpi；之前的完整时间段结果完整保留。
RViz 的三个速度话题现在显示窗口地图，轨迹话题只显示这 10 秒的 100 个位姿。

### 20.034 s 网络输出与 mean-only / CVaR 诊断

`cvar_results/2025-06-25-16-50-28_t020_diagnostics/` 保存本轮实际矫正图像、
三个速度的 score/support/q、叠加图、原生 float32 输出、投影对应及均值/CVaR 对照。
完整相机图为 1920×1440，网络输入 448×336，原生方形输出 56×56；去除内部 padding 后恢复图像几何。
q 在原生网格相乘后再恢复尺寸。重算 q 与本轮首次投影日志的 152/152/153 个有效样本逐点完全一致。

20.034 s 有效投影点 q 中位数为 0.736/0.660/0.605。最终地图最下部 y<=-11.75 m
在该帧尚未被投影（首次候选格最低 y=-11.25 m），最终颜色来自后续窗口帧。
0.4 m/s 下，这个下部区域的 cost100 占比：仅 EMA 均值为 8.5%，alpha=0.95 CVaR 为 68.4%。
一格实际 q 观测为 0.207–0.783，最终 mu=0.431、sigma=0.164，
mu-2.063*sigma=0.092，因此由 mean-only 的 cost50 变为 CVaR 的 cost100。

mean-only 仍保留 score*support 和 EMA。共同 290 格中，mean-only 的白色占比随速度为
35.9%/24.5%/12.4%，CVaR 为 15.5%/4.5%/0.7%。
当前 sigma 是投影 q 的时序波动，不能直接等同于已校准的真实通行风险；
mean-only 会平滑偶发低分，视觉连续性本身也不能证明其更准确。

6 m 的现有数据分析以栅格中心到 20–30 s base_link 轨迹折线的最小水平距离分组，
仅是覆盖估计与相关性分析，没有执行限距融合。0.4 m/s 有 144/290 格在整个窗口内始终>6 m，
下部区域约 94% 属于这一组；近组/远组 sigma 中位数分别为 0.090/0.157。
正式 6 m 对照须逐帧融合前过滤，单独报告覆盖与共同区域表现。

诊断图均为 300 dpi，PNG/PDF 与原生数据包为
`/mnt/nvme1n1/whd/cvar_results/2025-06-25-16-50-28_t020_diagnostics_figures_300dpi.zip`。
大体积全分辨率 float32 NPZ 单独保留；包内 `index.json` 说明其路径及内容。
本次为离线诊断，原有建图统计文件 SHA256 已核对保持一致；mean-only 应标注为 EMA 均值对照。

### 2026-10-29：20–30 s 内，6 m / 8 m 三速度对比

完整回放 `UNUSED_ROSBAG_25/2025-06-25-16-50-28.bag`，源会话
`.runtime/cvar_20261029_004130`：437/437 张图像完成推理，455 条完整 SLAM 位姿；
仅 `[20,30)` s 的 95 张图像发布给建图，独立订阅核验 95/95，实际形成 64 次共享投影事件。
速度 0.4/0.9/1.4 m/s，support≥0.15、EMA=0.2、CVaR α=0.95、分级阈值
[0.20,0.35,0.50,0.65]、分辨率 0.5 m。

`input_image` 现在可在 EMA trace 中记录 C++ 用图像 header 时间查询的 map→camera R,t。
该记录只增加诊断信息，不改变在线融合。离线先逐条重放并核验原始 GPU posterior，
再分别从空状态构建 6 m 和 8 m 地图：每条观测进入 EMA **之前**，检查栅格中心到
该图像时刻 base_link 的水平距离是否 ≤R。base_link 在相机坐标系中的固定偏移由
会话归档的 `calibration_used/mapping_rs_bistu.launch` 计算。不能用最终位姿裁剪地图，
不能把当前处理时刻的位姿当作图像时刻位姿。旧 trace 缺少 R,t，不能用于该精确距离重放。

```bash
# 使用模型 .venv 的 Python；先完整运行上述三速度会话并保留标定快照。
.venv/bin/python /mnt/nvme1n1/whd/ros-jazzy-ros1-bridge-builder/runtime/materialize_speed_maps.py \
  /mnt/nvme1n1/whd/ros-jazzy-ros1-bridge-builder/.runtime/cvar_20261029_004130 \
  --max-range-m 6 --output /absolute/path/to/new_empty_output
# --max-range-m 8 生成另一组；--bounds XMIN XMAX YMIN YMAX 固定相同显示范围。
```

输出目录：
`/mnt/nvme1n1/whd/cvar_results/2025-06-25-16-50-28_t020_030_range6m_8m_speed_comparison/`。
`range_6m` / `range_8m` 各有三速度单图与组合图，无标注版、轨迹起终点版，PNG/PDF 300 dpi；
顶层为两行三列对比图。`reference_unrestricted` 是同一回放的不限距对照。
所有图使用相同世界坐标范围、调色板及 100 条窗口内原始轨迹。

6 m 已知格子数为 138/138/140，8 m 为 226/226/229；代价 100 占各自已知区域
8.70%/8.70%/13.57% 和 28.32%/29.20%/31.88%。共同覆盖区域的平均代价变化较小，
不能把范围外变为未知直接解释成同一片地形预测准确率提高。

已验证：旋转及相机偏移下的距离、半径边界、远处旧观测不进入近距 EMA；
所有原始 GPU posterior、所有半径接受/拒绝记录、6 m≤8 m≤不限距逐格观测计数；
六个 RViz OccupancyGrid 与保存数组逐格相同，轨迹 100 条逐点相同。
RViz 配置 `.runtime/cvar_20261029_004130/CVaR_range_speed_comparison.rviz`，
默认显示 8 m、0.4 m/s；六张地图可逐一切换。

同一场景的 3 m 补充结果：
`/mnt/nvme1n1/whd/cvar_results/2025-06-25-16-50-28_t020_030_range3m_speed_comparison/`。
直接重放 `.runtime/cvar_20261029_004130` 的同一批 64 次投影记录，
`materialize_speed_maps.py --max-range-m 3`，从空 EMA 状态开始；其他参数及20–30秒窗口保持不变。
三速度有效格子数 45/45/46，面积 11.25/11.25/11.50 m²，接收观测数 655/655/661。
顶层标准图与 6 m/8 m 保持相同显示范围；`closeup` 图仅放大显示，保持 0.5 m 栅格与原始代价。
各速度文件夹同时包含无标注/轨迹标注的标准图与局部图，均为300 dpi PNG/PDF。
所有距离接受/拒绝记录、逐格观测计数属于6 m结果的子集、原始GPU后验、
RViz三图及100点轨迹均已核验。当前RViz使用 `CVaR_range3m_speed_comparison.rviz`，默认0.4 m/s。

### 2026-10-29：10月4日 bag，0.8 m/s、6 m、α=0.05/0.95

源会话 `.runtime/cvar_20261029_010649`，完整回放
`USED_ROSBAG_25/251004/2025-10-04-16-56-15.bag`（78.362 s），无20–30秒限制。
参考速度0.8 m/s、support≥0.15、EMA=0.2、阈值[0.20,0.35,0.50,0.65]、栅格0.5 m，
以及原单速度流程的 `/cloud_registered`、200 m世界存储均保持原实验配置。
1568/1568帧推理、779条完整SLAM位姿、829次实际投影，记录图像采集时刻的map→camera变换。

新增单速度导出器：
```bash
/mnt/nvme1n1/whd/uam2te/.venv/bin/python runtime/materialize_range_maps.py \
  .runtime/cvar_20261029_010649 --max-range-m 6 \
  --output /absolute/path/to/new_empty_output
```
它先核对每条原始GPU后验，再从空EMA状态重放每条观测，只有到图像时刻base_link水平距离≤6 m的栅格
参与更新。相机→base偏移从会话标定快照获取。ROS参数经过float32存储，参数一致性校验允许其表示误差；
重放使用trace中的实际gamma（0.20000000298023224）。两种alpha共用完全相同的mean/variance/count。

输出 `/mnt/nvme1n1/whd/cvar_results/2025-10-04-16-56-15_range6m_alpha005_095/`：
两alpha独立无标注图、轨迹起终点图，以及横向对比图，均有300 dpi PNG/PDF；保留融合统计和完整观测记录。
6 m内接收50316条栅格观测，已知1439格（359.75 m²）。α=0.05/0.95最高代价格占比为2.08%/15.08%。
原始GPU后验最大误差7.45e-9，alpha统计完全一致，α=0.95逐格代价不低于0.05。
`publish_alpha_maps.py`发布两张最终地图，已验证ROS数组逐格相同、779点轨迹逐点相同。
RViz配置为源会话中的 `CVaR_range6m_alpha_005.rviz` 和 `CVaR_range6m_alpha_095.rviz`。

### 2026-10-29：20–30秒单帧预测，5时刻×3速度×4结果

输出目录：
`/mnt/nvme1n1/whd/cvar_results/2025-06-25-16-50-28_t020_030_prediction_sequence/`。
目标时刻20/22/24/26/28秒，按图像header选择各目标时刻之后的第一帧，实际均约晚34.4 ms。
沿用 `.runtime/cvar_20261029_004130` 的标定、权重和0.4/0.9/1.4 m/s查询速度；
每张相机图只运行一次曲线预测，再查询三个速度。图像矫正复现原calibrator参数和裁剪，
20秒矫正图与原建图保存图像逐像素一致，三个速度的153/153/154条原投影q样本逐值完全一致。

`subplots/` 恰有60张独立结果图：score、support、二者乘积q、q叠加矫正原图；
均为1920×1440、300 dpi，统一RdYlGn [0,1]色标，叠加权重为相机60%+热图40%。
`sheets/` 为每速度5×4合版（4800×4740）及60子图总览（10800×4020），都有300 dpi PNG/PDF。
这是单帧网络输出，未应用support门槛、距离过滤、EMA或CVaR。
q在原生56×56模型网格先相乘，再去填充和恢复图像尺寸；不能用两个已恢复图直接相乘来替代。
`native_arrays/` 保存三个速度的float32 score/support/q，`provenance/deployment.py`
可恢复全分辨率标量图；`camera/`保存五张raw和五张rectified相机图，
`frame_times.csv`与`subplot_index.csv`给出精确时间戳和60张图的对应关系。

复现脚本归档于输出的provenance目录，分别为extract_prediction_sequence_20261029.py、
infer_prediction_sequence_20261029.py、render_prediction_sequence_20261029.py。
抽帧使用ros1-noetic内/usr/bin/python3和cv_bridge；推理使用挂载的uam2te/.venv；
无ROS话题发布，不改变当前正在显示的最终代价地图。


### 2026-10-29: June 18 full-bag map with 6 m range gate

Replayed `USED_ROSBAG_25/250618/2025-06-18-15-17-18.bag` in session `.runtime/cvar_20261029_100550`: all 865 images inferred and 906 SLAM poses recorded. Parameters: v=0.8 m/s, alpha=0.95, support>=0.15, EMA gamma=0.2, thresholds=[0.20,0.35,0.50,0.65], resolution=0.5 m. The previous `/cloud_registered` projection configuration is retained.

For each of 689 projection events, the world-cell center is tested against the image-acquisition base_link pose before EMA; the 6 m horizontal gate accepts 34,728 observations and produces 985 known cells (246.25 m²). CPU replay of all unrestricted observations agrees with recorded GPU posteriors (maximum absolute error 7.45e-9). Published range-map values and all 906 trajectory poses were verified against the exports.

Outputs: `/mnt/nvme1n1/whd/cvar_results/2025-06-18-15-17-18_range6m_alpha095/`; two 3000x1950 PNG figures at 300 dpi, matching PDFs, statistics, observation trace and source/calibration provenance. RViz on display :1 shows `/range_comparison/alpha_095/costmap`. `materialize_range_maps.py --alpha 0.95` exports only the requested alpha while retaining both statistical archives for consistency checks; `publish_alpha_maps.py` follows the output session alpha selection.


### 2026-10-29: June 27 full-bag three-speed comparison with 6 m range gate

Replayed `/mnt/sda/datasets/whd/bunker/UNUSED_ROSBAG_25/2025-06-27-14-30-04.bag` in `.runtime/cvar_20261029_103551`: all 582 input images inferred; 613 complete SLAM poses. Speeds 0.4/0.9/1.4 m/s share the same curve inference and projected candidate cells. Alpha=0.95, support>=0.15, EMA gamma=0.2, thresholds=[.20,.35,.50,.65], grid=0.5 m, pointcloud=`/cloud_registered_body`. Full bag is mapped without a time window.

Each projected observation is checked against the image-time base_link position before EMA, with a 6 m horizontal radius. Known cells: 715/714/733; accepted observations: 21319/21290/21944. Original GPU posteriors and shared projection geometry were verified; all three published maps and the entire trajectory exactly match the exported data.

Outputs: `/mnt/nvme1n1/whd/cvar_results/2025-06-27-14-30-04_range6m_speed_comparison/`, plus the sibling `_300dpi.zip`. Combined plain/trajectory figures are 7200x2460; each speed has 3000x1950 plain/trajectory figures, with matching PDFs at 300 dpi. RViz display :1 offers the three `/speed_comparison/v_*/costmap` layers, with 0.4 m/s enabled initially. Network-image displays are labeled as the 0.4 m/s query. Source, calibration, observations and transforms are archived under provenance/.


### 2026-10-29: June 27 road-center grass speed curve near 29.67 s

Extracted the closest image acquisition time, 29.634805983 s after bag start (35.194017 ms before target), from `2025-06-27-14-30-04.bag`. Rectification matches the actual mapping camera pipeline pixel-for-pixel on the stored 20 s reference frame. Selected a grass cell visually before inference: native row 36, column 30 (cropped row 29), rectified pixel center (1045.714286, 1011.428571).

Same checkpoint as session `.runtime/cvar_20261029_103551`; one image forward provides all queries. Score/support interpolation stays in logit space; q=score*support on the native cell. At speeds 0.4/0.9/1.4 m/s: scores 0.870900/0.776707/0.819168, support 0.983831/0.977522/0.941739, products 0.856819/0.759248/0.771442. Single-cell query results exactly match full-map queries. No temporal EMA, CVaR, range gate or support threshold is applied to these diagnostic curves. The black crosses are model queries, not measurements.

Output: `/mnt/nvme1n1/whd/cvar_results/2025-06-27-14-30-04_t029670_grass_speed_curve/`. Contains 300 dpi PNG/PDF curve, marked location and combined figure; dense/query/anchor CSV; original camera images; native logits; calibration and code provenance.


### 2026-10-29: Four user-marked points in the June 27 frame

Extended the 29.634805983 s single-frame curve analysis to four red-circle centers, numbered P1-P4 from top to bottom. The annotation was registered to the saved raw frame using 1,137 feature inliers (median residual 0.297 px, 95th percentile 0.964 px), then transformed through the archived camera undistortion/crop. Native (row,column) selections: P1=(36,36), P2=(38,32), P3=(42,33), P4=(46,25). Each marked center selects its containing cell, with no spatial averaging.

All queries reuse the original saved logits; no image forward or inference on red-painted pixels. The previous single-point values reproduce bitwise, and the 12 new queries at 0.4/0.9/1.4 m/s exactly match corresponding full-map entries. Curves remain single-frame score and score*support, without EMA/CVaR.

Output: `/mnt/nvme1n1/whd/cvar_results/2025-06-27-14-30-04_t029670_four_point_speed_curves/`. Includes marked full-frame locations, 2x2 curves, cropped-location/curve composite, four individual curves, 300 dpi PNG/PDF, CSV, native logits, coordinate registration and provenance.


### 2026-10-30: 实际部署耗时统计（June 27 bag）

完整回放 `2025-06-27-14-30-04.bag`（61.717 s，582 张相机图），1× 速率，并行运行 FAST-LIO-SAM、图像矫正、推理、EMCP 和 CVaR 导出。硬件为 RTX PRO 6000 Blackwell / EPYC 7A23；无 RViz、无逐观测 trace 写盘。单速度为 0.8 m/s，三速度为 0.4/0.9/1.4 m/s；alpha=0.95、support>=0.15、EMA=0.2、分档阈值 [.20,.35,.50,.65]、0.5 m 栅格。此处测量现有在线流程，**不包含此前离线重建的 6 m 距离门控**。三速度共用一次网络前向、融合三个通道；在线 CVaR 导出仍只计算第一个速度。

`runtime/profiling/` 提供可选计时 worker、ROS 包装器、嵌入式 Python hook、话题到达观察器、完整 bag 控制器与统计脚本。EMCP 的 `RuntimeTimingScope` 仅在设置 `UAM2TE_PROFILE_DIR` 时写日志。计时使用单调时钟；GPU 阶段同步 CUDA event，避免仅测得异步提交时间。每个模式先做 4 次模型预热，并逐数组核对插桩 worker 与原始实现一致；稳态统计剔除 bag 前 10 秒，吞吐量统计包含完整回放。

输出：`/mnt/nvme1n1/whd/cvar_results/2025-06-27-14-30-04_deployment_timing_20261030/`。
单速度平均值 / P95：网络前向 17.26 / 23.55 ms；完整推理回调 187.64 / 210.56 ms；EMCP 图像投影和融合 17.67 / 20.26 ms；CVaR 与代价图发布 7.87 / 9.49 ms。三速度对应平均值为 16.85、312.27、76.19、7.60 ms（最后一项仅一个速度）。详细分解见 `stage_timings_ms.csv`，父阶段包含子阶段，不能重复相加。

原始单速度部署处理 326/582 帧（5.28 FPS）；计时版单速度 309 帧（5.01 FPS）、三速度 188 帧（3.05 FPS）。本次计时版较原始版吞吐下降 5.2%，包含运行间波动；原始对照也保留轻量话题观察器。实际数据流瓶颈主要在全分辨率输出的编码、传输和解包，不能用网络前向倒数代表部署帧率。

同时间戳图像的观测延迟含传输/排队：原始单速度从 raw 图像到预测话题平均 566 ms；计时版单速度到融合完成平均 648 ms，三速度 1122 ms。不同话题观察器收到副本的顺序可能颠倒，因此短阶段的跨订阅者时差可能为负，不能当作独立矫正/通信耗时。最终 OccupancyGrid 没有输入图像身份，不报告精确图像到该话题的延迟。GridMap 配置 3 Hz、代价图配置 1 Hz，后者受回调触发量化实际约 0.83 Hz，并非计算吞吐上限。SLAM 保持运行，未插桩其内部计算。

该控制器固定复用 `.runtime/cvar_20261029_103551` 的 bag、模型和标定；ROS Master 11671、worker 8765 应空闲。输出目录必须不存在。例如在项目根目录依次执行：

```bash
python3 runtime/profiling/run_bag.py --mode single --output .runtime/deployment-timing-new/single_speed
python3 runtime/profiling/run_bag.py --mode multi --output .runtime/deployment-timing-new/three_speeds
python3 runtime/profiling/run_bag.py --mode single --baseline --output .runtime/deployment-timing-new/single_speed_baseline
/mnt/nvme1n1/whd/uam2te/.venv/bin/python runtime/profiling/summarize.py .runtime/deployment-timing-new
```

汇总脚本只读已保存的日志，可独立重绘 300 dpi PNG/PDF。原始日志、参数、逐帧时间、数组一致性核验、源码快照均保留在结果目录。
