# DEPLOY —— 换车部署清单（race_abot_411）

> 适用于「今天在这台车上跑通了，明天要换一台车」的情形。
> 不同小车的 **工作区路径 / master 地址 / 相机设备号 / 相机话题名 / 模型文件位置 / 环境变量**
> 都可能不一样 —— 本包已经对这几项做了自动适配，但仍有几步必须人工确认。

---

## 0. 一句话流程

```
放代码 → 编译 → 体检(check) → 启动(start) → 验证 
```

---

## 1. 把代码弄上车

**方式 A：git（网络好时）**

```bash
cd ~/catkin_ws/src
git clone https://github.com/x7in-code/ros_xiaoche.git race_abot_411   # 目录名必须 = 包名
```

**方式 B：离线包（网络差时，推荐）**

把 `race_abot_411.zip` 拷到车上任意位置，然后：

```bash
cd ~/catkin_ws/src
unzip ~/race_abot_411.zip          # 解压出来就已经是 race_abot_411/ 目录
```

> ⚠️ 解压后**不要手动改目录名**，目录名必须和 `package.xml` 里的 `<name>` 完全一致。

**换车时如果工作区不叫 `~/catkin_ws`**，先找一下：

```bash
ls -d ~/*/src 2>/dev/null          # 看看机器上有哪些 ROS 工作区
```

---

## 2. 体检（换车必做的第一步）

```bash
cd ~/catkin_ws/src/race_abot_411
bash tools/race_abot_411_start.sh check
```

只读、不动任何东西，会逐项报告：

| 检查项 | 说明 |
|---|---|
| 系统 / ROS 版本 | 应是 Ubuntu 20.04 + Noetic |
| master 地址 / 本机 IP | 判断网络配置 |
| 工作区路径 / 是否已编译 | |
| **编译白名单** | 若之前用过 `catkin_make --pkg X`，会一直只编译那一个包 |
| 6 个关键包是否可找到 | `race_abot_411` `abot_driver` `abot` `traffic_detector` `uvc_camera` `image_view` |
| **相机设备** `/dev/video*` | 不同车编号不同 |
| **模型权重**自动查找结果 | 找不到会列出全盘 `.pt` |
| Python 依赖 | `rospy cv2 cv_bridge numpy ultralytics yaml` |

**带 `!!` 的先解决，再往下走。**

---

## 3. 编译

```bash
cd ~/catkin_ws
catkin_make --pkg race_abot_411 abot_driver
source devel/setup.bash
```

**为什么不用裸 `catkin_make`？** 车上 `abot` 包有并行编译竞争缺陷（`abot_base.cpp` 没声明
依赖 `riki_msgs` 的消息生成），`catkin_make -j` 全量编译会随机失败。
我们只需要这两个包，绕开它最省事。

要全量编译（慢但能过）：
```bash
catkin_make -j1
```

> `--pkg` 会把白名单写进 CMake 缓存**并长期生效**。想恢复全量：
> `catkin_make -DCATKIN_WHITELIST_PACKAGES=""`

---

## 4. 启动

```bash
cd ~/catkin_ws/src/race_abot_411
bash tools/race_abot_411_start.sh
```

脚本会自动**清场 → roscore → 底盘驱动 → 相机（逐个试设备号直到出图）→ 主系统**，
并生成 `~/race_abot_env.sh`。

**每个新开的终端**要先对齐环境（这样就不会再出现 "Unable to communicate with master"）：

```bash
source ~/race_abot_env.sh
```

停止全部：
```bash
bash tools/race_abot_411_start.sh stop
```

**相机/模型/设备要覆盖时：**

```bash
# 相机在别的地方已经起了
USE_CAMERA=0 bash tools/race_abot_411_start.sh

# 指定相机设备
CAM_DEV=/dev/video2 bash tools/race_abot_411_start.sh

# 指定模型
MODEL=/path/to/aaabest.pt bash tools/race_abot_411_start.sh

# 连别人的 master（例如裁判机）
MASTER=http://172.21.129.23:11311 bash tools/race_abot_411_start.sh
```

---

## 5. 验证（3 条命令）

```bash
source ~/race_abot_env.sh

rosnode list | grep race_abot_411          # 应有 _node 和 _sign_detect
grep -nE "模型|类别|自检|ERROR" /tmp/race_abot_logs/race.log | tail -20
rostopic echo /race_abot_411/detections    # 拿标志对着相机
```

**判断标准：**

| 看到什么 | 说明 |
|---|---|
| `模型类别(13): [...]` | 模型加载成功（关键行） |
| `[自检] 相机 OK` | 巡线相机有数据 |
| `[自检] 里程计 OK` | `/odom` 有数据 |
| `"count": 0` 的 JSON 持续刷 | 检测链路通了 |
| 拿标志 → `"count": 1` 且 `class_name` 是规范名 | 归一化生效 |

---

## 6. 换车差异对照表（重点）

| 项目 | 今天这台车 | 换车后怎么办 |
|---|---|---|
| 工作区路径 | `/home/abot/catkin_ws` | `bash ...start.sh check` 会显示实际用的；不是这个就 `WS=... bash ...start.sh` |
| 模型权重 | `~/catkin_ws/src/traffic_detector/models/aaabest.pt` | **已自动查找**（留空即自动，优先 aaabest.pt）。找不到就 `MODEL=` 指定 |
| 相机设备 | `/dev/video0` | **脚本会逐个试** `/dev/video*` 直到出图；也可 `CAM_DEV=` 指定 |
| 相机话题 | `/camera/rgb/image_raw` | **节点自动挑选**（候选：rgb/right/front/left/back/usb_cam/image_raw） |
| master 地址 | `ROS_MASTER_URI=http://127.0.0.1:11311` | 脚本内部统一设好并写进 `~/race_abot_env.sh`；要连共享 master 用 `MASTER=` |
| 环境变量 | `CAMERA=usb` 等 | 本包**不依赖** `CAMERA`；只在 `start_camera:=true`（用 abot 的 cam.launch）时才需要 |
| 底盘控制话题 | `/akm_vel`（`abot_driver/Akm`） | 若那台车是 `cmd_vel`，改 `config/..._params.yaml` 的 `topics.cmd` |
| 超声话题 | `/sonar` | 没有也不影响，避障退化为低速直行 |

---

## 7. 故障排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `Unable to communicate with master` | 新终端没对齐环境 / master 没跑 | `source ~/race_abot_env.sh`；`pgrep -a rosmaster` 没输出就 `bash ...start.sh` |
| 话题存在但收不到数据 | 发布者与订阅者的地址不一致（127.0.0.1 vs 局域网 IP 混用） | 用 `start.sh` 统一启动；`rostopic info <话题>` 看两边地址是否同一个 |
| `模型文件不存在` | 那台车的 .pt 在别处 | `find ~ -name "*.pt"`，或 `MODEL=` 指定 |
| `模型类别` 一直不出现 | Jetson 上首次加载慢（20~60s） | 等待后再 grep；或 `tail -30 /tmp/race_abot_logs/race.log` |
| 相机没画面 | 设备号不对 / 被占用 | 脚本会自动换设备；仍不行 `sudo fuser -k /dev/video0` |
| 编译报 `riki_msgs/Velocities.h 没有那个文件` | `abot` 包并行编译竞争 | `catkin_make --pkg race_abot_411 abot_driver` |
| 编译报 `catkin_package ... not listed as build dependency` | CMakeLists 的 CATKIN_DEPENDS | 本包已修复（纯 Python 包留空） |
| roslaunch 报 `xterm: command not found` | 没装 xterm 却传了 `node_prefix:="xterm -e"` | 不传即可（默认已改为空） |
| `is not a package` | 目录名 ≠ 包名 | 目录必须是 `race_abot_411` |
| 日志目录超 1GB | ROS 日志堆积 | `rosclean purge -y` |

---

## 8. 跑之前还想改的话

- **所有可调参数**：`config/race_abot_411_params.yaml`（唯一的调参入口）
- **任务顺序 / 标志映射 / 类别别名**：`config/race_abot_411_route_a.yaml`
- **不要**直接改 `scripts/` 里的数值参数 —— 现场改 yaml 就够，且不会被 git 冲突烦到
