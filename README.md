# race_abot_411 —— 无人车自动驾驶任务挑战赛（A 路线）独立实现

> 按《附件3-无人车自动驾驶任务挑战赛比赛规则》+《标识.pdf / 标识-B路线.pdf》重写的一套干净代码。
> **不修改原 `traffic_detector` / `abot_project` 任何文件**，本包所有文件以 `race_abot_411_` 前缀命名，
> 与其他队伍共用同一辆小车时不会互相覆盖。

---

## 1. 与原代码的关系

| | 原 `traffic_detector` | 本包 `race_abot_411` |
|---|---|---|
| 任务组织 | 隐式顺序约束（`turn_left>=3` 才允许倒车…） | **显式任务队列**（读 `config/race_abot_411_route_a.yaml`） |
| 类别名 | 代码里写死 `cefang`/`turn`… | **归一化映射**，换模型不改代码 |
| 动作 | `publish_cmd_vel + sleep` 开环 | **里程计/超声闭环 + 超时兜底** |
| 文件 | 25 个脚本大量重复 | **7 个文件，无重复** |
| 避障/单边桥/S弯/车库搬运 | 完全没有 | **已实现** |

原来那 25 个脚本里只有 `core_new.py` 是现行主程序，其余是历史版本（`08.py`、`123456.py`、`xiaotao（b）.py`、
`xunji/xunjib/xunji1/xunji1gai/xunjibgai`…，其中 `08=123456=mmmdd=xunji`、`keepB=xunjibgai` 是整文件重复）。
**本包不复用、也不修改它们**；如需清理请自行确认后删除（见第 8 节）。

---

## 2. 目录结构

```
race_abot_411/
├── package.xml
├── CMakeLists.txt
├── launch/
│   ├── race_abot_411_a.launch         # A 路线一键启动
│   └── race_abot_411_debug.launch     # 调试（可视化 + 独立终端）
├── config/
│   ├── race_abot_411_params.yaml      # 全部可调参数（现场只改这个）
│   └── race_abot_411_route_a.yaml     # A 路线任务队列 + 标志→动作映射 + 类别归一化
├── scripts/
│   ├── race_abot_411_common.py        # 公共：角度/EMA/类别归一化/底盘接口 AkmIO
│   ├── race_abot_411_lane.py          # 巡线：HSV 黄线 + 分带取中 + 平滑限幅
│   ├── race_abot_411_actions.py       # 动作库（含避障/过桥/S弯/泊车/车库搬运）
│   ├── race_abot_411_node.py          # 主节点：任务队列状态机
│   ├── race_abot_411_sign_detect.py   # 标志检测（YOLOv8 + 中心区域 + 归一化）
│   ├── race_abot_411_fake_sign.py     # 调试：伪造标志（无模型/无场地也能联调）
│   └── race_abot_411_diagnostics.py   # 现场自检（上车第一步先跑）
└── tools/
    ├── race_abot_411_record.sh        # 录 bag
    └── race_abot_411_rename.sh        # 一键改前缀成你们队名
```

---

## 3. 与规则的对应关系（A 路线）

| 规则任务点 | 分值 | 触发标志 | 动作函数 | 状态 |
|---|---|---|---|---|
| 绿灯启动 | 5 | `green_light` | `start` | ✅ |
| 转向标识 | 5 | `turn_left` / `turn_right` | `turn_left/right`（90°闭环） | ✅ |
| 单边桥 ×2 | 10 | `single_bridge` | `cross_bridge`（低速居中） | ✅ 代码就绪，**标志需补标注** |
| 直角弯 ×2 | 10 | 无标志 | `lane_only`（巡线通过） | ✅ |
| 避障区 | 15 | `obstacle` | `obstacle_avoid`（`/sonar` 绕障） | ✅ 代码就绪，**标志需补标注** |
| 侧方停车 | 20 | `parallel_parking` | `parallel_parking` | ✅ 参数化动作串 |
| 倒入车库 A | 20 | `reverse_parking` | `reverse_parking` | ✅ 参数化动作串 |
| 车库 A→B | 20 | 无标志 | `garage_transfer` | ✅ 固定串（后续可换定位） |
| 定点停车 | 5 | `stop` | `stop_wait` | ✅ |
| S 弯正进 | 5 | 无标志 | `s_curve_in` | ✅ 巡线到末端 |
| 反向倒出 S 弯 | 5 | `reverse` | `s_curve_out`（倒车巡线） | ✅ |
| 终点 | 10 | `finish` | `finish` | ✅ |

**标志清单（10 种 + 红绿灯）**：左转、右转、停留(红STOP)、障碍物(黄跑车)、倒车(黄三角)、
翻转(循环箭头)、终点(FINISH)、侧方位停车(蓝P)、单边桥、倒车入库 + 红/绿灯。

---

## 4. 部署到车上

### 4.1 放到哪个文件夹

放小车上 ROS 工作区的 **`src/`** 目录里，即 **`~/catkin_ws/src/`**（本车工作区路径由编译缓存确认）：

```
~/catkin_ws/
├── src/
│   ├── abot_project/          ← 车上原有，别动
│   ├── traffic_detector/      ← 车上原有，别动
│   └── race_abot_411/         ← ★ 本包放这里（与其他包并存，不覆盖任何东西）
├── build/
└── devel/
```

### 4.2 放进去不等于能跑，要走完这三步

```bash
# ① 拷进去（会新建一个 race_abot_411 目录）
scp -r race_abot_411 abot@<车IP>:~/catkin_ws/src/

# ② 加可执行权限 + 编译（让 catkin 注册这个包）
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
chmod +x src/race_abot_411/scripts/*.py src/race_abot_411/tools/*.sh
catkin_make                    # 全量；想快可以: catkin_make --pkg race_abot_411 abot_driver
source devel/setup.bash        # ★ 每次开新终端都要 source

# ③ 验证包能被找到（能打印出 help 就说明装好了）
rosrun race_abot_411 race_abot_411_diagnostics.py \
  _params_yaml:=$(rospack find race_abot_411)/config/race_abot_411_params.yaml \
  _route_yaml:=$(rospack find race_abot_411)/config/race_abot_411_route_a.yaml
```

> 为什么不能"放进去直接 roslaunch"：`roslaunch race_abot_411 ...` 要找的是**包名**，
> 而包名是靠 `catkin_make` + `source devel/setup.bash` 注册进 ROS 环境的。
> 不编译、不 source，就会报 `[race_abot_411] is not a package`。

### 4.3 跑起来之前必须确认的四件事

| # | 项目 | 怎么确认 | 不做会怎样 |
|---|---|---|---|
| 1 | 模型路径 | 改 `config/race_abot_411_params.yaml` 的 `detect.model_path`（默认 `/home/abot/yolov8/best.pt`） | 检测节点报"模型文件不存在"，一个标志都识别不到 |
| 2 | 环境变量 | `CAMERA`、`CMM_MODE=serial` 等已设（通常在 `~/.bashrc`） | `cam.launch` 直接报 `env XXX is not set` |
| 3 | 底盘与相机 | `abot_driver` 能出 `/odom`、`/battery`；相机出 `/camera/rgb/image_raw` | 车不动 / 巡线失效（自检脚本会标红） |
| 4 | ultralytics | `python3 -c "import ultralytics"` | 标志检测节点起不来（自检脚本会警告） |

超声 `/sonar` 没有也能跑，只是避障会退化为"低速直行通过"。

### 4.4 如果车上没有 abot 包

`race_abot_411_a.launch` 默认会 `include $(find abot)/launch/cam.launch` 一起起相机。
车上若没有 `abot` 这个包，就加 `start_camera:=false`，自己在别处起相机即可：
```bash
roslaunch race_abot_411 race_abot_411_a.launch start_camera:=false
```

---

## 5. 启动

```bash
# A 路线正常跑
roslaunch race_abot_411 race_abot_411_a.launch

# 换模型（强烈建议用 aaabest.pt，它有红/黄灯）
roslaunch race_abot_411 race_abot_411_a.launch model_path:=/home/abot/yolov8/aaabest.pt

# 调试模式（可视化 + 独立终端，键盘可用）
roslaunch race_abot_411 race_abot_411_debug.launch
```

**跑之前必须先改的路径**：`config/race_abot_411_params.yaml` 里的
`detect.model_path`（现在写的是 `/home/abot/yolov8/best.pt`）。

---

## 6. 调试

### 6.0 上车第一步：现场自检
```bash
rosrun race_abot_411 race_abot_411_diagnostics.py \
    _params_yaml:=$(rospack find race_abot_411)/config/race_abot_411_params.yaml \
    _route_yaml:=$(rospack find race_abot_411)/config/race_abot_411_route_a.yaml
```
会逐项检查：ROS 环境 / `abot_driver.Akm` 与 cv_bridge / 模型文件 / 参数与路线文件 /
任务队列与总分 / 标志映射完整性 / 类别缺口 / 相机·里程计·超声·控制话题是否真有数据。
**红色项必须先解决再上车。**

### 6.1 无模型 / 无场地：伪造标志联调
```bash
# 终端1
roslaunch race_abot_411 race_abot_411_a.launch start_camera:=false model_path:=/dev/null
# 终端2：交互式发标志
rosrun race_abot_411 race_abot_411_fake_sign.py interactive
# 或者按路线文件顺序自动发一遍
rosrun race_abot_411 race_abot_411_fake_sign.py route ~/catkin_ws/src/race_abot_411/config/race_abot_411_route_a.yaml
```

### 6.2 键盘调试（需 `xterm -e` 启动主控）
| 键 | 作用 |
|---|---|
| `空格` | 开/关巡线 |
| `1`~`9` | 手动触发：1左转 2倒车入库 3侧方停车 4右转 5过单边桥 6停留 7倒车出S弯 8翻转 9避障 |
| `n` | 跳过当前任务 |
| `r` | 重置队列 |
| `s` | 打印状态（进度/转向/偏航/超声） |
| `q` | 退出 |

### 6.3 录包回放
```bash
bash tools/race_abot_411_record.sh lap1
rosbag play --loop ~/bags/lap1.bag
```

---

## 7. 调参入口

**只改 `config/race_abot_411_params.yaml`**（或 launch 里用 rosparam 覆盖）：
- 巡线：`lane.speed` / `lane.steering_gain` / `lane.steering_ema` / `lane.single_line_offset_px`
- 标志：`detect.confidence` / `detect.center_*_ratio`
- 动作：`actions.turn_*` / `actions.parallel_parking.slots` / `actions.avoid.*` / `actions.bridge.*`
- 安全：`safety.no_steer_timeout_s` / `safety.max_task_time_s`

**任务顺序**只改 `config/race_abot_411_route_a.yaml` 的 `tasks` 列表，代码不动。

---

## 8. 已知待办

1. **A 路线的确切任务顺序需在真实赛道核对一次**（规则文字没给出"左转×3 / 翻转"的插点位置）。
   核对后只改 `race_abot_411_route_a.yaml` 即可。
2. **必须补两个标志的标注并重训**：`obstacle`（黄色跑车，15 分）、`single_bridge`（通过单边桥，10 分）——
   现有 4 个权重都没有这两类。详见 `race_abot_411_route_a.yaml` 的 `class_coverage`。
3. 红灯：只有 `aaabest.pt` 含 `red`，建议换它。
4. 原 `traffic_detector/scripts/` 有 20+ 个重复历史脚本可清理（`08.py`、`123456.py`、
   `xiaotao（b）.py`、`xunji*`、`mmmdd`、`yuan`、`car/car1`、`15*`、`keep*` 等），
   但**本次未删除任何文件**，需要清理请先备份再自行确认。

---

## 9. 改成你们队名（共用小车辨识度）

```bash
bash tools/race_abot_411_rename.sh                              # 干跑，先看会改什么
bash tools/race_abot_411_rename.sh --apply race_你的队名         # 真正改
```

> 注意：新名字必须**全小写**（字母/数字/下划线，以字母开头，不能有连字符、空格、中文），
> 例如 `race_abot_411`、`race_xiaoche`。改完全局替换（文件名+目录名+文件内容），
> 所以节点名和话题名也会一起变，正好避免和其他队伍撞名；但队友脚本里写死的旧话题名要同步改。
> 改完后清缓存再编译：`rm -rf build/<新包名>* devel/lib/python3/dist-packages/<新包名>* && catkin_make`

---

## 10. 变更记录

### v1.1（本次完善）
- **包名/前缀改为 `race_abot_411`**（按需求加 411 后缀；全包统一前缀，与队友隔离）。
- **新增 `race_abot_411_diagnostics.py` 现场自检节点**：上车先跑，一次性报告环境/依赖/模型/
  配置/话题/类别缺口，避免"上车才发现没数据"。
- **红灯等待改为非阻塞**：原先红灯时会在主循环里同步 `sleep`，把巡线、相机超时保护、
  任务看门狗全卡住；现在放到独立线程，主循环继续运行。
- **任务看门狗真正生效**：`max_task_time_s` 现在会强制中止超时任务并恢复巡线
  （之前只打印一句日志，任务仍会一直挂着）；动作库的按段执行与停留动作也接入了中止检查。
- **新增空闲补发停止指令**：既不巡线也不执行动作时，按 `idle_stop_hz` 周期补发 0 速度，
  防止底盘保持上一帧速度继续跑（原版没动作时完全不发指令）。
- **新增启动 3 秒自检报告**：启动后自动报告相机/里程计/超声/检测四路是否有数据。
- **里程计失联告警**：超过 `odom_stale_warn_s` 没收到 `/odom` 会周期告警（闭环动作会失效）。
- 显示节流由 `time` 取模改为帧计数，刷新更稳定；`loginfo_throttle` 替代重复日志刷屏。

### v1.0
- 首个版本：任务队列状态机 + 巡线 + 标志检测 + 动作库（含避障/过桥/S弯/泊车/车库搬运）。
