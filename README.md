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
├── README.md                          # 本文
├── DEPLOY.md                          # ★ 换车部署清单（明天换车先看这个）
├── launch/
│   ├── race_abot_411_a.launch         # A 路线一键启动
│   └── race_abot_411_debug.launch     # 调试（可视化）
├── config/
│   ├── race_abot_411_params.yaml      # 全部可调参数（现场只改这个）
│   └── race_abot_411_route_a.yaml     # A 路线任务队列 + 标志→动作映射 + 类别归一化
├── scripts/
│   ├── race_abot_411_common.py        # 公共：角度/EMA/类别归一化/底盘接口/换车自适应
│   ├── race_abot_411_lane.py          # 巡线：HSV 黄线 + 分带取中 + 平滑限幅
│   ├── race_abot_411_actions.py       # 动作库（含避障/过桥/S弯/泊车/车库搬运）
│   ├── race_abot_411_node.py          # 主节点：任务队列状态机
│   ├── race_abot_411_sign_detect.py   # 标志检测（YOLOv8 + 中心区域 + 归一化）
│   ├── race_abot_411_fake_sign.py     # 调试：伪造标志（无模型/无场地也能联调）
│   └── race_abot_411_diagnostics.py   # 现场自检（上车第一步先跑）
└── tools/
    ├── race_abot_411_start.sh         # ★ 一键启动 / 体检 / 停止（换车友好）
    ├── race_abot_411_update.sh        # 车上一键更新（git pull + 修 CRLF + 编译）
    ├── race_abot_411_record.sh        # 录 bag
    └── race_abot_411_rename.sh        # 一键改前缀成你们队名
```

---

## 3. 与规则的对应关系（A 路线）

| 规则任务点 | 分值 | 触发标志 | 动作函数 | 状态 |
|---|---|---|---|---|
| 绿灯启动 | 5 | `green_light` | `start` | ✅ |
| 转向标识 | 5 | `turn_left` / `turn_right` | `turn_left/right`（90°闭环） | ✅ |
| 单边桥 ×2 | 10 | `single_bridge`(实为 `danbain`) | `cross_bridge`（低速居中） | ✅ 类别已确认存在 |
| 直角弯 ×2 | 10 | 无标志 | `lane_only`（巡线通过） | ✅ |
| 避障区 | 15 | `obstacle`(实为 `zhangai`) | `obstacle_avoid`（`/sonar` 绕障） | ✅ 类别已确认存在 |
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

### 4.0 从小车 Git 拉取代码（推荐方式）

仓库：`https://github.com/x7in-code/ros_xiaoche.git`（分支 `main`）

> ⚠️ **注意：仓库名是 `ros_xiaoche`，但包名是 `race_abot_411`**。
> 直接在 `src` 里 `git clone` 会得到 `src/ros_xiaoche/`，**目录名与包名不一致，catkin 会警告/不认**。
> 所以 **必须指定目标目录名**：

```bash
# 0) 车上装 git（如果还没有）
sudo apt install -y git

# 1) clone 到 src，并指定目录名为包名（关键！）
cd ~/catkin_ws/src
git clone https://github.com/x7in-code/ros_xiaoche.git race_abot_411
# 网络慢可加 --depth 1 只拉最新一次提交：
# git clone --depth 1 https://github.com/x7in-code/ros_xiaoche.git race_abot_411

# 2) 确认目录名和包名一致
grep "<name>" race_abot_411/package.xml      # 应输出 race_abot_411

# 3) 加权限 + 编译 + source
chmod +x race_abot_411/scripts/*.py race_abot_411/tools/*.sh
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

**以后更新代码**（队友推了新提交）：
```bash
bash ~/catkin_ws/src/race_abot_411/tools/race_abot_411_update.sh
# 该脚本 = git pull + 修 CRLF + chmod + catkin_make + source，一条命令搞定
```

私有仓库需要认证：HTTPS 用「用户名 + Personal Access Token」，或配 SSH key（`ssh-keygen -t ed25519` 后把公钥加到 GitHub）。

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
| 1 | 模型路径 | **留空即自动查找**（`params.yaml` 的 `detect.model_path` 默认空，会按候选顺序找 aaabest.pt）；想固定就填绝对路径 | 自动也找不到时，检测节点报"模型文件不存在"，一个标志都识别不到 |
| 2 | 环境变量 | 只有 `start_camera:=true`（用 abot 的 cam.launch）时才需要 `CAMERA`、`CMM_MODE` 等 | 那种情况下 `cam.launch` 会报 `env XXX is not set` |
| 3 | 底盘与相机 | `abot_driver` 能出 `/akm_vel`、`/odom`、`/battery`；相机有图 | 车不动 / 巡线失效（自检脚本会标红） |
| 4 | ultralytics | `python3 -c "import ultralytics"` | 标志检测节点起不来（自检脚本会警告） |

超声 `/sonar` 没有也能跑，只是避障会退化为"低速直行通过"。

### 4.4 如果车上没有 abot 包

`race_abot_411_a.launch` 的 `start_camera:=true` 会 `include $(find abot)/launch/cam.launch`。
车上若没有 `abot` 这个包，用默认的 `start_camera:=false`（**默认已是 false**），自己在别处起相机即可。

### 4.5 换一辆车（明天不用同一台车）

**一句话**：`放代码 → 编译 → 体检 → 启动`，细节看 **[DEPLOY.md](DEPLOY.md)**。

```bash
cd ~/catkin_ws/src/race_abot_411

bash tools/race_abot_411_start.sh check    # ① 只读体检：ROS/工作区/白名单/相机设备/模型/依赖
cd ~/catkin_ws && catkin_make --pkg race_abot_411 abot_driver && source devel/setup.bash   # ②
cd ~/catkin_ws/src/race_abot_411
bash tools/race_abot_411_start.sh          # ③ 一键启动（自动清场+master+驱动+相机+主系统）
```

本包对换车做了这些**自动适配**，一般不用改代码：

| 会变的 | 自动适配方式 |
|---|---|
| 模型权重路径 | `detect.model_path` 留空 → 按候选顺序 + glob 查找，**优先 aaabest.pt** |
| 相机话题名 | `topics.camera_*` 留空 → 自动挑一路**真的在发布**的话题 |
| 相机设备号 | 启动脚本逐个试 `/dev/video*`，**直到出图**为止 |
| master 地址 | 启动脚本统一设好并写进 `~/race_abot_env.sh`；连共享 master 用 `MASTER=` 覆盖 |

**每个新终端**记得先对齐环境（避免 `Unable to communicate with master`）：

```bash
source ~/race_abot_env.sh
```

---

## 5. 启动

```bash
# 推荐：一键启动（自动清场 + master + 底盘 + 相机 + 主系统）
bash tools/race_abot_411_start.sh
bash tools/race_abot_411_start.sh check     # 只体检
bash tools/race_abot_411_start.sh stop      # 全停

# 或者用 roslaunch
roslaunch race_abot_411 race_abot_411_a.launch          # 模型/相机话题都会自动找
roslaunch race_abot_411 race_abot_411_a.launch model_path:=/路径/aaabest.pt   # 手动指定模型
roslaunch race_abot_411 race_abot_411_debug.launch      # 调试模式（可视化）
```

> `detect.model_path` 留空时**不需要再改路径**——节点会自动查找并打印实际使用的路径
> （`模型路径: /xxx/aaabest.pt`）。日志里同时会打印 `模型类别(13): [...]`，这是判断加载成功的标志。

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
2. ~~必须补两个标志的标注并重训~~ —— **已排除**。2026-10-16 实机跑通 `aaabest.pt`，
   实测它有 13 类，其中 `zhangai`(=障碍物, 15分) 与 `danbain`(=单边桥, 10分) **本来就有**。
   之前判断"缺失"是因为解包 .pt 时用英文关键词搜索而漏掉了拼音命名，已更正。
   ⚠️ 注意拼写是 **`danbain`** 不是 `danbian`（原 `core_new.py` 判断的 `'danbian'` 拼错，
   那个分支从来没生效过）；`class_alias` 里两种拼写都已收录。
3. **统一用 `aaabest.pt`**：4 个权重里只有它同时含 `red`/`yellow`(红绿灯) + `zhangai` + `danbain`，
   是唯一能覆盖全部任务点的模型。
4. `yellow`（黄灯）暂未映射，会被忽略 —— 规则只要求红/绿。
5. 原 `traffic_detector/scripts/` 有 20+ 个重复历史脚本可清理（`08.py`、`123456.py`、
   `xiaotao（b）.py`、`xunji*`、`mmmdd`、`yuan`、`car/car1`、`15*`、`keep*` 等），
   但**本次未删除任何文件**，需要清理请先备份再自行确认。
6. 小车实测注意：`roscore` 要单独终端常驻；多网卡环境建议
   `export ROS_MASTER_URI=http://127.0.0.1:11311` + `export ROS_IP=127.0.0.1`
   （`tools/race_abot_411_start.sh` 已内置）。

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

### v1.2 —— 实机跑通 + 换车自适应
- **实机验证通过**：在 Ubuntu 20.04 + Noetic 的 abot 小车上跑通了
  编译 → 包注册 → 主节点 → 检测节点 → 相机 → 底盘 → 模型加载（13 类）全链路。
- **★ 类别别名按实机更正**：`aaabest.pt` 用的是**拼音**，
  真实类别为 `zhangai`（障碍物，15 分项）与 `danbain`（单边桥，10 分项，
  注意拼写不是 `danbian`）。已加入别名表并实测映射全绿 —— **25 分项无需重新标注训练**。
- **★ 换车自适应（本次核心）**：新增 `resolve_camera_topic()` 与 `resolve_model_path()`，
  相机话题和模型权重都可以**留空自动找**：
  - 相机：按候选列表挑一路**真的在发布**的话题，配置的没发布就自动回退并打 WARN；
  - 模型：按候选路径 + glob 搜索，**优先 `aaabest.pt`**（只有它含红/黄灯）。
- **★ 启动脚本重写为 `start | check | stop` 三模式**：
  - `check` 只读体检：ROS/工作区/**编译白名单**/关键包/`/dev/video*`/模型/依赖；
  - `start` 相机**逐个试 `/dev/video*` 直到真正出图**（用 `rostopic hz` 验证，不只是看发布者）；
  - 生成 `~/race_abot_env.sh`，解决"新终端连不上 master"；
  - 支持 `MODEL= / CAM_DEV= / USE_CAMERA=0 / MASTER= / WS=` 覆盖。
- **修复模型路径被 launch 覆盖**：`race_abot_411_a.launch` 的 `model_path` 默认改为空，
  以 `params.yaml` 为唯一权威配置源（原先 launch 里的硬编码会覆盖 yaml，改 yaml 不生效）。
- **`camera_sign` 默认不再写死 `/camera/right/image_raw`**（实机那台车没有这个话题，
  只有 `/camera/rgb` 与 `/camera/back`），改为留空自动挑选。
- **`debug.launch` 默认值改稳**：`start_camera=false`、`node_prefix=""`
  （原先默认 `xterm -e`，新车没装 xterm 会直接起不来）。
- **新增 `DEPLOY.md`**：换车部署清单 + 换车差异对照表 + 故障排查表。
- 新增 `model_candidates` / `model_globs` / `camera_candidates` 配置项。

### v1.1
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
- **新增 `.gitattributes`（`* text=auto eol=lf`）+ `.gitignore`**：从仓库层面强制 LF 换行，
  防止有人在 Windows 提交 CRLF 后，小车 clone 下来报 `bad interpreter: /usr/bin/env python3^M`。
- **新增 `tools/race_abot_411_update.sh`**：车上一键更新（git pull → 修 CRLF → chmod → catkin_make → source）。

### v1.0
- 首个版本：任务队列状态机 + 巡线 + 标志检测 + 动作库（含避障/过桥/S弯/泊车/车库搬运）。
