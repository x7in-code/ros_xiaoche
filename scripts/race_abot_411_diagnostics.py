#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
race_abot_411_diagnostics.py —— 现场自检（上车第一步先跑这个）

检查项：
  1. roscore / ROS 版本
  2. 依赖包与消息类型是否可用（abot_driver/Akm、cv_bridge）
  3. 模型文件是否存在
  4. 参数文件 / 路线文件是否可读、任务队列是否完整
  5. 关键话题是否有人在发（相机 / 里程计 / 超声 / 检测）
  6. 类别覆盖缺口（哪些标志现有模型识别不了）

用法：
  rosrun race_abot_411 race_abot_411_diagnostics.py
  rosrun race_abot_411 race_abot_411_diagnostics.py _params_yaml:=/路径/params.yaml
"""

import os
import sys

import rospy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from race_abot_411_common import load_yaml  # noqa: E402

OK = "  [OK]   "
WARN = "  [警告] "
FAIL = "  [失败] "


def hr(title):
    print("\n" + "=" * 62)
    print("  " + title)
    print("=" * 62)


def check_ros():
    hr("1. ROS 环境")
    try:
        distro = rospy.get_param('/rosdistro', '?')
        print(OK + "ROS 发行版: %s" % distro)
    except Exception:
        print(WARN + "无法读取 /rosdistro 参数")
    print(OK + "Python: %s" % sys.version.split()[0])
    if sys.version_info[0] < 3:
        print(FAIL + "必须是 Python 3（ROS Noetic）")


def check_deps():
    hr("2. 依赖与消息类型")
    try:
        from abot_driver.msg import Akm
        print(OK + "abot_driver/Akm 消息可用（linear/angular/diff）")
    except Exception as e:
        print(FAIL + "abot_driver/Akm 不可用: %s" % e)
        print("         -> 先编译 abot_driver: catkin_make --pkg abot_driver")
    try:
        import cv_bridge  # noqa: F401
        print(OK + "cv_bridge 可用")
    except Exception as e:
        print(FAIL + "cv_bridge 不可用: %s" % e)
    try:
        import cv2
        print(OK + "opencv 可用 (%s)" % cv2.__version__)
    except Exception as e:
        print(FAIL + "opencv 不可用: %s" % e)
    try:
        import numpy
        print(OK + "numpy 可用 (%s)" % numpy.__version__)
    except Exception as e:
        print(FAIL + "numpy 不可用: %s" % e)
    try:
        import yaml  # noqa: F401
        print(OK + "pyyaml 可用")
    except Exception as e:
        print(FAIL + "pyyaml 不可用: %s" % e)


def check_model(params):
    hr("3. YOLO 模型")
    p = ((params.get('detect') or {}).get('model_path')) or ''
    if not p:
        print(WARN + "params 里没写 detect.model_path")
        return
    if os.path.isfile(p):
        size = os.path.getsize(p) / 1024.0 / 1024.0
        print(OK + "模型存在: %s (%.1f MB)" % (p, size))
    else:
        print(FAIL + "模型不存在: %s" % p)
        print("         -> 改 config/race_abot_411_params.yaml 的 detect.model_path")
    try:
        import ultralytics
        print(OK + "ultralytics %s" % ultralytics.__version__)
    except Exception:
        print(WARN + "ultralytics 未安装 -> pip3 install ultralytics")


def check_config(params, route, params_path, route_path):
    hr("4. 配置文件与任务队列")
    for name, path, cfg in (("参数", params_path, params),
                            ("路线", route_path, route)):
        if path and os.path.isfile(path):
            print(OK + "%s文件可读: %s" % (name, path))
        else:
            print(WARN + "%s文件未提供或不存在: %s" % (name, path))
    tasks = route.get('tasks') or []
    if not tasks:
        print(FAIL + "任务队列为空！")
        return
    total = 0
    print(OK + "任务队列 %d 项：" % len(tasks))
    for t in tasks:
        total += t.get('score', 0)
        print("         #%-4s %-18s trigger=%-5s sign=%-18s action=%s"
              % (t.get('id'), t.get('name', ''), t.get('trigger', ''),
                 str(t.get('sign')), t.get('action', '')))
    print(OK + "官方口径总分: %d" % total)

    # 标志映射完整性
    sign_action = route.get('sign_action') or {}
    miss = [t.get('sign') for t in tasks
            if t.get('sign') and t.get('sign') not in sign_action]
    if miss:
        print(WARN + "以下标志在 sign_action 里没有映射: %s" % sorted(set(miss)))
    else:
        print(OK + "所有任务用到的标志都有动作映射")

    # 类别覆盖缺口
    cov = route.get('class_coverage') or {}
    gaps = [k for k, v in cov.items() if not v.get('have')]
    if gaps:
        print(FAIL + "以下标志现有模型没有，需要补标注重训: %s" % gaps)
        for g in gaps:
            print("         - %s: %s" % (g, (cov.get(g) or {}).get('note', '')))
    else:
        print(OK + "类别覆盖完整")


def check_topics(params):
    hr("5. 话题连通性（每个等 2 秒）")
    tp = params.get('topics') or {}
    published = dict(rospy.get_published_topics())

    def probe(desc, topic, msg_type):
        if not topic:
            print(WARN + "%s: 未配置话题名" % desc)
            return
        if topic not in published:
            print(WARN + "%s: 话题 %s 当前无人发布" % (desc, topic))
            return
        try:
            rospy.wait_for_message(topic, msg_type, timeout=2.0)
            print(OK + "%s: %s 有数据" % (desc, topic))
        except Exception:
            print(WARN + "%s: 话题存在但 2 秒内没收到消息 (%s)" % (desc, topic))

    from sensor_msgs.msg import Image
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32

    probe("前视相机", tp.get('camera_front'), Image)
    probe("标志相机", tp.get('camera_sign'), Image)
    probe("里程计", tp.get('odom'), Odometry)
    probe("超声", tp.get('sonar'), Float32)

    cmd = tp.get('cmd')
    if cmd in published:
        print(OK + "控制输出: %s 已被订阅/发布" % cmd)
    else:
        print(WARN + "控制输出: %s 当前无人订阅（底盘驱动没起？）" % cmd)


def main():
    rospy.init_node('race_abot_411_diagnostics', anonymous=True, disable_signals=True)
    params_path = rospy.get_param('~params_yaml', '')
    route_path = rospy.get_param('~route_yaml', '')
    params = load_yaml(params_path)
    route = load_yaml(route_path)

    print("\n" + "#" * 62)
    print("#  race_abot_411 现场自检")
    print("#" * 62)

    check_ros()
    check_deps()
    check_model(params)
    check_config(params, route, params_path, route_path)
    check_topics(params)

    hr("结论")
    print("  红色/失败项必须解决；警告项按需处理。")
    print("  全部通过后即可： roslaunch race_abot_411 race_abot_411_a.launch")
    print("")


if __name__ == '__main__':
    main()
