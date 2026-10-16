#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
race_abot_411_common.py —— 公共工具（被主节点 / 动作库 import）

提供：
  - 角度/数值工具：wrap_angle / clamp / rate_limit
  - 类别名归一化：build_alias_index / normalize_class
  - 参数加载：load_yaml / merge_params
  - 底盘接口：AkmIO（发布 abot_driver/Akm，读里程计 yaw）
"""

import math
import os
import threading
import time

import rospy
import yaml
from abot_driver.msg import Akm
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ----------------------------------------------------------------------
# 数值工具
# ----------------------------------------------------------------------
def wrap_angle(a):
    """把角度规范到 [-pi, pi]"""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def rate_limit(new, old, max_step):
    """限制单步变化量，防抖"""
    d = new - old
    if d > max_step:
        d = max_step
    elif d < -max_step:
        d = -max_step
    return old + d


class EMA(object):
    """指数移动平均"""

    def __init__(self, alpha=0.35):
        self.alpha = alpha
        self.value = None

    def update(self, x):
        if self.value is None:
            self.value = x
        else:
            self.value = self.alpha * x + (1.0 - self.alpha) * self.value
        return self.value

    def reset(self):
        self.value = None


# ----------------------------------------------------------------------
# 类别名归一化
# ----------------------------------------------------------------------
def build_alias_index(class_alias):
    """
    输入: {规范名: [别名...]}
    输出: {任意名字(小写): 规范名}
    """
    index = {}
    for canonical, aliases in (class_alias or {}).items():
        index[canonical.lower()] = canonical
        for a in (aliases or []):
            index[str(a).lower()] = canonical
    return index


def normalize_class(name, alias_index):
    """把检测到的原始类别名映射成规范名；映射不到就原样返回"""
    if not name:
        return name
    return alias_index.get(str(name).lower(), name)


# ----------------------------------------------------------------------
# 参数加载
# ----------------------------------------------------------------------
def load_yaml(path):
    """安全加载 yaml；失败返回 {}"""
    try:
        if path and os.path.isfile(path):
            with open(path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f) or {}
    except Exception as e:
        rospy.logwarn("加载 yaml 失败 %s: %s", path, e)
    return {}


def deep_merge(base, override):
    """override 覆盖 base（递归）"""
    if not isinstance(base, dict):
        return override
    out = dict(base)
    for k, v in (override or {}).items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


# ----------------------------------------------------------------------
# 底盘接口
# ----------------------------------------------------------------------
class AkmIO(object):
    """
    统一底盘出口：
      - 发布 abot_driver/Akm 到 cmd_topic（默认 /akm_vel）
      - 同时兼容标准 Twist（有些车桥接用 /cmd_vel）
      - 订阅 odom 维护 current_yaw / current_pose
    """

    def __init__(self, cmd_topic='/akm_vel', odom_topic='/odom',
                 also_publish_twist=False, twist_topic='/cmd_vel',
                 diff_ratio=0.27):
        self.cmd_topic = cmd_topic
        self.diff_ratio = diff_ratio

        self.current_yaw = 0.0
        self.current_pose = None
        self.last_odom_time = 0.0
        self._lock = threading.Lock()

        self.pub = rospy.Publisher(cmd_topic, Akm, queue_size=10)
        self.pub_twist = None
        if also_publish_twist:
            self.pub_twist = rospy.Publisher(twist_topic, Twist, queue_size=10)

        self.sub = rospy.Subscriber(odom_topic, Odometry, self._odom_cb)

    def _odom_cb(self, msg):
        from tf.transformations import euler_from_quaternion
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        with self._lock:
            self.current_pose = msg.pose.pose
            self.current_yaw = yaw
            self.last_odom_time = time.time()

    def get_yaw(self):
        with self._lock:
            return self.current_yaw

    def get_pose(self):
        with self._lock:
            return self.current_pose

    def odom_healthy(self, timeout=1.0):
        return (time.time() - self.last_odom_time) < timeout

    # ---- 控制输出 ----
    def drive(self, linear, angular, diff=None):
        """发一帧速度指令"""
        if diff is None:
            diff = angular * self.diff_ratio
        cmd = Akm()
        cmd.linear = float(linear)
        cmd.angular = float(angular)
        cmd.diff = float(diff)
        self.pub.publish(cmd)
        if self.pub_twist is not None:
            t = Twist()
            t.linear.x = float(linear)
            t.angular.z = float(angular)
            self.pub_twist.publish(t)

    def stop(self, times=1):
        for _ in range(max(1, times)):
            self.drive(0.0, 0.0, 0.0)

    def drive_for(self, linear, angular, duration, rate_hz=20, watch=None):
        """
        持续发速度 duration 秒。
        watch: 可选回调 () -> bool，返回 True 表示提前结束（例如超时/到位判定）
        返回实际持续时间
        """
        r = rospy.Rate(rate_hz)
        t0 = time.time()
        while not rospy.is_shutdown():
            if time.time() - t0 >= duration:
                break
            if watch is not None and watch():
                break
            self.drive(linear, angular)
            r.sleep()
        self.stop()
        return time.time() - t0

    def rotate_to(self, target_yaw, max_angular=1.2, tolerance=0.10,
                  timeout=8.0, linear=0.0, rate_hz=20, invert=False):
        """
        原地/边走边转到目标偏航角（闭环 + 超时兜底）。
        返回 True=正常到位, False=超时
        """
        r = rospy.Rate(rate_hz)
        t0 = time.time()
        ok = False
        while not rospy.is_shutdown():
            if time.time() - t0 > timeout:
                rospy.logwarn("rotate_to 超时 (target=%.2f rad)", target_yaw)
                break
            err = wrap_angle(target_yaw - self.get_yaw())
            if abs(err) < tolerance:
                ok = True
                break
            ang = clamp(err * 2.0, -max_angular, max_angular)
            if invert:
                ang = -ang
            self.drive(linear, ang)
            r.sleep()
        self.stop()
        return ok

    def rotate_by(self, delta_rad, **kw):
        """相对当前角度转 delta_rad（正=左转）"""
        target = wrap_angle(self.get_yaw() + delta_rad)
        return self.rotate_to(target, **kw)

    def drive_until(self, linear, angular, condition, timeout=10.0, rate_hz=20):
        """
        边走边等到 condition() 为 True，或超时。
        用于"走到某距离/某标志"的地方。
        返回 True=条件满足, False=超时
        """
        r = rospy.Rate(rate_hz)
        t0 = time.time()
        while not rospy.is_shutdown():
            if condition():
                self.stop()
                return True
            if time.time() - t0 > timeout:
                rospy.logwarn("drive_until 超时")
                self.stop()
                return False
            self.drive(linear, angular)
            r.sleep()
        self.stop()
        return False
