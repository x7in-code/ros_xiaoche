#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
race_abot_411_node.py —— 主节点：任务队列状态机 + 巡线 + 动作调度

设计要点（相对原 traffic_detector/core_new.py 的改进）：
  1. 【任务队列】不再是隐式顺序约束，而是读 race_abot_411_route_a.yaml 的显式队列，
     每步只判断"检测到的标志 == 队首期望"，命中就执行并前进。加/改任务点只改 yaml。
  2. 【类别归一化】检测回调先做别名映射，换模型不用改代码。
  3. 【动作闭环】所有动作在 race_abot_411_actions.py 里，靠里程计/超声闭环 + 超时兜底。
  4. 【可中断】动作执行期间若又检测到新标志，会置 pending，动作可提前退出。
  5. 【安全】相机超时停车 / 任务看门狗超时强制中止 / 空闲补发停止指令 /
     里程计失联告警 / 启动 3 秒后自动自检报告。

调试接口（keyboard_enabled=true 时）：
  空格 = 开/关巡线      1..9 = 手动触发动作
  n    = 跳过当前任务    s    = 打印状态
  r    = 重置队列        q    = 退出
"""

import json
import os
import select
import sys
import termios
import threading
import time
import tty

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from race_abot_411_common import (AkmIO, build_alias_index, deep_merge,  # noqa: E402
                              load_yaml, normalize_class, clamp,
                              resolve_camera_topic)
from race_abot_411_lane import LaneFollower          # noqa: E402
from race_abot_411_actions import RaceActions        # noqa: E402


class KeyboardInput(object):
    """非阻塞读单个按键"""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *a):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def get_char(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


class RaceAbotNode(object):

    def __init__(self):
        rospy.init_node('race_abot_411_node', anonymous=False)

        # ---------- 参数 ----------
        params_yaml = rospy.get_param('~params_yaml', '')
        route_yaml = rospy.get_param('~route_yaml', '')
        self.cfg_params = load_yaml(params_yaml)
        self.cfg_route = load_yaml(route_yaml)

        # 私有命名空间里的 rosparam 优先覆盖 yaml（launch 里 load 的、或单独 <param> 的）
        over = {}
        for key in ('topics', 'detect', 'lane', 'actions', 'safety', 'debug'):
            v = rospy.get_param('~' + key, None)
            if isinstance(v, dict):
                over[key] = v
        self.P = deep_merge(self.cfg_params, over)
        P = self.P

        tp = P.get('topics') or {}
        dbg = P.get('debug') or {}
        saf = P.get('safety') or {}
        lp = P.get('lane') or {}

        # 巡线相机：配置优先，配置的话题没人发布就自动换成实际在用的那路（换车必备）
        want_front = (rospy.get_param('~camera_front', '')
                      or tp.get('camera_front') or '')
        self.topic_cam_front, _cands = resolve_camera_topic(
            want_front, tp.get('camera_candidates'), name="前视相机")
        self.topic_cmd = tp.get('cmd', '/akm_vel')
        self.topic_odom = tp.get('odom', '/odom')
        self.topic_sonar = tp.get('sonar', '/sonar')
        self.topic_det = rospy.get_param('~detection_topic', '/race_abot_411/detections')

        self.debug = bool(dbg.get('enabled', True))
        self.show_lane = bool(dbg.get('show_lane_window', True))
        self.keyboard_enabled = bool(dbg.get('keyboard_enabled', True))

        self.no_steer_timeout = float(saf.get('no_steer_timeout_s', 1.0))
        self.max_task_time = float(saf.get('max_task_time_s', 25.0))
        self.watchdog_period = float(saf.get('watchdog_period_s', 10.0))
        self.odom_stale_warn = float(saf.get('odom_stale_warn_s', 2.0))
        self.idle_stop_hz = float(saf.get('idle_stop_hz', 5.0))

        self.lane_speed = float(lp.get('speed', 0.20))
        self.lane_reverse_speed = float(lp.get('reverse_speed', 0.12))
        self.slow_turn_threshold = float(lp.get('slow_turn_threshold', 0.25))
        self.slow_turn_factor = float(lp.get('slow_turn_speed_factor', 0.65))

        self.stop_wait_s = float((P.get('actions') or {}).get('stop_wait_s', 3.0))

        # ---------- 队列 ----------
        self.queue = list(self.cfg_route.get('tasks') or [])
        self.alias = build_alias_index(self.cfg_route.get('class_alias', {}))
        self.idx = 0
        self.pending_sign = None
        self.pending_time = 0.0
        self.pending_ttl = float((P.get('detect') or {}).get('sign_cooldown', 3.0)) + 5.0

        # ---------- 状态 ----------
        self.lane_enabled = False
        self.action_running = False
        self.current_task = None
        self.last_image_time = time.time()
        self.last_steer_time = time.time()
        self.sonar_d = None
        self.sonar_time = 0.0
        self.running = True

        self._abort = False              # 看门狗强制中止标记，动作库会读它提前退出
        self.task_start_time = 0.0
        self.frame_count = 0
        self.last_idle_stop = 0.0
        self.holding_for_red = False
        self.startup_reported = False
        self.odom_data_seen = False
        self.sonar_data_seen = False
        self.det_data_seen = False
        self._lock = threading.Lock()

        # ---------- ROS 组件 ----------
        self.bridge = CvBridge()
        self.io = AkmIO(cmd_topic=self.topic_cmd, odom_topic=self.topic_odom,
                        diff_ratio=float(lp.get('diff_ratio', 0.27)))
        # 周期检查 odom 是否活着（用于自检报告与告警）
        self._odom_watch = rospy.Timer(rospy.Duration(0.5), self._odom_tick)

        self.lane = LaneFollower(P)
        self.actions = RaceActions(
            self.io, P,
            get_sonar=self.get_sonar,
            lane_step=self.lane_step,
            set_lane=self.set_lane,
            should_stop=self.should_stop,
        )

        self.sub_img = rospy.Subscriber(self.topic_cam_front, Image,
                                        self.image_cb, queue_size=1)
        self.sub_det = rospy.Subscriber(self.topic_det, String,
                                        self.detection_cb, queue_size=10)
        self.sub_sonar = rospy.Subscriber(self.topic_sonar, Float32,
                                          self.sonar_cb, queue_size=1)
        rospy.on_shutdown(self.on_shutdown)

        if self.keyboard_enabled:
            t = threading.Thread(target=self.keyboard_loop)
            t.daemon = True
            t.start()

        rospy.loginfo("=" * 60)
        rospy.loginfo("race_abot_411 主节点启动 | 路线=%s | 任务数=%d",
                      self.cfg_route.get('route_name', '?'), len(self.queue))
        for i, t in enumerate(self.queue):
            rospy.loginfo("  [%02d] %-16s trigger=%-5s sign=%-18s action=%s",
                          i + 1, t.get('name', ''), t.get('trigger', ''),
                          str(t.get('sign')), t.get('action', ''))
        rospy.loginfo("巡线相机=%s | 标志检测=%s | 控制=%s | 里程计=%s",
                      self.topic_cam_front, self.topic_det,
                      self.topic_cmd, self.topic_odom)
        rospy.loginfo("=" * 60)

    # ==================================================================
    # 接口
    # ==================================================================
    def _odom_tick(self, _evt):
        if self.io.odom_healthy(self.odom_stale_warn):
            self.odom_data_seen = True

    def get_sonar(self):
        if time.time() - self.sonar_time > 1.0:
            return None
        return self.sonar_d

    def set_lane(self, v):
        self.lane_enabled = bool(v)
        if self.lane_enabled:
            self.last_image_time = time.time()
            self.last_steer_time = time.time()

    def should_stop(self):
        """动作库会不断调用；返回 True 表示当前动作应尽快退出"""
        return self._abort or (self.pending_sign is not None)

    def lane_step(self, speed=None, reverse=False, gain=None):
        """用最新一帧的巡线转向发一帧速度"""
        steering = self.lane.steering
        if gain is not None:
            steering = clamp(steering * gain, -1.0, 1.0)
        sp = speed if speed is not None else (
            self.lane_reverse_speed if reverse else self.lane_speed)
        if not reverse:
            if abs(steering) > self.slow_turn_threshold:
                sp *= self.slow_turn_factor
            self.io.drive(sp, steering)
        else:
            self.io.drive(-sp, -steering)
        self.last_steer_time = time.time()
        return True

    # ==================================================================
    # 回调
    # ==================================================================
    def sonar_cb(self, msg):
        self.sonar_d = float(msg.data)
        self.sonar_time = time.time()
        self.sonar_data_seen = True

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr_throttle(5.0, "cv_bridge: %s", e)
            return
        self.last_image_time = time.time()
        _, dbg, info = self.lane.update(frame)

        self.frame_count += 1
        if self.debug and self.show_lane and (self.frame_count % 6 == 0):
            try:
                cv2.imshow('race_abot_411 lane', dbg)
                cv2.waitKey(1)
            except Exception:
                pass

    def detection_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        dets = data.get('detections') or []
        if not dets:
            return
        best = max(dets, key=lambda d: d.get('confidence', 0))
        name = normalize_class(best.get('class_name', ''), self.alias)
        self.det_data_seen = True

        now = time.time()
        with self._lock:
            if self.action_running:
                self.pending_sign = name
                self.pending_time = now
                rospy.loginfo("[检测] 动作执行中，记录 pending=%s", name)
                return
            self.pending_sign = name
            self.pending_time = now
        rospy.loginfo("[检测] %s (raw=%s, conf=%.2f)",
                      name, best.get('raw_name'), best.get('confidence', 0))

    # ==================================================================
    # 队列推进
    # ==================================================================
    def _take_pending(self):
        with self._lock:
            s, self.pending_sign = self.pending_sign, None
            return s

    def _pending_valid(self):
        if self.pending_sign is None:
            return False
        if time.time() - self.pending_time > self.pending_ttl:
            self.pending_sign = None
            return False
        return True

    def process_queue(self):
        """在主循环里以 20Hz 调用"""
        if self.action_running:
            return
        if self.idx >= len(self.queue):
            return

        task = self.queue[self.idx]
        trig = task.get('trigger', 'sign')

        if trig == 'auto':
            self.start_task(task)
            return

        if not self._pending_valid():
            return

        want = task.get('sign')
        got = self.pending_sign

        if trig == 'light':
            if got == 'red_light':
                self._take_pending()
                self._start_red_hold()
                return
            if got == want:
                self._take_pending()
                self.start_task(task)
            else:
                rospy.loginfo_throttle(3.0, "[队列] 等待 %s，当前 %s", want, got)
            return

        # trig == 'sign'
        if got == want:
            self._take_pending()
            self.start_task(task)
        else:
            rospy.loginfo_throttle(3.0, "[队列] 等待 %s，当前 %s", want, got)

    def _start_red_hold(self):
        """红灯等待：非阻塞（放线程里），不卡住巡线/安全检测"""
        if self.holding_for_red:
            return
        rospy.logwarn("[队列] 红灯，停车等待（非阻塞）")
        self.holding_for_red = True
        th = threading.Thread(target=self._red_hold_worker)
        th.daemon = True
        th.start()

    def _red_hold_worker(self):
        try:
            self.set_lane(False)
            t0 = time.time()
            while (not rospy.is_shutdown() and self.holding_for_red
                   and time.time() - t0 < self.stop_wait_s):
                self.io.stop()
                rospy.sleep(0.1)
        finally:
            self.io.stop(3)
            self.holding_for_red = False
            if not self.action_running:
                self.set_lane(True)

    def start_task(self, task):
        self.holding_for_red = False      # 让红灯等待线程立刻退出
        self._abort = False
        self.task_start_time = time.time()
        self.action_running = True
        self.current_task = task
        rospy.loginfo("─" * 50)
        rospy.loginfo("[队列] 执行 #%s %s -> 动作 %s",
                      task.get('id'), task.get('name'), task.get('action'))
        th = threading.Thread(target=self._task_worker, args=(task,))
        th.daemon = True
        th.start()

    def _task_worker(self, task):
        t0 = time.time()
        try:
            self.actions.run(task.get('action'))
        except Exception as e:
            rospy.logerr("[队列] 任务异常: %s", e)
            self.io.stop(3)
        dt = time.time() - t0
        if self._abort:
            rospy.logwarn("[队列] 任务 #%s 被看门狗强制中止（%.1fs）", task.get('id'), dt)
        self.action_running = False
        self.current_task = None
        self.task_start_time = 0.0
        self._abort = False
        with self._lock:
            self.idx += 1
        rospy.loginfo("[队列] 完成 #%s（%.1fs），进度 %d/%d",
                      task.get('id'), dt, self.idx, len(self.queue))
        if self.idx >= len(self.queue):
            rospy.loginfo("[队列] 全部任务完成")

    # ==================================================================
    # 键盘调试
    # ==================================================================
    KEYMAP = {
        '1': 'turn_left', '2': 'reverse_parking', '3': 'parallel_parking',
        '4': 'turn_right', '5': 'cross_bridge', '6': 'stop_wait',
        '7': 's_curve_out', '8': 'flip', '9': 'obstacle_avoid',
    }

    def keyboard_loop(self):
        try:
            with KeyboardInput() as kb:
                while self.running and not rospy.is_shutdown():
                    ch = kb.get_char()
                    if ch:
                        self.on_key(ch)
                    time.sleep(0.05)
        except Exception as e:
            rospy.logwarn("键盘线程不可用（无 tty？）: %s", e)

    def on_key(self, ch):
        if ch == ' ':
            self.set_lane(not self.lane_enabled)
            rospy.loginfo("巡线 -> %s", "开" if self.lane_enabled else "关")
        elif ch in self.KEYMAP:
            act = self.KEYMAP[ch]
            rospy.loginfo("手动触发动作: %s", act)
            th = threading.Thread(target=lambda: self.actions.run(act))
            th.daemon = True
            th.start()
        elif ch == 'n':
            if self.idx < len(self.queue):
                rospy.logwarn("跳过任务 #%s", self.queue[self.idx].get('id'))
                with self._lock:
                    self.idx += 1
        elif ch == 'r':
            with self._lock:
                self.idx = 0
                self.pending_sign = None
            rospy.logwarn("队列已重置")
        elif ch == 's':
            self.print_status()
        elif ch == 'q':
            self.running = False
            rospy.signal_shutdown("键盘退出")

    def print_status(self):
        cur = self.queue[self.idx].get('name') if self.idx < len(self.queue) else '全部完成'
        rospy.loginfo("─" * 40)
        rospy.loginfo("进度: %d/%d  当前: %s", self.idx, len(self.queue), cur)
        rospy.loginfo("巡线: %s  动作中: %s  转向: %+.2f (%s)",
                      self.lane_enabled, self.action_running,
                      self.lane.steering, "有效" if self.lane.valid else "丢线")
        rospy.loginfo("偏航: %.1f°  超声: %s  里程计: %s",
                      self.io.get_yaw() * 57.3,
                      "%.2fm" % self.sonar_d if self.sonar_d is not None else "无",
                      "OK" if self.io.odom_healthy(self.odom_stale_warn) else "无数据")
        rospy.loginfo("─" * 40)

    # ==================================================================
    # 启动自检报告
    # ==================================================================
    def startup_report(self):
        self.startup_reported = True
        img_ok = (time.time() - self.last_image_time) < 2.0
        rospy.loginfo("=" * 60)
        rospy.loginfo("[自检] 相机 %s | 里程计 %s | 超声 %s | 标志检测 %s",
                      "OK" if img_ok else "!! 无数据",
                      "OK" if self.odom_data_seen else "!! 无数据",
                      "OK" if self.sonar_data_seen else "-- 无（避障退化为低速直行）",
                      "OK" if self.det_data_seen else "-- 无（未起步时属正常）")
        if not img_ok:
            rospy.logerr("[自检] 收不到图像 %s，巡线无法工作！检查相机与 CAMERA 环境变量",
                         self.topic_cam_front)
        if not self.odom_data_seen:
            rospy.logwarn("[自检] 收不到里程计 %s，转弯/泊车闭环会失效；检查 abot_driver 是否在跑",
                          self.topic_odom)
        rospy.loginfo("=" * 60)

    # ==================================================================
    # 主循环
    # ==================================================================
    def run(self):
        rate = rospy.Rate(20)
        last_wd = time.time()
        start_time = time.time()

        while not rospy.is_shutdown() and self.running:
            self.process_queue()

            # 看门狗：任务超时强制中止（动作库会读 _abort 提前退出）
            if (self.action_running and self.task_start_time
                    and time.time() - self.task_start_time > self.max_task_time):
                rospy.logerr("[安全] 任务超时 %.1fs，强制中止并恢复巡线", self.max_task_time)
                self._abort = True
                self.io.stop(5)
                self.set_lane(True)

            # 自动巡线（动作执行时不做）
            if self.lane_enabled and not self.action_running:
                if time.time() - self.last_image_time > self.no_steer_timeout:
                    rospy.logwarn_throttle(2.0, "[安全] 相机超时，停车")
                    self.io.stop()
                else:
                    self.lane_step()

            # 空闲补发停止：既不巡线也不执行动作时，防止底盘保持上一帧速度
            if (not self.lane_enabled and not self.action_running
                    and not self.holding_for_red):
                if time.time() - self.last_idle_stop > 1.0 / max(0.1, self.idle_stop_hz):
                    self.io.stop()
                    self.last_idle_stop = time.time()

            # 启动 3 秒后做一次自检
            if not self.startup_reported and time.time() - start_time > 3.0:
                self.startup_report()

            # 心跳
            if time.time() - last_wd > self.watchdog_period:
                last_wd = time.time()
                cur = self.queue[self.idx].get('name') if self.idx < len(self.queue) else '完成'
                rospy.loginfo("[心跳] 进度 %d/%d | 当前 %s | 巡线 %s | 动作 %s | odom %s",
                              self.idx, len(self.queue), cur,
                              self.lane_enabled, self.action_running,
                              "OK" if self.io.odom_healthy(self.odom_stale_warn) else "无")
            rate.sleep()

    def on_shutdown(self):
        self.running = False
        self._abort = True
        self.holding_for_red = False
        try:
            self.io.stop(5)
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rospy.loginfo("race_abot_411 已退出")


def main():
    try:
        node = RaceAbotNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        import traceback
        rospy.logerr("启动失败: %s", e)
        traceback.print_exc()


if __name__ == '__main__':
    main()
