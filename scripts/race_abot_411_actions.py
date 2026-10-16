#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
race_abot_411_actions.py —— 动作库

把"每个标志对应要做的动作"集中在这里，主节点只负责按队列调度。

所有动作都遵守三条纪律：
  1. 闭环：能靠里程计/传感器判断的就不靠 sleep
  2. 超时：每个动作都有 timeout 兜底，绝不卡死
  3. 幂等收尾：动作结束一定发停止指令

接口约定：
  io              : race_abot_411_common.AkmIO 实例
  params          : 参数字典（对应 race_abot_411_params.yaml）
  get_sonar()     : 返回最近一次超声距离(米)，无数据返回 None
  lane_step()     : 让主节点用"最新一帧的巡线转向"发一帧速度，返回是否有效
  set_lane(bool)  : 开/关主循环的自动巡线
  should_stop()   : 返回 True 表示外部要求提前结束当前动作（例如检测到下一个标志）
"""

import math
import time

import rospy


class RaceActions(object):

    def __init__(self, io, params,
                 get_sonar=None, lane_step=None, set_lane=None,
                 should_stop=None):
        self.io = io
        self.p = params or {}
        self.get_sonar = get_sonar or (lambda: None)
        self.lane_step = lane_step or (lambda speed=None, reverse=False: False)
        self.set_lane = set_lane or (lambda v: None)
        self.should_stop = should_stop or (lambda: False)

    # ------------------------------------------------------------------
    # 内部工具
    # ------------------------------------------------------------------
    def _a(self, key, default=None):
        return (self.p.get('actions') or {}).get(key, default)

    def _lane_p(self, key, default=None):
        return (self.p.get('lane') or {}).get(key, default)

    def _run_slots(self, slots, timeout, name):
        """按 [linear, angular, duration] 序列执行，带总超时，且可被中止"""
        t0 = time.time()
        for i, s in enumerate(slots or []):
            if rospy.is_shutdown():
                break
            if self.should_stop():
                rospy.logwarn("%s 被中止（收到新标志或看门狗超时）", name)
                break
            if time.time() - t0 > timeout:
                rospy.logwarn("%s 总超时，提前结束", name)
                break
            lin, ang, dur = float(s[0]), float(s[1]), float(s[2])
            rospy.loginfo("%s 第%d段: v=%.2f w=%.2f t=%.2fs", name, i + 1, lin, ang, dur)
            self.io.drive_for(lin, ang, dur)
        self.io.stop(3)
        rospy.loginfo("%s 完成，用时 %.1fs", name, time.time() - t0)

    def _lane_drive(self, duration, speed=None, reverse=False, gain=None):
        """
        用主节点的巡线转向连续跑一段时间（用于过桥 / S弯正进）
        """
        r = rospy.Rate(20)
        t0 = time.time()
        while not rospy.is_shutdown():
            if time.time() - t0 >= duration:
                break
            if self.should_stop():
                rospy.loginfo("lane_drive 被外部终止")
                break
            self.lane_step(speed=speed, reverse=reverse, gain=gain)
            r.sleep()
        self.io.stop()

    # ==================================================================
    # 启动 / 结束
    # ==================================================================
    def start(self):
        """绿灯：起步并开启巡线"""
        rospy.loginfo("[动作] 绿灯起步 -> 开启巡线")
        self.set_lane(True)

    def finish(self):
        """终点：停车并关闭巡线，比赛结束"""
        rospy.loginfo("[动作] 到达终点，停车")
        self.set_lane(False)
        self.io.stop(5)

    # ==================================================================
    # 转向
    # ==================================================================
    def turn_left(self):
        self._do_turn(+1, "左转")

    def turn_right(self):
        self._do_turn(-1, "右转")

    def _do_turn(self, sign, name):
        """靠里程计偏航角闭环转 90°（左转 sign=+1，右转 sign=-1）"""
        deg = self._a('turn_target_deg', 90.0)
        rad = math.radians(deg) * sign
        max_w = self._a('turn_max_angular', 1.2)
        tol = self._a('turn_tolerance_rad', 0.10)
        timeout = self._a('turn_timeout_s', 8.0)
        lin = self._a('turn_linear', 0.25)

        rospy.loginfo("[动作] %s %.0f°（闭环）", name, deg)
        self.set_lane(False)
        self.io.drive(0.0, 0.0)
        rospy.sleep(0.2)
        ok = self.io.rotate_by(rad, max_angular=max_w, tolerance=tol,
                               timeout=timeout, linear=lin)
        rospy.loginfo("[动作] %s 结束 ok=%s", name, ok)
        self.set_lane(True)

    def flip(self):
        """翻转标志：180° 掉头"""
        deg = self._a('flip_target_deg', 180.0)
        max_w = self._a('flip_max_angular', 1.2)
        timeout = self._a('flip_timeout_s', 15.0)
        rospy.loginfo("[动作] 翻转 180°")
        self.set_lane(False)
        self.io.drive(0.0, 0.0)
        rospy.sleep(0.2)
        ok = self.io.rotate_by(math.radians(deg), max_angular=max_w,
                               tolerance=0.12, timeout=timeout)
        rospy.loginfo("[动作] 翻转结束 ok=%s", ok)
        self.set_lane(True)

    # ==================================================================
    # 停留 / 定点停车
    # ==================================================================
    def stop_wait(self):
        """停留标志（红底停）：停稳等待，然后可继续"""
        wait = self._a('stop_wait_s', 3.0)
        rospy.loginfo("[动作] 停留 %.1fs", wait)
        self.io.stop(3)
        t0 = time.time()
        while not rospy.is_shutdown() and time.time() - t0 < wait:
            if self.should_stop():
                rospy.loginfo("[动作] 停留被中止")
                break
            self.io.stop()
            rospy.sleep(0.1)
        rospy.loginfo("[动作] 停留结束")
        self.set_lane(True)

    # ==================================================================
    # 避障（15 分）
    # ==================================================================
    def obstacle_avoid(self):
        """
        避障绕行：
          1. 确认前方有障碍（超声距离 < 阈值）
          2. 先后退一点 → 侧向打角绕开 → 直行通过 → 回归巡线
          3. 全过程总超时保护；障碍物消失就提前结束
        """
        c = self._a('avoid', {}) or {}
        thr = c.get('sonar_threshold_m', 0.35)
        total_to = c.get('total_timeout_s', 15.0)

        rospy.loginfo("[动作] 避障开始（阈值 %.2fm）", thr)
        self.set_lane(False)
        t0 = time.time()

        # 1) 判断是否真的有障碍
        d = self.get_sonar()
        rospy.loginfo("[动作] 超声距离 = %s", ("%.2fm" % d) if d is not None else "无数据")

        has_obstacle = (d is not None and d < thr)

        if not has_obstacle:
            # 没读到障碍，保守起见也走一次小绕行
            rospy.loginfo("[动作] 未探测到障碍，低速直行通过")
            self.io.drive_for(c.get('forward_speed', 0.18), 0.0,
                              min(c.get('forward_s', 1.5), total_to))
            self.set_lane(True)
            return

        # 2) 后退
        self.io.drive_for(-abs(c.get('back_off_speed', 0.10)), 0.0,
                          c.get('back_off_s', 0.8))
        if time.time() - t0 > total_to:
            self._avoid_finish(); return

        # 3) 侧向打角绕开（默认先往左绕；如需往右把 side_turn_angular 取负）
        self.io.drive_for(c.get('side_turn_speed', 0.15),
                          c.get('side_turn_angular', 0.8),
                          c.get('side_turn_s', 1.6))
        if time.time() - t0 > total_to:
            self._avoid_finish(); return

        # 4) 直行通过障碍区（边走边看，障碍消失即停）
        def gone():
            dd = self.get_sonar()
            return (dd is not None and dd >= thr * 1.5)

        self.io.drive_until(c.get('forward_speed', 0.18), 0.0,
                            gone, timeout=min(c.get('forward_s', 1.5),
                                              max(0.1, total_to - (time.time() - t0))))
        if time.time() - t0 > total_to:
            self._avoid_finish(); return

        # 5) 回归巡线（反向打角回正）
        self.io.drive_for(c.get('forward_speed', 0.15),
                          -c.get('side_turn_angular', 0.8),
                          c.get('rejoin_s', 1.2))

        self._avoid_finish()
        rospy.loginfo("[动作] 避障完成，用时 %.1fs", time.time() - t0)

    def _avoid_finish(self):
        self.io.stop(3)
        self.set_lane(True)

    # ==================================================================
    # 单边桥（10 分）
    # ==================================================================
    def cross_bridge(self):
        """
        过单边桥：保持巡线居中修正，但降速匀速通过。
        """
        c = self._a('bridge', {}) or {}
        speed = c.get('speed', 0.10)
        dur = c.get('duration_s', 4.0)
        gain = c.get('steer_gain', 1.2)

        rospy.loginfo("[动作] 过单边桥：速度 %.2f，时长 %.1fs", speed, dur)
        # 过桥时保持巡线开启（用更强增益居中），但速度由参数压住
        self._lane_drive(dur, speed=speed, gain=gain)
        rospy.loginfo("[动作] 单边桥通过")
        self.set_lane(True)

    # ==================================================================
    # S 弯
    # ==================================================================
    def s_curve_in(self):
        """S 弯正进：巡线到末端（末端可由检测到'倒车'标志触发，或超时兜底）"""
        c = self._a('s_curve_in', {}) or {}
        speed = c.get('speed', 0.15)
        to = c.get('timeout_s', 15.0)
        rospy.loginfo("[动作] S弯正进（最长 %.1fs）", to)
        self._lane_drive(to, speed=speed)
        rospy.loginfo("[动作] S弯正进结束")

    def s_curve_out(self):
        """反向倒出 S 弯：倒车巡线"""
        c = self._a('s_curve_in', {}) or {}
        to = c.get('timeout_s', 15.0)
        rospy.loginfo("[动作] 倒车出S弯（最长 %.1fs）", to)
        self._lane_drive(to, reverse=True)
        rospy.loginfo("[动作] 倒车出S弯结束")

    # ==================================================================
    # 泊车（60 分里的 40 分）
    # ==================================================================
    def parallel_parking(self):
        """侧方停车：按参数化动作串执行（现场对每段时长微调）"""
        c = self._a('parallel_parking', {}) or {}
        rospy.loginfo("[动作] 侧方停车开始")
        self.set_lane(False)
        self._run_slots(c.get('slots'), c.get('total_timeout_s', 20.0), "侧方停车")
        self.set_lane(False)
        rospy.loginfo("[动作] 侧方停车结束（保持停车，等待下一标志）")

    def reverse_parking(self):
        """倒车入库：按参数化动作串执行"""
        c = self._a('reverse_parking', {}) or {}
        rospy.loginfo("[动作] 倒车入库开始")
        self.set_lane(False)
        self._run_slots(c.get('slots'), c.get('total_timeout_s', 20.0), "倒车入库")
        self.set_lane(False)
        rospy.loginfo("[动作] 倒车入库结束")

    def garage_transfer(self):
        """车库 A→B 搬运：无专用标志，用固定动作串（后续可换成定位导航）"""
        c = self._a('garage_transfer', {}) or {}
        rospy.loginfo("[动作] 车库搬运 A→B 开始")
        self.set_lane(False)
        self._run_slots(c.get('slots'), c.get('total_timeout_s', 18.0), "车库搬运")
        self.set_lane(True)
        rospy.loginfo("[动作] 车库搬运结束")

    # ==================================================================
    # 直通（无专用标志的几何任务，例如直角弯）
    # ==================================================================
    def lane_only(self):
        rospy.loginfo("[动作] 该任务点无需专用动作，继续巡线通过")
        self.set_lane(True)

    # ==================================================================
    # 调度
    # ==================================================================
    DISPATCH = {
        'start': 'start',
        'finish': 'finish',
        'turn_left': 'turn_left',
        'turn_right': 'turn_right',
        'flip': 'flip',
        'stop_wait': 'stop_wait',
        'obstacle_avoid': 'obstacle_avoid',
        'cross_bridge': 'cross_bridge',
        's_curve_in': 's_curve_in',
        's_curve_out': 's_curve_out',
        'parallel_parking': 'parallel_parking',
        'reverse_parking': 'reverse_parking',
        'garage_transfer': 'garage_transfer',
        'lane_only': 'lane_only',
    }

    def run(self, action_name):
        """按名字执行动作；返回 True 表示执行过"""
        method_name = self.DISPATCH.get(action_name)
        if not method_name:
            rospy.logwarn("[动作] 未知动作 '%s'，跳过", action_name)
            return False
        method = getattr(self, method_name, None)
        if method is None:
            rospy.logerr("[动作] 动作 '%s' 未实现", action_name)
            return False
        try:
            method()
        except Exception as e:
            rospy.logerr("[动作] 执行 '%s' 出错: %s", action_name, e)
            self.io.stop(3)
            self.set_lane(True)
        return True
