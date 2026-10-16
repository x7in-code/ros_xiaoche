#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
race_abot_411_fake_sign.py —— 伪造标志检测（调试用）

在没有模型 / 没有场地时，直接向 /race_abot_411/detections 发假消息，
用来单独联调 race_abot_411_node.py 的任务队列。

用法：
  rosrun race_abot_411 race_abot_411_fake_sign.py interactive
  rosrun race_abot_411 race_abot_411_fake_sign.py once turn_right
  rosrun race_abot_411 race_abot_411_fake_sign.py loop green_light 1.0
  rosrun race_abot_411 race_abot_411_fake_sign.py route ./race_abot_411_route_a.yaml   # 按路线顺序逐条发（每条间隔3s）
"""

import json
import os
import sys
import time

import rospy
import yaml
from std_msgs.msg import String

SIGNS = [
    'green_light', 'red_light', 'turn_left', 'turn_right', 'stop',
    'obstacle', 'reverse', 'flip', 'finish',
    'parallel_parking', 'single_bridge', 'reverse_parking',
]


def build(msg_class, conf=0.95):
    return json.dumps({
        "stamp": round(time.time(), 3),
        "frame_id": "camera",
        "detections": [{
            "class_name": msg_class,
            "raw_name": msg_class,
            "class_id": 0,
            "confidence": conf,
            "center_x": 320.0,
            "center_y": 240.0,
        }],
        "count": 1,
    })


def main():
    rospy.init_node('race_abot_411_fake_sign', anonymous=True)
    topic = rospy.get_param('~topic', '/race_abot_411/detections')
    pub = rospy.Publisher(topic, String, queue_size=10)

    args = sys.argv[1:]
    mode = args[0] if args else 'interactive'
    rest = args[1:]
    rospy.sleep(1.0)
    rospy.loginfo("伪造标志 -> %s", topic)

    if mode == 'once':
        if rest:
            pub.publish(String(data=build(rest[0])))
            rospy.loginfo("已发: %s", rest[0])
        rospy.sleep(0.5)

    elif mode == 'loop':
        if not rest:
            print("用法: loop <sign> [interval]"); return
        interval = float(rest[1]) if len(rest) > 1 else 1.0
        r = rospy.Rate(1.0 / interval)
        while not rospy.is_shutdown():
            pub.publish(String(data=build(rest[0])))
            rospy.loginfo("已发: %s", rest[0])
            r.sleep()

    elif mode == 'route':
        path = rest[0] if rest else ''
        if not os.path.isfile(path):
            print("找不到路线文件: %s" % path); return
        cfg = yaml.safe_load(open(path, encoding='utf-8')) or {}
        for t in cfg.get('tasks', []):
            if rospy.is_shutdown():
                break
            sign = t.get('sign')
            if not sign:
                rospy.loginfo("跳过（无标志，auto 任务）: %s", t.get('name'))
                continue
            rospy.loginfo("-> %s (%s)", sign, t.get('name'))
            pub.publish(String(data=build(sign)))
            time.sleep(3.0)

    else:  # interactive
        print("=" * 56)
        print(" 交互式伪造标志：输入类别名回车发送，q 退出")
        print(" 可用:", ', '.join(SIGNS))
        print(" 输入序号也可，如 1 = green_light")
        print("=" * 56)
        try:
            while not rospy.is_shutdown():
                raw = input('sign> ').strip()
                if not raw:
                    continue
                if raw in ('q', 'quit', 'exit'):
                    break
                if raw.isdigit() and 1 <= int(raw) <= len(SIGNS):
                    raw = SIGNS[int(raw) - 1]
                pub.publish(String(data=build(raw)))
                rospy.loginfo("已发: %s", raw)
        except (EOFError, KeyboardInterrupt):
            pass


if __name__ == '__main__':
    main()
