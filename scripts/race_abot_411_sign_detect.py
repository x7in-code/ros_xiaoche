#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
race_abot_411_sign_detect.py —— 标志检测节点

职责：
  1. 加载 YOLOv8 权重（ultralytics）
  2. 订阅标志相机，只保留"图像中心区域"内的检测（滤掉路侧干扰）
  3. 类别名归一化（不同模型命名不同 -> 统一成规范名）
  4. 发布 JSON 到 /race_abot_411/detections，另发可视化图

输出消息结构（std_msgs/String + JSON）：
{
  "stamp": 0.0, "frame_id": "camera",
  "detections": [{"class_name": "turn_right", "raw_name": "turn_right",
                  "class_id": 0, "confidence": 0.93,
                  "center_x": 320.0, "center_y": 240.0}],
  "count": 1
}
"""

import json
import os
import sys
import time

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from race_abot_411_common import build_alias_index, normalize_class, load_yaml  # noqa: E402


class SignDetector(object):

    def __init__(self):
        rospy.init_node('race_abot_411_sign_detect', anonymous=False)

        self.model_path = rospy.get_param('~model_path', '/home/abot/yolov8/best.pt')
        self.conf = float(rospy.get_param('~confidence', 0.70))
        self.iou = float(rospy.get_param('~iou', 0.45))
        self.device = rospy.get_param('~device', 'auto')
        self.img_topic = rospy.get_param('~image_topic', '/camera/right/image_raw')
        self.out_topic = rospy.get_param('~output_topic', '/race_abot_411/detections')
        self.vis_topic = rospy.get_param('~vis_topic', '/race_abot_411/sign_image')

        self.cw = float(rospy.get_param('~center_width_ratio', 0.45))
        self.ch = float(rospy.get_param('~center_height_ratio', 0.55))
        self.cshift = int(rospy.get_param('~center_y_shift', 60))

        # 类别归一化表：从路线 yaml 读，没有就用空表
        route_yaml = rospy.get_param('~route_yaml', '')
        cfg = load_yaml(route_yaml) if route_yaml else {}
        self.alias = build_alias_index(cfg.get('class_alias', {}))
        rospy.loginfo("类别别名表条数: %d", len(self.alias))

        self.model = None
        self._load_model()

        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.out_topic, String, queue_size=10)
        self.pub_vis = rospy.Publisher(self.vis_topic, Image, queue_size=1)
        self.sub = rospy.Subscriber(self.img_topic, Image, self.cb, queue_size=1)

        rospy.loginfo("标志检测节点已启动 | 模型=%s | 输入=%s | 输出=%s",
                      self.model_path, self.img_topic, self.out_topic)

    def _load_model(self):
        if not os.path.isfile(self.model_path):
            rospy.logerr("模型文件不存在: %s（请改 ~model_path 参数）", self.model_path)
            return
        try:
            from ultralytics import YOLO
            self.model = YOLO(self.model_path)
            try:
                names = list(self.model.names.values())
                rospy.loginfo("模型类别(%d): %s", len(names), names)
            except Exception:
                pass
        except Exception as e:
            rospy.logerr("加载 YOLO 失败: %s", e)

    def _center_bounds(self, w, h):
        bw = w * self.cw
        bh = h * self.ch
        cx, cy = w / 2.0, h / 2.0 + self.cshift
        return (int(cx - bw / 2), int(cy - bh / 2),
                int(cx + bw / 2), int(cy + bh / 2))

    def cb(self, msg):
        if self.model is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr("cv_bridge: %s", e)
            return

        h, w = frame.shape[:2]
        x1b, y1b, x2b, y2b = self._center_bounds(w, h)

        try:
            res = self.model.predict(frame, conf=self.conf, iou=self.iou,
                                     verbose=False)[0]
        except Exception as e:
            rospy.logerr("推理失败: %s", e)
            return

        dets = []
        vis = frame.copy()
        cv2.rectangle(vis, (x1b, y1b), (x2b, y2b), (255, 255, 0), 1)

        if res.boxes is not None and len(res.boxes) > 0:
            xyxy = res.boxes.xyxy.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()
            clss = res.boxes.cls.cpu().numpy()
            for i in range(len(xyxy)):
                bx1, by1, bx2, by2 = xyxy[i]
                cx = (bx1 + bx2) / 2.0
                cy = (by1 + by2) / 2.0
                if not (x1b <= cx <= x2b and y1b <= cy <= y2b):
                    continue
                raw = str(self.model.names[int(clss[i])])
                name = normalize_class(raw, self.alias)
                dets.append({
                    "class_name": name,
                    "raw_name": raw,
                    "class_id": int(clss[i]),
                    "confidence": round(float(confs[i]), 3),
                    "center_x": round(float(cx), 1),
                    "center_y": round(float(cy), 1),
                })
                color = (0, 255, 0)
                cv2.rectangle(vis, (int(bx1), int(by1)), (int(bx2), int(by2)),
                              color, 2)
                cv2.putText(vis, "%s %.2f" % (name, confs[i]),
                            (int(bx1), max(12, int(by1) - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        payload = {
            "stamp": round(time.time(), 3),
            "frame_id": msg.header.frame_id,
            "detections": dets,
            "count": len(dets),
        }
        self.pub.publish(String(data=json.dumps(payload)))
        if dets:
            rospy.loginfo("检测到: %s", [d['class_name'] for d in dets])

        try:
            self.pub_vis.publish(self.bridge.cv2_to_imgmsg(vis, 'bgr8'))
        except Exception:
            pass


def main():
    try:
        SignDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
