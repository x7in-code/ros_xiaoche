#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
race_abot_411_lane.py —— 黄线巡线（感知 + 转向计算）

思路（比 Hough 更稳、参数更少）：
  1. 取画面下方 ROI，BGR->HSV，双段黄色阈值得到掩膜
  2. 形态学去噪
  3. 把 ROI 横切成 N 条带，每条带找最左/最右黄色像素
       - 同时找到左右边界 -> 车道中心 = (左+右)/2
       - 只找到一条      -> 用 single_line_offset_px 推算中心
  4. 各条带的中心做加权平均 -> 车道中心 x
  5. steering = (图像中心 - 车道中心) / (宽*0.5)   （正=左转）
  6. 指数平滑 + 单步限幅 + 限幅
"""

import cv2
import numpy as np

from race_abot_411_common import EMA, clamp, rate_limit


class LaneFollower(object):

    def __init__(self, params):
        self.p = params or {}
        lp = self.p.get('lane') or {}

        self.roi_top = float(lp.get('roi_top', 0.45))
        self.roi_bottom = float(lp.get('roi_bottom', 0.95))
        self.bands = int(lp.get('bands', 5))
        self.min_px = int(lp.get('min_pixels_per_band', 120))
        self.use_both = bool(lp.get('use_both_lines', True))
        self.offset = float(lp.get('single_line_offset_px', 260))

        self.hsv1_lo = np.array(lp.get('hsv_lower1', [15, 80, 100]), dtype=np.uint8)
        self.hsv1_hi = np.array(lp.get('hsv_upper1', [40, 255, 255]), dtype=np.uint8)
        self.hsv2_lo = np.array(lp.get('hsv_lower2', [0, 0, 0]), dtype=np.uint8)
        self.hsv2_hi = np.array(lp.get('hsv_upper2', [0, 0, 0]), dtype=np.uint8)
        self.use_range2 = bool(np.any(self.hsv2_hi > 0))

        self.gain = float(lp.get('steering_gain', 1.6))
        self.max_steering = float(lp.get('max_steering', 1.0))
        self.max_step = float(lp.get('max_steering_step', 0.15))
        self.ema = EMA(float(lp.get('steering_ema', 0.35)))

        self.steering = 0.0
        self.lane_center = None
        self.valid = False
        self.no_lane_count = 0

    # ------------------------------------------------------------------
    def reset(self):
        self.steering = 0.0
        self.lane_center = None
        self.valid = False
        self.no_lane_count = 0
        self.ema.reset()

    # ------------------------------------------------------------------
    def _mask(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, self.hsv1_lo, self.hsv1_hi)
        if self.use_range2:
            m2 = cv2.inRange(hsv, self.hsv2_lo, self.hsv2_hi)
            m = cv2.bitwise_or(m, m2)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
        return m

    # ------------------------------------------------------------------
    def update(self, bgr):
        """
        输入一帧 BGR 图像，更新 self.steering。
        返回 (steering, debug_bgr, info)
        """
        h, w = bgr.shape[:2]
        y1 = int(h * self.roi_top)
        y2 = int(h * self.roi_bottom)
        roi = bgr[y1:y2, :]
        mask = self._mask(roi)
        mh = mask.shape[0]

        centers = []
        weights = []
        left_seen = 0
        right_seen = 0

        band_h = max(1, mh // self.bands)
        for b in range(self.bands):
            bs = b * band_h
            be = mh if b == self.bands - 1 else (b + 1) * band_h
            strip = mask[bs:be, :]
            cols = np.where(strip.sum(axis=0) > 0)[0]
            if cols.size < self.min_px:
                continue

            lx = float(cols.min())
            rx = float(cols.max())

            # 一条带里左右两端相距太近 -> 认为只有一条线
            if self.use_both and (rx - lx) > 0.25 * w:
                c = (lx + rx) / 2.0
                left_seen += 1
                right_seen += 1
            else:
                # 只有一条线：按它离画面中线偏左还是偏右，判断它是左线还是右线
                if (lx + rx) / 2.0 < w / 2.0:
                    c = lx + self.offset        # 是左线，车道中心在它右边
                    left_seen += 1
                else:
                    c = rx - self.offset        # 是右线，车道中心在它左边
                    right_seen += 1

            centers.append(c)
            # 越靠近画面下方的带越可信
            weights.append(1.0 + 1.0 * (b / max(1, self.bands - 1)))

        info = {'left': left_seen, 'right': right_seen, 'bands_ok': len(centers)}

        if centers:
            wsum = sum(weights)
            self.lane_center = sum(c * wt for c, wt in zip(centers, weights)) / wsum
            raw = (w / 2.0 - self.lane_center) / (w * 0.5)   # 正 = 需要左转
            raw = clamp(raw * self.gain, -self.max_steering, self.max_steering)
            self.ema.update(raw)
            target = self.ema.value
            self.steering = rate_limit(target, self.steering, self.max_step)
            self.valid = True
            self.no_lane_count = 0
        else:
            self.no_lane_count += 1
            self.valid = False
            # 丢线时缓慢回正，不要猛打
            self.steering = rate_limit(0.0, self.steering, self.max_step)

        info['steering'] = self.steering
        info['lane_center'] = self.lane_center
        info['valid'] = self.valid

        dbg = self._draw(bgr, y1, y2, mask, info)
        return self.steering, dbg, info

    # ------------------------------------------------------------------
    def _draw(self, bgr, y1, y2, mask, info):
        img = bgr.copy()
        h, w = img.shape[:2]
        cv2.rectangle(img, (0, y1), (w, y2), (0, 255, 255), 1)
        cv2.line(img, (w // 2, y1), (w // 2, y2), (255, 0, 0), 1)
        if self.lane_center is not None:
            cv2.line(img, (int(self.lane_center), y1),
                     (int(self.lane_center), y2), (0, 0, 255), 2)
        txt = "steer=%+.2f  L=%d R=%d  %s" % (
            self.steering, info.get('left', 0), info.get('right', 0),
            "OK" if info.get('valid') else "NO-LANE")
        cv2.putText(img, txt, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if info.get('valid') else (0, 0, 255), 2)
        small = cv2.resize(mask, (w // 3, max(1, (y2 - y1) // 3)))
        img[y1:y1 + small.shape[0], w - small.shape[1]:w] = cv2.cvtColor(
            small, cv2.COLOR_GRAY2BGR)
        return img
