#!/usr/bin/env python3
import time
import numpy as np
from collections import deque

from msgq.visionipc.visionipc_pyx import VisionIpcClient, VisionStreamType
import cereal.messaging as messaging
from openpilot.common.transformations.camera import get_view_frame_from_calib_frame, DEVICE_CAMERAS
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


# ==============================================================================
# 车道线类型检测器类
# ==============================================================================

class LaneLineDetector:
    """车道线实线/虚线检测器"""

    FULL_RES_WIDTH = 1928

    def __init__(self):
        self.params = Params()

        self._last_params = {
          "left": None,
          "right": None,
          "left_rel": None,
          "right_rel": None,
        }

        self.left_last_type = -1
        self.right_last_type = -1

        self.intrinsics = None
        self.stride = None
        self.w, self.h = None, None
        self.left_history = None
        self.right_history = None

        # ===================== 【新增】横向采样 & presence 参数 =====================
        self.line_y_threshold = 200        # Y 平面亮度阈值
        self.lateral_range_m = 0.3         # ±30cm
        self.lateral_samples = 15          # 【修改】横向采样点数 11->15
        self.min_x_presence_ratio = 0.7    # 实线阈值
        self.max_x_absence_ratio = 0.5     # 虚线阈值
        # ==========================================================================

        self.update_params()
        cloudlog.info("LaneLineDetector initialized")

    def update_params(self):
        """从 Params 系统更新可调参数"""
        try:
            self.lookahead_start = float(self.params.get("LaneDetectLookaheadStart", encoding='utf8') or "6.0")
        except Exception:
            self.lookahead_start = 6.0

        try:
            self.lookahead_end = float(self.params.get("LaneDetectLookaheadEnd", encoding='utf8') or "30.0")
        except Exception:
            self.lookahead_end = 30.0

        try:
            self.num_points = int(self.params.get("LaneDetectNumPoints", encoding='utf8') or "40")
        except Exception:
            self.num_points = 40

        try:
            self.relative_threshold_low = float(self.params.get("LaneDetectThresholdLow", encoding='utf8') or "0.095")
        except Exception:
            self.relative_threshold_low = 0.095

        try:
            self.relative_threshold_high = float(self.params.get("LaneDetectThresholdHigh", encoding='utf8') or "0.105")
        except Exception:
            self.relative_threshold_high = 0.105

        try:
            self.prob_threshold = float(self.params.get("LaneDetectProbThreshold", encoding='utf8') or "0.3")
        except Exception:
            self.prob_threshold = 0.3

        try:
            new_history_frames = int(self.params.get("LaneDetectHistoryFrames", encoding='utf8') or "5")
        except Exception:
            new_history_frames = 5

        self.history_frames = new_history_frames

        if self.left_history is None or self.right_history is None:
            self.left_history = deque(maxlen=self.history_frames)
            self.right_history = deque(maxlen=self.history_frames)
        elif len(self.left_history) != 0 and self.left_history.maxlen != self.history_frames:
            self.left_history = deque(self.left_history, maxlen=self.history_frames)
            self.right_history = deque(self.right_history, maxlen=self.history_frames)

    def init_camera(self, sm, vipc_client):
        if self.intrinsics is not None:
            return True
        if not sm.updated['deviceState'] or not sm.updated['roadCameraState']:
            return False
        try:
            device_type = str(sm['deviceState'].deviceType)
            sensor = str(sm['roadCameraState'].sensor)
            camera = DEVICE_CAMERAS[(device_type, sensor)]

            self.stride = vipc_client.stride
            self.w = vipc_client.width
            self.h = vipc_client.height

            scale = self.w / self.FULL_RES_WIDTH
            self.intrinsics = camera.fcam.intrinsics * scale
            self.intrinsics[2, 2] = 1.0
            cloudlog.info(f"Camera initialized: {self.w}x{self.h}, device={device_type}")
            return True
        except Exception as e:
            cloudlog.error(f"Camera initialization failed: {e}")
            return False

    # ===================== 【修改】完整 update 函数 =====================
    def update(self, sm, yuv_buf):
        result = {
            'left': -1,
            'right': -1,
            'left_rel_std': 0.0,
            'right_rel_std': 0.0,
            # 【新增】保证 Presence 字段存在
            'left_x_presence': 0.0,
            'right_x_presence': 0.0,
            'left_max_run': 0,
            'right_max_run': 0,
        }

        if not sm.updated['modelV2'] or not sm.updated['liveCalibration']:
            return result

        model = sm['modelV2']
        calib = sm['liveCalibration']

        try:
            imgff = np.frombuffer(yuv_buf.data, dtype=np.uint8)
            y_plane = imgff[: self.stride * self.h]
            y_data = y_plane.reshape(self.h, self.stride)[:, :self.w]
        except Exception as e:
            cloudlog.error(f"YUV extraction failed: {e}")
            return result

        try:
            extrinsic_matrix_full = get_view_frame_from_calib_frame(
                calib.rpyCalib[0],
                0.0,
                0.0,
                0.0
            )
        except Exception as e:
            cloudlog.error(f"Calibration frame conversion failed: {e}")
            return result

        for i, line_idx in enumerate([1, 2]):
            try:
                line = model.laneLines[line_idx]
                line_prob = model.laneLineProbs[line_idx]
            except IndexError:
                continue

            side_key = 'left' if i == 0 else 'right'
            current_history = self.left_history if i == 0 else self.right_history
            last_type = self.left_last_type if i == 0 else self.right_last_type

            if line_prob < self.prob_threshold:
                current_history.append(None)
                continue

            xs, ys, zs = np.array(line.x), np.array(line.y), np.array(line.z)
            if len(xs) < 10:
                current_history.append(None)
                continue

            sample_xs = np.linspace(self.lookahead_start, self.lookahead_end, self.num_points)
            sample_ys = np.interp(sample_xs, xs, ys)
            sample_zs = np.interp(sample_xs, xs, zs)

            # ===================== 【新增】横向 ±30cm 扫描 + 按 x 聚合 presence =====================
            y_offsets = np.linspace(-self.lateral_range_m, self.lateral_range_m, self.lateral_samples)
            per_x_has_line = []
            all_pixel_values = []

            for k in range(self.num_points):
                base_x = sample_xs[k]
                base_y = sample_ys[k]
                base_z = sample_zs[k]
                has_line_at_x = False

                for dy in y_offsets:
                    local_point_homo = np.array([base_x, base_y + dy, base_z, 1.0])
                    view_point_homo = extrinsic_matrix_full @ local_point_homo

                    if view_point_homo[2] <= 0:
                        continue

                    u = int(view_point_homo[0] / view_point_homo[2] * self.intrinsics[0, 0] + self.intrinsics[0, 2])
                    v = int(view_point_homo[1] / view_point_homo[2] * self.intrinsics[1, 1] + self.intrinsics[1, 2])

                    if 0 <= u < self.w and 0 <= v < self.h:
                        y_val = int(y_data[v, u])
                        all_pixel_values.append(y_val)
                        if y_val > self.line_y_threshold:
                            has_line_at_x = True
                            break
                per_x_has_line.append(has_line_at_x)

            valid_x = [v for v in per_x_has_line if v is not None]

            if len(valid_x) < self.num_points // 2:
                cur_type = -1
            else:
                x_presence_ratio = np.mean(valid_x)

                # 统计最大连续段
                max_run = 0
                current_run = 0
                for v in valid_x:
                    if v:
                        current_run += 1
                        max_run = max(max_run, current_run)
                    else:
                        current_run = 0

                # 判定 + 迟滞
                if x_presence_ratio > self.min_x_presence_ratio:
                    cur_type = 1
                elif x_presence_ratio < self.max_x_absence_ratio:
                    cur_type = 0
                else:
                    cur_type = last_type

                if cur_type == 1 and max_run < self.num_points * 0.4:
                    cur_type = last_type

            current_history.append(cur_type if cur_type >= 0 else None)
            if i == 0:
                self.left_last_type = cur_type
            else:
                self.right_last_type = cur_type

            # 记录结果
            result[side_key] = cur_type
            result[f'{side_key}_rel_std'] = np.std(all_pixel_values) / max(np.mean(all_pixel_values), 1.0)
            result[f'{side_key}_x_presence'] = np.mean(valid_x) if valid_x else 0.0
            result[f'{side_key}_max_run'] = max_run if valid_x else 0

        return result
    # ===================== 【修改结束】update =====================

    def publish_result(self, pm, result):
        try:
            if result['left'] != self._last_params["left"]:
                self.params.put_nonblocking("LaneLineTypeLeft", str(result['left']))
                self._last_params["left"] = result['left']

            if result['right'] != self._last_params["right"]:
                self.params.put_nonblocking("LaneLineTypeRight", str(result['right']))
                self._last_params["right"] = result['right']

            left_rel = round(result['left_rel_std'], 4)
            if left_rel != self._last_params["left_rel"]:
                self.params.put_nonblocking("LaneLineRelStdLeft", f"{left_rel:.4f}")
                self._last_params["left_rel"] = left_rel

            right_rel = round(result['right_rel_std'], 4)
            if right_rel != self._last_params["right_rel"]:
                self.params.put_nonblocking("LaneLineRelStdRight", f"{right_rel:.4f}")
                self._last_params["right_rel"] = right_rel

        except Exception as e:
            cloudlog.warning(f"Failed to publish results to Params: {e}")

# ==============================================================================
# 主程序 (独立测试)
# ==============================================================================

def main():
    detector = LaneLineDetector()
    sm = messaging.SubMaster(['modelV2', 'liveCalibration', 'deviceState', 'roadCameraState'])
    vipc_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)

    while not vipc_client.connect(False):
        time.sleep(0.2)

    while True:
        sm.update(0)
        if detector.init_camera(sm, vipc_client):
            break
        time.sleep(0.1)

    while True:
        sm.update(0)
        yuv_buf = vipc_client.recv()

        result = detector.update(sm, yuv_buf)
        detector.publish_result(None, result)

        left_type = ['虚线', '实线', '不确定/丢失'][result['left'] if result['left'] >= 0 else 2]
        right_type = ['虚线', '实线', '不确定/丢失'][result['right'] if result['right'] >= 0 else 2]

        print(f"\033[2J\033[H", end="")
        print(f"=== 车道线识别 (Res: {detector.w}x{detector.h}) ===")
        print(f"左侧: {left_type}  (AvgRel: {result['left_rel_std']:.3f}, Presence: {result['left_x_presence']:.2f})")
        print(f"右侧: {right_type}  (AvgRel: {result['right_rel_std']:.3f}, Presence: {result['right_x_presence']:.2f})")
        print("----------------------------------")

if __name__ == "__main__":
    main()
