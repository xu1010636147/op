import av
import cv2
import threading
import queue
import os

from openpilot.common.swaglog import cloudlog

# 相机掉线后的重连退避：0.5s 起、翻倍至 5s 封顶。
# USB 设备重新枚举通常约 1 秒完成，所以头两次重试就能命中，恢复在 1~3 秒内。
INITIAL_BACKOFF = 0.5
MAX_BACKOFF = 5.0


class Camera:
    def __init__(self, cam_type_state, stream_type, camera_id):
        try:
            camera_id = int(camera_id)
        except ValueError: # allow strings, ex: /dev/video0
            pass
        self.cam_type_state = cam_type_state
        self.stream_type = stream_type
        self.cur_frame_id = 0

        self.container = av.open(camera_id)
        assert self.container.streams.video, f"Can't open video stream for camera {camera_id}"
        self.video_stream = self.container.streams.video[0]
        self.W = self.video_stream.codec_context.width
        self.H = self.video_stream.codec_context.height

    @classmethod
    def bgr2nv12(self, bgr):
        frame = av.VideoFrame.from_ndarray(bgr, format='bgr24')
        return frame.reformat(format='nv12').to_ndarray()

    def read_frames(self):
        for frame in self.container.decode(self.video_stream):
            img = frame.to_rgb().to_ndarray()[:,:, ::-1] # convert to bgr24
            yuv = Camera.bgr2nv12(img)
            yield yuv.data.tobytes()
        self.container.close()

class CameraMJPG:
    def __init__(self, cam_type_state, stream_type, camera_id, width, height,
                 num_workers=None, max_queue_size=10):
        try:
            camera_id = int(camera_id)
        except ValueError:
            pass

        self.cam_type_state = cam_type_state
        self.stream_type = stream_type
        self.camera_id = camera_id
        self.cur_frame_id = 0
        self.fps = None

        # 分辨率由调用方给定，而不是从设备读回。原因：visionipc 的 buffer 尺寸在
        # create_buffers 时就固定了，而 create_buffers 必须先于 start_listener()。
        # 若从设备读回，camerad 的启动就被绑在"相机此刻已枚举完成"上 —— 而设备刚插上
        # 或刚重枚举的那一瞬间恰恰打不开，会让整个 camerad 起不来。
        # 契约：实际协商出的分辨率必须与之一致，见 _open_camera()。
        self.W = width
        self.H = height
        self.current_format = "MJPG"

        # 打开动作交给读线程：构造不再因硬件不在位而抛异常。
        self.cap = None

        # 多线程相关。默认按 cpu_count 起，但双摄时就是 2×核心数 个转换线程；
        # 转换工作总量固定、队列里每帧只会被一个线程取走，线程过多只增加调度与 GIL 开销。
        # 可用 CAM_NUM_WORKERS 覆盖。
        if num_workers is None:
            num_workers = int(os.getenv("CAM_NUM_WORKERS") or 0) or (os.cpu_count() or 4)
        self.num_workers = num_workers
        self.frame_queue = queue.Queue(maxsize=max_queue_size)
        self.output_queue = queue.Queue(maxsize=max_queue_size)
        self.stop_event = threading.Event()
        self.read_thread = threading.Thread(target=self._frame_reader)

        self.read_thread.start()

    def _configure_camera_format(self, cap, target_fourcc):
        fourcc = cv2.VideoWriter_fourcc(*target_fourcc)
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        cap.set(cv2.CAP_PROP_FPS, 20)

    @staticmethod
    def _get_current_format(cap):
        fourcc_code = int(cap.get(cv2.CAP_PROP_FOURCC))
        return ''.join([chr((fourcc_code >> 8 * i) & 0xFF) for i in range(4)])

    def _open_camera(self):
        """打开并配置一路相机，成功返回句柄，失败释放句柄后抛出。"""
        cap = cv2.VideoCapture(self.camera_id)
        try:
            if not cap.isOpened():
                raise IOError(f"无法打开摄像头设备 {self.camera_id}")

            self._configure_camera_format(cap, "MJPG")

            # 分辨率不符时必须失败重试，不能放行：visionipc 的 send() 带有
            # buf.len == len(data) 的断言，喂进尺寸不符的帧会直接终止整个 camerad 进程。
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if (w, h) != (self.W, self.H):
                raise IOError(f"分辨率不符 {w}x{h} != {self.W}x{self.H}（vipc buffer 已固定）")

            self.current_format = self._get_current_format(cap)
            self.fps = cap.get(cv2.CAP_PROP_FPS)
            return cap
        except Exception:
            cap.release()
            raise

    def _release_cap(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def _drain_queues(self):
        # 重连时丢弃掉线前积压的帧，避免它们带着新的 frame_id 被送出。
        # 不补 task_done()：没有任何地方 join 这两个队列，未完成任务计数无意义。
        for q in (self.frame_queue, self.output_queue):
            while True:
                try:
                    q.get_nowait()
                except queue.Empty:
                    break

    @staticmethod
    def _bgr_to_nv12(bgr_frame):
        frame = av.VideoFrame.from_ndarray(bgr_frame, format='bgr24')
        return frame.reformat(format='nv12').to_ndarray().data.tobytes()

    def _frame_reader(self):
        # 掉线自愈：读失败不置 stop_event，退避后重开设备。
        # stop_event 只由外部（read_frames 的 finally / __del__）设置，用于显式关闭，
        # 因此"显式关闭"与"设备掉线"是两条互不干扰的路径。
        backoff = INITIAL_BACKOFF
        while not self.stop_event.is_set():
            if self.cap is None:
                try:
                    self.cap = self._open_camera()
                except Exception as e:
                    cloudlog.error(f"{self.camera_id}: 打开失败: {e}，{backoff:.1f}s 后重试")
                    self.stop_event.wait(backoff)
                    backoff = min(backoff * 2, MAX_BACKOFF)
                    continue
                backoff = INITIAL_BACKOFF
                cloudlog.warning(f"{self.camera_id}: 已打开 {self.W}x{self.H} "
                                 f"{self.current_format} {self.fps}fps")

            if not self.frame_queue.full():
                ret, frame = self.cap.read()
                if not ret:
                    cloudlog.error(f"{self.camera_id}: 读帧失败，重连")
                    self._release_cap()
                    self._drain_queues()
                    backoff = INITIAL_BACKOFF
                    continue
                self.frame_queue.put(frame)
            else:
                self.stop_event.wait(0.01)

        # 由本线程负责释放：主线程若与 read() 并发 release 同一个句柄是未定义行为。
        self._release_cap()

    def _frame_worker(self):
        while not self.stop_event.is_set() or not self.frame_queue.empty():
            try:
                frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                # 丢帧而不是 raise：worker 线程里抛异常会静默少一个转换线程。
                if self.current_format == "MJPG" and frame.shape != (self.H, self.W, 3):
                    cloudlog.error(f"{self.camera_id}: 帧形状异常 {frame.shape} != "
                                   f"({self.H}, {self.W}, 3)，丢弃")
                    continue
                self.output_queue.put(self._bgr_to_nv12(frame))
            finally:
                self.frame_queue.task_done()

    def read_frames(self):
        workers = []
        for _ in range(self.num_workers):
            t = threading.Thread(target=self._frame_worker)
            t.start()
            workers.append(t)

        try:
            while not self.stop_event.is_set() or not self.output_queue.empty():
                try:
                    yuv_bytes = self.output_queue.get(timeout=0.5)
                    yield yuv_bytes
                    self.output_queue.task_done()
                except queue.Empty:
                    if self.stop_event.is_set():
                        break
        finally:
            self.stop_event.set()
            # 带上限：半死的 USB 设备可能让 read() 一直阻塞，不该拖住关闭流程。
            # 句柄由读线程自己释放，所以超时后放弃等待也不会留下竞态。
            self.read_thread.join(timeout=2.0)
            for t in workers:
                t.join()

    def __del__(self):
        self.stop_event.set()
