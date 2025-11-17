import numpy as np
import sounddevice as sd
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

SAMPLE_RATE = 16000
FRAME_MS    = 50
THRESHOLD   = 0.02
SMOOTH_WIN  = 8
PUBLISH_HZ  = 20


class NoiseCore:
    def __init__(self, threshold=THRESHOLD, smooth_win=SMOOTH_WIN):
        self.th = threshold
        self.buf = deque(maxlen=smooth_win)
        self.smooth = 0.0
        self.is_loud = False

    def process(self, frame: np.ndarray):
        rms = float(np.sqrt(np.mean(frame**2)))
        self.buf.append(rms)
        self.smooth = float(np.mean(self.buf))
        self.is_loud = self.smooth > self.th
        return rms, self.smooth, self.is_loud

class NoiseNode(Node):
    def __init__(self):
        super().__init__('noise_monitor')

        # === 1) 参数声明（默认值） ===
        self.declare_parameter('threshold', 0.02)     # 噪声阈值
        self.declare_parameter('frame_ms', 50)        # 每帧时长(ms)
        self.declare_parameter('smooth_win', 8)       # 平滑窗口(帧)
        self.declare_parameter('device_index', -1)    # -1=默认设备；>=0 指定设备
        self.declare_parameter('publish_hz', 20.0)    # 发布频率(Hz)

        # === 2) 读取参数 ===
        THRESHOLD  = float(self.get_parameter('threshold').value)
        FRAME_MS   = int(self.get_parameter('frame_ms').value)
        SMOOTH_WIN = int(self.get_parameter('smooth_win').value)
        DEVICE     = int(self.get_parameter('device_index').value)
        PUBLISH_HZ = float(self.get_parameter('publish_hz').value)

        # === 3) 初始化算法/发布器/定时器 ===
        self.core = NoiseCore(threshold=THRESHOLD, smooth_win=SMOOTH_WIN)
        frame_size = int(SAMPLE_RATE * FRAME_MS / 1000)

        self.pub_vol  = self.create_publisher(Float32, '/noise/volume', 10)
        self.pub_loud = self.create_publisher(Bool,    '/noise/is_loud', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.on_timer)
        self.latest = None

        # === 4) 尝试打开输入流（无麦克风时给出友好提示并退出） ===
        try:
            self.stream = sd.InputStream(
                device=None if DEVICE < 0 else DEVICE,
                samplerate=SAMPLE_RATE, channels=1,
                blocksize=frame_size, dtype='float32',
                callback=self.on_audio
            )
            self.stream.start()
            self.get_logger().info(
                f'Audio stream started (device={DEVICE}, frame_ms={FRAME_MS}, threshold={THRESHOLD})'
            )
        except Exception as e:
            self.get_logger().error(
                f'Failed to start audio stream (likely no mic connected): {e}'
            )
            # 没有麦克风就不要继续 spin，抛出即可；等设备到了再运行
            raise


    def on_audio(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f'Audio status: {status}')
        self.latest = self.core.process(indata)

    def on_timer(self):
        if self.latest is None:
            return
        _, smooth, loud = self.latest
        self.pub_vol.publish(Float32(data=float(smooth)))
        self.pub_loud.publish(Bool(data=bool(loud)))

def main():
    rclpy.init()
    node = NoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
