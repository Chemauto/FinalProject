from __future__ import annotations
"""图片输入模块：支持仿真截图、USB 摄像头、ROS2 image topic、回退默认图片。"""
import os
from pathlib import Path

try:
    import cv2
except Exception:
    cv2 = None

# ── ROS2 可选导入（Go2 真机摄像头） ──────────────────────
_rclpy_ok = False
try:
    import sys as _sys
    import io as _io
    _saved_stderr = _sys.stderr
    _sys.stderr = _io.StringIO()
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        _rclpy_ok = True
    finally:
        _sys.stderr = _saved_stderr
except Exception:
    CvBridge = None

Video_Port = "video3"
image_path = str(Path(__file__).resolve().parent / "assets" / "2.png")
envtest_image_path = os.getenv("FINALPROJECT_VLM_IMAGE_PATH", "/tmp/envtest_front_camera.png")

# ── ROS2 image topic（TODO: 替换为实际 topic） ─────────────
ROS2_IMAGE_TOPIC = os.getenv(
    "FINALPROJECT_VLM_ROS2_IMAGE_TOPIC",
    "/go2/wrist_camera/image_raw",
)


class ImageSource:
    """负责提供最终可用的图片路径。"""

    def __init__(
        self,
        Video_Port: int | str = Video_Port,
        default_image: str | None = image_path,
        live_image: str | None = envtest_image_path,
    ):
        self.Video_Port = Video_Port
        self.default_image = Path(default_image or image_path)
        self.live_image = Path(live_image).expanduser() if live_image else None
        self.assets_dir = Path(__file__).resolve().parent / "assets"
        self._ros2_latest_path: Path | None = None

    def _capture_from_camera(self) -> Path | None:
        if cv2 is None:
            return None

        port = self.Video_Port
        if not isinstance(port, int):
            text = str(port).strip().lower()
            if text.startswith("video"):
                text = text[5:]
            port = int(text or 0)

        device_path = Path(f"/dev/video{port}")
        if not device_path.exists():
            return None

        cap = cv2.VideoCapture(str(device_path))
        if not cap or not cap.isOpened():
            if cap:
                cap.release()
            return None

        ok, frame = cap.read()
        cap.release()
        if ok:
            output = self.assets_dir / "camera_capture.jpg"
            cv2.imwrite(str(output), frame)
            return output
        return None

    def _read_live_envtest_image(self) -> Path | None:
        if self.live_image is None:
            return None

        path = self.live_image.expanduser().resolve()
        if path.is_file() and path.stat().st_size > 0:
            return path
        return None

    def _capture_from_ros2_topic(self) -> Path | None:
        if not _rclpy_ok or CvBridge is None:
            return None

        robot_type = os.getenv("FINALPROJECT_ROBOT_TYPE", "sim").strip().lower()
        if robot_type != "go2":
            return None

        try:
            bridge = CvBridge()
            topic = ROS2_IMAGE_TOPIC

            class _ImageListener(Node):
                def __init__(self):
                    super().__init__("finalproject_vlm_image_listener")
                    self.latest_frame = None
                    self.received = False
                    self.sub = self.create_subscription(
                        Image, topic, self._callback, 10,
                    )

                def _callback(self, msg: Image):
                    self.latest_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
                    self.received = True

            if not rclpy.ok():
                rclpy.init()
            node = _ImageListener()
            rclpy.spin_once(node, timeout_sec=0.5)

            if node.latest_frame is not None:
                output = self.assets_dir / "ros2_camera_capture.jpg"
                cv2.imwrite(str(output), node.latest_frame)
                node.destroy_node()
                return output

            node.destroy_node()
        except Exception as e:
            print(f"[VLM] ROS2 image capture 失败: {e}", file=os.sys.stderr if hasattr(os, 'sys') else __import__('sys').stderr)
        return None

    def get_image(self, image_path: str | None = None) -> Path:
        if image_path:
            path = Path(image_path).expanduser().resolve()
            if path.is_file():
                return path

        live_envtest_image = self._read_live_envtest_image()
        if live_envtest_image:
            return live_envtest_image

        ros2_image = self._capture_from_ros2_topic()
        if ros2_image:
            return ros2_image

        camera_image = self._capture_from_camera()
        if camera_image:
            return camera_image

        path = self.default_image.expanduser().resolve()
        if path.is_file():
            return path
        raise FileNotFoundError(f"默认图片不存在: {path}")
