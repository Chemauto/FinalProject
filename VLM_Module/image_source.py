from __future__ import annotations
"""图片输入模块：优先读取摄像头，否则回退到默认图片。"""
from pathlib import Path

try:
    import cv2
except Exception:
    cv2 = None

Video_Port = "video3"
image_path = str(Path(__file__).resolve().parent / "assets" / "0.png")


class ImageSource:
    """负责提供最终可用的图片路径。"""

    def __init__(self, Video_Port: int | str = Video_Port, default_image: str | None = image_path):
        """初始化摄像头端口和默认图片路径。"""
        self.Video_Port = Video_Port
        self.default_image = Path(default_image or image_path)
        self.assets_dir = Path(__file__).resolve().parent / "assets"

    def _capture_from_camera(self) -> Path | None:
        """严格从指定摄像头端口抓取图片。"""
        if cv2 is None:
            print("[ImageSource] 未安装 cv2，跳过摄像头读取")
            return None

        port = self.Video_Port
        if not isinstance(port, int):
            text = str(port).strip().lower()
            if text.startswith("video"):
                text = text[5:]
            port = int(text or 0)

        device_path = Path(f"/dev/video{port}")
        print(f"[ImageSource] 尝试读取摄像头: {device_path}")
        if not device_path.exists():
            print(f"[ImageSource] 摄像头不存在: {device_path}")
            return None

        cap = cv2.VideoCapture(str(device_path))
        if not cap or not cap.isOpened():
            if cap:
                cap.release()
            print(f"[ImageSource] 摄像头打开失败: {device_path}")
            return None

        ok, frame = cap.read()
        cap.release()
        if ok:
            output = self.assets_dir / "camera_capture.jpg"
            cv2.imwrite(str(output), frame)
            print(f"[ImageSource] 摄像头读取成功，图片已保存到: {output}")
            return output
        print(f"[ImageSource] 摄像头读取失败: {device_path}")
        return None


    def get_image(self, image_path: str | None = None) -> Path:
        """优先使用输入图片，否则尝试摄像头，失败后回退到默认图片。"""
        if image_path:
            path = Path(image_path).expanduser().resolve()
            if path.is_file():
                return path

        camera_image = self._capture_from_camera()
        if camera_image:
            return camera_image

        path = self.default_image.expanduser().resolve()
        if path.is_file():
            print(f"[ImageSource] 使用默认图片: {path}")
            return path
        raise FileNotFoundError(f"默认图片不存在: {path}")
