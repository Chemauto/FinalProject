import os
from pathlib import Path

try:
    import cv2
except Exception:
    cv2 = None

DEFAULT_IMAGE = Path(__file__).resolve().parent / "assets" / "1.png"
OLD_PROJECT_IMAGE = Path("/home/xcj/work/FinalProject/Data_Module/assets/2.png")


class ImageSource:
    def __init__(self, default_image=None):
        self.default_image = Path(default_image or DEFAULT_IMAGE)

    def get_image(self, image_path=None):
        for candidate in _candidate_paths(image_path, self.default_image):
            if candidate and candidate.is_file() and candidate.stat().st_size > 0:
                return candidate
        camera_image = _capture_from_camera()
        if camera_image:
            return camera_image
        raise FileNotFoundError("未找到可用图片，请传入image_path或设置VISION_IMAGE_PATH")
#按优先级获取图片路径：显式路径、环境变量、默认图、摄像头


def _candidate_paths(image_path, default_image):
    values = [
        image_path,
        os.getenv("VISION_IMAGE_PATH"),
        os.getenv("FINALPROJECT_VLM_IMAGE_PATH"),
        default_image,
        OLD_PROJECT_IMAGE,
    ]
    return [Path(item).expanduser().resolve() for item in values if item]
#生成候选图片路径列表


def _capture_from_camera():
    if cv2 is None:
        return None
    port = os.getenv("VISION_CAMERA_PORT", "0").replace("video", "")
    device = Path(f"/dev/video{port or 0}")
    if not device.exists():
        return None
    cap = cv2.VideoCapture(str(device))
    if not cap or not cap.isOpened():
        return None
    ok, frame = cap.read()
    cap.release()
    if not ok:
        return None
    output = Path(__file__).resolve().parent / "assets" / "camera_capture.jpg"
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), frame)
    return output
#尝试从本机摄像头抓一帧图片
