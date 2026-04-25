from Vision.vlm import VisionCore


def observe_environment(image_path="", emit=None):
    vision = VisionCore()
    visual_context = vision.describe_structured(image_path or None)
    scene_facts = VisionCore.build_scene_facts(visual_context)
    if emit:
        emit("status", scene_facts.get("summary", "环境观察完成"))
    return {
        "status": "success",
        "image_source": str(vision.last_image_path or ""),
        "visual_context": visual_context,
        "scene_facts": scene_facts,
    }
#观察环境图片，返回视觉语义和规划用场景事实
