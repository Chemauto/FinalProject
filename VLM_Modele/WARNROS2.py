import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavigateClient(Node):

    def __init__(self):
        super().__init__('navigate_client')
        self._client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

    def send_goal(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._client.wait_for_server()
        self._client.send_goal_async(goal)


def send_navigate_goal(parameters):
    """
    parameters example:
    {
      "target": "table"
    }
    """
    rclpy.init()
    node = NavigateClient()

    pose = PoseStamped()
    pose.header.frame_id = "map"

    # 示例：硬编码 table 位置（真实系统中来自地图 / perception）
    if parameters.get("target") == "table":
        pose.pose.position.x = 1.2
        pose.pose.position.y = 0.5
        pose.pose.orientation.w = 1.0

    node.send_goal(pose)
    rclpy.spin_once(node)
    rclpy.shutdown()
