# -*- coding: utf-8 -*-
# ROS_Module/ros_module/nodes/simulator_node.py
import sys
import pygame
import math
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# --- Constants ---
WIDTH, HEIGHT = 800, 600
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
ROBOT_SIZE = 20
ROBOT_SPEED = 3  # Pixels per frame
ROBOT_TURN_SPEED = 3  # Degrees per frame

# --- Robot Class (adapted from Dora_Module/simulator.py) ---
class Robot:
    """
    Represents the robot in the simulator.
    """

    def __init__(self, x, y):
        """
        Initializes the Robot.

        Args:
            x (int): The initial x-coordinate of the robot.
            y (int): The initial y-coordinate of the robot.
        """
        self.x = x
        self.y = y
        self.angle = 0  # In degrees
        self.target_x = x
        self.target_y = y
        self.target_angle = 0

    def set_navigation_goal(self, params):
        """
        Processes navigation commands to set a new target.

        Args:
            params (dict): The navigation parameters.
        """
        print(f"SIM: Received navigation command with params: {params}")
        location = params.get('location', 'current')
        direction = params.get('direction', None)
        distance_str = params.get('distance', "0cm")
        angle_str = params.get('angle', None)

        # Simplified distance parsing (cm -> pixels)
        try:
            if 'cm' in distance_str:
                distance = float(distance_str.replace('cm', ''))
            elif 'm' in distance_str:
                distance = float(distance_str.replace('m', '')) * 100
            else:
                distance = 0
        except ValueError:
            distance = 0

        # Determine target angle based on direction or explicit angle
        new_target_angle = None
        if angle_str:
            try:
                angle_value = float(angle_str.replace('deg', ''))
                # ROS standard is counter-clockwise, so we might need to adjust based on final system behavior
                new_target_angle = (self.angle - angle_value) % 360 # Assuming angle is relative turn
            except ValueError:
                print(f"SIM: Invalid angle format: {angle_str}")
        elif direction:
            if direction == 'front':
                new_target_angle = self.angle
            elif direction == 'back':
                new_target_angle = (self.angle - 180) % 360
            elif direction == 'left':
                new_target_angle = (self.angle + 90) % 360
            elif direction == 'right':
                new_target_angle = (self.angle - 90) % 360
        
        if new_target_angle is not None:
            self.target_angle = new_target_angle

        # Calculate target position using the determined target_angle
        # Note: In ROS, positive X is forward, positive Y is left. Pygame's Y is inverted.
        rad_angle = math.radians(self.angle) # Use current angle for forward movement direction
        if direction: # Only move if a direction is specified
            self.target_x = self.x + distance * math.cos(rad_angle)
            self.target_y = self.y - distance * math.sin(rad_angle) # Pygame Y is inverted
        
        print(f"SIM: New Target: ({self.target_x:.2f}, {self.target_y:.2f}) at angle {self.target_angle:.2f}")


    def update(self):
        """
        Moves the robot towards its target position and angle.
        """
        # Update angle
        angle_diff = (self.target_angle - self.angle + 180) % 360 - 180
        if abs(angle_diff) > ROBOT_TURN_SPEED:
            self.angle += ROBOT_TURN_SPEED if angle_diff > 0 else -ROBOT_TURN_SPEED
            self.angle %= 360
        else:
            self.angle = self.target_angle

        # Update position
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dist = math.sqrt(dx**2 + dy**2)

        if dist > ROBOT_SPEED:
            self.x += (dx / dist) * ROBOT_SPEED
            self.y += (dy / dist) * ROBOT_SPEED
        else:
            self.x = self.target_x
            self.y = self.target_y


    def draw(self, screen):
        """
        Draws the robot as a triangle.

        Args:
            screen (pygame.Surface): The screen to draw on.
        """
        rad = math.radians(self.angle)
        p1 = (
            self.x + ROBOT_SIZE * math.cos(rad),
            self.y - ROBOT_SIZE * math.sin(rad)
        )
        p2 = (
            self.x + ROBOT_SIZE * math.cos(rad + math.pi * 4/5),
            self.y - ROBOT_SIZE * math.sin(rad + math.pi * 4/5)
        )
        p3 = (
            self.x + ROBOT_SIZE * math.cos(rad - math.pi * 4/5),
            self.y - ROBOT_SIZE * math.sin(rad - math.pi * 4/5)
        )
        pygame.draw.polygon(screen, RED, [p1, p2, p3])


# --- ROS2 Node ---
class SimulatorNode(Node):
    """
    A ROS2 node that simulates a robot.
    """

    def __init__(self, robot):
        """
        Initializes the SimulatorNode.

        Args:
            robot (Robot): The robot to simulate.
        """
        super().__init__('simulator_node')
        self.robot = robot
        self.subscription = self.create_subscription(
            String,
            'robot_command',
            self.command_callback,
            10)
        self.get_logger().info('Simulator Node is running. Waiting for commands...')

    def command_callback(self, msg):
        """
        Callback function for the robot command subscriber.

        Args:
            msg (std_msgs.msg.String): The received message.
        """
        self.get_logger().info(f'Received command: "{msg.data}"')
        try:
            command_data = json.loads(msg.data)
            action = command_data.get("action")
            parameters = command_data.get("parameters", {})
            if action == "navigate":
                self.robot.set_navigation_goal(parameters)
            else:
                self.get_logger().info(f"Received unhandled action: {action}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse JSON command: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")


# --- Main ---
def main(args=None):
    """
    Main function for the simulator.
    """
    rclpy.init(args=args)
    
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("ROS2 Robot Simulator")
    clock = pygame.time.Clock()

    robot = Robot(WIDTH // 2, HEIGHT // 2)
    simulator_node = SimulatorNode(robot)
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # Process ROS2 messages
        rclpy.spin_once(simulator_node, timeout_sec=0.01)

        # Update and draw simulation
        robot.update()
        screen.fill(WHITE)
        robot.draw(screen)
        pygame.display.flip()

        clock.tick(60)

    pygame.quit()
    simulator_node.destroy_node()
    rclpy.shutdown()
    print("SIM: Simulator stopped.")


if __name__ == "__main__":
    main()
