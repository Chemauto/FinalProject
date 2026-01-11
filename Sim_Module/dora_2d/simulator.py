# -*- coding: utf-8 -*-
# Dora_Module/simulator.py
import sys
import pygame
import math
import pyarrow as pa
from dora import Node

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

# --- Constants ---
WIDTH, HEIGHT = 800, 600
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRID_COLOR = (220, 220, 220) # Light grey for the grid
GRID_SPACING = 50
ROBOT_BODY_COLOR = (60, 120, 180) # A nice blue
ROBOT_CABIN_COLOR = (255, 200, 0) # A gold/yellow
ROBOT_SIZE = 25
ROBOT_SPEED = 3  # Pixels per frame
ROBOT_TURN_SPEED = 3  # Degrees per frame

# --- Robot Class ---
class Robot:
    """
    Represents the robot in the simulator.
    Handles movement, rotation, and drawing.
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
        self.angle = 0  # In degrees, 0 is East, 90 is North
        self.target_x = x
        self.target_y = y
        self.target_angle = 0

    def set_navigation_goal(self, params):
        """
        Processes navigation commands to set a new target based on robot's frame of reference.
        - "move" commands change position relative to current angle.
        - "turn" commands change angle, not position.

        Args:
            params (dict): The navigation parameters.
        """
        print(f"SIM: Received navigation command with params: {params}")
        
        # --- Handle Turning ---
        angle_str = params.get('angle', None)
        if angle_str:
            try:
                angle_value = float(angle_str.replace('deg', ''))
                # 正角度 = 左转（逆时针），负角度 = 右转（顺时针）
                # Pygame: Y轴向下，所以加角度是逆时针（左转）
                self.target_angle = (self.angle + angle_value) % 360
                print(f"SIM: Turning {angle_value}°, New Target Angle: {self.target_angle:.2f}")
                return # If it's a turn command, we don't move
            except ValueError:
                print(f"SIM: Invalid angle format: {angle_str}")
                return

        # --- Handle Movement ---
        direction = params.get('direction', None)
        distance_str = params.get('distance', "0cm")

        try:
            if 'cm' in distance_str:
                distance = float(distance_str.replace('cm', ''))
            elif 'm' in distance_str:
                distance = float(distance_str.replace('m', '')) * 100
            else:
                distance = 0
        except ValueError:
            distance = 0
            
        if distance == 0:
            return

        # Calculate movement vector based on direction relative to robot's current angle
        move_angle_rad = math.radians(self.angle) # Default to 'forward'

        if direction == 'back':
            move_angle_rad += math.pi # 180 degrees
        elif direction == 'left':
            move_angle_rad -= math.pi / 2 # -90 degrees
        elif direction == 'right':
            move_angle_rad += math.pi / 2 # +90 degrees
        
        # Update target position
        self.target_x = self.x + distance * math.cos(move_angle_rad)
        self.target_y = self.y - distance * math.sin(move_angle_rad) # Pygame Y is inverted
        print(f"SIM: New Target Position: ({self.target_x:.2f}, {self.target_y:.2f})")

    def update(self):
        """
        Moves the robot smoothly towards its target position and angle.
        """
        # --- Update Angle ---
        angle_diff = (self.target_angle - self.angle + 180) % 360 - 180
        if abs(angle_diff) > ROBOT_TURN_SPEED:
            self.angle += ROBOT_TURN_SPEED if angle_diff > 0 else -ROBOT_TURN_SPEED
            self.angle %= 360
        else:
            self.angle = self.target_angle

        # --- Update Position ---
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
        Draws a more appealing robot shape (a directional arrow/ship).

        Args:
            screen (pygame.Surface): The screen to draw on.
        """
        rad = math.radians(self.angle)
        
        # Define points for a more complex shape (like an arrowhead or spaceship)
        # Point 1: Nose
        p1 = (
            self.x + ROBOT_SIZE * math.cos(rad),
            self.y - ROBOT_SIZE * math.sin(rad)
        )
        # Point 2: Right wing
        p2 = (
            self.x + ROBOT_SIZE * 0.8 * math.cos(rad - math.pi * 3/4),
            self.y - ROBOT_SIZE * 0.8 * math.sin(rad - math.pi * 3/4)
        )
        # Point 3: Tail center
        p3 = (
            self.x + ROBOT_SIZE * 0.4 * math.cos(rad + math.pi),
            self.y - ROBOT_SIZE * 0.4 * math.sin(rad + math.pi)
        )
        # Point 4: Left wing
        p4 = (
            self.x + ROBOT_SIZE * 0.8 * math.cos(rad + math.pi * 3/4),
            self.y - ROBOT_SIZE * 0.8 * math.sin(rad + math.pi * 3/4)
        )
        
        # Draw the main body
        pygame.draw.polygon(screen, ROBOT_BODY_COLOR, [p1, p2, p3, p4])
        
        # Draw a "cabin" to make direction even clearer
        cabin_center = (
            self.x + ROBOT_SIZE * 0.1 * math.cos(rad),
            self.y - ROBOT_SIZE * 0.1 * math.sin(rad)
        )
        cabin_radius = ROBOT_SIZE * 0.25
        pygame.draw.circle(screen, ROBOT_CABIN_COLOR, cabin_center, cabin_radius)

# --- Helper Functions ---
def draw_grid(screen):
    """Draws a grid on the screen."""
    for x in range(0, WIDTH, GRID_SPACING):
        pygame.draw.line(screen, GRID_COLOR, (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, GRID_SPACING):
        pygame.draw.line(screen, GRID_COLOR, (0, y), (WIDTH, y))

# --- Main ---
if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Dora Robot Simulator")
    clock = pygame.time.Clock()

    robot = Robot(WIDTH // 2, HEIGHT // 2)
    node = Node()
    running = True

    print("SIM: Simulator running. Waiting for Dora commands...")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # Check for Dora input
        try:
            dora_event = node.next(timeout=0.01) # Non-blocking read
            if dora_event:
                if dora_event["type"] == "INPUT":
                    if dora_event["id"] == "teleop":
                        command_data = dora_event["value"][0].as_py()
                        action = command_data.get("action")
                        parameters = command_data.get("parameters", {})

                        # 处理所有导航相关的 action
                        if action in ["navigate", "turn", "turn_left", "turn_right"]:
                            robot.set_navigation_goal(parameters)
                        elif action == "stop":
                            # 停止机器人
                            robot.target_x = robot.x
                            robot.target_y = robot.y
                            print(f"SIM: Robot stopped")
                        else:
                            print(f"SIM: Received unhandled action: {action}")
                elif dora_event["type"] == "STOP":
                    print("SIM: Received STOP event, exiting...")
                    running = False

        except Exception as e:
            # This can happen on timeout, which is expected
            pass

        # Update and draw
        robot.update()
        screen.fill(WHITE)
        draw_grid(screen) # Draw the grid background
        robot.draw(screen)
        pygame.display.flip()

        clock.tick(60)

    pygame.quit()
    print("SIM: Simulator stopped.")
