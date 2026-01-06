# -*- coding: utf-8 -*-
# ChaseBot_Project/simulator.py
import sys
import pygame
import math
import random
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

PLAYER_COLOR = (60, 120, 180) # Blue for the PlayerBot
PLAYER_CABIN_COLOR = (255, 200, 0) # Gold/yellow for PlayerBot cabin
TARGET_COLOR = (50, 180, 50) # Green for the TargetBot
ROBOT_SIZE = 25
ROBOT_SPEED = 3  # Pixels per frame
ROBOT_TURN_SPEED = 3  # Degrees per frame

# --- Robot Base Class ---
class BaseRobot:
    """
    Base class for robots in the simulator.
    """
    def __init__(self, x, y, color, cabin_color=None):
        self.x = x
        self.y = y
        self.angle = random.randint(0, 359) # Initial random orientation
        self.color = color
        self.cabin_color = cabin_color if cabin_color else color # Cabin for directional robots

    def draw(self, screen):
        """
        Draws a directional robot shape (arrowhead/spaceship).
        """
        rad = math.radians(self.angle)
        
        # Define points for a more complex shape (like an arrowhead or spaceship)
        p1 = (
            self.x + ROBOT_SIZE * math.cos(rad),
            self.y - ROBOT_SIZE * math.sin(rad)
        )
        p2 = (
            self.x + ROBOT_SIZE * 0.8 * math.cos(rad - math.pi * 3/4),
            self.y - ROBOT_SIZE * 0.8 * math.sin(rad - math.pi * 3/4)
        )
        p3 = (
            self.x + ROBOT_SIZE * 0.4 * math.cos(rad + math.pi),
            self.y - ROBOT_SIZE * 0.4 * math.sin(rad + math.pi)
        )
        p4 = (
            self.x + ROBOT_SIZE * 0.8 * math.cos(rad + math.pi * 3/4),
            self.y - ROBOT_SIZE * 0.8 * math.sin(rad + math.pi * 3/4)
        )
        
        pygame.draw.polygon(screen, self.color, [p1, p2, p3, p4])
        
        # Draw a "cabin" to make direction even clearer, only if cabin_color is different
        if self.cabin_color != self.color:
            cabin_center = (
                self.x + ROBOT_SIZE * 0.1 * math.cos(rad),
                self.y - ROBOT_SIZE * 0.1 * math.sin(rad)
            )
            cabin_radius = ROBOT_SIZE * 0.25
            pygame.draw.circle(screen, self.cabin_color, cabin_center, cabin_radius)

# --- PlayerBot Class ---
class PlayerBot(BaseRobot):
    """
    The robot controlled by the LLM agent.
    """
    def __init__(self, x, y):
        super().__init__(x, y, PLAYER_COLOR, PLAYER_CABIN_COLOR)
        self.target_x = x
        self.target_y = y
        self.target_angle = self.angle

    def set_command_goal(self, params):
        """
        Processes commands to set a new target based on robot's frame of reference.
        - "move" commands change position relative to current angle.
        - "turn" commands change angle, not position.
        - "succeeded" means the bot has reached its goal.

        Args:
            params (dict): The command parameters.
        """
        print(f"SIM: Received command with params: {params}")
        
        # --- Handle Turning ---
        angle_str = params.get('angle', None)
        if angle_str:
            try:
                angle_value = float(angle_str.replace('deg', ''))
                # Turning is relative to the current angle. 
                # We add a minus sign to invert the angle, because LLM's "right is positive"
                # is the opposite of Pygame's "counter-clockwise (left) is positive".
                self.target_angle = (self.angle - angle_value) % 360
                print(f"SIM: New Target Angle: {self.target_angle:.2f}")
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
        move_angle_rad = math.radians(self.angle) # Default to 'front'

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

# --- TargetBot Class ---
class TargetBot(BaseRobot):
    """
    A randomly placed, stationary target robot.
    """
    def __init__(self):
        # Random initial position within bounds, ensuring it's not too close to edges
        x = random.randint(ROBOT_SIZE * 2, WIDTH - ROBOT_SIZE * 2)
        y = random.randint(ROBOT_SIZE * 2, HEIGHT - ROBOT_SIZE * 2)
        super().__init__(x, y, TARGET_COLOR)

    def draw(self, screen):
        """
        Draws the TargetBot as a simple circle for distinction.
        """
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), ROBOT_SIZE)

# --- Helper Functions ---
def draw_grid(screen):
    """Draws a grid on the screen."""
    for x in range(0, WIDTH, GRID_SPACING):
        pygame.draw.line(screen, GRID_COLOR, (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, GRID_SPACING):
        pygame.draw.line(screen, GRID_COLOR, (0, y), (WIDTH, y))

# --- Main Simulation Loop ---
if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("ChaseBot Simulator")
    clock = pygame.time.Clock()

    player_bot = PlayerBot(WIDTH // 2, HEIGHT // 2)
    target_bot = TargetBot()
    
    node = Node("simulator")
    running = True

    print("SIM: Simulator running. Waiting for Dora commands from LLM agent...")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        # Check for commands from Dora (LLM agent)
        try:
            dora_event = node.next(timeout=0.01) # Non-blocking read
            if dora_event and dora_event["type"] == "INPUT":
                if dora_event["id"] == "command":
                    command_data = dora_event["value"][0].as_py()
                    action = command_data.get("action")
                    parameters = command_data.get("parameters", {})
                    
                    if action == "navigate":
                        player_bot.set_command_goal(parameters)
                    elif action == "succeeded":
                        print("SIM: PlayerBot reached target. Stopping simulator.")
                        running = False
                    else:
                        print(f"SIM: Received unhandled action: {action}")

        except Exception as e:
            # This can happen on timeout, which is expected
            pass

        # Update bots
        player_bot.update()

        # Check for success condition
        dist_x = player_bot.x - target_bot.x
        dist_y = player_bot.y - target_bot.y
        if math.sqrt(dist_x**2 + dist_y**2) < ROBOT_SIZE:
            print("SIM: SUCCESS! PlayerBot has reached the TargetBot.")
            running = False

        # Draw everything
        screen.fill(WHITE)
        draw_grid(screen)
        player_bot.draw(screen)
        target_bot.draw(screen)
        pygame.display.flip()

        # Send game state to LLM agent
        game_state = {
            "player_bot": {"x": player_bot.x, "y": player_bot.y, "angle": player_bot.angle},
            "target_bot": {"x": target_bot.x, "y": target_bot.y}
        }
        node.send_output("game_state", pa.array([game_state]))

        clock.tick(60) # Limit to 60 FPS

    pygame.quit()
    print("SIM: Simulator stopped.")
