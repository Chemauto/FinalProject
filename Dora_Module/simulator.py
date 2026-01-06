import pygame
import math
import pyarrow as pa
from dora import Node

# --- Constants ---
WIDTH, HEIGHT = 800, 600
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
ROBOT_SIZE = 20
ROBOT_SPEED = 3  # Pixels per frame
ROBOT_TURN_SPEED = 3  # Degrees per frame

# --- Robot Class ---
class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angle = 0  # In degrees
        self.target_x = x
        self.target_y = y
        self.target_angle = 0

    def set_navigation_goal(self, params):
        """Processes navigation commands to set a new target."""
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
                new_target_angle = (self.angle + angle_value) % 360
            except ValueError:
                print(f"SIM: Invalid angle format: {angle_str}")
        elif direction:
            if direction == 'front':
                new_target_angle = self.angle
            elif direction == 'back':
                new_target_angle = (self.angle - 180) % 360
            elif direction == 'left':
                new_target_angle = (self.angle - 90) % 360
            elif direction == 'right':
                new_target_angle = (self.angle + 90) % 360
        
        if new_target_angle is not None:
            self.target_angle = new_target_angle

        # Calculate target position using the determined target_angle
        rad_angle = math.radians(self.target_angle)
        self.target_x = self.x + distance * math.cos(rad_angle)
        self.target_y = self.y - distance * math.sin(rad_angle) # Pygame Y is inverted
        print(f"SIM: New Target: ({self.target_x:.2f}, {self.target_y:.2f}) at angle {self.target_angle:.2f}")


    def update(self):
        """Moves the robot towards its target position and angle."""
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
        """Draws the robot as a triangle."""
        # Center point
        center = (self.x, self.y)
        
        # Points of the triangle relative to the center
        rad = math.radians(self.angle)
        p1 = (
            self.x + ROBOT_SIZE * math.cos(rad),
            self.y - ROBOT_SIZE * math.sin(rad) # Pygame Y is inverted
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
            if dora_event and dora_event["type"] == "INPUT":
                if dora_event["id"] == "teleop":
                    command_data = dora_event["value"][0].as_py()
                    action = command_data.get("action")
                    parameters = command_data.get("parameters", {})
                    if action == "navigate":
                        robot.set_navigation_goal(parameters)
                    else:
                        print(f"SIM: Received unhandled action: {action}")

        except Exception as e:
            # This can happen on timeout, which is expected
            pass

        # Update and draw
        robot.update()
        screen.fill(WHITE)
        robot.draw(screen)
        pygame.display.flip()

        clock.tick(60)

    pygame.quit()
    print("SIM: Simulator stopped.")
