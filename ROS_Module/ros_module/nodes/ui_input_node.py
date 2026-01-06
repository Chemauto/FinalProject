# -*- coding: utf-8 -*-
# ROS_Module/ros_module/nodes/ui_input_node.py
import sys
import tkinter as tk
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

class TkinterUI:
    """
    A Tkinter UI for inputting robot commands.
    """

    def __init__(self, node):
        """
        Initializes the TkinterUI.

        Args:
            node (UIInputNode): The ROS2 node to use for publishing commands.
        """
        self.node = node
        self.root = tk.Tk()
        self.root.title("Robot Command Input (ROS2)")
        self.root.geometry("400x120")

        self.label = tk.Label(self.root, text="Enter command for the robot:")
        self.label.pack(pady=5)

        self.entry = tk.Entry(self.root, width=50)
        self.entry.pack(pady=5)
        self.entry.bind("<Return>", self.send_command_event)

        self.send_button = tk.Button(self.root, text="Send Command", command=self.send_command)
        self.send_button.pack(pady=5)
        
        self.status_label = tk.Label(self.root, text="Status: Ready")
        self.status_label.pack(pady=5)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def send_command(self):
        """
        Sends the command from the entry box.
        """
        command = self.entry.get()
        if command:
            self.node.publish_command(command)
            self.entry.delete(0, tk.END)
            self.status_label.config(text=f"Status: Sent '{command}'")

    def send_command_event(self, event):
        """
        Handles the event of pressing Enter in the entry box.

        Args:
            event (tk.Event): The event.
        """
        self.send_command()

    def on_closing(self):
        """
        Handles the event of closing the UI window.
        """
        self.node.get_logger().info("UI window closed. Shutting down ROS2 node.")
        self.root.destroy()
        # This will cause rclpy.spin() to exit
        rclpy.shutdown()

    def run(self):
        """
        Runs the Tkinter main loop.
        """
        self.root.mainloop()

class UIInputNode(Node):
    """
    A ROS2 node for inputting user commands.
    """

    def __init__(self):
        """
        Initializes the UIInputNode.
        """
        super().__init__('ui_input_node')
        self.publisher_ = self.create_publisher(String, 'user_command', 10)
        self.get_logger().info("UI Input Node for ROS2 is running.")

    def publish_command(self, command):
        """
        Publishes a command.

        Args:
            command (str): The command to publish.
        """
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    """
    Main function for the UI input node.
    """
    rclpy.init(args=args)
    ui_input_node = UIInputNode()
    
    # Create and run the Tkinter UI in a separate thread
    app = TkinterUI(ui_input_node)
    ui_thread = threading.Thread(target=app.run)
    ui_thread.start()
    
    # Spin the ROS2 node in the main thread
    try:
        rclpy.spin(ui_input_node)
    except KeyboardInterrupt:
        pass
    finally:
        ui_input_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        ui_thread.join()

if __name__ == '__main__':
    main()
