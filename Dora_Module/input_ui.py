# Dora_Module/input_ui.py
import tkinter as tk
import pyarrow as pa
from dora import Node

class DoraInputUI:
    def __init__(self, node):
        self.node = node
        
        self.root = tk.Tk()
        self.root.title("Robot Command Input")
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
        command = self.entry.get()
        if command:
            print(f"UI sending command: {command}", flush=True)
            self.node.send_output("user_command", pa.array([command]))
            self.entry.delete(0, tk.END)
            self.status_label.config(text=f"Status: Sent '{command}'")

    def send_command_event(self, event):
        self.send_command()

    def on_closing(self):
        print("UI window closed. Stopping node.", flush=True)
        self.root.destroy()

    def run(self):
        self.root.mainloop()

def main():
    node = Node("ui-input-node")
    print("UI Input Node is running.", flush=True)
    
    app = DoraInputUI(node)
    app.run()
    
    print("UI Input Node finished.", flush=True)

if __name__ == "__main__":
    main()
