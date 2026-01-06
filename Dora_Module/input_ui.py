# -*- coding: utf-8 -*-
# Dora_Module/input_ui.py
import sys
import tkinter as tk
from tkinter import font as tkFont
import pyarrow as pa
from dora import Node

# Reconfigure stdout to use UTF-8 encoding on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

class DoraInputUI:
    """
    A modern, dark-themed Tkinter UI for inputting robot commands.
    """

    def __init__(self, node):
        """
        Initializes the DoraInputUI.

        Args:
            node (dora.Node): The Dora node to use for sending commands.
        """
        self.node = node
        
        # --- UI Style Configuration ---
        self.BG_COLOR = "#2E2E2E"
        self.FG_COLOR = "#F0F0F0"
        self.BUTTON_COLOR = "#4A4A4A"
        self.BUTTON_ACTIVE_COLOR = "#5A5A5A"
        self.ENTRY_COLOR = "#3C3C3C"
        self.SUCCESS_COLOR = "#4CAF50"

        # --- Window Setup ---
        self.root = tk.Tk()
        self.root.title("Robot Command Input")
        self.root.geometry("450x150")
        self.root.configure(bg=self.BG_COLOR)
        
        # --- Font Configuration ---
        self.default_font = tkFont.Font(family="Segoe UI", size=10)
        self.label_font = tkFont.Font(family="Segoe UI", size=11, weight="bold")
        
        # --- UI Elements ---
        main_frame = tk.Frame(self.root, bg=self.BG_COLOR, padx=15, pady=15)
        main_frame.pack(expand=True, fill=tk.BOTH)

        self.label = tk.Label(
            main_frame, 
            text="Enter command for the robot:", 
            font=self.label_font,
            bg=self.BG_COLOR, 
            fg=self.FG_COLOR
        )
        self.label.pack(pady=(0, 10))

        self.entry = tk.Entry(
            main_frame, 
            width=50, 
            font=self.default_font,
            bg=self.ENTRY_COLOR,
            fg=self.FG_COLOR,
            insertbackground=self.FG_COLOR, # Cursor color
            borderwidth=0,
            highlightthickness=1,
            highlightbackground=self.BUTTON_COLOR,
            highlightcolor=self.FG_COLOR
        )
        self.entry.pack(pady=5, ipady=5) # ipady for internal padding
        self.entry.bind("<Return>", self.send_command_event)

        self.send_button = tk.Button(
            main_frame, 
            text="Send Command", 
            command=self.send_command,
            font=self.default_font,
            bg=self.BUTTON_COLOR,
            fg=self.FG_COLOR,
            activebackground=self.BUTTON_ACTIVE_COLOR,
            activeforeground=self.FG_COLOR,
            borderwidth=0,
            highlightthickness=0,
            pady=5,
            padx=10
        )
        self.send_button.pack(pady=10)
        
        self.status_label = tk.Label(
            self.root, 
            text="Status: Ready", 
            font=self.default_font,
            bg=self.BG_COLOR,
            fg=self.FG_COLOR
        )
        self.status_label.pack(pady=(0, 5), fill=tk.X)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.entry.focus_set()
        
    def send_command(self):
        """
        Sends the command from the entry box and provides visual feedback.
        """
        command = self.entry.get()
        if command:
            print(f"UI sending command: {command}", flush=True)
            self.node.send_output("user_command", pa.array([command]))
            self.entry.delete(0, tk.END)
            self.status_label.config(text=f"Status: Sent '{command}'", fg=self.SUCCESS_COLOR)
            # Reset status message after a few seconds
            self.root.after(3000, lambda: self.status_label.config(text="Status: Ready", fg=self.FG_COLOR))

    def send_command_event(self, event):
        """
        Handles the event of pressing Enter in the entry box.
        """
        self.send_command()

    def on_closing(self):
        """
        Handles the event of closing the UI window.
        """
        print("UI window closed. Stopping node.", flush=True)
        self.root.destroy()

    def run(self):
        """
        Runs the Tkinter main loop.
        """
        self.root.mainloop()

def main():
    """
    Main function for the UI input node.
    """

    node = Node("ui-input-node")
    print("UI Input Node is running.", flush=True)
    
    app = DoraInputUI(node)
    app.run()
    
    print("UI Input Node finished.", flush=True)

if __name__ == "__main__":
    main()
