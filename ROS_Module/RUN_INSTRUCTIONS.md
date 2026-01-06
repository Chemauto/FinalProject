# How to Run the ROS2 Module

## 1. Prerequisites

-   **ROS2 Installation**: You need to have a ROS2 distribution installed on your system (e.g., Foxy, Galactic, Humble). Make sure to source the ROS2 setup file.
-   **Conda Environment**: The Conda environment `qwen3` should be created and activated as described in the main `README.md`.
-   **Python Dependencies**: In addition to the dependencies for the `VLM_Modele`, you will need to install the following Python packages in your `qwen3` conda environment:
    ```bash
    pip install colcon-common-extensions pygame
    ```

## 2. Building the ROS2 Package

1.  **Open a terminal** and navigate to the `ROS_Module` directory:
    ```bash
    cd path/to/your/project/ROS_Module
    ```

2.  **Source your ROS2 installation**. For example, on Windows with ROS2 Foxy:
    ```bash
    call C:\dev\ros2_foxy\local_setup.bat
    ```
    *Adjust the path to your ROS2 installation accordingly.*

3.  **Build the `ros_module` package** using `colcon`:
    ```bash
    colcon build
    ```
    This command will build the package and create `install`, `build`, and `log` directories inside the `ROS_Module` directory.

## 3. Running the ROS2 Module

1.  **Open a new terminal** and navigate to the `ROS_Module` directory.

2.  **Source the ROS2 installation and the local workspace setup**:
    ```bash
    call C:\dev\ros2_foxy\local_setup.bat
    cd path/to/your/project/ROS_Module
    call install\setup.bat
    ```

3.  **Run the launch file**:
    ```bash
    ros2 launch ros_module ros_module.launch.py
    ```

This will start all three nodes: the UI input, the LLM receiver, and the simulator. Two windows should pop up: one for the robot command input and one for the Pygame simulation. You can then enter commands in the input window to control the robot in the simulator.

## 4. Troubleshooting

-   **`colcon` not found**: Make sure you have installed `colcon-common-extensions` and that your Python scripts directory is in your PATH.
-   **ROS2 commands not found**: Make sure you have sourced your ROS2 installation correctly.
-   **`ros2 launch` command fails**: Make sure you have sourced the local workspace setup file (`install/setup.bat` or `install/setup.bash`).
-   **Python dependencies not found**: Make sure you have installed all the required Python packages in your `qwen3` conda environment.
-   **UI not showing up**: Make sure you have a graphical environment and that you can run other GUI applications.
-   **Simulator not showing up**: Make sure you have installed `pygame`.
-   **LLM not working**: Make sure you have set up your API key correctly in the `.env` file in the `VLM_Modele` directory.
