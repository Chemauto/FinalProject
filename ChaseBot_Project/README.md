# ChaseBot Project README

This project demonstrates an AI agent powered by a Large Language Model (LLM) that controls a robot to chase a target in a 2D simulated environment.

## 1. Overview

The project uses the `dora-rs` framework to orchestrate a closed-loop system where the LLM acts as the robot's brain. The system follows an "Observe-Think-Act" cycle:

-   **Observe**: A simulator sends out the current `game_state` (positions of the player and target robots).
-   **Think**: An LLM agent receives this state, analyzes the situation, and decides on the next move.
-   **Act**: The agent sends a command back to the simulator, which executes the move.

## 2. Core Components

-   `simulator.py`: A Pygame-based simulator that manages two robots (PlayerBot and TargetBot), handles physics, renders the environment, and streams the `game_state`.
-   `llm_agent.py`: The core AI agent. It receives the `game_state`, queries the LLM with a carefully designed prompt, and sends the resulting command to the simulator.
-   `chase_bot_prompt.yaml`: The prompt template that guides the LLM's reasoning process.
-   `chase_bot.yaml`: The Dora dataflow configuration file that defines and connects the `simulator` and `llm_agent` nodes.
-   `.env`: (User-created) File to store the `Test_API_KEY`.

## 3. Environment Setup

1.  **Activate Conda Environment**:
    ```bash
    # Use the same environment as the previous project
    conda activate qwen3
    ```
2.  **Install Dependencies**:
    The required dependencies are the same as the previous project (`dora-rs`, `pygame`, `openai`, `python-dotenv`, `pyyaml`, `pyarrow`). If you are using the `qwen3` environment, no new installations are needed.

3.  **Configure API Key**:
    -   Go to the `VLM_Modele` directory from the parent project.
    -   Ensure the `.env` file there contains your valid API key, as `llm_agent.py` will reference it.
    ```
    Test_API_KEY="sk-your_api_key_here"
    ```

## 4. How to Run

1.  **Navigate to the Project Directory**:
    ```powershell
    cd D:\MyFiles\Documents\毕设相关材料\Project\ChaseBot_Project
    ```

2.  **Start Dora Services**:
    ```powershell
    # Make sure the dora daemon is running
    dora up
    ```

3.  **Run the ChaseBot Dataflow**:
    ```powershell
    dora start chase_bot.yaml --attach
    ```
    This command will start both the simulator and the LLM agent. A Pygame window will pop up showing the blue `PlayerBot` automatically moving towards the green `TargetBot`. The terminal will display the LLM's thoughts and actions in real-time.
