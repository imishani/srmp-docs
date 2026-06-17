Agent Mode
==========

SRMP provides an **Agent Mode** - an LLM-powered motion planning assistant that allows you to interact with SRMP using natural language instead of writing Python code directly.

.. note::

   Agent Mode requires an LLM API key. By default, it uses Google's Gemini (free tier available).

Installation
------------

Agent Mode requires additional dependencies:

.. code-block:: console

   (.venv) $ pip install openai

Set up your API key:

.. code-block:: console

   # For Gemini (default, free)
   $ export GEMINI_API_KEY=your_api_key_here

   # For Claude (Anthropic)
   $ export ANTHROPIC_API_KEY=your_api_key_here

   # For OpenAI
   $ export OPENAI_API_KEY=your_api_key_here

Quick Start
-----------

**GUI Mode** - Web-based interface with 3D visualization:

.. code-block:: console

   (.venv) $ python -m agent.gui

This opens a Viser 3D viewer with an embedded chat panel. You can interact with the planner using natural language while watching the robot move in real-time.

**CLI Mode** - Terminal-based interface:

.. code-block:: console

   (.venv) $ python -m agent.cli

Example Interaction
-------------------

Here's an example conversation with the agent. Robots, obstacles, and planner settings persist between messages, so you can build up a scene incrementally:

.. code-block:: text

   You: Load a Panda robot
   Agent: Done! Loaded Panda robot.

   You: Add a box obstacle at [0.5, 0, 0.3]
   Agent: Added box obstacle.

   You: Plan a motion to reach [0.4, 0.2, 0.5]
   Agent: Planned trajectory with 52 waypoints.

   You: Now plan a motion to reach underneath the box
   Agent: Planned trajectory with 48 waypoints to position [0.5, 0, 0.15].

The agent understands the full SRMP API and can:

- Load robot models (URDF/SRDF)
- Add obstacles (boxes, spheres, meshes, point clouds)
- Configure planners (wA*, ARA*, xECBS, etc.)
- Plan single and multi-robot motions
- Compute forward/inverse kinematics
- Attach/detach objects to end-effectors

Command Line Options
--------------------

Both CLI and GUI modes support the following options:

.. code-block:: console

   python -m agent.cli [OPTIONS]
   python -m agent.gui [OPTIONS]

Available options:

- ``--backend {gemini|ollama|groq|anthropic|openai}`` - Select the LLM backend (default: gemini)
- ``--model MODEL_NAME`` - Specify a particular model (e.g., ``claude-sonnet-4-20250514``)
- ``--context "TEXT"`` - Pre-populate context with URDF paths or robot information
- ``--timeout SECONDS`` - Timeout for code execution (default: 30)
- ``--max-iters N`` - Maximum agent iterations per request (default: 20)
- ``--no-stream`` - Disable streaming output

Supported LLM Backends
----------------------

Agent Mode supports multiple LLM providers:

+-------------+------------------+-------------------------------------------+
| Backend     | Environment Var  | Notes                                     |
+=============+==================+===========================================+
| Gemini      | GEMINI_API_KEY   | Free tier available (default)             |
+-------------+------------------+-------------------------------------------+
| Anthropic   | ANTHROPIC_API_KEY| Claude models                             |
+-------------+------------------+-------------------------------------------+
| OpenAI      | OPENAI_API_KEY   | GPT-4o and other models                   |
+-------------+------------------+-------------------------------------------+
| Groq        | GROQ_API_KEY     | Fast cloud inference (free tier)          |
+-------------+------------------+-------------------------------------------+
| Ollama      | (local)          | Local models, no API key needed           |
+-------------+------------------+-------------------------------------------+

Using Ollama for Local Inference
--------------------------------

For privacy or offline usage, you can use Ollama with local models:

.. code-block:: console

   # Install and start Ollama
   $ ollama serve

   # Pull a model
   $ ollama pull llama3

   # Run agent with Ollama
   $ python -m agent.cli --backend ollama --model llama3

CLI Special Commands
--------------------

In CLI mode, you can use these special commands:

- ``/help`` - Show help information
- ``/reset`` - Reset the conversation and planner state
- ``/quit`` - Exit the agent

For multi-line input, end lines with ``\`` to continue:

.. code-block:: text

   You: Load these robots: \
        - panda0 at position [0, 0, 0] \
        - panda1 at position [1, 0, 0]

GUI Features
------------

The GUI mode provides additional features:

- **3D Visualization**: See robots and obstacles in a web-based Viser viewer
- **Live Trajectory Animation**: Watch planned trajectories execute in real-time
- **Backend Presets**: Quickly switch between LLM providers via dropdown
- **Persistent State**: The planner state persists across conversation turns

How It Works
------------

Agent Mode uses a tool-calling loop:

1. You send a natural language request
2. The agent queries the LLM backend
3. The LLM decides to either:

   - Execute Python code using the SRMP API
   - Provide a final answer

4. Code execution results are fed back to the LLM for reasoning
5. The loop continues until the task is complete

The agent maintains persistent state, so variables and the planner object persist across conversation turns. This allows for iterative development and debugging.
