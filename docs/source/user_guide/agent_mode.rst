Agent Mode
==========

SRMP provides an **Agent Mode** - an LLM-powered motion planning assistant that allows you to interact with SRMP using natural language instead of writing Python code directly.

.. note::

   Agent Mode needs an LLM: an API key for a cloud backend, a local Ollama model, or a
   logged-in ``claude`` CLI for the Claude Agent SDK backend. By default it uses Google's
   Gemini (free tier available).

Installation
------------

Install the client library for the backend you want:

.. code-block:: console

   (.venv) $ pip install "srmp[agent]"        # Gemini (the default)
   (.venv) $ pip install anthropic            # Claude via the Anthropic API
   (.venv) $ pip install openai               # OpenAI, Groq and Ollama
   (.venv) $ pip install claude-agent-sdk     # Claude Agent SDK (see below)

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

   (.venv) $ python -m srmp.agent.gui

This opens a Viser 3D viewer with an embedded chat panel. You can interact with the planner using natural language while watching the robot move in real-time.

**CLI Mode** - Terminal-based interface:

.. code-block:: console

   (.venv) $ python -m srmp.agent.cli

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
- Add obstacles (boxes, spheres, meshes, point clouds), with colors
- Configure planners (wA*, ARA*, MGS, xECBS, etc.)
- Plan single and multi-robot motions
- Compute forward/inverse kinematics
- Attach/detach objects to end-effectors

Command Line Options
--------------------

Both CLI and GUI modes support the following options:

.. code-block:: console

   (.venv) $ python -m srmp.agent.cli [OPTIONS]
   (.venv) $ python -m srmp.agent.gui [OPTIONS]

Options for both modes:

- ``--backend`` - The LLM backend (default: ``gemini``). The CLI accepts ``gemini``,
  ``ollama``, ``groq``, ``anthropic``, ``openai`` and ``claude-agent-sdk``; the GUI accepts
  the same list except ``claude-agent-sdk``, which you pick from its **Model** dropdown.
- ``--model MODEL_NAME`` - A specific model (default: the backend's own default)
- ``--api-key KEY`` / ``--base-url URL`` - Override the API key or endpoint
- ``--context "TEXT"`` - Extra text for the system prompt, such as URDF paths
- ``--timeout SECONDS`` - Advisory timeout per code execution (default: 60 in the CLI,
  120 in the GUI). The GUI runs code in-process and cannot enforce it.
- ``--max-iters N`` - Maximum tool calls per request, for every backend (default: 50).
  A request that hits the limit stops with a message saying so; send ``continue`` to pick
  up where it left off.
- ``--port PORT`` - Viser server port (default: 8080)

CLI only:

- ``--viser`` - Open a 3D viewer next to the terminal session
- ``--no-stream`` - Wait for the full response instead of streaming it

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
| Claude      | (none)           | Claude Agent SDK harness; uses your       |
| Agent SDK   |                  | logged-in ``claude`` CLI                  |
+-------------+------------------+-------------------------------------------+

Claude Agent SDK
~~~~~~~~~~~~~~~~

The Claude Agent SDK backend runs the full Claude Agent SDK harness instead of SRMP's own
tool-calling loop. It authenticates through your existing ``claude`` CLI login
(subscription or API key), so it needs no ``ANTHROPIC_API_KEY``. In the GUI, choose
**Claude Agent SDK** in the **Model** dropdown; in the CLI, pass
``--backend claude-agent-sdk``. It keeps its own session, so switching to or from it starts
a new conversation; the 3D scene is kept.

Using Ollama for Local Inference
--------------------------------

For privacy or offline usage, you can use Ollama with local models:

.. code-block:: console

   # Install and start Ollama
   $ ollama serve

   # Pull a model
   $ ollama pull llama3

   # Run agent with Ollama
   $ python -m srmp.agent.cli --backend ollama --model llama3

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

- **3D Visualization**: See robots and obstacles in a web-based Viser viewer. The agent
  works in this viewer; it does not open a second one.
- **Live Trajectory Animation**: Watch planned trajectories execute in real-time
- **Readable Replies**: Replies render as markdown, including tables, and each code step
  the agent ran can be expanded to show the code and its output
- **Backend Presets**: Quickly switch between LLM providers via dropdown
- **Persistent State**: The planner state persists across conversation turns
- **Stop Button**: Interrupt an in-flight agent run at any time
- **Save Chat / Load Chat**: Write the conversation to a file and restore it later —
  see :ref:`agent-chat-persistence` below

.. _agent-chat-persistence:

Saving and Restoring a Conversation
-----------------------------------

A conversation can be written to a file and picked up again later, from either the Python
API — see :class:`~srmp.agent.MotionPlanningAgent` in the :doc:`api` for the full agent
interface — or the GUI's **Save Chat** / **Load Chat** buttons in the **AI Agent** panel:

.. code-block:: python

   agent.save_chat("session.json")
   agent.save_chat("session.json", scene="bin_picking.json")   # record provenance
   agent.load_chat("session.json")

Loading restores **the messages only** — no saved code is re-executed, and the Python
interpreter is not reset. See :doc:`persistence` for what that means in practice and for
saving scenes and plans alongside the transcript.

How It Works
------------

Agent Mode uses a tool-calling loop:

1. You send a natural language request
2. The agent queries the LLM backend
3. The LLM decides to either:

   - Execute Python code using the SRMP API
   - Provide a final answer

4. Code execution results are fed back to the LLM for reasoning
5. The loop continues until the task is complete, or until ``--max-iters`` tool calls

The agent maintains persistent state, so variables and the planner object persist across conversation turns. This allows for iterative development and debugging.
