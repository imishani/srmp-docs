Coding Assistants (Claude Code, Codex, Gemini, Cursor)
======================================================

SRMP ships two pieces that let an AI coding assistant write SRMP code and plan motions
for you:

- a **skill**: instructions and verified code samples the assistant reads when you ask
  for something SRMP-related. Every code sample in it is executed in SRMP's test suite.
- an **MCP server** (``srmp mcp``): a persistent SRMP session the assistant drives with
  tools. It can load robots, build a scene, plan, check the result and open a 3D viewer.

You can use either piece on its own. With only the skill, the assistant writes scripts and
runs them itself. The MCP server adds a live scene that the assistant builds step by step.

Supported assistants are Claude Code, OpenAI Codex, Gemini CLI, Google Antigravity
and Cursor.

.. note::

   This is different from :doc:`agent_mode`, which is SRMP's own chat interface with its
   own LLM backends. The skill and MCP server plug SRMP into the assistant you already use.

Install
-------

.. code-block:: console

   $ pip install "srmp[mcp]"

The MCP server needs Python 3.10 or newer and the ``mcp`` extra. The skill alone works on
every Python version SRMP supports.

Then set up each assistant. ``--for`` accepts ``claude``, ``codex``, ``gemini``,
``antigravity``, ``cursor`` or ``all``.

.. code-block:: bash

   # Skill: copied into the assistant's skills directory.
   # Project scope (current directory) is the default; --user installs under your home.
   srmp skill install --for claude
   srmp skill install --for codex --user

   # MCP server: prints the exact command or config snippet for that assistant.
   # It never edits your config files; you run or paste what it prints.
   srmp mcp install --for claude
   srmp mcp install --for cursor

Re-run ``srmp skill install`` after upgrading SRMP so the skill matches your installed
version. ``srmp skill path`` prints where the bundled skill lives.

Claude Code plugin
~~~~~~~~~~~~~~~~~~

Claude Code users can install the skill and the MCP server together as a plugin. You still
need ``pip install "srmp[mcp]"`` first, because the plugin's MCP server runs the ``srmp``
command.

.. code-block:: text

   /plugin marketplace add imishani/srmp-assistant
   /plugin install srmp@srmp

Try it
------

Ask your assistant something like:

   *Load a Panda, put a 10 cm box at (0.5, 0.2, 0.4), and plan to the pose
   (0.64, -0.07, 0.5) with the gripper pointing down. Show me the result in the viewer.*

With the MCP server connected, the assistant builds the scene in a live session, plans,
checks that the trajectory is not empty, and gives you a viewer URL. Without it, the
assistant writes an equivalent script using the conventions in the skill.

MCP tools
---------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Tool
     - What it does
   * - ``run_srmp_code(code, timeout_s=60)``
     - Runs Python in the persistent SRMP session. ``srmp``, ``np`` and a ``planner``
       are predefined. Robots, objects and variables survive between calls. Output from
       SRMP's C++ core is included in the result. Long output is truncated at 20,000
       characters per stream.
   * - ``describe_scene()``
     - Returns JSON with ``srmp_version``, ``robots`` (name, dof, qpos,
       vamp_supported), ``objects`` (with ``attached``) and ``validity_backends``.
   * - ``start_visualizer(port=None)``
     - Starts the 3D viewer and returns its URL. A second call returns the same URL.
   * - ``reset_session()``
     - Restarts the session: a fresh ``planner``, an empty scene, no variables.

The skill's reference pages are also served as MCP resources (``srmp://docs/<file>``)
for assistants that do not load skills.

The viewer is local-only
~~~~~~~~~~~~~~~~~~~~~~~~

The viewer binds to ``127.0.0.1``, so only your own machine can reach it. Open the URL
the tool returns in a local browser. If the assistant runs on a remote host, forward the
port yourself, for example ``ssh -L 8080:127.0.0.1:8080 remote-host``.

Code runs on your machine
~~~~~~~~~~~~~~~~~~~~~~~~~

``run_srmp_code`` executes Python with your user's permissions, the same as the
assistant's own shell tool. Your assistant's tool-approval prompt is the safeguard. Review
what it wants to run, as you would for any other command.

Troubleshooting
---------------

**The MCP server will not start, or says it requires Python 3.10.**
   The ``mcp`` package needs Python 3.10 or newer. Check ``python --version`` for the
   interpreter that runs ``srmp``.

**``ModuleNotFoundError: mcp``.**
   Install the extra: ``pip install "srmp[mcp]"``.

**A tool result starts with ``SESSION LOST``.**
   The SRMP session was restarted after a crash, a timeout or ``reset_session``.
   Robots, objects and variables are gone, and the assistant should re-run its setup
   code. Raise ``timeout_s`` for long plans.

**The assistant does not use the skill.**
   Restart the assistant after installing. Check that you installed into the scope you
   expect: project, the default, or ``--user``.
