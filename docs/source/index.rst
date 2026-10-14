

Welcome to Search-Based Robot Motion Planning (SRMP) documentation!
===================================================================

**SRMP** is a motion planning software for robotic manipulation, leveraging state-of-the-art search-based algorithms. It ensures consistent and predictable motions, backed by rigorous theoretical guarantees. Additionally, SRMP can efficiently plan for up to dozens of manipulators while guaranteeing collision-free execution—both between robots and with the environment—while maintaining motion consistency and predictability.

Have questions or want to chat with the community? Join our `Discord channel <https://discord.gg/JwAWmqE6N>`_.

.. raw:: html

   <style>
     .srmp-videos { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 1.25rem; margin: 1.5rem 0; }
     .srmp-videos figure { margin: 0; }
     .srmp-videos video { width: 100%; height: auto; border-radius: 6px; box-shadow: 0 2px 6px rgba(0, 0, 0, 0.15); }
     .srmp-videos figcaption { font-size: 0.9em; text-align: center; margin-top: 0.4rem; opacity: 0.85; }
     @media (max-width: 720px) { .srmp-videos { grid-template-columns: 1fr; } }
   </style>
   <div class="srmp-videos">
     <figure>
       <video src="_static/assets/srmp_consistency.mp4" poster="_static/assets/srmp_consistency.jpg" autoplay muted loop playsinline preload="metadata"></video>
       <figcaption>SRMP vs. OMPL on similar queries: SRMP returns consistent motions.</figcaption>
     </figure>
     <figure>
       <video src="_static/assets/srmp_multi_arm.mp4" poster="_static/assets/srmp_multi_arm.jpg" autoplay muted loop playsinline preload="metadata"></video>
       <figcaption>Multi-arm planning in simulation and on real robots.</figcaption>
     </figure>
     <figure>
       <video src="_static/assets/srmp_agent_mode.mp4" poster="_static/assets/srmp_agent_mode.jpg" autoplay muted loop playsinline preload="metadata"></video>
       <figcaption>Asking the agent to build a small warehouse with arms and test their reachability.</figcaption>
     </figure>
     <figure>
       <video src="_static/assets/srmp_python_api.mp4" poster="_static/assets/srmp_python_api.jpg" autoplay muted loop playsinline preload="metadata"></video>
       <figcaption>Planning from Python and running the plan in a simulator and in MoveIt!.</figcaption>
     </figure>
   </div>

Key Features
============

- **Multi-Robot Motion Planning**: First-of-its-kind support for planning coordinated motions in multi-manipulator systems.
- **Reliable and Consistent Trajectories**: Generates predictable and repeatable motions, making it ideal for high-precision and safety-critical applications.
- **Agent Mode**: LLM-powered natural language interface for motion planning - describe what you want in plain English and let the agent handle the code.
- **Seamless Integration**: Compatible with major simulators, including MuJoCo, Sapien, Genesis, PyBullet and Isaac.
- **Fast Collision Checking**: Optional SIMD-vectorized checks with VAMP, extensible to any robot through :doc:`plugins <user_guide/vamp_plugins>`.
- **Multi-Lingual**: Available in both Python and C++ for easy integration into research and industrial workflows.
- **MoveIt! Plugin**: Enables deployment on real-world robotic systems with minimal setup.

Getting Started
===============

To install SRMP and begin planning motions for your robotic system, follow our :doc:`installation guide <user_guide/usage>`.
Check out the :doc:`quick start tutorial <examples/single_robot>` for an example of how to set up and run SRMP for your application.

Why SRMP?
=========

Existing motion planning frameworks often struggle with the demands of high-stakes applications, where predictability and repeatability are critical. 
SRMP addresses these challenges by leveraging search-based planning methods, ensuring motions that are both efficient and reliable. 
Whether you're working on robotic manipulation, industrial automation, or large-scale multi-robot coordination, SRMP provides a powerful solution tailored to your needs.

Learn More
==========

.. - `Documentation <https://srmp-docs.readthedocs.io>`_

- :doc:`Getting Started <user_guide/usage>`
- :doc:`Agent Mode <user_guide/agent_mode>`
- :doc:`API Reference <user_guide/api>`
- :doc:`Release Notes <user_guide/release_notes>`
- `Discord <https://discord.gg/JwAWmqE6N>`_

.. toctree::
   :hidden:
   :maxdepth: 2

   user_guide/index
   examples/index
