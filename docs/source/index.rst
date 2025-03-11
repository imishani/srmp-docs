Welcome to Search-Based Robot Motion Planning (SRMP) documentation!
===================================

**SRMP** is a motion planning software for robotic manipulation, leveraging state-of-the-art search-based algorithms. It ensures consistent and predictable motions, backed by rigorous theoretical guarantees. Additionally, SRMP can efficiently plan for up to dozens of manipulators while guaranteeing collision-free execution—both between robots and with the environment—while maintaining motion consistency and predictability.

Key Features
============

- **Multi-Robot Motion Planning**: First-of-its-kind support for planning coordinated motions in multi-manipulator systems.
- **Reliable and Consistent Trajectories**: Generates predictable and repeatable motions, making it ideal for high-precision and safety-critical applications.
- **Seamless Integration**: Compatible with major simulators, including MuJoCo, Sapien, Genesis, PyBullet and Isaac.
- **Multi-Lingual**: Available in both Python and C++ for easy integration into research and industrial workflows.
- **MoveIt! Plugin**: Enables deployment on real-world robotic systems with minimal setup.

Getting Started
===============

To install SRMP and begin planning motions for your robotic system, follow our :doc:`installation guide <installation>`.  
Check out the :doc:`quick start tutorial <quick_start>` for an example of how to set up and run SRMP for your application.

Why SRMP?
=========

Existing motion planning frameworks often struggle with the demands of high-stakes applications, where predictability and repeatability are critical. 
SRMP addresses these challenges by leveraging search-based planning methods, ensuring motions that are both efficient and reliable. 
Whether you're working on robotic manipulation, industrial automation, or large-scale multi-robot coordination, SRMP provides a powerful solution tailored to your needs.

Learn More
==========

- `Documentation <https://srmp-docs.readthedocs.io>`_
- :doc:`Tutorials <tutorials>`
- :doc:`API Reference <api>`


.. Check out the :doc:`usage` section for further information, including
.. how to :ref:`installation` the project.

.. .. note::

..    This project is under active development.

.. Contents
.. --------

.. .. toctree::

..    usage
..    api
