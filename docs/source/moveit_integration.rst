MoveIt Integration
==================

SRMP MoveIt! Plugin Demo
~~~~~~~~~~~~~~~~~~~~~~~~

.. image:: _static/assets/srmp_moveit2.gif
   :align: center
   :width: 800px
   :alt: SRMP MoveIt! Plugin Demo

SRMP provides seamless integration with MoveIt through a plugin. We provide two ways to set up SRMP with MoveIt! The first is a pre-configured Docker container. This package contains everything you need to run SRMP with MoveIt, including the SRMP libraries, MoveIt planners, and a pre-built user workspace. The second is a ROS package that you can install and use in your existing ROS workspace. Let's start with the Docker container.

Dockerized SRMP MoveIt! Plugin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Download Docker Package
-----------------------

You can download the complete SRMP MoveIt! Docker package here:

.. raw:: html

   <div style="text-align: center; margin: 20px 0;">
       <a href="_static/assets/srmp-docker.zip" download="srmp-docker.zip" 
          style="display: inline-block; padding: 15px 30px; background-color: #2980b9; color: white; text-decoration: none; border-radius: 5px; font-weight: bold; font-size: 16px;">
           📦 Download SRMP Docker Package
       </a>
       <p style="margin-top: 10px; color: #666; font-size: 14px;">
           Size: ~44MB | Includes MoveIt, SRMP libraries, and pre-built workspace
       </p>
   </div>

What's Included
---------------

- **MoveIt**: Complete MoveIt installation with all planners and tools (ROS, Ubuntu 20.04)
- **SRMP Libraries**: Full SRMP implementation with MoveIt plugin
- **User Workspace**: Pre-built workspace with SRMP MoveIt plugin and resources
- **RViz**: ROS visualization tool for robot simulation
- **Robot Configuration**: Panda robot configuration for testing and development

Quick Start
-----------

Prerequisites
-------------

- Docker installed on your system

Installation Steps
------------------

1. **Extract the package**:
    .. code-block:: bash

      unzip srmp-docker.zip
      cd docker_release

2. **Build the Docker image**:
    .. code-block:: bash

      ./build-srmp-docker.sh

   .. note::
      This may take 10-15 minutes on first build as it downloads and compiles all dependencies.

3. **Run the container**:
    .. code-block:: bash

      ./run-srmp-docker.sh

4. **Set up the workspace** (inside the container):
    .. code-block:: bash

      # Navigate to the workspace
      cd /workspace/srmp_user_ws
      
      # Build the workspace
      catkin build
      
      # Source the workspace
      source devel/setup.bash

5. **Test SRMP with MoveIt**: this command will start the MoveIt! demo with the Panda robot and show RViz.
    .. code-block:: bash

      # Launch the demo with Panda robot
      roslaunch panda_two_moveit_config demo.launch

Once the container is running and the workspace is set up, you can use SRMP for motion planning:

Installing the SRMP MoveIt! Plugin Locally (No Docker)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

While the Docker solution is simpler and (likely) easier to use, we also provide a debian package that you can install and use in your existing ROS workspace.

Download Local Package
-----------------------

You can download the SRMP MoveIt! local installation package here:

.. raw:: html

   <div style="text-align: center; margin: 20px 0;">
       <a href="_static/assets/srmp-local.zip" download="srmp-local.zip" 
          style="display: inline-block; padding: 15px 30px; background-color: #27ae60; color: white; text-decoration: none; border-radius: 5px; font-weight: bold; font-size: 16px;">
           📦 Download SRMP Local Package
       </a>
       <p style="margin-top: 10px; color: #666; font-size: 14px;">
           Size: ~44MB | Includes SRMP debian package and workspace
       </p>
   </div>

Prerequisites
-------------

- **Ubuntu 20.04** (required for compatibility)
- **ROS Noetic** installed and configured
- **MoveIt!** installed in your ROS workspace
- **Standard ROS development tools** (catkin, rosdep, etc.)

Installation Steps
------------------

1. **Extract the package**:
    .. code-block:: bash

      unzip srmp-local.zip
      cd local_release

2. **Install the SRMP debian package**:
    .. code-block:: bash

      sudo dpkg -i srmp-moveit-plugin_*.deb

3. **Navigate to the provided workspace**:
    .. code-block:: bash

      cd srmp_user_ws

4. **Build the workspace**:
    .. code-block:: bash

      catkin build

5. **Source the workspace**:
    .. code-block:: bash

      source devel/setup.bash

6. **Test SRMP with MoveIt**:
    .. code-block:: bash

      # Launch the demo with Panda robot
      roslaunch panda_two_moveit_config demo.launch
