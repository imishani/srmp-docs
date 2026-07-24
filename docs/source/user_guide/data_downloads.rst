Data Downloads
==============

This page provides access to robot models, mesh files, and configuration data used in SRMP examples and tutorials.

.. tip::

   **Automatic Downloads**: You can skip manual downloads by using the :doc:`robot_registry`.
   Just call ``planner.add_robot("panda")`` and the data downloads automatically!

   .. code-block:: python

      # No manual download needed - just use the robot name
      planner.add_robot("panda")

Complete Download
-----------------

Download the complete archive containing all Panda robot description files and mesh assets:

:download:`data.zip <../_static/data/data.zip>` - Full robot descriptions and meshes

Robot Registry Archives
-----------------------

These archives are used by the :doc:`robot_registry` for automatic downloads. You can also download them manually:

**Individual Robot Archives:**

- :download:`panda.tar.gz <../_static/robots/panda.tar.gz>` - Franka Emika Panda 7-DOF manipulator
- :download:`panda_on_rail.tar.gz <../_static/robots/panda_on_rail.tar.gz>` - Panda mounted on linear rail (8-DOF)
- :download:`panda_stick.tar.gz <../_static/robots/panda_stick.tar.gz>` - Panda 7-DOF manipulator with rigid stick end-effector (no gripper, for pushing tasks)
- :download:`panda_wristcam.tar.gz <../_static/robots/panda_wristcam.tar.gz>` - Panda 7-DOF manipulator with wrist-mounted RealSense D415 camera
- :download:`so101.tar.gz <../_static/robots/so101.tar.gz>` - SO101 6-DOF manipulator with gripper
- :download:`so107.tar.gz <../_static/robots/so107.tar.gz>` - SO107 7-DOF manipulator with gripper (SO101 + forearm roll)
- :download:`koch.tar.gz <../_static/robots/koch.tar.gz>` - Koch v1.1 6-DOF manipulator arm with gripper
- :download:`xarm7_ability.tar.gz <../_static/robots/xarm7_ability.tar.gz>` - UFactory XArm7 7-DOF arm with Psyonic Ability dexterous hand
- :download:`xarm6.tar.gz <../_static/robots/xarm6.tar.gz>` - UFactory XArm6 6-DOF manipulator (no gripper)
- :download:`xarm6_robotiq.tar.gz <../_static/robots/xarm6_robotiq.tar.gz>` - UFactory XArm6 6-DOF manipulator with Robotiq 2F-85 gripper
- :download:`widowxai.tar.gz <../_static/robots/widowxai.tar.gz>` - Trossen Robotics WidowX AI 6-DOF manipulator with gripper
- :download:`fetch.tar.gz <../_static/robots/fetch.tar.gz>` - Fetch mobile manipulator (holonomic base + torso + 7-DOF arm + gripper)
- :download:`xlerobot.tar.gz <../_static/robots/xlerobot.tar.gz>` - XLeRobot bimanual mobile manipulator (holonomic base + dual arms + head)
- :download:`kinova_gen3.tar.gz <../_static/robots/kinova_gen3.tar.gz>` - Kinova Gen3 7-DOF manipulator with vision module
- :download:`kinova_gen3_6dof.tar.gz <../_static/robots/kinova_gen3_6dof.tar.gz>` - Kinova Gen3 6-DOF manipulator with vision module
- :download:`kinova_gen3_lite.tar.gz <../_static/robots/kinova_gen3_lite.tar.gz>` - Kinova Gen3 Lite compact 6-DOF manipulator

**All Robots:**

- :download:`all_robots.tar.gz <../_static/robots/all_robots.tar.gz>` - All available robots in one archive

Each archive contains the URDF, SRDF, and mesh files needed for planning.

Robot Models
------------

**Panda Robot Configurations**

The following files contain URDF and SRDF configurations for the Franka Panda robot in various setups:

Single Panda Robot:
~~~~~~~~~~~~~~~~~~~

- :download:`panda.urdf <../_static/data/panda/panda.urdf>` - Base Panda robot URDF
- :download:`panda.srdf <../_static/data/panda/panda.srdf>` - Base Panda robot SRDF

Multi-Robot Configurations:
~~~~~~~~~~~~~~~~~~~~~~~~~~~

- :download:`panda0.urdf <../_static/data/panda/panda0.urdf>` - First robot configuration
- :download:`panda0.srdf <../_static/data/panda/panda0.srdf>` - First robot SRDF
- :download:`panda1.urdf <../_static/data/panda/panda1.urdf>` - Second robot configuration
- :download:`panda1.srdf <../_static/data/panda/panda1.srdf>` - Second robot SRDF
- :download:`panda2.urdf <../_static/data/panda/panda2.urdf>` - Third robot configuration
- :download:`panda2.srdf <../_static/data/panda/panda2.srdf>` - Third robot SRDF

Rail-Mounted Configuration:
~~~~~~~~~~~~~~~~~~~~~~~~~~~

- :download:`panda_on_rail.urdf <../_static/data/panda/panda_on_rail.urdf>` - Panda robot mounted on linear rail
- :download:`panda_on_rail.srdf <../_static/data/panda/panda_on_rail.srdf>` - Rail-mounted Panda SRDF

Mesh Files
----------

**Panda Robot Mesh Files**

- :download:`franka_description.tar.gz <../_static/data/panda/franka_description.tar.gz>` - Complete Panda robot mesh files archive

**Environment Mesh Files**

- :download:`crate.stl <../_static/data/meshes/crate.stl>` - Crate mesh for environment setup

The Panda robot mesh archive contains all necessary STL files for collision detection and visualization. Extract the archive to access individual mesh files organized in the standard franka_description directory structure.

Usage
-----

To use these files in your SRMP projects:

1. Download the required URDF/SRDF files for your robot configuration
2. Place them in your project's robot description directory
3. Update your SRMP configuration to point to the downloaded files
4. Ensure the mesh file paths in the URDF correctly reference your mesh directory

For detailed usage instructions, see the :doc:`examples section <../examples/single_robot>`.