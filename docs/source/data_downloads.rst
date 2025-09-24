Data Downloads
==============

This page provides access to robot models, mesh files, and configuration data used in SRMP examples and tutorials.

Robot Models
------------

**Panda Robot Configurations**

The following files contain URDF and SRDF configurations for the Franka Panda robot in various setups:

Single Panda Robot:
~~~~~~~~~~~~~~~~~~~

- :download:`panda.urdf <_static/data/panda/panda.urdf>` - Base Panda robot URDF
- :download:`panda.srdf <_static/data/panda/panda.srdf>` - Base Panda robot SRDF

Multi-Robot Configurations:
~~~~~~~~~~~~~~~~~~~~~~~~~~~

- :download:`panda0.urdf <_static/data/panda/panda0.urdf>` - First robot configuration
- :download:`panda0.srdf <_static/data/panda/panda0.srdf>` - First robot SRDF
- :download:`panda1.urdf <_static/data/panda/panda1.urdf>` - Second robot configuration
- :download:`panda1.srdf <_static/data/panda/panda1.srdf>` - Second robot SRDF
- :download:`panda2.urdf <_static/data/panda/panda2.urdf>` - Third robot configuration
- :download:`panda2.srdf <_static/data/panda/panda2.srdf>` - Third robot SRDF

Rail-Mounted Configuration:
~~~~~~~~~~~~~~~~~~~~~~~~~~~

- :download:`panda_on_rail.urdf <_static/data/panda/panda_on_rail.urdf>` - Panda robot mounted on linear rail
- :download:`panda_on_rail.srdf <_static/data/panda/panda_on_rail.srdf>` - Rail-mounted Panda SRDF

Mesh Files
----------

All necessary mesh files for collision detection and visualization are included in the robot descriptions above. The mesh files are organized within the robot model directories and will be automatically accessible when using the URDF files.

Usage
-----

To use these files in your SRMP projects:

1. Download the required URDF/SRDF files for your robot configuration
2. Place them in your project's robot description directory
3. Update your SRMP configuration to point to the downloaded files
4. Ensure the mesh file paths in the URDF correctly reference your mesh directory

For detailed usage instructions, see the :doc:`examples section <examples/single_robot>`.