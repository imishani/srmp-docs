Robot Registry
==============

SRMP includes a robot registry that allows you to download and use robots by name, without manually managing URDF/SRDF file paths.

Quick Start
-----------

The simplest way to add a robot to your planner:

.. code-block:: python

   from srmp import PlannerInterface

   planner = PlannerInterface()

   # Add robot by name - downloads automatically if needed
   planner.add_robot("panda")

That's it! The robot data is downloaded automatically on first use and cached locally.

Available Robots
----------------

The following robots are available in the registry:

.. list-table::
   :header-rows: 1
   :widths: 20 50 30

   * - Name
     - Description
     - End Effector
   * - ``panda``
     - Franka Emika Panda 7-DOF manipulator
     - ``panda_hand``
   * - ``panda_on_rail``
     - Panda mounted on linear rail (8-DOF)
     - ``panda_hand``
   * - ``so101``
     - SO101 6-DOF manipulator with gripper
     - ``gripper_frame_link``

Using the Registry
------------------

Adding Robots
~~~~~~~~~~~~~

Add a robot from the registry:

.. code-block:: python

   # Simple usage
   planner.add_robot("panda")

   # With custom articulation name
   planner.add_robot("panda", name="my_panda")

   # Override default parameters
   planner.add_robot("panda",
                     name="custom_panda",
                     planned=True,
                     gravity=np.array([0, 0, -9.81]))

Downloading Robots
~~~~~~~~~~~~~~~~~~

You can pre-download robots before using them:

.. code-block:: python

   import srmp.robots as robots

   # Download a specific robot
   robots.download("panda")

   # Download all available robots
   robots.download_all()

Listing Available Robots
~~~~~~~~~~~~~~~~~~~~~~~~

View what robots are available:

.. code-block:: python

   import srmp.robots as robots

   available = robots.list_available()
   print(available)
   # {'remote': ['panda', 'panda_on_rail', 'so101'],
   #  'local': ['panda'],  # already downloaded
   #  'custom': []}        # user-registered

Getting Robot Information
~~~~~~~~~~~~~~~~~~~~~~~~~

Get detailed information about a robot:

.. code-block:: python

   robot = robots.get("panda")
   print(robot.urdf_path)      # Path to URDF
   print(robot.srdf_path)      # Path to SRDF
   print(robot.end_effector)   # "panda_hand"
   print(robot.default_qpos)   # Default joint configuration
   print(robot.joint_names)    # List of joint names

Custom Robots
-------------

Register your own robots for easy reuse:

.. code-block:: python

   import srmp.robots as robots

   # Register a custom robot
   robots.register(
       name="my_robot",
       urdf_path="/path/to/my_robot.urdf",
       srdf_path="/path/to/my_robot.srdf",
       end_effector="tool0",
       description="My custom robot arm",
       default_qpos=[0, 0, 0, 0, 0, 0],
       joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
   )

   # Now use it by name
   planner.add_robot("my_robot")

   # Unregister when no longer needed
   robots.unregister("my_robot")

Path Fallback
~~~~~~~~~~~~~

If you pass a file path instead of a robot name, ``add_robot`` works like ``add_articulation``:

.. code-block:: python

   # Using file path directly
   planner.add_robot(
       "/path/to/robot.urdf",
       srdf_path="/path/to/robot.srdf",
       end_effector="tool0",
       name="my_robot"
   )

Cache Configuration
-------------------

Robot data is cached in ``~/.cache/srmp/robots/`` by default. You can customize this:

.. code-block:: python

   import srmp.robots as robots

   # Set custom cache directory
   robots.set_cache_dir("/shared/robots")

   # Or use environment variable
   # export SRMP_ROBOTS_DIR=/shared/robots

   # Check current cache directory
   print(robots.get_cache_dir())

API Reference
-------------

Registry Functions
~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Function
     - Description
   * - ``robots.download(name, force=False)``
     - Download a robot by name
   * - ``robots.download_all(force=False)``
     - Download all available robots
   * - ``robots.get(name)``
     - Get RobotInfo for a robot
   * - ``robots.info(name)``
     - Alias for ``get()``
   * - ``robots.list_available()``
     - List remote, local, and custom robots
   * - ``robots.register(...)``
     - Register a custom robot
   * - ``robots.unregister(name)``
     - Remove a custom robot registration
   * - ``robots.get_cache_dir()``
     - Get the cache directory path
   * - ``robots.set_cache_dir(path)``
     - Set a custom cache directory

PlannerInterface Method
~~~~~~~~~~~~~~~~~~~~~~~

.. py:method:: PlannerInterface.add_robot(robot, name=None, srdf_path=None, end_effector=None, planned=True, gravity=None, link_names=None, joint_names=None)

   Add a robot from the registry or by file path.

   :param str robot: Robot name from registry, or path to URDF file
   :param str name: Override the articulation name (default: robot name)
   :param str srdf_path: Override SRDF path (required if using file path)
   :param str end_effector: Override end effector link name (required if using file path)
   :param bool planned: Include in planning (default: True)
   :param numpy.ndarray gravity: Gravity vector (default: [0, 0, 0])
   :param list link_names: Override link names
   :param list joint_names: Override joint names
   :raises RobotNotFoundError: If robot not in registry and not a valid path
   :raises ValueError: If using file path without srdf_path and end_effector

Exceptions
~~~~~~~~~~

.. py:exception:: srmp.robots.RobotNotFoundError

   Raised when a robot is not found in the registry. The error message includes
   a list of available robots and instructions for downloading or registering.

.. py:exception:: srmp.robots.DownloadError

   Raised when downloading robot data fails.
