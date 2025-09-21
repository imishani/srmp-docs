API Reference
=============

Core Classes
------------

PlannerInterface
~~~~~~~~~~~~~~~~

The main interface for robot motion planning.

.. class:: srmp.PlannerInterface()

   The primary class for creating and configuring planners.

   **Methods:**

   .. method:: add_articulation(urdf_path, srdf_path, name, end_effector, planned=True)

      Add a robot to the planning scene.

      :param str urdf_path: Path to the URDF file describing the robot
      :param str srdf_path: Path to the SRDF file with semantic information
      :param str name: Unique name for this robot instance
      :param str end_effector: Name of the end-effector link
      :param bool planned: Whether this robot should be planned for (default: True)

   .. method:: set_base_pose(name, pose)

      Set the base pose of a robot.

      :param str name: Name of the robot
      :param Pose pose: Base pose of the robot

   .. method:: make_planner(articulation_names, planner_context)

      Configure and initialize the planner.

      :param list articulation_names: List of robot names to plan for
      :param dict planner_context: Dictionary containing planner configuration

      **Single Robot Planner Context Options:**

      - ``planner_id``: Planner type ("wAstar", "ARAstar", "MHAstar", "wPASE", "Astar")
      - ``heuristic``: Heuristic function ("bfs", "joint_euclidean", etc.)
      - ``weight``: Weight for weighted planners (string, e.g., "10.0")
      - ``weight_delta``: Weight reduction per iteration for ARAstar
      - ``final_weight``: Final weight for ARAstar
      - ``inadmissible_heuristics``: Heuristics for MHAstar
      - ``w1``, ``w2``: Weights for MHAstar
      - ``num_threads``: Number of threads for parallel planners

      **Multi-Robot Planner Context Options:**

      - ``planner_id``: "xECBS" for multi-robot planning
      - ``weight_low_level_heuristic``: Weight for low-level heuristic
      - ``high_level_focal_suboptimality``: High-level focal weight
      - ``low_level_focal_suboptimality``: Low-level focal weight
      - ``heuristic_{robot_name}``: Heuristic for specific robot
      - ``mprim_path_{robot_name}``: Motion primitive file for specific robot

   .. method:: plan(start_state, goal_constraint)

      Plan a trajectory for a single robot.

      :param numpy.ndarray start_state: Starting joint configuration
      :param GoalConstraint goal_constraint: Goal specification
      :returns: Trajectory object containing the planned path
      :rtype: Trajectory

   .. method:: plan_multi(start_states, goal_constraints)

      Plan trajectories for multiple robots simultaneously.

      :param dict start_states: Dictionary mapping robot names to start configurations
      :param dict goal_constraints: Dictionary mapping robot names to goal constraints
      :returns: Dictionary mapping robot names to their trajectories
      :rtype: dict

   .. method:: add_box(name, size, pose)

      Add a box obstacle to the environment.

      :param str name: Unique name for the box
      :param numpy.ndarray size: Box dimensions [x, y, z]
      :param Pose pose: Box pose in world frame

   .. method:: add_sphere(name, radius, pose)

      Add a sphere obstacle to the environment.

      :param str name: Unique name for the sphere
      :param float radius: Sphere radius
      :param Pose pose: Sphere pose in world frame

   .. method:: add_cylinder(name, radius, height, pose)

      Add a cylinder obstacle to the environment.

      :param str name: Unique name for the cylinder
      :param float radius: Cylinder radius
      :param float height: Cylinder height
      :param Pose pose: Cylinder pose in world frame

   .. method:: add_mesh(name, mesh_path, scale, pose)

      Add a mesh obstacle to the environment.

      :param str name: Unique name for the mesh
      :param str mesh_path: Path to mesh file (STL, OBJ, etc.)
      :param numpy.ndarray scale: Scaling factors [x, y, z]
      :param Pose pose: Mesh pose in world frame

   .. method:: add_point_cloud(name, vertices, resolution=0.01)

      Add a point cloud as a collision object to the environment.

      :param str name: Unique name for the point cloud
      :param numpy.ndarray vertices: Point cloud vertices as Nx3 matrix where each row is [x, y, z]
      :param float resolution: Voxel resolution for octomap representation (default: 0.01)

   .. method:: remove_object(name)

      Remove an object from the environment.

      :param str name: Name of the object to remove

   .. method:: read_sim(sim, sim_type, articulations=None)

      Import objects from a simulation environment.

      :param sim: Simulation object
      :param str sim_type: Type of simulator ("sapien", "genesis", "pybullet", "mujoco", "swift")
      :param list articulations: List of articulation names to exclude from import

   .. method:: print_available_planners()

      Print available planners and their descriptions.

Data Types
----------

Pose
~~~~

.. class:: srmp.Pose()

   Represents a 6DOF pose (position and orientation).

   **Attributes:**

   .. attribute:: p

      Position as numpy array [x, y, z]

      :type: numpy.ndarray

   .. attribute:: q

      Orientation as quaternion [w, x, y, z]

      :type: numpy.ndarray

GoalConstraint
~~~~~~~~~~~~~~

.. class:: srmp.GoalConstraint(goal_type, target)

   Represents a goal constraint for planning.

   :param GoalType goal_type: Type of goal constraint
   :param target: Target specification (joint angles or poses)

GoalType
~~~~~~~~

.. class:: srmp.GoalType

   Enumeration of goal constraint types.

   .. attribute:: JOINTS

      Goal specified as joint angles

   .. attribute:: POSE

      Goal specified as end-effector pose

Trajectory
~~~~~~~~~~

.. class:: srmp.Trajectory

   Represents a planned trajectory.

   **Attributes:**

   .. attribute:: positions

      List of joint configurations along the trajectory

      :type: list

Examples
--------

Basic Single Robot Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import numpy as np

   # Create planner
   planner = srmp.PlannerInterface()

   # Add robot
   planner.add_articulation(
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",
       name="panda",
       end_effector="panda_hand"
   )

   # Add obstacle
   obstacle_pose = srmp.Pose()
   obstacle_pose.p = np.array([0.5, 0.2, 0.4])
   planner.add_box("obstacle", np.array([0.1, 0.1, 0.4]), obstacle_pose)

   # Configure planner
   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10.0"
   })

   # Plan trajectory
   start_state = np.radians([0, -45, 0, -135, 0, 90, 45])

   goal_pose = srmp.Pose()
   goal_pose.p = np.array([0.6, 0.0, 0.5])
   goal_pose.q = np.array([0, 0, 0, 1])
   goal = srmp.GoalConstraint(srmp.GoalType.POSE, [goal_pose])

   trajectory = planner.plan(start_state, goal)

Multi-Robot Example
~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import numpy as np

   # Create planner
   planner = srmp.PlannerInterface()

   # Add two robots
   for i in range(2):
       planner.add_articulation(
           urdf_path=f"/path/to/panda{i}.urdf",
           srdf_path=f"/path/to/panda{i}.srdf",
           name=f"panda{i}",
           end_effector=f"panda{i}_hand"
       )

   # Set base poses
   for i in range(2):
       pose = srmp.Pose()
       pose.p = np.array([(-1)**i * 0.5, 0.5, 0])
       pose.q = np.array([1, 0, 0, 0])
       planner.set_base_pose(f"panda{i}", pose)

   # Configure multi-robot planner
   planner.make_planner(
       ["panda0", "panda1"],
       {
           "planner_id": "xECBS",
           "weight_low_level_heuristic": "55.0",
           "high_level_focal_suboptimality": "1.8",
           "low_level_focal_suboptimality": "1.0",
           "heuristic_panda0": "joint_euclidean_remove_time",
           "heuristic_panda1": "joint_euclidean_remove_time"
       }
   )

   # Plan trajectories
   start_states = {
       "panda0": np.radians([-40, 0, 0, -85, 0, 57, 0]),
       "panda1": np.radians([-40, 0, 0, -85, 0, 57, 0])
   }

   goal_states = {
       "panda0": np.radians([40, 0, 0, -70, 0, 50, 0]),
       "panda1": np.radians([40, 0, 0, -95, 0, 67, 0])
   }

   goal_constraints = {}
   for name, goal_state in goal_states.items():
       goal_constraints[name] = srmp.GoalConstraint(srmp.GoalType.JOINTS, [goal_state])

   trajectories = planner.plan_multi(start_states, goal_constraints)

Simulator Integration Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import sapien
   import numpy as np

   # Create SAPIEN scene
   scene = sapien.Scene()
   scene.add_ground()

   # Add some objects to scene
   builder = scene.create_actor_builder()
   builder.add_box_collision(half_size=[0.1, 0.1, 0.1])
   builder.add_box_visual(half_size=[0.1, 0.1, 0.1])
   box = builder.build_kinematic()
   box.set_pose(sapien.Pose([0.5, 0.0, 0.5]))

   # Create planner
   planner = srmp.PlannerInterface()

   # Add robot
   planner.add_articulation(
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",
       name="panda",
       end_effector="panda_hand"
   )

   # Import scene objects automatically
   planner.read_sim(scene, "sapien")

   # Continue with planning as usual...

Point Cloud Example
~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import numpy as np

   # Create planner
   planner = srmp.PlannerInterface()

   # Add robot
   planner.add_articulation(
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",
       name="panda",
       end_effector="panda_hand"
   )

   # Load point cloud from file (example formats: PLY, PCD, or custom)
   # For this example, we'll generate a synthetic point cloud

   # Generate a point cloud representing a table surface
   table_points = []
   for x in np.linspace(0.2, 0.8, 30):
       for y in np.linspace(-0.3, 0.3, 20):
           table_points.append([x, y, 0.4])  # Table at height 0.4m

   table_cloud = np.array(table_points)

   # Generate a point cloud representing a wall
   wall_points = []
   for y in np.linspace(-0.5, 0.5, 40):
       for z in np.linspace(0.0, 1.5, 60):
           wall_points.append([0.9, y, z])  # Wall at x=0.9m

   wall_cloud = np.array(wall_points)

   # Add point clouds to planner with different resolutions
   planner.add_point_cloud("table_surface", table_cloud, resolution=0.01)
   planner.add_point_cloud("wall", wall_cloud, resolution=0.02)

   # Configure planner
   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10.0"
   })

   # Plan around point cloud obstacles
   start_state = np.radians([0, -45, 0, -135, 0, 90, 45])

   # Goal pose that requires navigating around the point cloud obstacles
   goal_pose = srmp.Pose()
   goal_pose.p = np.array([0.7, 0.1, 0.6])  # Above the table, near the wall
   goal_pose.q = np.array([0, 0, 0, 1])
   goal = srmp.GoalConstraint(srmp.GoalType.POSE, [goal_pose])

   trajectory = planner.plan(start_state, goal)

   if trajectory:
       print(f"Successfully planned around point cloud obstacles")
       print(f"Trajectory length: {len(trajectory.positions)} waypoints")
   else:
       print("Planning failed - point cloud obstacles may block all paths")

   # Point cloud loading from files (common formats)
   def load_point_cloud_from_ply(filename):
       """Load point cloud from PLY file"""
       # This is a simplified example - use libraries like Open3D for robust loading
       points = []
       with open(filename, 'r') as f:
           lines = f.readlines()
           # Skip PLY header, find vertex data
           vertex_start = False
           for line in lines:
               if line.strip() == "end_header":
                   vertex_start = True
                   continue
               if vertex_start and line.strip():
                   coords = line.strip().split()
                   if len(coords) >= 3:
                       points.append([float(coords[0]), float(coords[1]), float(coords[2])])
       return np.array(points)

   def load_point_cloud_from_txt(filename):
       """Load point cloud from simple text file (x y z per line)"""
       return np.loadtxt(filename)

   # Usage with file loading
   # point_cloud = load_point_cloud_from_ply("/path/to/scan.ply")
   # planner.add_point_cloud("scanned_object", point_cloud, resolution=0.005)

   # Point cloud from sensor data (example with simulated LiDAR-style data)
   def generate_lidar_point_cloud(robot_pose, num_rays=360, max_range=5.0):
       """Generate simulated LiDAR point cloud"""
       points = []
       for i in range(num_rays):
           angle = 2 * np.pi * i / num_rays
           # Simulate ray hitting objects at various distances
           distance = np.random.uniform(0.5, max_range)
           x = robot_pose[0] + distance * np.cos(angle)
           y = robot_pose[1] + distance * np.sin(angle)
           z = robot_pose[2] + np.random.uniform(-0.1, 0.1)  # Some height variation
           points.append([x, y, z])
       return np.array(points)

   # Simulate sensor-based point cloud
   robot_position = [0, 0, 0.5]
   sensor_cloud = generate_lidar_point_cloud(robot_position)
   planner.add_point_cloud("sensor_obstacles", sensor_cloud, resolution=0.03)
