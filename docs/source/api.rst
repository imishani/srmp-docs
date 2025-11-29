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

   .. method:: add_articulation(name, end_effector, 
                                urdf_path, srdf_path='', 
                                link_names: List[str] = [],
                                joint_names: List[str] = [],
                                gravity: NDArray[np.float64] = np.array([0, 0, 0]), 
                                planned=True)

      Add a robot to the planning scene. The `srdf_path` argument is optional — if you
      don't have an SRDF file, you can omit this argument or pass an empty string.

      :param str name: Unique name for this robot instance
      :param str end_effector: Name of the end-effector link
      :param str urdf_path: Path to the URDF file describing the robot
      :param str srdf_path: Path to the SRDF file with semantic information (optional, default: "")
      :param list link_names: List of link names to include (default: all links)
      :param list joint_names: List of joint names to include (default: all joints)
      :param numpy.ndarray gravity: Gravity vector for the robot (default: [0, 0, 0])
      :param bool planned: Whether this robot should be planned for (default: True)

   .. method:: remove_articulation(name)

      Remove an articulation from the planning world.

      :param str name: Name of the articulation to remove

   .. method:: set_base_pose(name, pose)

      Set the base pose of a robot.

      :param str name: Name of the robot
      :param Pose pose: Base pose of the robot

   .. method:: make_planner(articulation_names, planner_context)

      Configure and initialize the planner.

      :param list articulation_names: List of robot names to plan for
      :param dict planner_context: Dictionary containing planner configuration

      **Required Parameters:**

      - ``planner_id`` (string): Planner algorithm to use

      **Single Robot Planner Context Options:**

      Available single-robot planners: "Astar", "wAstar", "ARAstar", "MHAstar", "wPASE"

      **General Parameters:**

      - ``heuristic`` (string): Heuristic function ("bfs", "joint_euclidean", "joint_euclidean_remove_time"). Default: "bfs"
      - ``resolution`` (string): Joint angle discretization in degrees. Default: "1"
      - ``mprim_path`` (string): Path to motion primitives file. Default: auto-generated based on DOF
      - ``time_limit`` or ``allowed_planning_time`` (string): Planning time limit in seconds. Default: "10"

      **Planner-Specific Parameters:**

      *A\* ("Astar"):*
        Uses only general parameters above.

      *Weighted A\* ("wAstar"):*
        - ``weight`` (string): Heuristic weight. Default: "50"

      *ARA\* ("ARAstar"):*
        - ``weight`` (string): Initial heuristic weight. Default: "50"
        - ``weight_delta`` (string): Weight reduction per iteration. Default: "10.0"
        - ``final_weight`` (string): Final weight to reach. Default: "1.0"

      *MHA\* ("MHAstar"):*
        - ``heuristic`` (string): Anchor heuristic. Default: "joint_euclidean"
        - ``inadmissible_heuristics`` (vector<string>): List of inadmissible heuristics. Default: ["bfs"]
        - ``w1`` (string): Anchor heuristic weight. Default: "20"
        - ``w2`` (string): Inadmissible heuristic weight. Default: "5"

      *wPASE ("wPASE"):*
        - ``heuristic`` (string): Primary heuristic. Default: "joint_euclidean"
        - ``i_heuristic`` (string): Secondary heuristic. Default: "joint_euclidean"
        - ``weight`` (string): Primary heuristic weight. Default: "50"
        - ``i_weight`` (string): Secondary heuristic weight. Default: "100.0"
        - ``num_threads`` (string): Number of parallel threads. Default: "4"

      **Multi-Robot Planner Context Options:**

      Available multi-robot planners: "E-CBS", "xECBS"

      **Required Multi-Robot Parameters:**

      - ``planner_id``: "E-CBS" or "xECBS"

      **Agent-Specific Parameters (per robot):**

      For each robot with name ``{robot_name}``:

      - ``heuristic_{robot_name}`` (string): Heuristic for this robot. Default: "bfs" (E-CBS), "joint_euclidean_remove_time" (xECBS)
      - ``mprim_path_{robot_name}`` (string): Motion primitives path for this robot. Default: auto-generated timed version
      - ``resolution_{robot_name}`` (string): Discretization for this robot. Default: "1"

      **E-CBS/xECBS Parameters:**

      - ``weight_low_level_heuristic`` (string): Low-level search weight. Default: "1.0" (E-CBS), "55.0" (xECBS)
      - ``high_level_focal_suboptimality`` (string): High-level focal search bound. Default: "1.3"
      - ``low_level_focal_suboptimality`` (string): Low-level focal search bound. Default: "1.3"

   .. method:: plan(start, goal_constraint)

      Plan a trajectory for a single robot.

      :param numpy.ndarray start: Starting joint configuration
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

   Note: `add_articulation` accepts an optional `srdf_path` parameter. Many examples
   below include SRDF paths for completeness, but you can call `add_articulation`
   with only the `urdf_path` if no SRDF is required for your use case.

   .. method:: get_articulation_names()

      Return the list of articulation names currently present in the planning world.

      :returns: List of articulation names

   .. method:: get_object_names()

      Return the list of object names currently present in the planning world.

      :returns: List of object names

   .. method:: has_articulation(name)

      Check whether an articulation with the given name exists in the planning world.

      :param str name: Articulation name
      :returns: True if the articulation exists, False otherwise

   .. method:: has_object(name)

      Check whether an object with the given name exists in the planning world.

      :param str name: Object name
      :returns: True if the object exists, False otherwise

   .. method:: is_articulation_planned(name)

      Check whether a named articulation is configured to be planned.

      :param str name: Articulation name
      :returns: True if the articulation is planned, False otherwise

   .. method:: set_articulation_planned(name, planned)

      Enable or disable planning for a specific articulation.

      :param str name: Articulation name
      :param bool planned: Whether to plan for this articulation

   .. method:: is_object_attached(name)

      Query whether an object is currently attached to a robot.

      :param str name: Object name
      :returns: True if attached, False otherwise

   .. method:: attach_object(name, art_name, link_id, touch_links=None)

      Attach an existing object to a robot link so it moves with the robot.

      :param str name: Object name
      :param str art_name: Articulation name to attach to
      :param int link_id: Index of the link to attach the object to
      :param list touch_links: Optional list of link names allowed to touch the object

   .. method:: detach_object(name, also_remove=False)

      Detach an attached object from its robot. Optionally remove it from the world.

      :param str name: Object name
      :param bool also_remove: If True, remove the object from the world after detaching
      :returns: True if successful

   .. method:: detach_all_objects(also_remove=False)

      Detach all attached objects. Optionally remove them from the world.

      :param bool also_remove: If True, remove detached objects from the world
      :returns: True if successful

   .. method:: is_state_colliding(articulation_name="")

      Check whether the current state (optionally for a specific articulation) is in collision.

      :param str articulation_name: Optional articulation name to check
      :returns: True if a collision is detected

   .. method:: is_robot_colliding_with_objects(art_name)

      Check if the specified robot is colliding with any objects in the planning world.

      :param str art_name: Articulation name
      :returns: True if collision detected with environment objects

   .. method:: distance_to_self_collision()

      Get the minimum signed distance to self-collision for all robots.

      :returns: Minimum self-collision distance (float)

   .. method:: distance_to_robot_collision()

      Get the minimum signed distance between robots and environment objects.

      :returns: Minimum robot-to-environment collision distance (float)

   .. method:: distance_to_collision()

      Get the minimum distance to any collision (self or environment).

      :returns: Minimum distance to collision (float)

   .. method:: set_allowed_collision(name1, name2, allowed)

      Set whether collisions between two named objects are allowed.

      :param str name1: First object name
      :param str name2: Second object name
      :param bool allowed: True to allow collisions, False to disallow

   .. method:: set_qpos(name, qpos)

      Set the joint positions for a named articulation.

      :param str name: Articulation name
      :param numpy.ndarray qpos: Joint positions (1D array)

   .. method:: set_qpos_all(state)

      Set the joint positions for all planned articulations using a concatenated state vector.

      :param numpy.ndarray state: Concatenated joint positions for all planned articulations

   .. method:: update_attached_bodies_pose()

      Update the poses of all objects attached to robots based on current robot joint states.

   .. method:: compute_fk(articulation_name, qpos)

      Compute forward kinematics for a specific articulation and joint configuration.

      :param str articulation_name: Articulation name
      :param numpy.ndarray qpos: Joint positions
      :returns: Pose of the end-effector (Pose)

   .. method:: compute_ik(articulation_name, ee_pose, init_state_val)

      Compute inverse kinematics (CLIK) for a desired end-effector pose.

      :param str articulation_name: Articulation name
      :param list ee_pose: Desired end-effector pose [x, y, z, roll, pitch, yaw]
      :param list init_state_val: Initial joint configuration for IK solver
      :returns: Tuple `(success: bool, joint_state: list)`

   .. method:: reset(reset_robots=True)

      Reset the planner interface. When `reset_robots` is True, all articulations and objects
      are removed; when False, only planner caches and internal data are reset.

      :param bool reset_robots: Whether to remove robots and objects during reset

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

   .. attribute:: velocities

      List of joint velocities along the trajectory

      :type: list

   .. attribute:: accelerations

      List of joint accelerations along the trajectory

      :type: list


Motion Primitives Configuration
-------------------------------

Motion primitives define the discrete actions available to the robot during planning. SRMP uses YAML configuration files to define motion primitive families and their properties.

YAML Configuration Files
~~~~~~~~~~~~~~~~~~~~~~~~~

**File Structure:**

.. code-block:: yaml

   <family_name>:
     <primitive_name>:
       mprim_sequence:
         - [0, 0, 0, ...]  # Always starts with origin (all zeros)
         - [delta1, delta2, ...]  # Delta values from origin
         - [delta1, delta2, ...]  # Additional steps (optional)
       mprim_sequence_transition_costs: [cost1, cost2, 0]  # Last is always 0
       mprim_sequence_transition_times: [time1, time2, 0]  # Optional timing
       generate_negative: true/false  # Whether to generate negative deltas

**Key Components:**

- **Family Name**: Groups related primitives (e.g., ``long_primitives``, ``short_primitives``)
- **Primitive Name**: Unique identifier for each motion primitive
- **mprim_sequence**: Sequence of states, always starting with zeros (origin)
- **mprim_sequence_transition_costs**: Cost for each transition in the sequence
- **mprim_sequence_transition_times**: Optional time constraints for each transition
- **generate_negative**: Automatically creates negative versions of primitives

**Units:**

- **Joint space (manipulators)**: Degrees for angular movements

**Example: 7DOF Manipulator Primitives**

.. code-block:: yaml

   long_primitives:
     joint0:
       mprim_sequence:
         - [0, 0, 0, 0, 0, 0, 0]  # Origin state
         - [15, 0, 0, 0, 0, 0, 0]  # Move joint 0 by 15 degrees
       mprim_sequence_transition_costs: [1, 0]
       generate_negative: true

     joint1:
       mprim_sequence:
         - [0, 0, 0, 0, 0, 0, 0]
         - [0, 15, 0, 0, 0, 0, 0]  # Move joint 1 by 15 degrees
       mprim_sequence_transition_costs: [1, 0]
       generate_negative: true

   short_primitives:
     joint0:
       mprim_sequence:
         - [0, 0, 0, 0, 0, 0, 0]
         - [7, 0, 0, 0, 0, 0, 0]  # Move joint 0 by 7 degrees
       mprim_sequence_transition_costs: [1, 0]
       generate_negative: true

**Example: Timed Motion Primitives**

.. code-block:: yaml

   long_primitives:
     joint0:
       mprim_sequence:
         - [0, 0, 0, 0, 0, 0, 0]
         - [15, 0, 0, 0, 0, 0, 0]
       mprim_sequence_transition_costs: [1, 0]
       mprim_sequence_transition_times: [1, 0]  # 1 time unit per transition
       generate_negative: true

Creating Custom Motion Primitives
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Step 1: Define YAML Configuration**

1. Create a new YAML file with appropriate naming (e.g., ``custom_7dof_mprim.yaml``)
2. Define motion primitive families based on your robot's requirements
3. Specify primitives for each joint
4. Set appropriate costs and timing constraints

**Step 2: File Naming Convention**

- **Manipulators**: ``<robot_type>_<dof>dof[_additional_info]_mprim.yaml``
- **Timed variants**: Include ``_timed`` in the filename for multi-robot coordination

**Step 3: Integration with Planner**

.. code-block:: python

   # Use custom motion primitives
   planner.make_planner(["robot_name"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "mprim_path": "/path/to/custom_7dof_mprim.yaml"
   })

   # For multi-robot with custom primitives per robot
   planner.make_planner(["robot1", "robot2"], {
       "planner_id": "xECBS",
       "mprim_path_robot1": "/path/to/robot1_timed_mprim.yaml",
       "mprim_path_robot2": "/path/to/robot2_timed_mprim.yaml"
   })

**Design Guidelines:**

1. **Start Simple**: Begin with single-joint movements before complex combinations
2. **Balance Resolution vs Speed**: More primitives = finer control but slower planning
3. **Cost Weighting**: Use costs to prefer certain types of movements
4. **Symmetric Movements**: Use ``generate_negative: true`` for symmetric joint movements
5. **Multi-Robot**: Use timed primitives (``_timed_mprim.yaml``) for coordination

**Available Primitive Files:**

The SRMP package includes pre-configured motion primitive files for manipulators:

- ``manip_6dof_mprim.yaml`` - 6DOF manipulator primitives
- ``manip_7dof_mprim.yaml`` - 7DOF manipulator primitives
- ``manip_7dof_timed_mprim.yaml`` - 7DOF with timing for multi-robot coordination



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

      # Configure multi-robot planner (add per-robot heuristic and mprim paths like tests)
      articulation_names = ["panda0", "panda1"]
      planner_context = {
         "planner_id": "xECBS",
         "weight_low_level_heuristic": "55.0",
         "high_level_focal_suboptimality": "1.8",
         "low_level_focal_suboptimality": "1.0",
      }
      for name in articulation_names:
         planner_context[f"heuristic_{name}"] = "joint_euclidean_remove_time"
         planner_context[f"mprim_path_{name}"] = "/path/to/config/manip_7dof_timed_mprim.yaml"

      planner.make_planner(articulation_names=articulation_names, planner_context=planner_context)

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

