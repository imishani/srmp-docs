Usage
=====

.. _installation:

Installation
------------

SRMP supports Linux (Ubuntu>=22.04) and macOS (arm64), and Python 3.9 through 3.14.
Windows is not yet supported.

To use SRMP, first install it using pip:

.. code-block:: console

   (.venv) $ pip install srmp

.. note::

   If you encounter installation issues, see the `Troubleshooting Installation`_ section below.

Then, import it in your script and follow the instructions on the docs:

.. code-block:: python

   import srmp
   import numpy as np

Basic Single Robot Planning
---------------------------

Create a planner interface:

.. code-block:: python

   planner = srmp.PlannerInterface()

Add a robot model to the world. The easiest way is to use the :doc:`robot_registry`:

.. code-block:: python

   # Add robot by name - downloads automatically if needed
   planner.add_robot("panda")

   # Or with a custom articulation name
   planner.add_robot("panda", name="my_panda")

Alternatively, you can use ``add_articulation`` with explicit file paths:

.. note::

   You can download pre-configured robot models (URDF/SRDF files) from our :doc:`data_downloads` page,
   or use the robot registry which downloads them automatically.

.. code-block:: python

   # With SRDF (recommended when semantic info is available)
   planner.add_articulation(name="panda",
                            end_effector="panda_hand",
                            urdf_path="/path/to/panda.urdf",
                            srdf_path="/path/to/panda.srdf")

   # Or without SRDF (URDF only)
   planner.add_articulation(name="panda",
                            end_effector="panda_hand",
                            urdf_path="/path/to/panda.urdf")

Gripper Control
~~~~~~~~~~~~~~~

If a robot has a gripper, its finger joints can be classified separately from the arm move
group so the planner never has to reason about them. Pass ``gripper_joint_names`` when adding
the robot (registry robots like ``"yam"`` or ``"so101"`` set this automatically), then drive
the gripper with ``set_gripper_qpos`` instead of ``set_qpos``:

.. code-block:: python

   # Registry robots with a known gripper classify it automatically
   planner.add_robot("yam")

   # Or classify explicitly when using add_articulation / a custom robot
   planner.add_articulation(name="my_arm",
                            end_effector="tool0",
                            urdf_path="/path/to/my_arm.urdf",
                            gripper_joint_names=["left_finger_joint", "right_finger_joint"])

   # Query and drive the gripper independently of the arm
   gripper_joints = planner.get_gripper_joint_names("my_arm")
   planner.set_gripper_qpos("my_arm", [0.02, 0.02])  # e.g. open the gripper

   # set_qpos / plan() are unaffected — they only ever see the arm move-group joints
   move_group_joints = planner.get_move_group_joint_names("my_arm")

Jacobians, Velocity Control & Manipulability
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Beyond joint-space planning, SRMP exposes a link's Jacobian directly — useful for velocity
control, manipulability/singularity analysis, or anything else built on top of it:

.. code-block:: python

   planner.set_qpos("panda", start_qpos)

   # (6, move_group_dof) Jacobian at the current qpos, world frame
   J = planner.get_jacobian("panda", "panda_hand")

   # Or at an arbitrary move-group qpos, without touching the robot's real state
   J_at_q = planner.compute_jacobian("panda", some_other_qpos, "panda_hand")

   # Map a desired end-effector twist [linear; angular] to joint velocities via damped
   # least squares (damping=0 gives the plain Moore-Penrose pseudo-inverse solution)
   twist = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])  # 10 cm/s along world x
   qdot = planner.compute_joint_velocities("panda", "panda_hand", twist, damping=0.05)

   # Yoshikawa manipulability index (0 at a singularity, higher = more dexterous)
   manipulability = planner.get_manipulability("panda", "panda_hand")

Screw Motion Planning
~~~~~~~~~~~~~~~~~~~~~

``plan_screw`` plans a helical (screw) motion for a link — rotation about an axis combined
with translation along it — by closing the loop on the Jacobian at each small step (a
resolved-rate/velocity controller, not an IK solve), checking collision and joint limits
along the way. It's the natural fit for tasks like turning a valve, opening a hinged door,
or driving a screw/bolt:

.. code-block:: python

   # Pure rotation (pitch=0) about an explicit axis -- e.g. turning a valve 90 degrees
   traj = planner.plan_screw(
       "panda", "panda_hand",
       axis_point=[0.4, 0.0, 0.3],     # a point on the valve's rotation axis, world frame
       axis_direction=[0, 0, 1],       # axis points straight up
       angle=np.radians(90),
   )

   # Or drive straight to a target pose -- the screw connecting the current pose to it
   # is derived automatically
   target_pose = srmp.Pose(p=[0.4, 0.1, 0.35], q=[1, 0, 0, 0])
   traj = planner.plan_screw("panda", "panda_hand", end_pose=target_pose)

``start_qpos`` defaults to the robot's current qpos (:meth:`~srmp.PlannerInterface.get_qpos`)
if omitted, so you can chain screw motions or mix them with regular ``plan()`` calls without
tracking the qpos yourself. A nonzero ``pitch`` (linear distance per full revolution) turns
the same rotate-in-place motion into a true screw thread, e.g. for driving a bolt:

.. code-block:: python

   traj = planner.plan_screw(
       "panda", "panda_hand",
       axis_point=[0.4, 0.0, 0.3], axis_direction=[0, 0, 1],
       pitch=0.002,                     # 2mm of travel per full revolution
       angle=np.radians(720),           # two full turns
   )

``plan_screw`` raises ``RuntimeError`` on collision, a joint-limit violation, a kinematic
singularity (stalled progress), or exceeding ``max_steps`` — wrap it in ``try``/``except``
if the motion might be infeasible. See :meth:`~srmp.PlannerInterface.plan_screw` for the
full parameter reference.

Add objects to the environment:

.. code-block:: python

   # Add a box obstacle
   obstacle_pose = srmp.Pose()
   obstacle_pose.p = np.array([0.5, 0.2, 0.4])
   obstacle_size = np.array([0.1, 0.1, 0.4])
   planner.add_box("box", obstacle_size, obstacle_pose)

   # Add a mesh
   mesh_pose = srmp.Pose()
   mesh_pose.p = np.array([0.3, 0.3, 0.5])
   planner.add_mesh("mesh_object", mesh_path="/path/to/mesh.stl",
                    scale=np.array([1.0, 1.0, 1.0]), pose=mesh_pose)

   # Add a point cloud
   # Generate sample point cloud data (Nx3 array)
   point_cloud = np.random.rand(1000, 3) * 0.5 + np.array([0.2, 0.2, 0.3])
   planner.add_point_cloud("point_cloud_obstacle", point_cloud, resolution=0.02)

Define the start configuration:

.. code-block:: python

   start_state = np.array([50, 47, -10, -35, -22, 93, 39])
   start_state = np.radians(start_state)

Define goal constraints:

.. code-block:: python

   # Goal as joint angles
   goal_state = np.array([21, 29, -30, -104, -162, 52, -118])
   goal_state = np.radians(goal_state)
   goal_joints = srmp.GoalConstraint(srmp.GoalType.JOINTS, [goal_state])

   # Goal as end-effector pose
   goal_pose = srmp.Pose()
   goal_pose.p = np.array([0.642, -0.068, 0.505])
   goal_pose.q = np.array([0.0, 0.0, 0.0, 1.0])  # normalized quaternion
   goal_ee = srmp.GoalConstraint(srmp.GoalType.POSE, [goal_pose])

Configure the planner:

.. code-block:: python

   # Available planners and configurations
   planner.print_available_planners()

   # wAstar planner with BFS heuristic
   planner.make_planner(["panda"], {"planner_id": "wAstar",
                                    "heuristic": "bfs",
                                    "weight": "10."})

   # ARA* planner with parameters
   planner.make_planner(["panda"], {"planner_id": "ARAstar",
                                    "heuristic": "bfs",
                                    "weight": "10.",
                                    "weight_delta": "1.",
                                    "final_weight": "1."})

   # PA*SE
   planner.make_planner(["panda"], {"planner_id": "wPASE",
                                    "heuristic": "bfs", "weight": "50.", "time_limit": "5.",
                                    "num_threads": "8"})


Compute a trajectory:

.. code-block:: python

   trajectory = planner.plan(start_state, goal_joints)
   # or
   trajectory = planner.plan(start_state, goal_ee)

   # Access trajectory data
   print(f"Trajectory length: {len(trajectory.positions)}")
   for i, position in enumerate(trajectory.positions):
       print(f"Step {i}: {position}")

Multi-Robot Planning
--------------------

Add multiple robots to the world. You can use the registry with custom names:

.. code-block:: python

   planner = srmp.PlannerInterface()

   # Add robots using the registry
   planner.add_robot("panda", name="panda0")
   planner.add_robot("panda", name="panda1")

Or use explicit file paths with multi-robot URDF configurations:

.. note::

   Multi-robot URDF/SRDF configurations are available for download on the :doc:`data_downloads` page.

.. code-block:: python

   planner = srmp.PlannerInterface()

   # Add first robot
   planner.add_articulation(
       name="panda0",
       end_effector="panda0_hand",
       urdf_path="/path/to/panda0.urdf",
       srdf_path="/path/to/panda0.srdf"
   )

   # Add second robot
   planner.add_articulation(
       name="panda1",
       end_effector="panda1_hand",
       urdf_path="/path/to/panda1.urdf",
       srdf_path="/path/to/panda1.srdf"
   )

Set base poses for robots:

.. code-block:: python

   # Set base pose for panda0
   pose0 = srmp.Pose()
   pose0.p = np.array([-0.5, 0.5, 0])
   pose0.q = np.array([1, 0, 0, 0])
   planner.set_base_pose("panda0", pose0)

   # Set base pose for panda1
   pose1 = srmp.Pose()
   pose1.p = np.array([0.5, 0.3, 0])
   pose1.q = np.array([0, 0, 0, 1])
   planner.set_base_pose("panda1", pose1)

Configure multi-robot planner:

.. code-block:: python

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

Define start and goal states for multiple robots:

.. code-block:: python

   start_states = {
       "panda0": np.radians([-40, 0, 0, -85, 0, 57, 0]),
       "panda1": np.radians([-40, 0, 0, -85, 0, 57, 0])
   }

   goal_states = {
       "panda0": np.radians([40, 0, 0, -70, 0, 50, 0]),
       "panda1": np.radians([40, 0, 0, -95, 0, 67, 0])
   }

   # Create goal constraints
   goal_constraints = {}
   for art_name in goal_states.keys():
       goal_constraints[art_name] = srmp.GoalConstraint(srmp.GoalType.JOINTS, [goal_states[art_name]])

Plan for multiple robots:

.. code-block:: python

   trajectories = planner.plan_multi(start_states, goal_constraints)

   # Access individual robot trajectories
   for robot_name, trajectory in trajectories.items():
       print(f"Robot {robot_name}: {len(trajectory.positions)} waypoints")

Simulator Integration
---------------------

SRMP supports integration with multiple simulators. The planner can automatically read collision objects from the simulation environment:

Genesis Integration:

.. code-block:: python

   import genesis as gs

   # Create Genesis scene
   scene = gs.Scene()
   # ... add objects to scene ...

   # Read objects from Genesis
   planner.read_sim(scene, "genesis")

PyBullet Integration:

.. code-block:: python

   import pybullet as p

   # Create PyBullet simulation
   physics_client = p.connect(p.GUI)
   # ... add objects to simulation ...

   # Read objects from PyBullet (exclude articulated bodies)
   planner.read_sim(physics_client, "pybullet", articulations=["panda"])

SAPIEN Integration:

.. code-block:: python

   import sapien

   # Create SAPIEN scene
   scene = sapien.Scene()
   # ... add objects to scene ...

   # Read objects from SAPIEN
   planner.read_sim(scene, "sapien")

Available Planners
------------------

SRMP provides several search-based planning algorithms:

- **wAstar**: Weighted A* - Fast single-goal planning
- **ARAstar**: Anytime Repairing A* - Iteratively improves solution quality
- **MHAstar**: Multi-heuristic A* - Uses multiple heuristics for better performance
- **wPASE**: Weighted PASE - Parallel search for improved performance
- **Astar**: Standard A* - Optimal but potentially slower
- **E-CBS**: Enhanced Conflict-Based Search - For multi-robot coordination
- **xECBS**: Experience Accelerated Conflict-Based Search - For multi-robot coordination

You can view available planners programmatically:

.. code-block:: python

   planner.print_available_planners()

Environment Management
----------------------

Remove objects from the environment:

.. code-block:: python

   planner.remove_object("box")

Supported geometric primitives:

- **Boxes**: `add_box(name, size, pose)`
- **Spheres**: `add_sphere(name, radius, pose)`
- **Cylinders**: `add_cylinder(name, radius, height, pose)`
- **Meshes**: `add_mesh(name, mesh_path, scale, pose)`
- **Point Clouds**: `add_point_cloud(name, vertices, resolution)`

See the :doc:`API <api>` for detailed method signatures and additional functionality.

Troubleshooting Installation
-----------------------------

**Conda Environment Issues**

If you're using conda and encounter C++ library compatibility issues:

.. code-block:: console

   $ conda install -c conda-forge libstdcxx-ng

**Missing NumPy**

If you get import errors related to NumPy:

.. code-block:: console

   $ pip install numpy
