Usage
=====

.. _installation:

Installation
------------

SRMP supports Ubuntu>=22.04 and Python 3.9, 3.10, 3.11, 3.12, and 3.13.

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

.. note::

   You can download pre-configured robot models (URDF/SRDF files) from our :doc:`data_downloads` page.

Create a planner interface:

.. code-block:: python

   planner = srmp.PlannerInterface()

Add a robot model to the world:

.. code-block:: python

   planner.add_articulation(urdf_path="/path/to/panda.urdf",
                            srdf_path="/path/to/panda.srdf",
                            name="panda",
                            end_effector="panda_hand")

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

   # MHA* planner
   planner.make_planner(["panda"], {"planner_id": "MHAstar",
                                    "inadmissible_heuristics": "bfs",
                                    "w1": "100.",
                                    "w2": "100."})

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

.. note::

   Multi-robot URDF/SRDF configurations are available for download on the :doc:`data_downloads` page.

Add multiple robots to the world:

.. code-block:: python

   planner = srmp.PlannerInterface()

   # Add first robot
   planner.add_articulation(
       urdf_path="/path/to/panda0.urdf",
       srdf_path="/path/to/panda0.srdf",
       name="panda0",
       end_effector="panda0_hand"
   )

   # Add second robot
   planner.add_articulation(
       urdf_path="/path/to/panda1.urdf",
       srdf_path="/path/to/panda1.srdf",
       name="panda1",
       end_effector="panda1_hand"
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
       "heuristic_panda0": "joint_euclidean_remove_time",
       "heuristic_panda1": "joint_euclidean_remove_time",
   }
   planner.make_planner(articulation_names, planner_context)

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
