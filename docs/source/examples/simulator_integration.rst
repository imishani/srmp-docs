Simulator Integration Examples
===============================

SRMP provides seamless integration with popular robotics simulators. This section demonstrates how to use SRMP with different simulation platforms.

SAPIEN Integration
------------------

SAPIEN is a high-performance physics simulation platform. Here's how to integrate SRMP with SAPIEN:

Basic SAPIEN Setup
~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import sapien
   import numpy as np

   # Create SAPIEN scene
   scene = sapien.Scene()
   scene.add_ground(0, render_material=[0.5, 0.5, 0.5])

   # Add lighting
   scene.set_ambient_light([0.5, 0.5, 0.5])
   scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)

   # Load robot using SAPIEN's URDF loader
   loader = scene.create_urdf_loader()
   loader.fix_root_link = True

   robot = loader.load("/path/to/panda.urdf")
   robot.set_root_pose(sapien.Pose([0, 0, 0.1], [1, 0, 0, 0]))

   # Disable gravity for planning
   for link in robot.links:
       link.disable_gravity = True

   # Set initial configuration
   q_initial = np.radians([0, -45, 0, -135, 0, 90, 45])
   qpos = robot.get_qpos()
   qpos[:7] = q_initial
   robot.set_qpos(qpos)

   # Create SRMP planner
   planner = srmp.PlannerInterface()
   planner.add_articulation(
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",
       name="panda",
       end_effector="panda_hand"
   )

   # Automatically import scene objects
   planner.read_sim(scene, "sapien")

   print("SAPIEN scene imported successfully")

Adding Objects to SAPIEN Scene
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import sapien
   import numpy as np

   # Create scene
   scene = sapien.Scene()
   scene.add_ground()

   # Add various objects that will become obstacles
   builder = scene.create_actor_builder()

   # Box obstacle
   builder.add_box_collision(half_size=[0.1, 0.1, 0.1])
   builder.add_box_visual(half_size=[0.1, 0.1, 0.1], material=[0.8, 0.2, 0.2])
   box = builder.build_kinematic()
   box.set_pose(sapien.Pose([0.5, 0.0, 0.5]))

   # Sphere obstacle
   builder = scene.create_actor_builder()
   builder.add_sphere_collision(radius=0.08)
   builder.add_sphere_visual(radius=0.08, material=[0.2, 0.8, 0.2])
   sphere = builder.build_kinematic()
   sphere.set_pose(sapien.Pose([0.3, -0.3, 0.6]))

   # Cylinder obstacle
   builder = scene.create_actor_builder()
   builder.add_cylinder_collision(radius=0.05, half_length=0.15)
   builder.add_cylinder_visual(radius=0.05, half_length=0.15, material=[0.2, 0.2, 0.8])
   cylinder = builder.build_kinematic()
   cylinder.set_pose(sapien.Pose([-0.2, 0.4, 0.65]))

   # Load robot
   loader = scene.create_urdf_loader()
   loader.fix_root_link = True
   robot = loader.load("/path/to/panda.urdf")
   robot.set_root_pose(sapien.Pose([0, 0, 0.1]))

   # Create planner and import scene
   planner = srmp.PlannerInterface()
   planner.add_articulation(
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",
       name="panda",
       end_effector="panda_hand"
   )

   # Import all collision objects automatically
   planner.read_sim(scene, "sapien")

   # Plan trajectory
   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10.0"
   })

   start_state = np.radians([0, -45, 0, -135, 0, 90, 45])
   goal_state = np.radians([45, -30, 0, -120, 0, 90, 0])
   goal = srmp.GoalConstraint(srmp.GoalType.JOINTS, goal_state)

   trajectory = planner.plan(start_state, goal)
   print(f"Planned trajectory with {len(trajectory.positions)} waypoints")

Executing Trajectory in SAPIEN
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import sapien
   import numpy as np
   import time

   # ... (setup scene and plan trajectory as above) ...

   # Create viewer for visualization
   viewer = scene.create_viewer()
   viewer.set_camera_xyz(-2, 0, 1.5)
   viewer.set_camera_rpy(0, -0.5, 0)

   # Execute planned trajectory
   for i, waypoint in enumerate(trajectory.positions):
       # Set robot configuration
       qpos = robot.get_qpos()
       qpos[:7] = waypoint
       robot.set_qpos(qpos)

       # Step simulation
       scene.step()
       scene.update_render()
       viewer.render()

       # Control playback speed
       time.sleep(0.1)

       # Allow user to quit
       if viewer.window.key_down('q'):
           break

   # Keep viewer open
   while not viewer.closed:
       scene.step()
       viewer.render()

PyBullet Integration
--------------------

PyBullet is a popular open-source physics simulator. Here's how to use it with SRMP:

Basic PyBullet Setup
~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import pybullet as p
   import pybullet_data
   import numpy as np

   # Connect to PyBullet
   physics_client = p.connect(p.GUI)  # Use p.DIRECT for headless
   p.setAdditionalSearchPath(pybullet_data.getDataPath())
   p.setGravity(0, 0, -9.8)

   # Load ground plane
   plane_id = p.loadURDF("plane.urdf")

   # Load robot
   robot_id = p.loadURDF(
       "/path/to/panda.urdf",
       basePosition=[0, 0, 0.1],
       baseOrientation=[0, 0, 0, 1],
       useFixedBase=True
   )

   # Add obstacles
   # Box obstacle
   box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
   box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1],
                                    rgbaColor=[0.8, 0.2, 0.2, 1])
   box_id = p.createMultiBody(
       baseCollisionShapeIndex=box_collision,
       baseVisualShapeIndex=box_visual,
       basePosition=[0.5, 0, 0.5]
   )

   # Sphere obstacle
   sphere_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.08)
   sphere_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.08,
                                       rgbaColor=[0.2, 0.8, 0.2, 1])
   sphere_id = p.createMultiBody(
       baseCollisionShapeIndex=sphere_collision,
       baseVisualShapeIndex=sphere_visual,
       basePosition=[0.3, -0.3, 0.6]
   )

   # Create SRMP planner
   planner = srmp.PlannerInterface()
   planner.add_articulation(
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",
       name="panda",
       end_effector="panda_hand"
   )

   # Import PyBullet scene (exclude robot from collision checking)
   planner.read_sim(physics_client, "pybullet", articulations=["panda"])

   print("PyBullet scene imported successfully")

PyBullet Multi-Robot Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import pybullet as p
   import pybullet_data
   import numpy as np

   # Setup PyBullet
   physics_client = p.connect(p.GUI)
   p.setAdditionalSearchPath(pybullet_data.getDataPath())
   p.setGravity(0, 0, 0)  # No gravity for planning

   # Load two robots
   robot0_id = p.loadURDF(
       "/path/to/panda0.urdf",
       basePosition=[-0.5, 0, 0.1],
       baseOrientation=[0, 0, 0, 1],
       useFixedBase=True
   )

   robot1_id = p.loadURDF(
       "/path/to/panda1.urdf",
       basePosition=[0.5, 0, 0.1],
       baseOrientation=[0, 0, 1, 0],  # 180-degree rotation
       useFixedBase=True
   )

   # Add shared workspace obstacles
   obstacles = []
   for i, pos in enumerate([[0, 0.4, 0.8], [0, -0.4, 0.6]]):
       collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.1])
       visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.1])
       obstacle_id = p.createMultiBody(
           baseCollisionShapeIndex=collision,
           baseVisualShapeIndex=visual,
           basePosition=pos
       )
       obstacles.append(obstacle_id)

   # Setup SRMP planner
   planner = srmp.PlannerInterface()

   # Add both robots
   for i in range(2):
       planner.add_articulation(
           urdf_path=f"/path/to/panda{i}.urdf",
           srdf_path=f"/path/to/panda{i}.srdf",
           name=f"panda{i}",
           end_effector=f"panda{i}_hand"
       )

   # Set base poses to match PyBullet
   pose0 = srmp.Pose()
   pose0.p = np.array([-0.5, 0, 0])
   pose0.q = np.array([0, 0, 0, 1])
   planner.set_base_pose("panda0", pose0)

   pose1 = srmp.Pose()
   pose1.p = np.array([0.5, 0, 0])
   pose1.q = np.array([0, 0, 1, 0])
   planner.set_base_pose("panda1", pose1)

   # Import scene obstacles
   planner.read_sim(physics_client, "pybullet", articulations=["panda0", "panda1"])

   # Configure multi-robot planner
   planner.make_planner(
       ["panda0", "panda1"],
       {
           "planner_id": "xECBS",
           "weight_low_level_heuristic": "30.0",
           "high_level_focal_suboptimality": "1.5",
           "low_level_focal_suboptimality": "1.0",
           "heuristic_panda0": "joint_euclidean_remove_time",
           "heuristic_panda1": "joint_euclidean_remove_time"
       }
   )

   # Plan coordinated motion
   start_states = {
       "panda0": np.radians([30, -45, 0, -90, 0, 45, 0]),
       "panda1": np.radians([-30, -45, 0, -90, 0, 45, 0])
   }

   goal_states = {
       "panda0": np.radians([-30, -45, 0, -90, 0, 45, 0]),
       "panda1": np.radians([30, -45, 0, -90, 0, 45, 0])
   }

   goal_constraints = {}
   for name in ["panda0", "panda1"]:
       goal_constraints[name] = srmp.GoalConstraint(
           srmp.GoalType.JOINTS, [goal_states[name]]
       )

   trajectories = planner.plan_multi(start_states, goal_constraints)

   if trajectories:
       print("Multi-robot PyBullet planning successful!")
       # Execute trajectories...

Genesis Integration
-------------------

Genesis is a high-performance physics simulation platform with advanced rendering capabilities:

Genesis Setup and Planning
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import genesis as gs
   import numpy as np

   # Initialize Genesis
   gs.init(backend=gs.cpu)

   # Create scene
   sim_options = gs.options.SimOptions(gravity=(0, 0, 0))  # No gravity for planning
   vis_options = gs.options.VisOptions(show_world_frame=False)
   scene = gs.Scene(show_viewer=True, sim_options=sim_options, vis_options=vis_options)

   # Add ground plane
   plane = scene.add_entity(gs.morphs.Plane())

   # Add base platform
   base = scene.add_entity(
       gs.morphs.Box(
           size=(2, 2, 0.001),
           pos=(0, 0, 0.05),
           fixed=True
       )
   )

   # Add obstacles as meshes
   crate_mesh = gs.morphs.Mesh(
       file='/path/to/crate.stl',
       scale=(0.01, 0.01, 0.01),
       pos=(0.53, 0.1, 0.1),
       quat=(0.7073883, 0, 0, 0.7068252),
       fixed=True,
       convexify=False
   )
   crate1 = scene.add_entity(crate_mesh)

   # Add more crates at different positions
   positions = [(0.13, 0.1, 0.1), (-0.27, 0.1, 0.1), (-0.67, 0.1, 0.1)]
   colors = [(1, 1, 0.94, 1), (220/255, 205/255, 152/255, 1), (0.24, 0.26, 0.33, 1)]

   for pos, color in zip(positions, colors):
       crate = scene.add_entity(
           gs.morphs.Mesh(
               file='/path/to/crate.stl',
               scale=(0.01, 0.01, 0.01),
               pos=pos,
               quat=(0.7073883, 0, 0, 0.7068252),
               fixed=True,
               convexify=False
           )
       )

   # Add robot
   robot = scene.add_entity(
       gs.morphs.MJCF(
           file='xml/franka_emika_panda/panda.xml',
           pos=[0, 0.4, 0.1],
           quat=[0.7073883, 0, 0, -0.7068252]
       )
   )

   # Build scene
   scene.build()

   # Create SRMP planner
   planner = srmp.PlannerInterface()
   planner.add_articulation(
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",
       name="panda",
       end_effector="panda_hand"
   )

   # Set robot base pose to match Genesis
   robot_pose = srmp.Pose()
   robot_pose.p = np.array([0, 0.4, 0])
   robot_pose.q = np.array([0.7073883, 0, 0, -0.7068252])
   planner.set_base_pose("panda", robot_pose)

   # Import Genesis scene
   planner.read_sim(scene, "genesis")

   print("Genesis scene imported successfully")

Genesis Trajectory Execution with Recording
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import genesis as gs
   import numpy as np
   import time

   # ... (setup scene and plan trajectory as above) ...

   # Plan trajectory through multiple waypoints
   waypoints = [
       [11, 3, 14, -144, -1, 147, 26 + 45],
       [43, 47, 17, -76, -14, 120, 62 + 45],
       [-38, 38, -26, -94, 20, 127, -71 + 45],
       [-14, 3, -12, -143, 1, 147, -26 + 45],
       [11, 3, 14, -144, -1, 147, 26 + 45]
   ]

   # Plan trajectory segments
   all_trajectories = []
   for i in range(len(waypoints) - 1):
       start_state = np.radians(waypoints[i])
       goal_state = np.radians(waypoints[i + 1])
       goal = srmp.GoalConstraint(srmp.GoalType.JOINTS, [goal_state])

       trajectory = planner.plan(start_state, goal)
       all_trajectories.extend(trajectory.positions)

   # Setup camera for recording
   camera = scene.add_camera(
       res=[1280, 1280],
       pos=[0, 0.4, 4 * np.sin(np.deg2rad(30))],
       lookat=[0, 0, 0]
   )

   # Get joint indices for the robot
   joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
   dof_indices = [robot.get_joint(name).dof_idx_local for name in joint_names]

   # Execute trajectory with recording
   camera.start_recording()
   dt = 0.01

   for i, configuration in enumerate(all_trajectories):
       # Update camera position for circular motion
       radius = 4.0
       angle = -i * np.pi / len(all_trajectories)
       camera_x = radius / np.cos(np.deg2rad(45)) * np.cos(angle)
       camera_y = radius / np.cos(np.deg2rad(45)) * np.sin(angle)
       camera_z = 4 * np.sin(np.deg2rad(30))

       scene.viewer.set_camera_pose(
           pos=[camera_x, camera_y, camera_z],
           lookat=[0, 0, 0]
       )
       camera.set_pose(
           pos=[camera_x, camera_y, camera_z],
           lookat=[0, 0, 0]
       )

       # Set robot configuration
       robot.set_dofs_position(configuration, dof_indices)

       # Step simulation and render
       scene.step()
       camera.render()
       time.sleep(dt)

   # Stop recording and save
   camera.stop_recording(save_to_filename='/path/to/output/video.mp4', fps=10)

   # Keep viewer open
   while scene.viewer:
       scene.step()

MuJoCo Integration
------------------

MuJoCo integration for high-fidelity physics simulation:

.. code-block:: python

   import srmp
   import mujoco_py as mj
   import numpy as np

   # Load MuJoCo model
   model = mj.load_model_from_path("/path/to/scene.xml")
   sim = mj.MjSim(model)

   # Create viewer (optional)
   viewer = mj.MjViewer(sim)

   # Create SRMP planner
   planner = srmp.PlannerInterface()
   planner.add_articulation(
       urdf_path="/path/to/panda.urdf",
       srdf_path="/path/to/panda.srdf",
       name="panda",
       end_effector="panda_hand"
   )

   # Import MuJoCo scene
   planner.read_sim(sim, "mujoco")

   # Plan and execute trajectory
   # ... (similar to other examples) ...

