Single Robot Planning Examples
==============================

This section provides comprehensive examples for single robot motion planning with SRMP.

Basic Planning Example
----------------------

Here's a simple example showing the complete workflow for planning a trajectory for a single Panda robot:

.. code-block:: python

   import srmp
   import numpy as np
   import time

   # Create planner interface
   planner = srmp.PlannerInterface()

   # Add robot to the scene (downloads automatically if needed)
   planner.add_robot("panda")

   # Or use explicit paths:
   # planner.add_articulation(
   #     urdf_path="/path/to/panda.urdf",
   #     srdf_path="/path/to/panda.srdf",
   #     name="panda",
   #     end_effector="panda_hand"
   # )

   # Define start and goal configurations
   start_state = np.array([50, 47, -10, -35, -22, 93, 39])
   start_state = np.radians(start_state)

   goal_state = np.array([21, 29, -30, -104, -162, 52, -118])
   goal_state = np.radians(goal_state)

   # Create goal constraint
   goal_constraint = srmp.GoalConstraint(srmp.GoalType.JOINTS, [goal_state])

   # Configure planner
   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10."
   })

   # Plan trajectory
   start_time = time.time()
   trajectory = planner.plan(start_state, goal_constraint)
   planning_time = time.time() - start_time

   print(f"Planning completed in {planning_time:.3f} seconds")
   print(f"Trajectory has {len(trajectory.positions)} waypoints")

Planning with Obstacles
-----------------------

This example shows how to add obstacles to the environment and plan around them:

.. code-block:: python

   import srmp
   import numpy as np

   # Create planner and add robot
   planner = srmp.PlannerInterface()
   planner.add_robot("panda")

   # Add various obstacles
   # Box obstacle
   box_pose = srmp.Pose()
   box_pose.p = np.array([0.5, 0.2, 0.4])
   planner.add_box("box_obstacle", np.array([0.1, 0.1, 0.4]), box_pose)

   # Sphere obstacle
   sphere_pose = srmp.Pose()
   sphere_pose.p = np.array([0.3, -0.3, 0.6])
   planner.add_sphere("sphere_obstacle", 0.1, sphere_pose)

   # Cylinder obstacle
   cylinder_pose = srmp.Pose()
   cylinder_pose.p = np.array([-0.2, 0.4, 0.5])
   planner.add_cylinder("cylinder_obstacle", 0.05, 0.3, cylinder_pose)

   # Configure planner for obstacle avoidance
   planner.make_planner(["panda"], {
       "planner_id": "ARAstar",
       "heuristic": "bfs",
       "weight": "10.",
       "weight_delta": "1.",
       "final_weight": "1."
   })

   # Plan with obstacles
   start_state = np.radians([50, 47, -10, -35, -22, 93, 39])

   # Goal as end-effector pose
   goal_pose = srmp.Pose()
   goal_pose.p = np.array([0.642, -0.068, 0.505])
   goal_pose.q = np.array([0.0, 0.0, 0.0, 1.0])
   goal_constraint = srmp.GoalConstraint(srmp.GoalType.POSE, [goal_pose])

   trajectory = planner.plan(start_state, goal_constraint)
   print(f"Planned trajectory with {len(trajectory.positions)} waypoints")

Different Planner Algorithms
-----------------------------

SRMP supports various planning algorithms. Here's how to use different planners:

wAstar (Weighted A*)
~~~~~~~~~~~~~~~~~~~~

Fast planning with suboptimal solutions:

.. code-block:: python

   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "joint_euclidean",
       "weight": "50."  # Higher weight = faster but less optimal
   })

ARAstar (Anytime Repairing A*)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Iteratively improves solution quality:

.. code-block:: python

   planner.make_planner(["panda"], {
       "planner_id": "ARAstar",
       "heuristic": "bfs",
       "weight": "10.",
       "weight_delta": "1.",   # Weight reduction per iteration
       "final_weight": "1."    # Final weight (1.0 = optimal)
   })

MHAstar (Multi-Heuristic A*)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Uses multiple heuristics for better performance:

.. code-block:: python

   planner.make_planner(["panda"], {
       "planner_id": "MHAstar",
       "inadmissible_heuristics": "bfs",
       "w1": "100.",  # Weight for anchor heuristic
       "w2": "100."   # Weight for inadmissible heuristics
   })

wPASE (Weighted Parallel A* Search)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Parallel search for improved performance:

.. code-block:: python

   planner.make_planner(["panda"], {
       "planner_id": "wPASE",
       "heuristic": "bfs",
       "weight": "50.",
       "num_threads": "8"  # Number of parallel threads
   })

Planning to End-Effector Poses
-------------------------------

This example shows how to plan to specific end-effector poses rather than joint configurations:

.. code-block:: python

   import srmp
   import numpy as np

   # Setup planner and robot
   planner = srmp.PlannerInterface()
   planner.add_robot("panda")

   # Configure planner
   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10."
   })

   # Start configuration
   start_state = np.radians([0, -45, 0, -135, 0, 90, 45])

   # Goal as 6DOF pose
   goal_pose = srmp.Pose()
   goal_pose.p = np.array([0.6, -0.1, 0.5])    # Position [x, y, z]
   goal_pose.q = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion [x, y, z, w]

   # Normalize quaternion to be safe
   goal_pose.q = goal_pose.q / np.linalg.norm(goal_pose.q)

   goal_constraint = srmp.GoalConstraint(srmp.GoalType.POSE, [goal_pose])

   # Plan to pose
   trajectory = planner.plan(start_state, goal_constraint)

   print(f"Planned trajectory to pose: {goal_pose.p}")
   print(f"Trajectory length: {len(trajectory.positions)}")


.. _collision-aware-ik:

Collision-Aware IK
------------------

Sometimes you want the *joint configuration* behind a pose rather than a plan to it — to
seed another routine, to check a grasp is achievable before committing to it, or to turn a
pose goal into a joint goal. :meth:`~srmp.PlannerInterface.compute_ik` answers that
kinematically, but it does not look at the planning world and will return configurations
that drive the arm through obstacles.
:meth:`~srmp.PlannerInterface.collision_aware_ik` samples IK solutions with escalating
random restarts and returns only ones that are collision-free:

.. code-block:: python

   import srmp
   import numpy as np

   planner = srmp.PlannerInterface()
   planner.add_robot("panda")
   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10."
   })

   start_state = np.radians([0, -45, 0, -135, 0, 90, 45])
   planner.set_qpos("panda", start_state)

   # A shelf right above the target -- plain compute_ik would happily solve into it
   shelf_pose = srmp.Pose()
   shelf_pose.p = np.array([0.55, -0.1, 0.62])
   planner.add_box("shelf", np.array([0.4, 0.6, 0.02]), shelf_pose)

   target_pose = srmp.Pose()
   target_pose.p = np.array([0.55, -0.1, 0.45])
   target_pose.q = np.array([0.0, 1.0, 0.0, 0.0])

   q, status = planner.collision_aware_ik("panda", target_pose, start_state)

   if status == "found":
       # Turn the pose into a joint goal -- the planner now searches to a configuration
       # already known to be reachable and collision-free
       goal = srmp.GoalConstraint(srmp.GoalType.JOINTS, [q])
       trajectory = planner.plan(start_state, goal)
       print(f"Planned {len(trajectory.positions)} waypoints to the IK solution")
   elif status == "blocked":
       print("Reachable, but every IK solution collides -- clear the shelf or re-grasp")
   else:  # "unreachable"
       print("No IK solution exists -- move the base or pick a different target")

Distinguishing ``'blocked'`` from ``'unreachable'`` is the point of the ``status`` return: a
bare failure flag cannot tell you whether to move the obstacle or move the robot.

The search escalates its seeds instead of jumping straight to random restarts — first
``q_seed`` itself, then ``perturbation_steps`` attempts with growing Gaussian noise (up to
``perturbation_sigma_max`` radians), then uniform samples within the joint limits. That
biases the answer toward configurations near ``q_seed``. By default it returns the first
collision-free solution found within ``timeout``; ``best_of=True`` spends the whole budget
and returns the one closest to ``q_seed`` instead:

.. code-block:: python

   # A hard pose in a cluttered scene: search longer, stay near the current configuration
   q, status = planner.collision_aware_ik(
       "panda", target_pose, planner.get_qpos("panda"),
       timeout=0.25,
       perturbation_steps=10,
       perturbation_sigma_max=0.5,
       best_of=True,
   )

On success the robot is left at the returned configuration, so you can chain straight into
:meth:`~srmp.PlannerInterface.plan_screw` or a gripper action without another
:meth:`~srmp.PlannerInterface.set_qpos`. On failure it is restored to the configuration it
held when the call started. Attached visualizers see only that final state — not the
rejected candidates checked along the way.

Trajectory Analysis
-------------------

After planning, you can analyze and visualize the resulting trajectory:

.. code-block:: python

   import srmp
   import numpy as np
   import matplotlib.pyplot as plt

   # ... (setup planner and plan trajectory as before) ...
   trajectory = planner.plan(start_state, goal_constraint)

   # Extract trajectory data
   positions = trajectory.positions
   num_waypoints = len(positions)

   print(f"Trajectory Statistics:")
   print(f"  Number of waypoints: {num_waypoints}")
   print(f"  Start configuration: {np.degrees(positions[0])}")
   print(f"  Goal configuration: {np.degrees(positions[-1])}")

   # Calculate joint ranges of motion
   joint_ranges = []
   for joint_idx in range(7):  # 7 joints for Panda
       joint_values = [pos[joint_idx] for pos in positions]
       joint_range = max(joint_values) - min(joint_values)
       joint_ranges.append(np.degrees(joint_range))
       print(f"  Joint {joint_idx+1} range: {joint_range:.2f} degrees")

   # Plot trajectory (optional - requires matplotlib)
   try:
       fig, axes = plt.subplots(7, 1, figsize=(10, 14))
       for joint_idx in range(7):
           joint_trajectory = [np.degrees(pos[joint_idx]) for pos in positions]
           axes[joint_idx].plot(joint_trajectory)
           axes[joint_idx].set_ylabel(f'Joint {joint_idx+1} (deg)')
           axes[joint_idx].grid(True)

       axes[-1].set_xlabel('Waypoint')
       plt.title('Joint Trajectories')
       plt.tight_layout()
       plt.show()
   except ImportError:
       print("matplotlib not available for plotting")

Pick-and-Place with a Gripper
------------------------------

Robots with a gripper (like ``yam``) classify their finger joints separately from the arm
move group, so the planner never has to reason about them. Open/close the gripper with
:meth:`~srmp.PlannerInterface.set_gripper_qpos` before and after planning the arm motion —
it never affects :meth:`~srmp.PlannerInterface.set_qpos`/:meth:`~srmp.PlannerInterface.plan`,
which only ever see the arm's move-group joints.

A realistic pick-and-place is usually two phases: a search-planned, obstacle-avoiding motion
to a *pre-grasp* pose, followed by a short, direct final approach into the object. That final
segment rarely needs a full search plan — it's typically a straight, controlled motion right
up to the grasp, often combined with a small final twist to align the fingers, which makes it
a natural fit for :meth:`~srmp.PlannerInterface.plan_screw` instead of another
:meth:`~srmp.PlannerInterface.plan` call:

.. code-block:: python

   import srmp
   import srmp.robots as robots
   import numpy as np

   # yam classifies its two gripper finger joints automatically (see the robot registry)
   planner = srmp.PlannerInterface()
   name = planner.add_robot("yam")

   gripper_joints = planner.get_gripper_joint_names(name)
   print(f"Gripper joints: {gripper_joints}")

   # Open the gripper before approaching the object
   planner.set_gripper_qpos(name, [0.02, 0.02])

   planner.make_planner([name], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10."
   })

   start_state = np.array(robots.get("yam").default_qpos)
   link_name = robots.get("yam").end_effector

   # Phase 1: search-plan to a pre-grasp pose, offset back from the object. The slight
   # orientation difference from the final grasp is optional -- it just makes phase 2 a
   # translate-and-twist rather than a straight line; both are supported.
   pre_grasp_pose = srmp.Pose()
   pre_grasp_pose.p = np.array([0.2, 0.0, 0.2])
   pre_grasp_pose.q = np.array([0.2588, 0.0, 0.0, 0.9659])  # 150 degrees about z
   pre_grasp_goal = srmp.GoalConstraint(srmp.GoalType.POSE, [pre_grasp_pose])

   trajectory = planner.plan(start_state, pre_grasp_goal)
   print(f"Reached pre-grasp in {len(trajectory.positions)} waypoints")
   planner.set_qpos(name, trajectory.positions[-1])

   # Phase 2: final approach with plan_screw -- a short translate-and-twist right up to
   # the object. start_qpos defaults to the robot's current qpos, so it picks up exactly
   # where phase 1 left off. Joint limits and collision are checked at every step; pass
   # collision_aware=False if closing on the object trips the collision check.
   grasp_pose = srmp.Pose()
   grasp_pose.p = np.array([0.3, 0.0, 0.2])
   grasp_pose.q = np.array([0.0, 0.0, 0.0, 1.0])  # 180 degrees about z

   approach = planner.plan_screw(name, link_name, end_pose=grasp_pose)
   print(f"Final approach in {len(approach.positions)} steps")
   planner.set_qpos(name, approach.positions[-1])

   # Close the gripper to grasp — arm move-group state is untouched
   planner.set_gripper_qpos(name, [0.0, 0.0])

If the grasp needs no reorientation, give ``end_pose`` the pre-grasp orientation and the
approach becomes a straight line — see :ref:`screw-translation-only`.

To check up front that the grasp pose is actually achievable in the current scene — and, if
it is not, whether the problem is reach or clutter — run
:meth:`~srmp.PlannerInterface.collision_aware_ik` on it before planning phase 1. See
:ref:`collision-aware-ik`.

Screw Motion: Turning a Valve
------------------------------

Tasks that rotate about (and optionally translate along) an axis — turning a valve,
opening a hinged door, driving a screw — are naturally described as a *screw motion*
rather than a single end-effector pose. :meth:`~srmp.PlannerInterface.plan_screw` walks
the end effector along that helical path directly, closing the loop on the Jacobian at
each small step (no IK/FK solve involved), checking joint limits at every step and — unless
you pass ``collision_aware=False`` — collision as well:

.. code-block:: python

   import srmp
   import numpy as np

   planner = srmp.PlannerInterface()
   planner.add_robot("panda")

   start_qpos = np.radians([0, -45, 0, -135, 0, 90, 45])
   planner.set_qpos("panda", start_qpos)

   # Rotate the wrist 90 degrees about a vertical axis through a point on the valve
   # (pitch=0 means pure rotation, like a valve or a hinged door)
   traj = planner.plan_screw(
       "panda", "panda_hand",
       axis_point=[0.4, 0.0, 0.3],
       axis_direction=[0, 0, 1],
       angle=np.radians(90),
   )
   print(f"Turned the valve in {len(traj.positions)} steps")

   # start_qpos defaults to the robot's current qpos, so a second screw motion can
   # continue right where the first left off without tracking qpos yourself
   traj2 = planner.plan_screw(
       "panda", "panda_hand",
       axis_point=[0.4, 0.0, 0.3],
       axis_direction=[0, 0, 1],
       angle=np.radians(90),
   )

A nonzero ``pitch`` turns the same rotate-in-place motion into a true screw thread —
useful for driving a bolt, where the end effector should advance along the axis as it
turns:

.. code-block:: python

   traj = planner.plan_screw(
       "panda", "panda_hand",
       axis_point=[0.4, 0.0, 0.3],
       axis_direction=[0, 0, 1],
       pitch=0.002,              # 2 mm of travel per full revolution
       angle=np.radians(720),    # two full turns
   )

You can also aim directly at a target pose and let ``plan_screw`` derive the connecting
screw automatically:

.. code-block:: python

   target_pose = planner.get_link_pose("panda", "panda_hand")
   target_pose.p += np.array([0.0, 0.1, 0.0])                  # 10 cm sideways
   target_pose.q = np.array([0.92388, 0.0, 0.0, 0.38268])      # and 45 degrees about z

   traj = planner.plan_screw("panda", "panda_hand", end_pose=target_pose)

.. _screw-translation-only:

Translation-only motion
~~~~~~~~~~~~~~~~~~~~~~~

A target pose that keeps the **current orientation** is a pure translation. No screw axis
exists for such a motion — any axis parallel to the travel direction reproduces it — so
``plan_screw`` handles it as a special case, taking the twist directly as
``[delta_p, 0]``. That makes it the straightforward way to run a straight-line move, with
no rotation to invent and no IK solve:

.. code-block:: python

   # Straight 10 cm sideways, orientation untouched
   target_pose = planner.get_link_pose("panda", "panda_hand")
   target_pose.p += np.array([0.0, 0.1, 0.0])

   traj = planner.plan_screw("panda", "panda_hand", end_pose=target_pose)

This is the usual shape of a final grasp approach: advance along the gripper's approach
axis without reorienting.

.. code-block:: python

   # Advance 5 cm along the tool's +z, keeping orientation
   ee_pose = planner.get_link_pose("panda", "panda_hand")
   approach_dir = ee_pose.to_transformation_matrix()[:3, 2]   # tool +z in world frame

   target_pose = planner.get_link_pose("panda", "panda_hand")
   target_pose.p += 0.05 * approach_dir

   traj = planner.plan_screw("panda", "panda_hand", end_pose=target_pose)

.. versionchanged:: 0.1.4.7

   Pure translations are supported. Earlier versions raised ``RuntimeError`` ("a screw
   motion requires a nonzero rotation") whenever ``end_pose`` shared the current
   orientation, which forced an artificial orientation offset on straight-line moves.

Closing on an object without aborting on contact
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A final approach deliberately drives the gripper *into* contact range, so the per-step
collision check can abort a motion that is doing exactly what you asked. Pass
``collision_aware=False`` to sweep the path on kinematics alone:

.. code-block:: python

   traj = planner.plan_screw(
       "panda", "panda_hand",
       end_pose=grasp_pose,
       collision_aware=False,
   )

Joint limits are still enforced. Only the collision check is dropped, so the returned
trajectory may pass through obstacles — keep the segment short and validate it yourself
if anything other than the grasp target is nearby.

.. versionadded:: 0.1.4.7
   The ``collision_aware`` parameter.

Wrap the call in ``try``/``except`` if the motion might be infeasible — ``plan_screw``
raises ``RuntimeError`` on collision (when ``collision_aware`` is left on), a joint-limit
violation, a kinematic singularity (stalled progress), or exceeding ``max_steps``:

.. code-block:: python

   try:
       traj = planner.plan_screw(
           "panda", "panda_hand",
           axis_direction=[0, 0, 1], angle=np.radians(720),
       )
   except RuntimeError as e:
       print(f"Screw motion infeasible: {e}")

Planning with Point Clouds
---------------------------

SRMP supports point cloud obstacles for sensor-based planning:

.. code-block:: python

   import srmp
   import numpy as np

   # Create planner and add robot
   planner = srmp.PlannerInterface()
   planner.add_robot("panda")

   # Generate sample point cloud (table surface)
   table_points = []
   for x in np.linspace(0.3, 0.7, 30):
       for y in np.linspace(-0.2, 0.2, 15):
           table_points.append([x, y, 0.4])  # Table at height 0.4m

   table_cloud = np.array(table_points)

   # Add point cloud with specified resolution
   planner.add_point_cloud("table", table_cloud, resolution=0.01)

   # Configure planner and plan trajectory
   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10.0"
   })

   start_state = np.radians([0, -30, 0, -120, 0, 90, 45])

   goal_pose = srmp.Pose()
   goal_pose.p = np.array([0.5, 0.0, 0.6])  # Above table
   goal_pose.q = np.array([0, 0, 0, 1])
   goal_constraint = srmp.GoalConstraint(srmp.GoalType.POSE, [goal_pose])

   trajectory = planner.plan(start_state, goal_constraint)

   if trajectory:
       print(f"Planned around point cloud: {len(trajectory.positions)} waypoints")

   # Load point cloud from file
   def load_point_cloud_from_file(filename):
       """Load point cloud from XYZ text file"""
       return np.loadtxt(filename, usecols=(0, 1, 2))

   # Usage: point_cloud = load_point_cloud_from_file("/path/to/points.txt")
   # planner.add_point_cloud("loaded_obstacles", point_cloud, resolution=0.02)