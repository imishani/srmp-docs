Kinematics: IK, Jacobians and Screw Motions
===========================================

Besides planning, the planner answers kinematic questions about any robot in it: where a link
is, which joint values put it somewhere, and how it moves for a small joint motion. All poses
are in the world frame, including for robots moved with
:meth:`~srmp.PlannerInterface.set_base_pose`.

Link poses
----------

.. code-block:: python

   import numpy as np
   import srmp

   planner = srmp.PlannerInterface()
   planner.add_robot("panda")
   start_qpos = np.radians([0, -45, 0, -135, 0, 90, 45])
   planner.set_qpos("panda", start_qpos)

   hand = planner.get_link_pose("panda", "panda_hand")   # at the current joint values
   print(hand.p, hand.q)                                 # metres, [w, x, y, z]

.. _collision-aware-ik:

Collision-aware IK
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

The ``status`` tells the failures apart:

- ``'found'``: ``q`` is a valid, collision-free configuration.
- ``'unreachable'``: IK never succeeded within ``timeout``; the pose is most likely
  kinematically infeasible for this arm. Move the base or pick a different target (or raise
  ``timeout`` if you expect it to be reachable).
- ``'blocked'``: IK reached the pose at least once, but every solution found was in
  collision. The pose *is* reachable; something is in the way.

Target poses are in the world frame, the frame :meth:`~srmp.PlannerInterface.compute_fk`
returns, for both ``compute_ik`` and ``collision_aware_ik``. That holds for a robot whose
base has been moved with :meth:`~srmp.PlannerInterface.set_base_pose` too, so an FK pose
can be fed straight back into IK.

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

Jacobians and velocity control
------------------------------

.. code-block:: python

   import numpy as np
   import srmp

   planner = srmp.PlannerInterface()
   planner.add_robot("panda")
   start_qpos = np.radians([0, -45, 0, -135, 0, 90, 45])
   planner.set_qpos("panda", start_qpos)

   # (6, move_group_dof) Jacobian at the current qpos, world frame, rows [linear; angular]
   J = planner.get_jacobian("panda", "panda_hand")

   # Or at another qpos, without moving the robot
   J_at_q = planner.compute_jacobian("panda", np.radians([10, -45, 0, -135, 0, 90, 45]), "panda_hand")

   # Joint velocities for an end-effector twist [linear; angular], by damped least squares
   # (damping=0 gives the plain pseudo-inverse)
   twist = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])  # 10 cm/s along world x
   qdot = planner.compute_joint_velocities("panda", "panda_hand", twist, damping=0.05)

   # Yoshikawa manipulability: 0 at a singularity, larger is more dexterous
   print(planner.get_manipulability("panda", "panda_hand"))

Screw motions
-------------

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
turns. How far one call can turn is bounded by the arm's joint limits (about half a turn
for the Panda here); a longer drive repeats shorter turns with a regrasp in between:

.. code-block:: python

   planner.set_qpos("panda", start_qpos)   # back to the start of the example
   traj = planner.plan_screw(
       "panda", "panda_hand",
       axis_point=[0.4, 0.0, 0.3],
       axis_direction=[0, 0, 1],
       pitch=0.01,               # 1 cm of travel per full revolution
       angle=np.radians(180),    # half a turn, so 5 mm of travel
   )

You can also aim directly at a target pose and let ``plan_screw`` derive the connecting
screw automatically:

.. code-block:: python

   planner.set_qpos("panda", start_qpos)
   target_pose = planner.get_link_pose("panda", "panda_hand")
   target_pose.p += np.array([0.0, 0.1, 0.0])                  # 10 cm sideways
   yaw_45 = srmp.Pose(q=[0.92388, 0.0, 0.0, 0.38268])          # 45 degrees about world z
   target_pose.q = (yaw_45 * target_pose).q                    # applied to the current orientation

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

   grasp_pose = planner.get_link_pose("panda", "panda_hand")
   grasp_pose.p += 0.03 * grasp_pose.to_transformation_matrix()[:3, 2]   # 3 cm forward

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

