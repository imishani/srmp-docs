Your First Plan
===============

This page plans a motion for a Franka Panda around a box, first to a joint configuration and
then to a gripper pose, and shows it in the browser. Each block is a complete script.

Plan to a joint configuration
-----------------------------

.. code-block:: python

   import numpy as np
   import srmp

   planner = srmp.PlannerInterface()
   planner.add_robot("panda")  # downloaded from the robot registry on first use

   # A box in front of the robot
   planner.add_box("box", size=[0.1, 0.4, 0.4], pose=srmp.Pose(p=[0.45, 0.0, 0.2]))

   planner.make_planner(["panda"], {"planner_id": "wAstar", "heuristic": "bfs", "weight": "50"})

   start = np.radians([-60, 30, 0, -100, 0, 130, 45])
   goal = srmp.GoalConstraint(srmp.GoalType.JOINTS, [np.radians([60, 30, 0, -100, 0, 130, 45])])

   traj = planner.plan(start, goal)
   print(f"{len(traj.positions)} waypoints, {traj.times[-1]:.2f} s")

What each step does:

- :class:`~srmp.PlannerInterface` holds the world: robots, obstacles and the planner.
- :meth:`~srmp.PlannerInterface.add_robot` adds a robot from the
  :doc:`robot registry <../user_guide/robot_registry>`. Your own URDF works too, with
  :meth:`~srmp.PlannerInterface.add_articulation`.
- :meth:`~srmp.PlannerInterface.make_planner` picks the algorithm and its settings for the
  listed robots. Every value is a string. Weighted A* with the ``bfs`` heuristic is a good
  default; :doc:`../guides/choosing_a_planner` covers the others.
- :meth:`~srmp.PlannerInterface.plan` returns a :class:`~srmp.Trajectory`: ``positions``,
  ``velocities``, ``accelerations`` and ``times``, already smoothed and timed within the
  robot's joint limits (see :doc:`../guides/postprocessing`). If no plan is found, the
  trajectory is empty (``len(traj.positions) == 0``) and the console says why.

Plan to a gripper pose
----------------------

A pose goal names where the end effector should be; the planner finds the joint values.
Positions are in metres, quaternions are ``[w, x, y, z]``:

.. code-block:: python

   import numpy as np
   import srmp

   planner = srmp.PlannerInterface()
   planner.add_robot("panda")
   planner.add_box("box", size=[0.1, 0.4, 0.4], pose=srmp.Pose(p=[0.45, 0.0, 0.2]))
   planner.make_planner(["panda"], {"planner_id": "wAstar", "heuristic": "bfs", "weight": "50"})

   start = np.radians([-60, 30, 0, -100, 0, 130, 45])
   target = srmp.Pose(p=[0.4, 0.3, 0.5], q=[0, 1, 0, 0])  # gripper pointing down
   goal = srmp.GoalConstraint(srmp.GoalType.POSE, [target])

   traj = planner.plan(start, goal)
   planner.set_qpos("panda", traj.positions[-1])
   print(planner.get_link_pose("panda", "panda_hand").p)  # close to [0.4, 0.3, 0.5]

Watch it
--------

:meth:`~srmp.PlannerInterface.start_visualizer` opens a 3D view in the browser that follows
the planner's world:

.. code-block:: python

   import numpy as np
   import srmp

   planner = srmp.PlannerInterface()
   planner.add_robot("panda")
   planner.add_box("box", size=[0.1, 0.4, 0.4], pose=srmp.Pose(p=[0.45, 0.0, 0.2]))
   planner.make_planner(["panda"], {"planner_id": "wAstar", "heuristic": "bfs", "weight": "50"})

   start = np.radians([-60, 30, 0, -100, 0, 130, 45])
   goal = srmp.GoalConstraint(srmp.GoalType.JOINTS, [np.radians([60, 30, 0, -100, 0, 130, 45])])
   traj = planner.plan(start, goal)

   viz = planner.start_visualizer(type="viser", port=8080)
   print("Open", viz.url)
   viz.animate_trajectory(traj)
   input("Press Enter to exit")
   viz.stop()

The viewer also has controls to move the robot, set goals and plan by clicking; see
:doc:`../user_guide/visualization`.

Next steps
----------

- :doc:`../examples/single_robot`: obstacles, pick-and-place, point clouds.
- :doc:`../examples/multi_robot`: several arms planned together.
- :doc:`../concepts/how_srmp_plans`: what the planner does, and which settings matter.
- :doc:`../user_guide/agent_mode`: describe a task in plain language and let the agent write
  the code.
