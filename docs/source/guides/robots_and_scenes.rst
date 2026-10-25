Robots and Scenes
=================

Everything the planner knows lives in one :class:`~srmp.PlannerInterface`: the robots, their
base poses and joint values, and the obstacles around them.

Adding robots
-------------

:meth:`~srmp.PlannerInterface.add_robot` adds a robot from the
:doc:`robot registry <../user_guide/robot_registry>`, downloading it on first use. Give each
copy its own ``name`` when you add the same robot twice:

.. code-block:: python

   import numpy as np
   import srmp

   planner = srmp.PlannerInterface()
   planner.add_robot("panda", name="left")
   planner.add_robot("panda", name="right")
   planner.set_base_pose("left", srmp.Pose(p=[0.0, 0.4, 0.0]))
   planner.set_base_pose("right", srmp.Pose(p=[0.0, -0.4, 0.0]))

For a robot that is not in the registry, use :meth:`~srmp.PlannerInterface.add_articulation`
with its URDF (and SRDF, when you have one: it lists the link pairs that never collide, which
speeds up checks):

.. code-block:: python

   planner.add_articulation(name="my_arm",
                            end_effector="tool0",
                            urdf_path="/path/to/my_arm.urdf",
                            srdf_path="/path/to/my_arm.srdf")

:doc:`../user_guide/robot_registry` explains how to register your own robots so
``add_robot`` finds them.

Grippers
~~~~~~~~

Finger joints are kept out of the planned joints, so the planner never moves them. Registry
robots with a gripper (``yam``, ``so101`` …) classify theirs automatically; for your own
robot, name them with ``gripper_joint_names``. Move the fingers with
:meth:`~srmp.PlannerInterface.set_gripper_qpos`:

.. code-block:: python

   planner.add_articulation(name="my_arm",
                            end_effector="tool0",
                            urdf_path="/path/to/my_arm.urdf",
                            gripper_joint_names=["left_finger_joint", "right_finger_joint"])

   planner.set_gripper_qpos("my_arm", [0.02, 0.02])   # open
   planner.get_move_group_joint_names("my_arm")       # the joints plan() moves

Obstacles
---------

.. code-block:: python

   planner.add_box("table", size=[0.8, 1.4, 0.05], pose=srmp.Pose(p=[0.5, 0.0, -0.03]))
   planner.add_sphere("ball", radius=0.05, pose=srmp.Pose(p=[0.5, 0.0, 0.3]))
   planner.add_cylinder("post", radius=0.03, length=0.4, pose=srmp.Pose(p=[0.7, 0.0, 0.2]))
   planner.add_point_cloud("scan", np.random.rand(500, 3) * [0.2, 0.2, 0.1] + [0.6, -0.1, 0.0])

   planner.update_object_pose("ball", srmp.Pose(p=[0.5, 0.0, 0.4]))
   planner.remove_object("post")

   print(planner.is_state_colliding())        # any collision in the world?
   print(planner.is_state_colliding("left"))  # just this robot

Sizes are full extents in metres; a pose is the object's centre. Meshes come from a file or
from vertex and triangle arrays: ``add_mesh(name, mesh_path=..., scale=..., pose=...)``. Boxes,
spheres, cylinders and meshes take an optional ``color``, used only by visualizers.

To copy a whole scene from a simulator instead, see :doc:`../examples/simulator_integration`
(:meth:`~srmp.PlannerInterface.read_sim`). To save a scene and load it later, see
:doc:`../user_guide/persistence`.

Holding objects
---------------

:meth:`~srmp.PlannerInterface.attach_object` makes an object move with a robot link. From
then on it is checked against the world, other robots and the robot's own links; the links
touching it when you attach it (the hand, the fingers) are allowed to touch it.

.. code-block:: python

   planner = srmp.PlannerInterface()
   planner.add_robot("panda")
   planner.set_qpos("panda", np.radians([0, -45, 0, -135, 0, 90, 45]))

   hand = planner.get_link_pose("panda", "panda_hand")
   cube = srmp.Pose(p=hand.p + hand.to_transformation_matrix()[:3, 2] * 0.1, q=hand.q)
   planner.add_box("cube", size=[0.04, 0.04, 0.04], pose=cube)

   planner.attach_object("cube", "panda", "panda_hand", is_link_name=True)
   # ... plan: the cube moves with the hand and is collision-checked ...
   planner.detach_object("cube")

Objects attached to the same link may touch each other; objects on different links or robots
may not. :meth:`~srmp.PlannerInterface.set_allowed_collision` allows any other pair to touch,
by object, link or robot name. :doc:`../examples/single_robot` has a full pick-and-place.

.. _planning-volume:

The planning volume
-------------------

Every obstacle is also voxelized into an occupancy grid, which the BFS heuristic searches. The
default grid is a 2 m cube at 2 cm resolution spanning ``x, y ∈ [-1, 1]`` and ``z ∈ [0, 2]``.
That covers a table-top arm, but not a mobile base or a robot on a rail. **Obstacles outside
the grid are still collision-checked, but the heuristic does not see them**, so it can steer
the search into them and planning slows down. Set the volume when creating the planner, or later with
:meth:`~srmp.PlannerInterface.set_grid`:

.. code-block:: python

   config = srmp.GridConfig()
   config.origin_x, config.origin_y, config.origin_z = -1.5, -1.5, 0.0
   config.size_x, config.size_y, config.size_z = 3.0, 3.0, 2.0
   config.resolution = 0.03

   planner = srmp.PlannerInterface(config)   # or later: planner.set_grid(config)
   planner.add_box("box", size=[0.2, 0.2, 0.2], pose=srmp.Pose(p=[0.5, 0.0, 0.4]))

   grid = planner.get_grid()
   print(grid.bounds)         # ((-1.5, -1.5, 0.0), (1.5, 1.5, 2.0))
   print(grid.num_cells)      # (100, 100, 67)
   voxels = grid.get_occupied_voxels()                 # (N, 3) obstacle cell centres
   print(grid.get_distance_from_point(0.5, 0.0, 0.7))  # clearance at a point, metres

Halving the resolution multiplies memory and update cost by about eight, so tighten the
volume before refining the grid.

The grid is shared with the planner, so a handle stays current as objects are added and
removed (until :meth:`~srmp.PlannerInterface.set_grid` replaces it). It is read-only from
Python. Objects are voxelized as surfaces: the cells inside a solid box are not occupied, so
the distance at its centre is the distance to its nearest face. Distances are also bounded by
the grid's own faces.
