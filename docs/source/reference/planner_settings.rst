Planner Settings
================

Every key :meth:`~srmp.PlannerInterface.make_planner` accepts, by planner. All values are
strings. :doc:`../guides/choosing_a_planner` explains which planner to use, and
:doc:`../guides/postprocessing` covers the ``postprocess_*`` keys, which every single-robot
planner accepts.

Single-robot planners
---------------------

``planner_id`` is one of ``Astar``, ``wAstar``, ``ARAstar``, ``MHAstar``, ``wPASE``, ``MGS``
and ``vamp_rrtc``.

Common keys
~~~~~~~~~~~

- ``heuristic`` (string): Heuristic function ("bfs", "joint_euclidean", "joint_euclidean_remove_time"). Default: "bfs"
- ``resolution`` (string): Joint angle discretization in degrees. Default: "1"
- ``mprim_path`` (string): Path to motion primitives file. Default: the built-in primitives for the robot's joint count (see :doc:`motion_primitives`)
- ``time_limit`` or ``allowed_planning_time`` (string): Planning time limit in seconds. Default: "10"
- ``validity_backend`` (string): What checks states and motions for collisions — "coal"
  (default), "vamp" or "auto". See `Validity backends`_ below.

A* (``Astar``)
~~~~~~~~~~~~~~

Uses only the common keys.

Weighted A* (``wAstar``)
~~~~~~~~~~~~~~~~~~~~~~~~

- ``weight`` (string): Heuristic weight. Default: "50"

ARA* (``ARAstar``)
~~~~~~~~~~~~~~~~~~

- ``weight`` (string): Initial heuristic weight. Default: "50"
- ``weight_delta`` (string): Weight reduction per iteration. Default: "10.0"
- ``final_weight`` (string): Final weight to reach. Default: "1.0"

MHA* (``MHAstar``)
~~~~~~~~~~~~~~~~~~

- ``heuristic`` (string): Anchor heuristic. Default: "joint_euclidean"
- ``inadmissible_heuristics`` (vector<string>): List of inadmissible heuristics. Default: ["bfs"]
- ``w1`` (string): Anchor heuristic weight. Default: "20"
- ``w2`` (string): Inadmissible heuristic weight. Default: "5"

wPASE (``wPASE``)
~~~~~~~~~~~~~~~~~

- ``heuristic`` (string): Primary heuristic. Default: "joint_euclidean"
- ``i_heuristic`` (string): Secondary heuristic. Default: "joint_euclidean"
- ``weight`` (string): Primary heuristic weight. Default: "50"
- ``i_weight`` (string): Secondary heuristic weight. Default: "100.0"
- ``num_threads`` (string): Number of parallel threads. Default: "4"

Multi-Graph Search (``MGS``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Grows a start graph and a goal graph, adds extra graphs rooted at IK solutions of the
BFS heuristic's workspace attractors, and connects them. The extra graphs help it out
of the local minima that stall a single weighted search. :meth:`get_mgs_roots` reports
the roots it added; the viser GUI can draw them (see :doc:`../user_guide/visualization`).

- ``heuristic`` (string): State-to-state heuristic between graph roots. Default:
  "joint_steps" (joint distance in discretization steps). "bfs" is rejected, since it
  cannot measure the distance between two states. "joint_time_floor" suits mobile
  manipulators: it steers root graphs around obstacles on the floor.
- ``anchor_heuristic`` (string): Goal-distance heuristic of the start graph. Default: "bfs"
- ``weight`` (string): Suboptimality bound of the start-graph search. Default: "100"
- ``g_num`` (string): Maximum number of graphs, start and goal included; at least 2.
  Default: "10"
- ``graph_growth`` (string): "on_stall" (default) adds a root graph whenever the start
  graph makes no progress for ``stall_window`` expansions, up to ``g_num``; "upfront"
  creates all ``g_num - 2`` roots before searching. With "on_stall", a scene the start
  graph solves without stalling gets no extra graphs at all.
- ``stall_window`` (string): Expansions without a better heuristic value before a root
  is added. Default: "50"
- ``root_interval`` (string): Also add a root every this many expansions; "0" turns it
  off. Default: "0"
- ``root_selection`` (string): "frontier_nearest" (default) picks the root nearest the
  stalled frontier's end effector; "sequential" uses the attractors in order.
- ``goal_roots`` (string): For pose goals, graphs rooted at collision-free IK solutions
  of the goal pose. "1" (default) makes the search bidirectional; "0" searches from the
  start only.
- ``root_clearance`` (string): Minimum end-effector clearance of a root, in metres.
  Attractors sit on obstacle boundaries, so roots are pulled back into free space.
  Default: "0.05"
- ``nonblocking_roots`` (string): "false" (default, deterministic) waits for the next
  root when a stall asks for one; "true" keeps expanding while roots are prepared in
  the background.
- ``anchor_search`` (string): After the first start–goal connection, keep searching
  until the cost is within ``weight`` of optimal. Default: "true"
- ``lazy_edges`` (string): "true" checks a successor's motion only when it is expanded
  (Lazy Weighted A*). Default: "false"
- ``root_prefetch``, ``root_ik_threads``, ``expansion_threads``, ``snap_ik_seeds``
  (strings): How much root IK runs before the search starts, and how many threads the
  root IK, the edge checks and the goal-snap IK use. The defaults suit most scenes.

VAMP RRT-Connect (``vamp_rrtc``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A sampling-based planner that runs entirely on the VAMP backend, so it needs
``validity_backend`` "vamp" (or "auto" on a supported robot). It uses no motion
primitives or heuristic.

- ``rrtc_range`` (string): Maximum extension distance per step
- ``rrtc_max_iterations``, ``rrtc_max_samples`` (strings): Search budgets
- ``rrtc_simplify`` (string): "true" to simplify the returned path

Validity backends
-----------------

- "coal" (default): COAL collision checking against the full planning world. Works
  for every robot.
- "vamp": SIMD-vectorized checks from VAMP, much faster. It needs a VAMP model of the
  robot: one built into srmp, or a plugin (see :doc:`../user_guide/vamp_plugins`). Requesting it for a
  robot without one raises an error, as does a build compiled without VAMP (check
  :meth:`get_available_validity_backends`).
- "auto": "vamp" when every robot being planned supports it, "coal" otherwise. It
  never raises, and prints which backend it chose; :meth:`get_active_validity_backend`
  reports it too.

Multi-robot planners accept the same key and use one backend for the whole group.

A VAMP model is used only for a robot it actually describes. Besides its name and joint
count, its end-effector kinematics must agree with the robot's at several
configurations (within 2 mm / 0.01 rad); otherwise "auto" falls back to coal and "vamp"
raises, naming the disagreement. :meth:`is_robot_vamp_supported` answers this per robot.
Among the registry robots, ``panda``, ``ridgeback_ur10e``, ``static_ridgeback_ur10e``
and ``so107`` match a built-in model, and ``kinova_gen3_robotiq_2f85`` has a published
plugin that :meth:`add_robot` installs automatically. srmp also contains VAMP models
named ``ur5`` and ``fetch``, but they describe other robot models than the registry's
``ur5`` and ``fetch``, so those plan on coal.

.. warning::

   VAMP models keep the gripper fingers at a fixed opening: :meth:`set_gripper_qpos`
   does not reach them. With the fingers closed on or near an object, VAMP can miss a
   collision that coal reports. Use "coal" for grasping motions.

VAMP scene translation
----------------------

All optional, in metres unless noted. VAMP sees mesh
obstacles and attached meshes as sampled spheres; these keys tune that:

- ``mesh_sampling_resolution``: Surface sample spacing for mesh obstacles and attached
  meshes. Default: "0.01"
- ``pointcloud_point_radius``: Radius of point-cloud points; raised automatically to
  cover the mesh sample spacing when that is coarser. Default: "0.01"
- ``articulation_link_sampling_resolution``: Sample spacing for the mesh links of robots
  that are not being planned. Default: "0.05"
- ``attachment_spherization_resolution``: Sphere pitch for attached primitives (box,
  cylinder). Default: "0.05"
- ``attachment_max_spheres``: Sphere budget for one attached object. An attached mesh
  over it raises (raise the budget, or attach a primitive such as :meth:`add_box`
  instead); a primitive is coarsened to fit. Default: "512"

Multi-robot planners
--------------------

``planner_id`` is ``ECBS`` or ``xECBS``. Nothing else is required.

Multi-robot planners search timed states (joint values plus a time step) and take
joint-space goals only: pass :attr:`GoalType.JOINTS` constraints to :meth:`plan_multi`.

Per-robot keys
~~~~~~~~~~~~~~

All optional.

For each robot with name ``{robot_name}``:

- ``heuristic_{robot_name}`` (string): Heuristic for this robot: "bfs", "joint_euclidean"
  or "joint_euclidean_remove_time". Each is used in its time-aware form. Default: "bfs"
- ``mprim_path_{robot_name}`` (string): Motion primitives path for this robot. Default:
  the built-in timed primitives for its DOF
- ``resolution_{robot_name}`` (string): Discretization for this robot. Default: "1"

Search keys
~~~~~~~~~~~

- ``weight_low_level_heuristic`` (string): Low-level search weight. Default: "1.0"
- ``high_level_focal_suboptimality`` (string): High-level focal search bound. Default: "1.3"
- ``low_level_focal_suboptimality`` (string): Low-level focal search bound. Default: "1.3"

.. code-block:: python

   planner.make_planner(["panda0", "panda1"], {"planner_id": "xECBS", "time_limit": "20"})
   goals = {n: srmp.GoalConstraint(srmp.GoalType.JOINTS, [q]) for n, q in goal_qs.items()}
   trajs = planner.plan_multi(start_qs, goals)

