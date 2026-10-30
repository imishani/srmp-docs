Choosing a Planner
==================

:meth:`~srmp.PlannerInterface.make_planner` takes a ``planner_id`` and its settings. Every
setting is a string. This page says which planner to start with and when to switch;
:doc:`../reference/planner_settings` lists every setting.

Start with weighted A*
----------------------

.. code-block:: python

   planner.make_planner(["panda"], {"planner_id": "wAstar", "heuristic": "bfs", "weight": "50"})

Weighted A* (``wAstar``) searches the motion-primitive lattice (see
:doc:`../concepts/how_srmp_plans`) and returns a path whose cost is at most ``weight`` times
the best path on that lattice. It is deterministic: the same scene and query give the same
path, which is what makes SRMP's motions consistent.

- **weight**: higher plans faster with longer paths; lower gives shorter paths and plans
  slower. 10 to 100 is the useful range; ``50`` is a good default.
- **heuristic**: ``bfs`` (the default) runs a breadth-first search through the occupancy grid
  from the goal, so it routes the end effector around obstacles. It works for joint and pose
  goals and is the right choice in cluttered scenes. ``joint_euclidean`` is the straight-line
  joint distance: cheaper, but blind to obstacles.
- **time_limit**: seconds before giving up (default 10). A plan that is not found returns an
  empty trajectory.

When to switch
--------------

.. list-table::
   :header-rows: 1
   :widths: 18 42 40

   * - ``planner_id``
     - Use it when
     - Trade-off
   * - ``wAstar``
     - Default for one robot.
     - Path cost within ``weight`` of the lattice optimum.
   * - ``Astar``
     - You need the cheapest path on the lattice, on small problems.
     - Optimal, but often far slower than ``wAstar``.
   * - ``ARAstar``
     - You have a fixed time budget and want the best path within it.
     - Finds a path quickly with a high weight, then keeps lowering it until ``final_weight``
       or ``time_limit``.
   * - ``MHAstar``
     - One heuristic gets stuck (e.g. ``bfs`` pulls the hand into a dead end).
     - Explores with several heuristics; the anchor heuristic keeps the cost within
       ``w1 × w2`` of the optimum.
   * - ``wPASE``
     - Collision checks are expensive (meshes on coal) and you have spare cores.
     - Expands states in parallel with ``num_threads`` threads; same bound as ``wAstar``.
   * - ``MGS``
     - Hard scenes where weighted A* stalls: shelves, narrow openings, pose goals deep in
       clutter.
     - Grows several search graphs (from the start, the goal and extra roots) and connects
       them. More setup work per query.
   * - ``vamp_rrtc``
     - You want raw speed and VAMP supports the robot.
     - Sampling-based RRT-Connect on VAMP: no lattice, so none of the search planners'
       bounds or consistency apply.
   * - ``ECBS`` / ``xECBS``
     - Several robots that must move at the same time.
     - Plans every robot with time and resolves conflicts between them; see
       :doc:`../examples/multi_robot`.

.. code-block:: python

   # Pick one of these.

   # A fixed budget: the best path found in 2 seconds
   planner.make_planner(["panda"], {"planner_id": "ARAstar", "heuristic": "bfs",
                                    "weight": "50", "final_weight": "1", "time_limit": "2"})

   # Several heuristics: the anchor keeps the bound, bfs does the exploring
   planner.make_planner(["panda"], {"planner_id": "MHAstar", "heuristic": "joint_euclidean",
                                    "inadmissible_heuristics": "bfs", "w1": "20", "w2": "5"})

   # Multi-graph search; the defaults suit most scenes
   planner.make_planner(["panda"], {"planner_id": "MGS"})

   # Parallel expansion on 4 threads
   planner.make_planner(["panda"], {"planner_id": "wPASE", "heuristic": "bfs",
                                    "weight": "50", "num_threads": "4"})

``planner.print_available_planners()`` lists what your installed version supports.

Making any planner faster
-------------------------

**Use the VAMP backend.** ``"validity_backend": "vamp"`` checks collisions with SIMD-vectorized
sphere models instead of meshes, often an order of magnitude faster. It works with every
search planner above, for robots VAMP supports (``"auto"`` picks it when it can). VAMP keeps
the gripper fingers at a fixed opening, so use coal for grasping motions. See
:doc:`../reference/planner_settings` and :doc:`../user_guide/vamp_plugins`.

**Raise the weight** before anything else, if path length matters less than planning time.

**Size the planning volume to the scene.** The BFS heuristic runs over the occupancy grid, so
a grid much larger or finer than the workspace makes every new goal slower. See
:ref:`planning-volume`.

**Make the planner once, plan many times.** Call
:meth:`~srmp.PlannerInterface.make_planner` once and :meth:`~srmp.PlannerInterface.plan` per
query. The BFS for a goal is reused while the scene and the goal are unchanged.

``planner.get_planning_stats()`` after a plan reports where the time went (search, heuristic,
collision checks, post-processing).
