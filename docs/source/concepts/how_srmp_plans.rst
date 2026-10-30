How SRMP Plans
==============

SRMP plans by **search**: it builds a graph of joint configurations connected by small, fixed
motions, and searches it for the cheapest path from start to goal. Sampling-based planners
(RRT, PRM) draw random configurations instead; search on a fixed graph is what makes SRMP's
plans repeatable and gives them guarantees.

The search space: a lattice
---------------------------

Each robot's joint values are discretized (``resolution``, 1 degree by default). From any
configuration the planner may apply a **motion primitive**: a small change to one or more
joints. For a 7-joint arm the defaults are:

- **Long primitives**, used far from the goal: ±14° on each of the first four joints, the ones
  that carry the hand through the workspace.
- **Short primitives**, used near the goal: ±7° on every joint, for fine positioning.
- **Snap**, near a pose goal: an inverse-kinematics solve that tries to land exactly on the
  goal pose, so the goal does not have to lie on the lattice.

Every configuration reachable this way, with every motion between them, forms a lattice. The
primitives are YAML files you can replace (``mprim_path``; see
:doc:`../reference/motion_primitives`).

Checking states and motions
---------------------------

A configuration is valid if the robot is inside its joint limits and touches nothing: not the
obstacles, not its own links (apart from pairs the SRDF marks as never colliding), not other
robots, and not objects it holds. A motion between two configurations is checked at
intermediate points no more than 0.03 rad apart per joint.

Two **validity backends** do the checking:

- **coal** (default) checks the real meshes. It works for every robot.
- **VAMP** checks sphere approximations of the robot with SIMD instructions, many
  configurations at once. It is much faster, needs a VAMP model of the robot (built in, or a
  :doc:`plugin <../user_guide/vamp_plugins>`), and sees the gripper fingers at a fixed
  opening.

``validity_backend: "auto"`` uses VAMP when every robot being planned supports it, coal
otherwise.

The heuristic
-------------

A search is only fast if it knows which way the goal is. The **heuristic** estimates the
remaining cost from any configuration.

- ``bfs`` (default) runs a breadth-first search through the **occupancy grid** (the voxelized
  scene, see :ref:`planning-volume`) outward from the goal's end-effector position. Its value
  at a configuration is how far the end effector would travel *around* obstacles, plus an
  orientation term. This is what lets the planner reach into shelves and around clutter.
- ``joint_euclidean`` is the straight-line joint distance to a joint goal. Cheap, and fine in
  open space, but blind to obstacles.

The search
----------

Weighted A* expands the configurations with the lowest *cost so far + weight × heuristic*.
With weight 1 it is A* and returns the cheapest path on the lattice; a larger weight trusts
the heuristic more and finds a path faster, at most ``weight`` times more expensive than the
cheapest. The other planners vary this: anytime (ARA*), several heuristics (MHA*), parallel
expansion (PA*SE), several connected graphs (MGS). See :doc:`../guides/choosing_a_planner`.

What this buys:

- **Completeness on the lattice.** If a path exists on the lattice, the search finds it
  (given enough time). If none exists, it reports failure instead of retrying randomly.
- **Bounded cost.** The path is within a known factor of the best path on the lattice.
- **Consistency.** Weighted A* uses no randomness: the same query in the same scene gives
  the same path, and similar queries give similar paths, so robots behave predictably.
  (``vamp_rrtc`` is sampling-based and does not share this.)

The guarantees hold for the lattice, not for every possible motion: a path that needs a
finer move than the primitives allow can be missed. Smaller primitives or a finer
``resolution`` close that gap at the cost of planning time.

From path to trajectory
-----------------------

The search path is a chain of straight joint-space segments. SRMP then shortcuts and smooths
it, re-checking every change, and fits a spline timed to the robot's velocity and
acceleration limits. See :doc:`../guides/postprocessing`.

Several robots
--------------

For several robots planned together (``ECBS``, ``xECBS``), configurations carry a time step and
the primitives take time. Each robot is planned on its own; where two plans collide at the same
time, the planner adds a constraint and replans the robots involved, until no conflicts
remain. See :doc:`../examples/multi_robot`.
