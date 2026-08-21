API Reference
=============

Core Classes
------------

PlannerInterface
~~~~~~~~~~~~~~~~

The main interface for robot motion planning.

.. class:: srmp.PlannerInterface()

   The primary class for creating and configuring planners.

   **Methods:**

   .. method:: add_robot(robot, name=None, srdf_path=None, end_effector=None, planned=True, gravity=None, link_names=None, joint_names=None, gripper_joint_names=None)

      Add a robot from the registry or by file path. This is the recommended way to add robots.

      :param str robot: Robot name from registry (e.g., "panda", "so101"), or path to URDF file
      :param str name: Override the articulation name (default: robot name)
      :param str srdf_path: Override SRDF path (required if robot is a file path)
      :param str end_effector: Override end effector link name (required if robot is a file path)
      :param bool planned: Whether this robot should be planned for (default: True)
      :param numpy.ndarray gravity: Gravity vector for the robot (default: [0, 0, 0])
      :param list link_names: Override link names
      :param list joint_names: Override joint names
      :param list gripper_joint_names: Override the registry entry's gripper joint classification
         (default: use the registry's own ``gripper_joint_names`` for that robot, if any). Drive
         gripper joints with :meth:`set_gripper_qpos` instead of :meth:`set_qpos`.
      :returns: The actual name assigned to the robot. This may differ from ``name``/``robot``
         if an articulation with that name already exists in the scene — SRMP auto-suffixes it
         (e.g. ``"panda"`` → ``"panda1"``), which makes it possible to add the same robot/URDF
         multiple times without picking unique names yourself. Use the returned name, not the
         input, for subsequent calls like :meth:`set_base_pose` or :meth:`plan`.
      :rtype: str
      :raises RobotNotFoundError: If robot not in registry and not a valid path
      :raises ValueError: If using file path without srdf_path and end_effector

      See :doc:`robot_registry` for available robots and registry functions.

   .. method:: add_articulation(name, end_effector,
                                urdf_path, srdf_path='',
                                link_names: List[str] = [],
                                joint_names: List[str] = [],
                                gravity: NDArray[np.float64] = np.array([0, 0, 0]),
                                planned=True,
                                gripper_joint_names: List[str] = [])

      Add a robot to the planning scene with explicit file paths. The `srdf_path` argument is optional — if you
      don't have an SRDF file, you can omit this argument or pass an empty string.

      :param str name: Unique name for this robot instance
      :param str end_effector: Name of the end-effector link
      :param str urdf_path: Path to the URDF file describing the robot
      :param str srdf_path: Path to the SRDF file with semantic information (optional, default: "")
      :param list link_names: List of link names to include (default: all links)
      :param list joint_names: List of joint names to include (default: all joints)
      :param numpy.ndarray gravity: Gravity vector for the robot (default: [0, 0, 0])
      :param bool planned: Whether this robot should be planned for (default: True)
      :param list gripper_joint_names: Names of this robot's gripper joints, classified separately
         from the arm move group (default: []). Drive them with :meth:`set_gripper_qpos` instead of
         :meth:`set_qpos`.
      :returns: The actual name assigned to the articulation (auto-suffixed if ``name`` already
         exists in the scene, e.g. ``"panda"`` → ``"panda1"``). Use the returned value for
         subsequent calls when there's any chance of a name collision.
      :rtype: str

   .. method:: remove_articulation(name)

      Remove an articulation from the planning world.

      :param str name: Name of the articulation to remove

   .. method:: set_base_pose(name, pose)

      Set the base pose of a robot.

      :param str name: Name of the robot
      :param Pose pose: Base pose of the robot

   .. method:: make_planner(articulation_names, planner_context)

      Configure and initialize the planner.

      :param list articulation_names: List of robot names to plan for
      :param dict planner_context: Dictionary containing planner configuration

      **Required Parameters:**

      - ``planner_id`` (string): Planner algorithm to use

      **Single Robot Planner Context Options:**

      Available single-robot planners: "Astar", "wAstar", "ARAstar", "MHAstar", "wPASE"

      **General Parameters:**

      - ``heuristic`` (string): Heuristic function ("bfs", "joint_euclidean", "joint_euclidean_remove_time"). Default: "bfs"
      - ``resolution`` (string): Joint angle discretization in degrees. Default: "1"
      - ``mprim_path`` (string): Path to motion primitives file. Default: auto-generated based on DOF
      - ``time_limit`` or ``allowed_planning_time`` (string): Planning time limit in seconds. Default: "10"

      **Planner-Specific Parameters:**

      *A\* ("Astar"):*
        Uses only general parameters above.

      *Weighted A\* ("wAstar"):*
        - ``weight`` (string): Heuristic weight. Default: "50"

      *ARA\* ("ARAstar"):*
        - ``weight`` (string): Initial heuristic weight. Default: "50"
        - ``weight_delta`` (string): Weight reduction per iteration. Default: "10.0"
        - ``final_weight`` (string): Final weight to reach. Default: "1.0"

      *MHA\* ("MHAstar"):*
        - ``heuristic`` (string): Anchor heuristic. Default: "joint_euclidean"
        - ``inadmissible_heuristics`` (vector<string>): List of inadmissible heuristics. Default: ["bfs"]
        - ``w1`` (string): Anchor heuristic weight. Default: "20"
        - ``w2`` (string): Inadmissible heuristic weight. Default: "5"

      *wPASE ("wPASE"):*
        - ``heuristic`` (string): Primary heuristic. Default: "joint_euclidean"
        - ``i_heuristic`` (string): Secondary heuristic. Default: "joint_euclidean"
        - ``weight`` (string): Primary heuristic weight. Default: "50"
        - ``i_weight`` (string): Secondary heuristic weight. Default: "100.0"
        - ``num_threads`` (string): Number of parallel threads. Default: "4"

      **Multi-Robot Planner Context Options:**

      Available multi-robot planners: "E-CBS", "xECBS"

      **Required Multi-Robot Parameters:**

      - ``planner_id``: "E-CBS" or "xECBS"

      **Agent-Specific Parameters (per robot):**

      For each robot with name ``{robot_name}``:

      - ``heuristic_{robot_name}`` (string): Heuristic for this robot. Default: "bfs" (E-CBS), "joint_euclidean_remove_time" (xECBS)
      - ``mprim_path_{robot_name}`` (string): Motion primitives path for this robot. Default: auto-generated timed version
      - ``resolution_{robot_name}`` (string): Discretization for this robot. Default: "1"

      **E-CBS/xECBS Parameters:**

      - ``weight_low_level_heuristic`` (string): Low-level search weight. Default: "1.0" (E-CBS), "55.0" (xECBS)
      - ``high_level_focal_suboptimality`` (string): High-level focal search bound. Default: "1.3"
      - ``low_level_focal_suboptimality`` (string): Low-level focal search bound. Default: "1.3"

   .. method:: plan(start, goal_constraint)

      Plan a trajectory for a single robot.

      :param numpy.ndarray start: Starting joint configuration
      :param GoalConstraint goal_constraint: Goal specification
      :returns: Trajectory object containing the planned path
      :rtype: Trajectory

   .. method:: plan_multi(start_states, goal_constraints)

      Plan trajectories for multiple robots simultaneously.

      :param dict start_states: Dictionary mapping robot names to start configurations
      :param dict goal_constraints: Dictionary mapping robot names to goal constraints
      :returns: Dictionary mapping robot names to their trajectories
      :rtype: dict

   .. method:: plan_screw(articulation_name, link_name, start_qpos=None, end_pose=None, axis_point=None, axis_direction=None, pitch=0.0, angle=None, qpos_step=0.1, max_steps=10000, collision_aware=True)

      Plan a screw motion for ``link_name`` by closing the loop on its Jacobian — a
      resolved-rate/velocity controller that walks along the screw path in small steps,
      checking move-group joint limits at every step and, unless ``collision_aware`` is
      ``False``, collision as well (:meth:`is_state_colliding`). No IK/FK solve is involved.

      Specify the motion in exactly one of two ways:

      - ``end_pose``: the target pose for ``link_name``. The unique screw axis/pitch/angle
        connecting the current pose to it is derived automatically. A **pure translation**
        (``end_pose`` has the same orientation as the current pose) is a supported special
        case: no screw axis exists for it, so the twist is taken directly as
        ``[delta_p, 0]``. This makes ``plan_screw`` the natural way to run a straight-line
        approach — see :ref:`screw-translation-only`.
      - ``axis_direction`` + ``angle`` (with optional ``axis_point``/``pitch``): an explicit
        screw — rotate ``angle`` radians about the line through ``axis_point`` (default: the
        current position of ``link_name``) along ``axis_direction``, translating ``pitch``
        linear units per full revolution (default ``0`` = pure rotation, e.g. a hinge/door).

      :param str articulation_name: Articulation name
      :param str link_name: The link whose motion is being screwed (e.g. the end effector);
         also determines which Jacobian is used
      :param numpy.ndarray start_qpos: Move-group joint configuration to start from.
         Defaults to the articulation's current qpos (:meth:`get_qpos`) if omitted. Sets the
         articulation's current qpos as a side effect, same as :meth:`set_qpos`.
      :param Pose end_pose: Target pose for ``link_name`` (pose-to-pose mode)
      :param numpy.ndarray axis_point: A point on the screw axis, world frame (axis mode;
         defaults to the current position of ``link_name``)
      :param numpy.ndarray axis_direction: Direction of the screw axis, world frame, need
         not be unit length (axis mode; required)
      :param float pitch: Linear distance traveled per full revolution (axis mode only;
         ignored in pose-to-pose mode, where it's derived)
      :param float angle: Total rotation to sweep, radians; sign gives direction via the
         right-hand rule (axis mode; required)
      :param float qpos_step: Max joint-space step norm per iteration, radians (default: 0.1)
      :param int max_steps: Safety cap on the number of iterations (default: 10000)
      :param bool collision_aware: Whether to collision-check each step (default: ``True``).
         Set ``False`` to skip the :meth:`is_state_colliding` call and sweep the path on
         kinematics alone — useful for a short final approach that deliberately closes on an
         object, where contact-adjacent configurations would otherwise abort the motion.
         **Joint limits are still enforced either way**; only the collision check is
         dropped, so the returned trajectory may pass through obstacles and is your
         responsibility to validate.
      :returns: Joint-space Trajectory (positions only) tracing the screw motion
      :rtype: Trajectory
      :raises RuntimeError: Parameters are inconsistent; a step would collide (when
         ``collision_aware``) or violate a joint limit; progress stalls (kinematic
         singularity); or ``max_steps`` is exceeded. A pose-to-pose call whose start and end
         differ by a pure translation is supported and does **not** raise.

   .. method:: add_box(name, size, pose)

      Add a box obstacle to the environment.

      :param str name: Unique name for the box
      :param numpy.ndarray size: Box dimensions [x, y, z]
      :param Pose pose: Box pose in world frame

   .. method:: add_sphere(name, radius, pose)

      Add a sphere obstacle to the environment.

      :param str name: Unique name for the sphere
      :param float radius: Sphere radius
      :param Pose pose: Sphere pose in world frame

   .. method:: add_cylinder(name, radius, height, pose)

      Add a cylinder obstacle to the environment.

      :param str name: Unique name for the cylinder
      :param float radius: Cylinder radius
      :param float height: Cylinder height
      :param Pose pose: Cylinder pose in world frame

   .. method:: add_mesh(name, mesh_path=None, vertices=None, triangles=None, scale=np.ones(3), pose=None, convex=False)

      Add a mesh obstacle to the environment.  Either ``mesh_path`` **or** both
      ``vertices`` and ``triangles`` must be provided.

      :param str name: Unique name for the mesh
      :param str mesh_path: Path to a mesh file (STL, OBJ, DAE, …).  Mutually
          exclusive with ``vertices``/``triangles``.
      :param numpy.ndarray vertices: Mesh vertices as an Nx3 array.  Must be
          supplied together with ``triangles``.
      :param numpy.ndarray triangles: Triangle face indices as an Mx3 array of
          integer indices into ``vertices``.  Must be supplied together with
          ``vertices``.
      :param numpy.ndarray scale: Uniform or per-axis scale factors [x, y, z]
          (default: ``[1, 1, 1]``)
      :param Pose pose: Mesh pose in world frame
      :param bool convex: Treat mesh as convex hull during collision checking.
          Only applicable for file-based meshes (default: ``False``)

   .. method:: add_point_cloud(name, vertices, resolution=0.01)

      Add a point cloud as a collision object to the environment.

      :param str name: Unique name for the point cloud
      :param numpy.ndarray vertices: Point cloud vertices as Nx3 matrix where each row is [x, y, z]
      :param float resolution: Voxel resolution for octomap representation (default: 0.01)

   .. method:: remove_object(name)

      Remove an object from the environment.

      :param str name: Name of the object to remove

   .. method:: update_object_pose(name, new_pose)

      Move an existing object to a new pose, updating both the collision world and the
      occupancy grid in place. Use this instead of :meth:`remove_object` followed by a fresh
      ``add_*`` call — it keeps the object's name, geometry, and any attachment intact, and
      avoids rebuilding the grid region twice.

      :param str name: Name of the object to move
      :param Pose new_pose: New world pose of the object

      .. warning::

         Must not be called while a plan is running: it mutates state shared with the
         collision checker and the heuristics.

   .. method:: read_sim(sim, sim_type, articulations=None)

      Import objects from a simulation environment.

      :param sim: Simulation object
      :param str sim_type: Type of simulator ("sapien", "genesis", "pybullet", "mujoco", "swift")
      :param list articulations: List of articulation names to exclude from import
      :returns: For SAPIEN, a dict mapping each source actor name to the list of
         planning-world object names created from it — one actor can contribute several
         collision shapes, hence a list. ``None`` for backends that do not report it
         (Genesis, PyBullet, MuJoCo, Swift).
      :rtype: dict or None

      .. note::

         Imported SAPIEN objects are named ``"<actor>::<shape><n>"`` — e.g.
         ``"table::box0"``, ``"mug::convex_mesh0"`` — rather than the bare ``"box0"`` /
         ``"convex_mesh_0"`` used previously. The actor prefix is what makes a generated
         object traceable back to its source actor, which you need in order to address it
         later with :meth:`update_object_pose`, :meth:`attach_object`, or
         :meth:`remove_object`. Prefer the returned dict over reconstructing these names
         yourself:

         .. code-block:: python

            actor_objects = planner.read_sim(scene, "sapien")
            for obj_name in actor_objects["mug"]:
                planner.update_object_pose(obj_name, new_pose)

   .. method:: save_scene(path)

      Write the planning world to ``path`` as a JSON manifest. A ``.npz`` sidecar is written
      beside it when the scene holds geometry with no file behind it — meshes imported from a
      simulator, and point clouds. See :doc:`persistence`.

      :param path: Destination for the manifest. Parent directories are created.
      :returns: The manifest path
      :rtype: pathlib.Path

   .. method:: load_scene(path, clear=True)

      Rebuild this planner's world from a manifest written by :meth:`save_scene`.

      :param path: The manifest
      :param bool clear: Reset this planner first (default: ``True``). When ``False``, merge
         instead — and raise on a name collision rather than silently overwriting.

      .. note::

         A manifest's grid config cannot be applied to an already-constructed planner
         (planning-world bounds are fixed at construction); a warning is issued if the file
         has one. Use the module-level :func:`srmp.load_scene` to get a planner *built* with
         it.

   .. method:: save_plan(path)

      Write the most recent planning episode — the trajectories, start, goal, and planner
      context from the last :meth:`plan` or :meth:`plan_multi` — to ``path``.

      :param path: Destination. Parent directories are created.
      :returns: The path written
      :rtype: pathlib.Path
      :raises RuntimeError: If :meth:`plan`/:meth:`plan_multi` has not produced a real plan
         yet. A search that returns zero waypoints does not count, so a failed search cannot
         be persisted as a file that looks like a success. For a trajectory that did not come
         from :meth:`plan` — :meth:`plan_screw`, for instance — use the module-level
         :func:`srmp.save_plan` instead.

   .. method:: print_available_planners()

      Print available planners and their descriptions.

   Note: `add_articulation` accepts an optional `srdf_path` parameter. Many examples
   below include SRDF paths for completeness, but you can call `add_articulation`
   with only the `urdf_path` if no SRDF is required for your use case.

   .. method:: get_articulation_names()

      Return the list of articulation names currently present in the planning world.

      :returns: List of articulation names

   .. method:: get_object_names()

      Return the list of object names currently present in the planning world.

      :returns: List of object names

   .. method:: get_link_names(articulation_name)

      Return every link name for an articulation, in the order the underlying model
      defines them — so a name's position in this list is its link index.

      :param str articulation_name: Articulation name
      :returns: List of link names
      :rtype: list

   .. method:: get_link_index(articulation_name, link_name)

      Resolve a link name to its link index. Useful for the ``link_id`` argument of
      :meth:`attach_object`.

      :param str articulation_name: Articulation name
      :param str link_name: Link name
      :returns: Link index
      :rtype: int

   .. method:: get_link_pose(articulation_name, link_name)

      World pose of a link at the articulation's **current** qpos. Call :meth:`set_qpos`
      first to query a different configuration.

      :param str articulation_name: Articulation name
      :param str link_name: Link name
      :returns: Pose of the link in the world frame
      :rtype: Pose

   .. method:: has_articulation(name)

      Check whether an articulation with the given name exists in the planning world.

      :param str name: Articulation name
      :returns: True if the articulation exists, False otherwise

   .. method:: has_object(name)

      Check whether an object with the given name exists in the planning world.

      :param str name: Object name
      :returns: True if the object exists, False otherwise

   .. method:: is_articulation_planned(name)

      Check whether a named articulation is configured to be planned.

      :param str name: Articulation name
      :returns: True if the articulation is planned, False otherwise

   .. method:: set_articulation_planned(name, planned)

      Enable or disable planning for a specific articulation.

      :param str name: Articulation name
      :param bool planned: Whether to plan for this articulation

   .. method:: is_object_attached(name)

      Query whether an object is currently attached to a robot.

      :param str name: Object name
      :returns: True if attached, False otherwise

   .. method:: attach_object(name, art_name, link_id, touch_links=None)

      Attach an existing object to a robot link so it moves with the robot.

      :param str name: Object name
      :param str art_name: Articulation name to attach to
      :param int link_id: Index of the link to attach the object to
      :param list touch_links: Optional list of link names allowed to touch the object

   .. method:: detach_object(name, also_remove=False)

      Detach an attached object from its robot. Optionally remove it from the world.

      :param str name: Object name
      :param bool also_remove: If True, remove the object from the world after detaching
      :returns: True if successful

   .. method:: detach_all_objects(also_remove=False)

      Detach all attached objects. Optionally remove them from the world.

      :param bool also_remove: If True, remove detached objects from the world
      :returns: True if successful

   .. method:: is_state_colliding(articulation_name="")

      Check whether the current state (optionally for a specific articulation) is in collision.

      :param str articulation_name: Optional articulation name to check
      :returns: True if a collision is detected

   .. method:: is_robot_colliding_with_objects(art_name)

      Check if the specified robot is colliding with any objects in the planning world.

      :param str art_name: Articulation name
      :returns: True if collision detected with environment objects

   .. method:: distance_to_self_collision()

      Get the minimum signed distance to self-collision for all robots.

      :returns: Minimum self-collision distance (float)

   .. method:: distance_to_robot_collision()

      Get the minimum signed distance between robots and environment objects.

      :returns: Minimum robot-to-environment collision distance (float)

   .. method:: distance_to_collision()

      Get the minimum distance to any collision (self or environment).

      :returns: Minimum distance to collision (float)

   .. method:: set_allowed_collision(name1, name2, allowed)

      Set whether collisions between two named objects are allowed.

      :param str name1: First object name
      :param str name2: Second object name
      :param bool allowed: True to allow collisions, False to disallow

   .. method:: get_active_joint_names(articulation_name)

      Get the names of all active (user-defined) joints for an articulation. This includes
      every actuated joint (arm + fingers etc.), **not** filtered to the planning move group.
      Use :meth:`get_move_group_joint_names` for planning.

      :param str articulation_name: Name of the articulation
      :returns: List of active joint names

   .. method:: get_move_group_joint_names(articulation_name)

      Get the names of the move-group joints for an articulation — the joints in the kinematic
      chain to the end-effector that the planner actually controls. The start state passed to
      :meth:`plan` must have exactly ``len(get_move_group_joint_names(...))`` elements.

      :param str articulation_name: Name of the articulation
      :returns: List of move-group joint names

   .. method:: get_move_group_qpos_dim(articulation_name)

      Get the move-group qpos dimension (velocity-space DOF) for an articulation. This is the
      authoritative value to size planning states with — prefer it over
      ``len(get_move_group_joint_names(...))`` when mimic/fixed joints (which contribute 0 DOF)
      may appear in the move group.

      :param str articulation_name: Name of the articulation
      :returns: Move-group qpos dimension (int)

   .. method:: get_move_group_joint_limits(articulation_name)

      Get position limits for an articulation's move-group joints, in the same order as
      :meth:`set_qpos`/:meth:`plan`. Only single-DOF joints (revolute/prismatic/continuous)
      are supported — raises for anything else.

      :param str articulation_name: Name of the articulation
      :returns: ``(move_group_qpos_dim, 2)`` array of ``(lower, upper)`` bounds per joint
      :rtype: numpy.ndarray

   .. method:: set_qpos(name, qpos)

      Set the joint positions for a named articulation.

      :param str name: Articulation name
      :param numpy.ndarray qpos: Joint positions (1D array)

   .. method:: get_qpos(name)

      Get the current joint positions for a named articulation, in move-group order (the
      same order :meth:`set_qpos`/:meth:`plan` expect).

      :param str name: Articulation name
      :returns: Move-group qpos
      :rtype: numpy.ndarray

   .. method:: get_gripper_joint_names(articulation_name)

      Get the names of an articulation's gripper joints, as classified via the
      ``gripper_joint_names`` argument to :meth:`add_articulation`.

      :param str articulation_name: Name of the articulation
      :returns: List of gripper joint names (empty if none were classified)

   .. method:: set_gripper_qpos(articulation_name, gripper_qpos)

      Set joint angles for an articulation's gripper joints. Gripper joints are
      tracked separately from the arm move group, so this does not affect
      :meth:`set_qpos` or its expected qpos dimension.

      :param str articulation_name: Name of the articulation
      :param gripper_qpos: Joint angles for the gripper joints

   .. method:: set_qpos_all(state)

      Set the joint positions for all planned articulations using a concatenated state vector.

      :param numpy.ndarray state: Concatenated joint positions for all planned articulations

   .. method:: update_attached_bodies_pose()

      Update the poses of all objects attached to robots based on current robot joint states.

   .. method:: compute_fk(articulation_name, qpos)

      Compute forward kinematics for a specific articulation and joint configuration.

      :param str articulation_name: Articulation name
      :param numpy.ndarray qpos: Joint positions
      :returns: Pose of the end-effector (Pose)

   .. method:: compute_ik(articulation_name, ee_pose, init_state_val)

      Compute inverse kinematics (CLIK) for a desired end-effector pose.

      :param str articulation_name: Articulation name
      :param list ee_pose: Desired end-effector pose [x, y, z, roll, pitch, yaw]
      :param list init_state_val: Initial joint configuration for IK solver
      :returns: Tuple `(success: bool, joint_state: list)`

   .. method:: get_jacobian(articulation_name, link_name, local=False, move_group_only=True)

      Get the Jacobian of a link at the articulation's current qpos.

      :param str articulation_name: Articulation name
      :param str link_name: Name of the link
      :param bool local: If True, express the Jacobian in the link's own local frame; if
         False (default), world frame
      :param bool move_group_only: If True (default), return only the move-group columns
         (same order as :meth:`set_qpos`/:meth:`plan`); if False, return all columns (full
         model DOF, including gripper/frozen joints)
      :returns: ``(6, N)`` Jacobian array, ``N`` = move-group qpos dim or full model DOF
      :rtype: numpy.ndarray

   .. method:: compute_jacobian(articulation_name, qpos, link_name, local=False, move_group_only=True)

      Get the Jacobian of a link at an arbitrary qpos, without touching the articulation's
      current internal state.

      :param str articulation_name: Articulation name
      :param numpy.ndarray qpos: Move-group joint configuration (same order as
         :meth:`set_qpos`/:meth:`plan`) to evaluate the Jacobian at; non-move-group joints
         (gripper/frozen) are taken from the articulation's current state
      :param str link_name: Name of the link
      :param bool local: If True, express the Jacobian in the link's own local frame; if
         False (default), world frame
      :param bool move_group_only: If True (default), return only the move-group columns;
         if False, return all columns
      :returns: ``(6, N)`` Jacobian array
      :rtype: numpy.ndarray

   .. method:: compute_joint_velocities(articulation_name, link_name, twist, damping=0.0, local=False)

      Map a desired end-effector twist to move-group joint velocities via damped least
      squares — stays well-behaved near/at kinematic singularities (``damping=0`` gives the
      plain Moore-Penrose pseudo-inverse solution).

      :param str articulation_name: Articulation name
      :param str link_name: Name of the link
      :param numpy.ndarray twist: Desired 6D twist ``[linear; angular]``, in the frame
         selected by ``local``
      :param float damping: Damping factor; higher trades tracking accuracy near
         singularities for stability (default: 0.0)
      :param bool local: If True, ``twist`` is expressed in the link's own local frame; if
         False (default), world frame
      :returns: Move-group joint velocities (same order as :meth:`set_qpos`/:meth:`plan`)
      :rtype: numpy.ndarray

   .. method:: get_manipulability(articulation_name, link_name)

      Yoshikawa manipulability index of a link's move-group Jacobian: ``sqrt(det(J @ J.T))``.
      Zero at a kinematic singularity; higher means more dexterous. Frame-invariant (world
      vs. local Jacobian give the same value).

      :param str articulation_name: Articulation name
      :param str link_name: Name of the link
      :returns: Manipulability index (>= 0)
      :rtype: float

   .. method:: reset(reset_robots=True)

      Reset the planner interface. When `reset_robots` is True, all articulations and objects
      are removed; when False, only planner caches and internal data are reset.

      :param bool reset_robots: Whether to remove robots and objects during reset

   .. method:: start_visualizer(type="viser", **kwargs)

      Create, attach, and open a visualizer on this planner without subclassing
      :class:`ViserPlannerInterface`. This is the recommended way to add visualization to a
      plain :class:`~srmp.PlannerInterface`. See :doc:`visualization`.

      :param str type: ``"viser"`` (a ``"meshcat"`` backend exists in the codebase but is
         currently being reworked and isn't functional)
      :param kwargs: Additional arguments passed to the visualizer constructor (e.g. ``port``)
      :returns: The created visualizer instance

   .. method:: get_visualizer(index=0)

      Get a visualizer previously attached via :meth:`start_visualizer` or
      :meth:`attach_visualizer`.

      :param int index: Index of the visualizer to get (default: 0)
      :returns: The visualizer instance, or ``None`` if no visualizer is attached at that index

   .. method:: attach_visualizer(listener)

      Attach a :class:`VisualizerListener` to this planner so it is notified of scene changes
      (``add_robot``, ``add_box``, etc.) and kept in sync. Used internally by
      :meth:`start_visualizer`; call directly if you constructed a visualizer yourself.

      :param VisualizerListener listener: The visualizer/listener to attach

   .. method:: detach_visualizer(listener)

      Detach a previously attached visualizer. The visualizer stops receiving scene updates.

      :param VisualizerListener listener: The visualizer/listener to detach

MotionPlanningAgent
~~~~~~~~~~~~~~~~~~~

The LLM-driven assistant behind :doc:`agent_mode`, usable directly from Python. It pairs an
LLM backend with a persistent Python executor, so the planner and any variables you build up
survive across turns.

It is **not** re-exported at the top level, so import it from the subpackage:

.. code-block:: python

   from srmp.agent import MotionPlanningAgent

   with MotionPlanningAgent() as agent:
       print(agent.run("Add a panda and a table, then plan to a pose above the table"))

.. class:: srmp.agent.MotionPlanningAgent(system_prompt_extra="", backend=..., executor=None, max_tool_iterations=..., execution_timeout=60.0, include_viser=False)

   Supports the context-manager protocol; leaving the ``with`` block calls :meth:`close`.

   :param str system_prompt_extra: Extra context appended to the system prompt
   :param backend: An LLM backend instance. Omit it to default to ``AnthropicBackend``,
      which is the right choice for standalone scripting. Pass ``None`` *explicitly* to
      start without a working backend and hot-swap one in later through the
      :attr:`backend` setter — what the GUI does before the user picks a provider.
   :param WorkerExecutor executor: A pre-built executor to share, or ``None`` to create
      one. See :attr:`executor`.
   :param int max_tool_iterations: Safety cap on consecutive tool calls in a single turn
   :param float execution_timeout: Per-call timeout for the code executor, in seconds
      (default: 60.0)
   :param bool include_viser: Describe the Viser visualizer API in the system prompt too,
      so the agent knows it can drive the viewer

   **Methods:**

   .. method:: run(user_prompt, on_tool_output=None)

      Process one user turn, blocking until the LLM produces its final text. The agent may
      execute Python several times within the turn, up to ``max_tool_iterations``.

      :param str user_prompt: The user's natural-language request
      :param callable on_tool_output: Optional ``callback(code, result_dict)``, invoked after
         each tool execution — how the CLI echoes what the agent ran
      :returns: The LLM's final text response
      :rtype: str
      :raises AgentStoppedError: If :meth:`stop` was called during the turn

   .. method:: chat(user_prompt, on_tool_output=None)

      Streaming counterpart to :meth:`run`. Tool calls still execute synchronously; the final
      response is streamed token-by-token when the backend supports it (Anthropic,
      OpenAI-compatible) and yielded whole otherwise.

      :param str user_prompt: The user's natural-language request
      :param callable on_tool_output: Optional ``callback(code, result_dict)``
      :returns: Generator of string tokens from the final response
      :rtype: Generator[str, None, None]

      .. note::

         :meth:`stop` does **not** interrupt ``chat()`` — only :meth:`run` checks the stop
         flag. Use :meth:`run` if you need a turn to be interruptible.

   .. method:: stop()

      Signal the current :meth:`run` to abort once the in-flight LLM call returns, raising
      :class:`AgentStoppedError` in the thread running the turn. Safe to call from another
      thread — this is what the GUI's Stop button does. The flag is cleared at the start of
      the next :meth:`run`.

   .. method:: reset(restart_executor=True)

      Clear the conversation history.

      :param bool restart_executor: Also restart the executor subprocess, wiping all Python
         state including the planner and any variables (default: ``True``). Pass ``False`` to
         forget the conversation but keep the world you built.

   .. method:: save_chat(path, scene=None)

      Write this conversation to ``path`` as JSON. See :doc:`persistence`.

      :param path: Destination for the transcript
      :param str scene: Optional path of the scene this chat ran against, recorded as
         provenance. It cannot be discovered automatically — the planner lives in the
         executor subprocess.
      :returns: The path written
      :rtype: pathlib.Path

   .. method:: load_chat(path)

      Restore a saved conversation, replacing this agent's history. The messages only: no
      saved code is re-executed and the executor is not reset, so whatever you had defined
      before the load is still defined. A note is appended to the system prompt telling the
      model the transcript came from a file and that the interpreter and planning world may
      not match what it describes.

      :param path: The transcript file

   .. method:: close()

      Shut down the executor subprocess — but only if this agent created it. An executor
      passed in through the constructor is left running, since the caller that supplied it
      owns it.

   **Attributes:**

   .. attribute:: messages

      The conversation so far, as a list of dicts in OpenAI message format. A copy — mutating
      it does not affect the agent.

   .. attribute:: backend

      The LLM backend. Assignable: setting it hot-swaps the provider and **preserves the
      conversation history**, which is how the GUI's backend dropdown switches mid-session.

   .. attribute:: executor

      The persistent Python executor holding the planner and your variables. Read-only,
      exposed so a caller can hand the *same* live executor to another agent instance rather
      than starting a second subprocess with its own separate planner.

.. exception:: srmp.agent.agent.AgentStoppedError

   Raised by :meth:`~srmp.agent.MotionPlanningAgent.run` when :meth:`stop` was called during
   the turn. Import it from the module rather than the package — unlike
   ``MotionPlanningAgent``, it is not listed in ``srmp.agent.__all__``:

   .. code-block:: python

      from srmp.agent.agent import AgentStoppedError

Data Types
----------

Pose
~~~~

.. class:: srmp.Pose()

   Represents a 6DOF pose (position and orientation).

   **Attributes:**

   .. attribute:: p

      Position as numpy array [x, y, z]

      :type: numpy.ndarray

   .. attribute:: q

      Orientation as quaternion [x, y, z, w]

      :type: numpy.ndarray

GoalConstraint
~~~~~~~~~~~~~~

.. class:: srmp.GoalConstraint(goal_type, target)

   Represents a goal constraint for planning.

   :param GoalType goal_type: Type of goal constraint
   :param target: Target specification (joint angles or poses)

GoalType
~~~~~~~~

.. class:: srmp.GoalType

   Enumeration of goal constraint types.

   .. attribute:: JOINTS

      Goal specified as joint angles

   .. attribute:: POSE

      Goal specified as end-effector pose

Trajectory
~~~~~~~~~~

.. class:: srmp.Trajectory

   Represents a planned trajectory.

   **Attributes:**

   .. attribute:: positions

      List of joint configurations along the trajectory

      :type: list

   .. attribute:: velocities

      List of joint velocities along the trajectory

      :type: list

   .. attribute:: accelerations

      List of joint accelerations along the trajectory

      :type: list


Robot Registry
--------------

The ``srmp.robots`` module provides functions for managing robot data downloads and registration.
See :doc:`robot_registry` for detailed usage.

.. module:: srmp.robots

Functions
~~~~~~~~~

.. function:: download(name, force=False)

   Download a robot's data from the registry.

   :param str name: Robot name (e.g., "panda", "so101")
   :param bool force: Re-download even if already cached (default: False)
   :returns: Path to the downloaded robot directory
   :rtype: pathlib.Path
   :raises DownloadError: If download fails

.. function:: download_all(force=False)

   Download all available robots.

   :param bool force: Re-download even if already cached (default: False)
   :returns: Path to the robots cache directory
   :rtype: pathlib.Path

.. function:: get(name)

   Get robot information by name.

   :param str name: Robot name
   :returns: Robot information including paths and metadata
   :rtype: RobotInfo
   :raises RobotNotFoundError: If robot not found

.. function:: info(name)

   Alias for :func:`get`.

.. function:: list_available()

   List all available robots.

   :returns: Dictionary with keys "remote", "local", and "custom"
   :rtype: dict

.. function:: register(name, urdf_path, srdf_path, end_effector, description=None, default_qpos=None, joint_names=None, gripper_joint_names=None)

   Register a custom robot for easy reuse.

   :param str name: Unique name for the robot
   :param str urdf_path: Path to URDF file
   :param str srdf_path: Path to SRDF file
   :param str end_effector: End effector link name
   :param str description: Optional description
   :param list default_qpos: Optional default joint configuration
   :param list joint_names: Optional list of joint names
   :param list gripper_joint_names: Optional list of gripper joint names. When set, robots
      added via :meth:`~srmp.PlannerInterface.add_robot` use this classification by default,
      driving those joints with :meth:`~srmp.PlannerInterface.set_gripper_qpos`.

.. function:: unregister(name)

   Remove a custom robot registration.

   :param str name: Robot name to unregister

.. function:: get_cache_dir()

   Get the current robot cache directory.

   :returns: Path to cache directory
   :rtype: pathlib.Path

.. function:: set_cache_dir(path)

   Set a custom robot cache directory.

   :param str path: Path to new cache directory

Classes
~~~~~~~

.. class:: RobotInfo

   Information about a registered robot.

   **Attributes:**

   .. attribute:: name

      Robot name

      :type: str

   .. attribute:: urdf_path

      Path to URDF file

      :type: str

   .. attribute:: srdf_path

      Path to SRDF file

      :type: str

   .. attribute:: end_effector

      End effector link name

      :type: str

   .. attribute:: description

      Optional description

      :type: str or None

   .. attribute:: default_qpos

      Optional default joint configuration

      :type: list or None

   .. attribute:: joint_names

      Optional list of joint names

      :type: list or None

   .. attribute:: gripper_joint_names

      Optional list of gripper joint names, used by default when this robot is added via
      :meth:`~srmp.PlannerInterface.add_robot`

      :type: list or None

Exceptions
~~~~~~~~~~

.. exception:: RobotNotFoundError

   Raised when a robot is not found in the registry. The error message includes
   available robots and instructions for downloading or registering.

.. exception:: DownloadError

   Raised when downloading robot data fails.


Motion Primitives Configuration
-------------------------------

Motion primitives define the discrete actions available to the robot during planning. SRMP uses YAML configuration files to define motion primitive families and their properties.

YAML Configuration Files
~~~~~~~~~~~~~~~~~~~~~~~~~

**File Structure:**

.. code-block:: yaml

   <family_name>:
     <primitive_name>:
       mprim_sequence:
         - [0, 0, 0, ...]  # Always starts with origin (all zeros)
         - [delta1, delta2, ...]  # Delta values from origin
         - [delta1, delta2, ...]  # Additional steps (optional)
       mprim_sequence_transition_costs: [cost1, cost2, 0]  # Last is always 0
       mprim_sequence_transition_times: [time1, time2, 0]  # Optional timing
       generate_negative: true/false  # Whether to generate negative deltas

**Key Components:**

- **Family Name**: Groups related primitives (e.g., ``long_primitives``, ``short_primitives``)
- **Primitive Name**: Unique identifier for each motion primitive
- **mprim_sequence**: Sequence of states, always starting with zeros (origin)
- **mprim_sequence_transition_costs**: Cost for each transition in the sequence
- **mprim_sequence_transition_times**: Optional time constraints for each transition
- **generate_negative**: Automatically creates negative versions of primitives

**Units:**

- **Joint space (manipulators)**: Degrees for angular movements

**Example: 7DOF Manipulator Primitives**

.. code-block:: yaml

   long_primitives:
     joint0:
       mprim_sequence:
         - [0, 0, 0, 0, 0, 0, 0]  # Origin state
         - [15, 0, 0, 0, 0, 0, 0]  # Move joint 0 by 15 degrees
       mprim_sequence_transition_costs: [1, 0]
       generate_negative: true

     joint1:
       mprim_sequence:
         - [0, 0, 0, 0, 0, 0, 0]
         - [0, 15, 0, 0, 0, 0, 0]  # Move joint 1 by 15 degrees
       mprim_sequence_transition_costs: [1, 0]
       generate_negative: true

   short_primitives:
     joint0:
       mprim_sequence:
         - [0, 0, 0, 0, 0, 0, 0]
         - [7, 0, 0, 0, 0, 0, 0]  # Move joint 0 by 7 degrees
       mprim_sequence_transition_costs: [1, 0]
       generate_negative: true

**Example: Timed Motion Primitives**

.. code-block:: yaml

   long_primitives:
     joint0:
       mprim_sequence:
         - [0, 0, 0, 0, 0, 0, 0]
         - [15, 0, 0, 0, 0, 0, 0]
       mprim_sequence_transition_costs: [1, 0]
       mprim_sequence_transition_times: [1, 0]  # 1 time unit per transition
       generate_negative: true

Creating Custom Motion Primitives
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Step 1: Define YAML Configuration**

1. Create a new YAML file with appropriate naming (e.g., ``custom_7dof_mprim.yaml``)
2. Define motion primitive families based on your robot's requirements
3. Specify primitives for each joint
4. Set appropriate costs and timing constraints

**Step 2: File Naming Convention**

- **Manipulators**: ``<robot_type>_<dof>dof[_additional_info]_mprim.yaml``
- **Timed variants**: Include ``_timed`` in the filename for multi-robot coordination

**Step 3: Integration with Planner**

.. code-block:: python

   # Use custom motion primitives
   planner.make_planner(["robot_name"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "mprim_path": "/path/to/custom_7dof_mprim.yaml"
   })

   # For multi-robot with custom primitives per robot
   planner.make_planner(["robot1", "robot2"], {
       "planner_id": "xECBS",
       "mprim_path_robot1": "/path/to/robot1_timed_mprim.yaml",
       "mprim_path_robot2": "/path/to/robot2_timed_mprim.yaml"
   })

**Design Guidelines:**

1. **Start Simple**: Begin with single-joint movements before complex combinations
2. **Balance Resolution vs Speed**: More primitives = finer control but slower planning
3. **Cost Weighting**: Use costs to prefer certain types of movements
4. **Symmetric Movements**: Use ``generate_negative: true`` for symmetric joint movements
5. **Multi-Robot**: Use timed primitives (``_timed_mprim.yaml``) for coordination

**Available Primitive Files:**

The SRMP package includes pre-configured motion primitive files for manipulators:

- ``manip_6dof_mprim.yaml`` - 6DOF manipulator primitives
- ``manip_7dof_mprim.yaml`` - 7DOF manipulator primitives
- ``manip_7dof_timed_mprim.yaml`` - 7DOF with timing for multi-robot coordination



Examples
--------

Basic Single Robot Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import numpy as np

   # Create planner
   planner = srmp.PlannerInterface()

   # Add robot (downloads automatically if needed)
   planner.add_robot("panda")

   # Add obstacle
   obstacle_pose = srmp.Pose()
   obstacle_pose.p = np.array([0.5, 0.2, 0.4])
   planner.add_box("obstacle", np.array([0.1, 0.1, 0.4]), obstacle_pose)

   # Configure planner
   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10.0"
   })

   # Plan trajectory
   start_state = np.radians([0, -45, 0, -135, 0, 90, 45])

   goal_pose = srmp.Pose()
   goal_pose.p = np.array([0.6, 0.0, 0.5])
   goal_pose.q = np.array([0, 0, 0, 1])
   goal = srmp.GoalConstraint(srmp.GoalType.POSE, [goal_pose])

   trajectory = planner.plan(start_state, goal)



Multi-Robot Example
~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import numpy as np

   # Create planner
   planner = srmp.PlannerInterface()

   # Add two robots using the registry
   planner.add_robot("panda", name="panda0")
   planner.add_robot("panda", name="panda1")

   # Set base poses
   for i in range(2):
       pose = srmp.Pose()
       pose.p = np.array([(-1)**i * 0.5, 0.5, 0])
       pose.q = np.array([1, 0, 0, 0])
       planner.set_base_pose(f"panda{i}", pose)

   # Configure multi-robot planner
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

   # Plan trajectories
   start_states = {
       "panda0": np.radians([-40, 0, 0, -85, 0, 57, 0]),
       "panda1": np.radians([-40, 0, 0, -85, 0, 57, 0])
   }

   goal_states = {
       "panda0": np.radians([40, 0, 0, -70, 0, 50, 0]),
       "panda1": np.radians([40, 0, 0, -95, 0, 67, 0])
   }

   goal_constraints = {}
   for name, goal_state in goal_states.items():
       goal_constraints[name] = srmp.GoalConstraint(srmp.GoalType.JOINTS, [goal_state])

   trajectories = planner.plan_multi(start_states, goal_constraints)

Simulator Integration Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import sapien
   import numpy as np

   # Create SAPIEN scene
   scene = sapien.Scene()
   scene.add_ground()

   # Add some objects to scene
   builder = scene.create_actor_builder()
   builder.add_box_collision(half_size=[0.1, 0.1, 0.1])
   builder.add_box_visual(half_size=[0.1, 0.1, 0.1])
   box = builder.build_kinematic()
   box.set_pose(sapien.Pose([0.5, 0.0, 0.5]))

   # Create planner
   planner = srmp.PlannerInterface()

   # Add robot
   planner.add_robot("panda")

   # Import scene objects automatically
   planner.read_sim(scene, "sapien")

   # Continue with planning as usual...

Point Cloud Example
~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   import srmp
   import numpy as np

   # Create planner
   planner = srmp.PlannerInterface()

   # Add robot
   planner.add_robot("panda")

   # Load point cloud from file (example formats: PLY, PCD, or custom)
   # For this example, we'll generate a synthetic point cloud

   # Generate a point cloud representing a table surface
   table_points = []
   for x in np.linspace(0.2, 0.8, 30):
       for y in np.linspace(-0.3, 0.3, 20):
           table_points.append([x, y, 0.4])  # Table at height 0.4m

   table_cloud = np.array(table_points)

   # Generate a point cloud representing a wall
   wall_points = []
   for y in np.linspace(-0.5, 0.5, 40):
       for z in np.linspace(0.0, 1.5, 60):
           wall_points.append([0.9, y, z])  # Wall at x=0.9m

   wall_cloud = np.array(wall_points)

   # Add point clouds to planner with different resolutions
   planner.add_point_cloud("table_surface", table_cloud, resolution=0.01)
   planner.add_point_cloud("wall", wall_cloud, resolution=0.02)

   # Configure planner
   planner.make_planner(["panda"], {
       "planner_id": "wAstar",
       "heuristic": "bfs",
       "weight": "10.0"
   })

   # Plan around point cloud obstacles
   start_state = np.radians([0, -45, 0, -135, 0, 90, 45])

   # Goal pose that requires navigating around the point cloud obstacles
   goal_pose = srmp.Pose()
   goal_pose.p = np.array([0.7, 0.1, 0.6])  # Above the table, near the wall
   goal_pose.q = np.array([0, 0, 0, 1])
   goal = srmp.GoalConstraint(srmp.GoalType.POSE, [goal_pose])

   trajectory = planner.plan(start_state, goal)

   if trajectory:
       print(f"Successfully planned around point cloud obstacles")
       print(f"Trajectory length: {len(trajectory.positions)} waypoints")
   else:
       print("Planning failed - point cloud obstacles may block all paths")

   # Point cloud loading from files (common formats)
   def load_point_cloud_from_ply(filename):
       """Load point cloud from PLY file"""
       # This is a simplified example - use libraries like Open3D for robust loading
       points = []
       with open(filename, 'r') as f:
           lines = f.readlines()
           # Skip PLY header, find vertex data
           vertex_start = False
           for line in lines:
               if line.strip() == "end_header":
                   vertex_start = True
                   continue
               if vertex_start and line.strip():
                   coords = line.strip().split()
                   if len(coords) >= 3:
                       points.append([float(coords[0]), float(coords[1]), float(coords[2])])
       return np.array(points)

   def load_point_cloud_from_txt(filename):
       """Load point cloud from simple text file (x y z per line)"""
       return np.loadtxt(filename)

   # Usage with file loading
   # point_cloud = load_point_cloud_from_ply("/path/to/scan.ply")
   # planner.add_point_cloud("scanned_object", point_cloud, resolution=0.005)

   # Point cloud from sensor data (example with simulated LiDAR-style data)
   def generate_lidar_point_cloud(robot_pose, num_rays=360, max_range=5.0):
       """Generate simulated LiDAR point cloud"""
       points = []
       for i in range(num_rays):
           angle = 2 * np.pi * i / num_rays
           # Simulate ray hitting objects at various distances
           distance = np.random.uniform(0.5, max_range)
           x = robot_pose[0] + distance * np.cos(angle)
           y = robot_pose[1] + distance * np.sin(angle)
           z = robot_pose[2] + np.random.uniform(-0.1, 0.1)  # Some height variation
           points.append([x, y, z])
       return np.array(points)

   # Simulate sensor-based point cloud
   robot_position = [0, 0, 0.5]
   sensor_cloud = generate_lidar_point_cloud(robot_position)
   planner.add_point_cloud("sensor_obstacles", sensor_cloud, resolution=0.03)


Visualization Classes
---------------------

SRMP ships an optional Viser-based visualizer, :class:`~srmp.ViserPlannerInterface`, which
keeps the 3D scene in sync automatically as robots and objects are added or removed. Get one
either via :meth:`~srmp.PlannerInterface.start_visualizer` on an existing planner, or by
constructing :class:`~srmp.ViserPlannerInterface` directly — both produce the same object.

See the :doc:`visualization` page for installation instructions and full
usage examples.

ViserPlannerInterface
~~~~~~~~~~~~~~~~~~~~~

.. class:: srmp.ViserPlannerInterface(port=8080, share=False)

   Interactive Viser-based visualizer.  Inherits all methods of
   :class:`~srmp.PlannerInterface`.  Provides *bidirectional* browser ↔ Python
   communication: sliders, dropdowns, and buttons in the browser directly invoke
   Python callbacks.

   :param int port: TCP port for the Viser web server (default: ``8080``)
   :param bool share: Request a public share URL from Viser
       (default: ``False``)

   ``viser`` and its ``trimesh`` dependency are installed automatically with ``srmp``.

   **Additional Methods:**

   .. method:: visualize(open_browser=True, add_grid=True)

      Start the Viser server (if not already running) and render the scene.

      :param bool open_browser: Unused; kept for API symmetry
      :param bool add_grid: Whether to render a ground grid
          (default: ``True``)

   .. method:: animate_trajectory(trajectories, dt=0.05, robot_name=None)

      Replay a planned trajectory in the 3D viewer.

      :param trajectories: A single :class:`~srmp.Trajectory` or a ``dict``
          mapping robot names to trajectories.
      :param float dt: Seconds between frames (default: ``0.05``)
      :param str robot_name: Robot name when a single trajectory is supplied.

   .. method:: add_robot_controls(robot_name)

      Add interactive joint sliders, visibility toggles, and a Reset button
      for the named robot.  Moving a slider updates both the Viser geometry
      and the planner backend in real time.

      :param str robot_name: Name of the robot

   .. method:: add_plan_controls(on_plan=None)

      Add a collapsible "Plan" folder to the sidebar for goal-driven planning.  Checking a
      robot's box places a draggable 6-DOF goal gizmo (plus a semi-transparent "ghost" preview
      robot) at its end-effector; dragging the gizmo updates the ghost via live IK.  Clicking
      "Plan to goal" runs the planner against the dragged goal(s) and animates the resulting
      trajectory.  Replaces the older single-robot ``add_ee_drag_control`` gizmo with a
      multi-robot, planner-integrated workflow.

      :param on_plan: Optional ``callable(prompt: str)``.  When provided (agent mode) the
          "Plan to goal" button calls ``on_plan(prompt)`` instead of planning directly.  When
          ``None`` (standalone mode) it builds the planner and animates the trajectory itself.

   .. method:: add_object_controls()

      Add drag gizmos to every object currently in the scene, so they can be repositioned
      directly in the browser.  Each gizmo moves the object's visual representation
      immediately; the underlying collision world is updated shortly after dragging stops.

      :returns: The folder handle (call ``.remove()`` to tear it down)

   .. method:: add_controls_panel(executor=None, on_result=None, busy_lock=None, on_plan=None, agent=None, agent_preset=None, agent_backend_error=None)

      Add (or update) the top-level "Controls" launcher panel, with on-demand buttons for
      Robot Controls, Object Controls, Plan Controls, Console, and AI Agent.  Called
      automatically at the end of :meth:`visualize`, so every ``ViserPlannerInterface`` /
      ``start_visualizer("viser")`` scene gets this launcher by default — it is the backbone
      of the :doc:`agent_mode` GUI. Calling it again only updates the stored callback
      parameters rather than duplicating the panel.

      :param executor: Forwarded to the Console panel, and reused for the "AI Agent" panel
      :param on_result: Forwarded to the Console panel
      :param busy_lock: Forwarded to the Console panel
      :param on_plan: Forwarded to :meth:`add_plan_controls`
      :param agent: Pre-built agent (see :doc:`agent_mode`). If provided, the "AI Agent" panel
          opens immediately instead of waiting for the button click
      :param agent_preset: Backend preset matching ``agent``'s initial backend
      :param agent_backend_error: Error string if ``agent``'s initial backend failed to construct

   .. method:: add_gui_controls()

      Add an interactive "Object Controls" panel to the browser sidebar.
      The panel includes type dropdown, position vector, size sliders, mesh
      path input, file-browser button, and **Add / Update / Remove** buttons
      that invoke Python callbacks.

   .. method:: add_obstacle_from_gui()

      Add an obstacle using the current GUI widget values.

      :returns: Name of the added object, or ``None`` on failure.
      :rtype: str or None

   .. method:: update_object_from_gui(object_name)

      Move an existing object to the position shown in the GUI.

      :param str object_name: Name of the object to update
      :returns: ``True`` if successful
      :rtype: bool

   .. method:: load_object_to_gui(object_name)

      Load an object's properties into the GUI widgets for editing.

      :param str object_name: Name of the object to load
      :returns: ``True`` if successful
      :rtype: bool

   .. method:: stop()

      Shut down the Viser web server.

   .. attribute:: url

      The Viser browser URL (e.g. ``http://localhost:8080``).  Returns an
      informational string if the server has not been started yet.

