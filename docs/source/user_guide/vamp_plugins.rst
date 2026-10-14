VAMP Plugins
============

The VAMP validity backend (``validity_backend: "vamp"`` or ``"auto"``, see :doc:`api`) checks
collisions with SIMD-vectorized sphere models, much faster than coal. It needs a VAMP model
of the robot. A few are built into srmp; any other robot can get one as a **plugin**: a
``.so`` that ``import srmp`` loads, built without recompiling srmp.

.. note::

   Plugins require Linux x86-64. Building one (``srmp vamp add-robot``) runs its tools in
   Docker containers when Docker is available.

Published plugins
-----------------

For registry robots with a published plugin, nothing needs doing: the first
:meth:`~srmp.PlannerInterface.add_robot` downloads it, checks it, installs it and uses it.

.. code-block:: python

   planner = srmp.PlannerInterface()
   planner.add_robot("kinova_gen3_robotiq_2f85")   # first time: download + a short check
   planner.is_robot_vamp_supported("kinova_gen3_robotiq_2f85")   # True

Before installing, srmp compares the published files against the checksums the registry
records, checks the plugin against the registry robot (kinematics, link coverage, joint
order, allowed-collision pairs), and runs an agreement test against coal. A plugin that fails
any of these is not installed, and the robot plans on coal. The first ``add_robot`` of such a
robot can take a minute or two; later ones load the installed plugin instantly. Without
network access, ``add_robot`` silently uses coal.

The same download is available as a command:

.. code-block:: console

   $ srmp vamp fetch kinova_gen3_robotiq_2f85

Plugins are tied to the srmp build that loads them (its *ABI tag*), so a new srmp release
comes with its own published plugins.

Building a plugin for your robot
--------------------------------

You need a **sphere-only** URDF of the robot: every link's collision geometry replaced by
spheres (for example with URDFApproxGeom or foam). srmp then generates the VAMP model with `cricket <https://github.com/CoMMALab/cricket>`_,
compiles it, checks it, and installs it:

.. code-block:: console

   $ srmp vamp add-robot --urdf my_robot_spherized.urdf --srdf my_robot_spherized.srdf \
         --end-effector tool0 --name my_robot

- cricket and the compiler run in two containers pulled from Docker Hub
  (``imishani/srmp-cricket`` and ``imishani/srmp-vamp-builder``, about 1.9 GB together); srmp
  asks before pulling. ``--build-images`` builds them locally instead, and a failed pull falls
  back to that on its own.
- ``--srdf`` is generated when omitted.
- ``--name`` must match the name you give the robot in :meth:`~srmp.PlannerInterface.add_robot`.

If you already have a cricket-generated header, build from it instead:

.. code-block:: console

   $ srmp vamp add-robot --from-module my_robot.hh --name my_robot \
         --spherized-urdf my_robot_spherized.urdf --spherized-srdf my_robot_spherized.srdf \
         --end-effector tool0

What the agreement test checks
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Before installing, ``add-robot`` samples configurations (``--samples``, default 2000) and
compares the plugin's verdicts with coal's on the **same** sphere URDF. A correct plugin agrees
exactly; any disagreement fails the build. This proves the plugin is the module it claims to be
and was built correctly.

It does **not** check that the spheres cover the robot: if a link has no spheres, both sides
agree that nothing is there. Give ``--mesh-robot`` (a registry name, or the mesh URDF with
``--mesh-srdf``) so ``add-robot`` also fails when a link that has collision geometry in the real
robot has no spheres.

Managing plugins
----------------

.. code-block:: console

   $ srmp vamp list              # installed plugins, load status and how each was checked
   $ srmp vamp remove my_robot   # delete a plugin
   $ srmp vamp rebuild-all       # rebuild plugins after an srmp upgrade changed the ABI tag

An installed plugin is never overwritten: ``remove`` it first, then add it again.

After an srmp upgrade, plugins built for the previous version stop loading, and ``import srmp``
warns once, naming them and ``srmp vamp rebuild-all``. Set ``SRMP_VAMP_AUTO_REBUILD=1`` to have
``import srmp`` rebuild them itself; it never downloads anything for that.

Where plugins live
------------------

``add-robot`` and ``fetch`` install into the first directory in ``SRMP_VAMP_PLUGIN_PATH`` when
that is set, otherwise ``~/.cache/srmp/vamp_plugins/``. ``import srmp`` loads plugins from every
directory in ``SRMP_VAMP_PLUGIN_PATH`` and then ``~/.cache/srmp/vamp_plugins/``.

Limits
------

- VAMP models keep the gripper fingers at a fixed opening, so ``set_gripper_qpos`` does not
  affect them. Use coal for motions where the fingers close on or near an object.
- A model is used only for a robot whose kinematics match it (see :doc:`api`); a plugin named
  after one robot will not be applied to a different one.
- Sphere models approximate the robot. Coverage depends on the spheres you provide.
