Usage
=====

.. _installation:

Installation
------------

To use SRMP, first install it using pip:

.. code-block:: console

   (.venv) $ pip install srmp

Then, import it in your script and follow the instructions on the docs:

>>> import srmp
>>> planner = srmp.PlannerInterface()

Add a robot model to the world

>>> planner.add_articulation(urdf_path="panda.urdf", 
                             srdf_path="panda.srdf", 
                             name="panda", 
                             end_effector="panda_hand")

Add objects to the environment.

>>> planner.add_box("box", size=np.array([1,2.1,1]), 
                pose=srmp.Pose(p=[0,0,0.4]))
>>> planner.add_mesh("shrimp", path="shrimp.stl", 
                 pose=srmp.Pose(p=[2,0,1], 
                                q=[0,0,0,1]))
Define the start configuration.

>>> q_start = np.radians([0, -45, 0, -135, 0, 90, 45])

Define goal constraint (e.g., end-effector pose).

>>> goal = srmp.GoalConstraint(srmp.GoalType.POSE, list(srmp.Pose([0.6, -0.0, 0.5])))

Configure the planner (e.g., specify type, time limit, heuristics, etc.)

>>> planner.make_planner(["panda"],{"planner_id": 
                      "ARAstar","time_limit":"5"}) 
Compute a trajectory from the start configuration to the goal condition.

>>> trajectory = planner.plan(["panda"], q_start, goal)

Add a second robot model to the world

>>> planner.add_articulation(urdf_path="panda.urdf", 
                         srdf_path="panda.srdf", 
                         name="panda2", 
                         end_effector="panda_hand")
>>> planner.set_base_pose(name="panda2", 
                      pose=srmp.Pose(p=[0.5, 0, 0], 
                                     q=[0,0,0,1]))

Compute a trajectory for both robots using multi-agent planning.

>>> goal_panda_2 = srmp.GoalConstraint(srmp.GoalType.POSE, list(srmp.Pose([0.6, -0.5, 1.0])))

>>> planner.make_planner(["panda", "panda2"], 
                     {"planner_id": "xECBS", 
                      "time_limit" : "5"}) 
>>> trajectory = planner.plan(["panda", "panda2"],
                          [q_start, q_start], 
                          [goal, goal_panda_2])
