Motion Primitives
=================

Motion primitives define the discrete actions available to the robot during planning. SRMP uses YAML configuration files to define motion primitive families and their properties.

YAML Configuration Files
-------------------------

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
---------------------------------

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
