OptimizationCost
================

Message Definition
-----------------

.. code-block:: none

    # cost types
    int8 TIME_OPTIMALITY=0
    int8 KINEMATIC_DD=1
    int8 KINEMATIC_CL=2
    int8 ROBOT_VEL=3
    int8 AGENT_VEL=4
    int8 ROBOT_ACC=5
    int8 AGENT_ACC=6
    int8 OBSTACLE=7
    int8 DYNAMIC_OBSTACLE=8
    int8 VIA_POINT=9
    int8 AGENT_ROBOT_SAFETY=10
    int8 AGENT_AGENT_SAFETY=11
    int8 AGENT_ROBOT_REL_VEL=12
    int8 AGENT_ROBOT_MIN_DIST=13
    int8 AGENT_ROBOT_VISIBILITY=14

    int8    type
    float64 cost
    float64[] costs_arr

Constants
---------

* ``TIME_OPTIMALITY`` (int8 = 0)
* ``KINEMATIC_DD`` (int8 = 1)
* ``KINEMATIC_CL`` (int8 = 2)
* ``ROBOT_VEL`` (int8 = 3)
* ``AGENT_VEL`` (int8 = 4)
* ``ROBOT_ACC`` (int8 = 5)
* ``AGENT_ACC`` (int8 = 6)
* ``OBSTACLE`` (int8 = 7)
* ``DYNAMIC_OBSTACLE`` (int8 = 8)
* ``VIA_POINT`` (int8 = 9)
* ``AGENT_ROBOT_SAFETY`` (int8 = 10)
* ``AGENT_AGENT_SAFETY`` (int8 = 11)
* ``AGENT_ROBOT_REL_VEL`` (int8 = 12)
* ``AGENT_ROBOT_MIN_DIST`` (int8 = 13)
* ``AGENT_ROBOT_VISIBILITY`` (int8 = 14)

Field Descriptions
-----------------

* ``type`` (int8)
    Cost type code (see constants above).

* ``cost`` (float64)
    The computed cost value.

* ``costs_arr`` (float64[])
    Optional per-component cost array.

Example Usage
------------

.. code-block:: python

    # Python
    from hateb_local_planner.msg import OptimizationCost

    oc = OptimizationCost()
    oc.type = OptimizationCost.TIME_OPTIMALITY
    oc.cost = 1.23
    oc.costs_arr = [0.5, 0.7]
