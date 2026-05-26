PlanningMode
============

Message Definition
-----------------

.. code-block:: none

    int8    plan_mode
    int8    predict_mode
    int32[] moving_humans
    int32[] still_humans

Field Descriptions
-----------------

* ``plan_mode`` (int8)
    Planner mode code (application-specific).

* ``predict_mode`` (int8)
    Prediction mode code (application-specific).

* ``moving_humans`` (int32[])
    IDs of humans currently classified as moving.

* ``still_humans`` (int32[])
    IDs of humans currently classified as still.

Example Usage
------------

.. code-block:: python

    # Python
    from hateb_local_planner.msg import PlanningMode

    pm = PlanningMode()
    pm.plan_mode = 0
    pm.predict_mode = 0
    pm.moving_humans = [1]
