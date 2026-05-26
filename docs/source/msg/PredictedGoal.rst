PredictedGoal
=============

Message Definition
-----------------

.. code-block:: none

    int32                       id
    geometry_msgs/Pose          goal                                                       

Field Descriptions
-----------------

* ``id`` (int32)
    Identifier for the predicted goal (usually agent id).

* ``goal`` (geometry_msgs/Pose)
    The predicted goal pose.

Links
-----

* `geometry_msgs/Pose Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    from agent_path_prediction.msg import PredictedGoal

    g = PredictedGoal()
    g.id = 1
    # g.goal.position.x = 1.0
