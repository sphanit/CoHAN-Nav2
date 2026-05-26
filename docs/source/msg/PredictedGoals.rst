PredictedGoals
==============

Message Definition
-----------------

.. code-block:: none

    std_msgs/Header header
    PredictedGoal[] goals

Field Descriptions
-----------------

* ``header`` (std_msgs/Header)
    Standard ROS message header containing timestamp and frame information.

* ``goals`` (PredictedGoal[])
    Array of predicted goals.

Links
-----

* `std_msgs/Header Message <http://docs.ros.org/en/api/std_msgs/html/msg/Header.html>`_
* :doc:`PredictedGoal Message <PredictedGoal>`

Example Usage
------------

.. code-block:: python

    # Python
    from agent_path_prediction.msg import PredictedGoals, PredictedGoal

    msg = PredictedGoals()
    # msg.header.frame_id = "map"
    msg.goals.append(PredictedGoal())
