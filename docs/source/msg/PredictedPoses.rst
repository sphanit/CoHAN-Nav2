PredictedPoses
==============

Message Definition
-----------------

.. code-block:: none

    int32                                       id
    geometry_msgs/PoseWithCovarianceStamped[]   poses
    geometry_msgs/TwistStamped                  start_velocity

Field Descriptions
-----------------

* ``id`` (int32)
    Identifier for the agent these poses belong to.

* ``poses`` (geometry_msgs/PoseWithCovarianceStamped[])
    Time-stamped poses with covariance for predicted positions.

* ``start_velocity`` (geometry_msgs/TwistStamped)
    The starting velocity for the predicted sequence.

Links
-----

* `geometry_msgs/PoseWithCovarianceStamped Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html>`_
* `geometry_msgs/TwistStamped Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistStamped.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    from agent_path_prediction.msg import PredictedPoses

    pp = PredictedPoses()
    pp.id = 1
    # pp.poses.append(...)  # use geometry_msgs.PoseWithCovarianceStamped
