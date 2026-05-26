AgentsInfo
==========

Message Definition
-----------------

.. code-block:: none

    HumanInfo[]           humans
    int32[]               visible
    int32[]               still
    int32[]               moving
    geometry_msgs/Pose2D  robot_pose
    # geometry_msgs/Twist   robot_vel                    

Field Descriptions
-----------------

* ``humans`` (HumanInfo[])
    Array of human information records.

* ``visible`` (int32[])
    IDs of visible humans.

* ``still`` (int32[])
    IDs of humans considered still.

* ``moving`` (int32[])
    IDs of humans considered moving.

* ``robot_pose`` (geometry_msgs/Pose2D)
    Robot's pose in 2D.

Links
-----

* :doc:`HumanInfo Message <HumanInfo>`
* `geometry_msgs/Pose2D Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose2D.html>`_

Example Usage
------------

.. code-block:: python

    # Python
    from agent_path_prediction.msg import AgentsInfo, HumanInfo

    info = AgentsInfo()
    info.humans = [HumanInfo()]
    info.visible = [1]
    info.moving = [1]
    # info.robot_pose.x = 0.0
