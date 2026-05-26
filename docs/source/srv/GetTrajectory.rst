GetTrajectory
=============

Service Definition
------------------

.. code-block:: none

    # Request
    geometry_msgs/PoseStamped    robot_goal
    int64[]                      agents_ids
    ---
    # Response
    bool                         success
    string                       message
    geometry_msgs/Twist          cmd_vel
    Trajectory                   robot_trajectory
    AgentTrajectoryArray        human_trajectories

Request Fields
--------------

* ``robot_goal`` (geometry_msgs/PoseStamped)
    Goal pose for the robot.

* ``agents_ids`` (int64[])
    Array of agent IDs to consider in trajectory planning.

Response Fields
---------------

* ``success`` (bool)
    Indicates if the trajectory generation was successful.

* ``message`` (string)
    Status message or error description.

* ``cmd_vel`` (geometry_msgs/Twist)
    Computed velocity command for the robot.

* ``robot_trajectory`` (Trajectory)
    Planned trajectory for the robot.

* ``human_trajectories`` (AgentTrajectoryArray)
    Predicted trajectories for the human agents.

Links
-----

* `geometry_msgs/PoseStamped Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html>`_
* `geometry_msgs/Twist Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html>`_
* :doc:`../msg/Trajectory`
* :doc:`../msg/AgentTrajectoryArray`

Example Usage (ROS 2)
---------------------

.. code-block:: python

    # ROS 2 Python client example
    import rclpy
    from rclpy.node import Node
    from cohan_msgs.srv import GetTrajectory
    from geometry_msgs.msg import PoseStamped

    rclpy.init()
    node = rclpy.create_node('get_trajectory_client')

    client = node.create_client(GetTrajectory, 'get_trajectory')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service not available')
        rclpy.shutdown()

    req = GetTrajectory.Request()
    req.robot_goal = PoseStamped()
    # req.robot_goal.header.frame_id = 'map'
    req.agents_ids = [1, 2]

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    if resp is None:
        node.get_logger().error('Service call failed')
    elif resp.success:
        robot_traj = resp.robot_trajectory
        human_trajs = resp.human_trajectories
        node.get_logger().info('Trajectory received')
    else:
        node.get_logger().warn('Failed to get trajectory: %s' % resp.message)

    node.destroy_node()
    rclpy.shutdown()
