Optimize
========

Service Definition
-----------------

.. code-block:: none

    # Get optimized timed elastic bands for given agents and robot plans

    # Request
    nav_msgs/Path              robot_plan
    AgentPathArray            agent_plan_array
    int64[]                   agents_ids
    ---
    # Response
    bool                      success
    string                    message
    geometry_msgs/Twist       cmd_vel
    Trajectory                robot_trajectory
    AgentTrajectoryArray     human_trajectories

Request Fields
-------------

* ``robot_plan`` (nav_msgs/Path)
    Initial path plan for the robot.

* ``agent_plan_array`` (AgentPathArray)
    Array of initial path plans for the agents.

* ``agents_ids`` (int64[])
    Array of agent IDs to consider in the optimization.

Response Fields
--------------

* ``success`` (bool)
    Indicates if the optimization was successful.

* ``message`` (string)
    Status message or error description.

* ``cmd_vel`` (geometry_msgs/Twist)
    Optimized velocity command for the robot.

* ``robot_trajectory`` (Trajectory)
    Optimized trajectory for the robot.

* ``human_trajectories`` (AgentTrajectoryArray)
    Optimized trajectories for the human agents.

Links
-----

* `nav_msgs/Path Message <http://docs.ros.org/en/api/nav_msgs/html/msg/Path.html>`_
* `geometry_msgs/Twist Message <http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html>`_
* :doc:`../msg/AgentPathArray`
* :doc:`../msg/Trajectory`
* :doc:`../msg/AgentTrajectoryArray`

Example Usage (ROS 2)
---------------------

.. code-block:: python

    # ROS 2 Python client example
    import rclpy
    from rclpy.node import Node
    from cohan_msgs.srv import Optimize
    from nav_msgs.msg import Path
    from cohan_msgs.msg import AgentPathArray

    rclpy.init()
    node = rclpy.create_node('optimize_client')

    client = node.create_client(Optimize, 'optimize')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service not available')
        rclpy.shutdown()

    req = Optimize.Request()
    req.robot_plan = Path()
    # req.agent_plan_array = AgentPathArray()
    req.agents_ids = [1, 2]

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    if resp is None:
        node.get_logger().error('Service call failed')
    elif resp.success:
        robot_traj = resp.robot_trajectory
        human_trajs = resp.human_trajectories
        cmd_vel = resp.cmd_vel
        node.get_logger().info('Optimization successful')
    else:
        node.get_logger().warn('Optimization failed: %s' % resp.message)

    node.destroy_node()
    rclpy.shutdown()
