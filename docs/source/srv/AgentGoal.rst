AgentGoal
=========

Service Definition
------------------

.. code-block:: none

    # Request
    agent_path_prediction/AgentPose[]  goals
    ---
    bool                               success
    string                             message

Request Fields
--------------

* ``goals`` (agent_path_prediction/AgentPose[])
    Array of goal poses for agents.

Response Fields
---------------

* ``success`` (bool)
    Indicates whether the operation succeeded.

* ``message`` (string)
    Optional status or error message.

Example Usage (ROS 2)
---------------------

.. code-block:: python

    # ROS 2 Python client example
    import rclpy
    from rclpy.node import Node
    from agent_path_prediction.srv import AgentGoal
    from agent_path_prediction.msg import AgentPose

    rclpy.init()
    node = rclpy.create_node('agent_goal_client')

    client = node.create_client(AgentGoal, 'agent_goal')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service not available')
        rclpy.shutdown()

    req = AgentGoal.Request()
    req.goals = [AgentPose()]

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    if resp is None:
        node.get_logger().error('Service call failed')
    elif resp.success:
        node.get_logger().info('Agent goals accepted')
    else:
        node.get_logger().warn('AgentGoal failed: %s' % resp.message)

    node.destroy_node()
    rclpy.shutdown()
