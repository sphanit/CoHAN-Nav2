AgentPosePredict
================

Service Definition
------------------

.. code-block:: none

    # request constants
    uint8 VELOCITY_OBSTACLE=0
    uint8 EXTERNAL=1
    uint8 BEHIND_ROBOT=2
    uint8 PREDICTED_GOAL=3
    # request fields
    uint8                                     type
    float64[]                                 predict_times
    int64[]                                   ids
    ---
    # response fields
    agent_path_prediction/PredictedPoses[]    predicted_agents_poses

Field Descriptions
------------------

* ``type`` (uint8)
    Prediction type (use the defined constants).

* ``predict_times`` (float64[])
    Times at which to predict poses.

* ``ids`` (int64[])
    Agent IDs to predict for.

* ``predicted_agents_poses`` (PredictedPoses[])
    Predicted poses for requested agents.

Example Usage (ROS 2)
---------------------

.. code-block:: python

    # ROS 2 Python client example
    import rclpy
    from rclpy.node import Node
    from agent_path_prediction.srv import AgentPosePredict

    rclpy.init()
    node = rclpy.create_node('agent_pose_predict_client')

    client = node.create_client(AgentPosePredict, 'agent_pose_predict')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service not available')
        rclpy.shutdown()

    req = AgentPosePredict.Request()
    req.type = 3  # PREDICTED_GOAL
    req.predict_times = [0.5, 1.0]
    req.ids = [1, 2]

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resp = future.result()
    if resp is None:
        node.get_logger().error('Service call failed')
    else:
        # resp.predicted_agents_poses is an array of PredictedPoses
        node.get_logger().info('Received predicted poses: %d' % len(resp.predicted_agents_poses))

    node.destroy_node()
    rclpy.shutdown()
