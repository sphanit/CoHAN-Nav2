CoHAN Sim
=========

CoHAN Sim is a simple 2D robot simulator aimed to support ROS1 and ROS2 versions. The current implementation is simple and uses SDL2 library to build simulation from an .yaml file using similar notations as ROS. It is very simple and straight-forward to use as you just have to modify your existing map (.yaml) files that are used by map_server.

Adding robot and map to the simulator
-------------------------------------

Suppose your current map.yaml (used by map_server) looks like this:

.. code-block:: yaml

    image: map.pgm
    resolution: 0.050000
    origin: [-20.000000, -20.000000, 0.000000]
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196

To simulate a robot in the current map add the following and save it. It will simulate the environment using the map and omni directional robot with a laser and keyboard control.

.. code-block:: yaml

    image: map.pgm
    resolution: 0.050000
    origin: [-20.000000, -20.000000, 0.000000]
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196


    ## CoHAN Sim Part
    # The following are set to move the robot if a keyboard is attached (Adjust them as needed)
    speed: 2.0              # Keyboard linear speed (m/s)
    angular_speed: 1.5708   # Keyboard angular speed (rad/s), default pi/2


    entities:
    - name: robot
        start_x: 2                  # Start position of robot in x
        start_y: 2                  # Start position of robot in y
        start_theta: 0              # Start orientation of robot
        radius: 0.25                # Radius of the robot
        color: [0, 100, 255, 255]   # Color of the robot in the simulator
        use_keyboard: true          # Attach keyboard control, if true, you can move the robot with keyboard
        use_differential: false     # If true, differential kinematics are used to move, otherwise holonomic motion is simulated
        use_laser: true             # Add a laser scanner or not
        laser_range: 7              # Maximum range of the laser
        laser_resolution: 1081      # Maximum number of laser points
        laser_angle: 4.71239        # Angular spread of the laser

Now you can run the simulator and move the robot.

.. code-block:: bash

    # In Terminal 1 
    roscore

.. code-block:: bash

    # In Terminal 2
    rosrun  cohan_sim simros_node -g path/to/map/map.yaml

    # DO NOT Forget to run roscore if you are using rosrun
    # Errors (if no roscore): Error in XmlRpcClient::writeRequest: write error (Connection refused).
    #                        Error in XmlRpcDispatch::work: couldn't find source iterator

    ## You can control the robot using the following keys.
    #######################################################################
    #                                                                     #
    # The simulator have the following GUI support.                       #
    #                                                                     #
    # Pressing r or R - Resets the Simulation                             #
    # Pressing z or Z - Resets the Zoom and Pan to default                #
    # Scroll Key - Zoom in/out                                            #
    # Hold left mouse button - Pan                                        #              
    #                                                                     #
    # For moving the robot to which keyboard is attached,                 #
    #                                                                     #
    #                       Move Forward                                  #
    #                           ^                                         #
    #                          i/I                                        #
    #  Move Left        < j/J       l/L >    Move Right                   #
    # (only holonomic)                      (only holonomic)              #
    #                          k/K                                        #
    #                           v                                         #
    #                      Move Backward                                 #
    #                                                                     #
    # For turnings,                                                       #
    #              Turn Left   << a/A   d/D >>   Turn right               #
    #                                                                     #
    # Turn Head Left   << Shift + a/A   Shift + d/D >>   Turn Head right  #
    #                                                                     #
    #######################################################################


Adding humans or more robots
----------------------------

We can add more than one entity to the simulator. Although the entity is treated like a robot, it can have different features than the first one (the main robot). This feature lets us simulate different kinds of robots or humans. To add two humans along with a robot, do the following:

.. code-block:: yaml

    image: map.pgm
    resolution: 0.050000
    origin: [-20.000000, -20.000000, 0.000000]
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196


    ## CoHAN Sim Part
    # The following are set to move the robot if a keyboard is attached (Adjust them as needed)
    speed: 2.0              # Keyboard linear speed (m/s)
    angular_speed: 1.5708   # Keyboard angular speed (rad/s), default pi/2


    entities:
    - name: robot
        start_x: 2                  # Start position of robot in x
        start_y: 2                  # Start position of robot in y
        start_theta: 0              # Start orientation of robot
        radius: 0.25                # Radius of the robot
        color: [0, 100, 255, 255]   # Color of the robot in the simulator
        use_keyboard: true          # Attach keyboard control, if true, you can move the robot with keyboard
        use_differential: false     # If true, differential kinematics are used to move, otherwise holonomic motion is simulated
        use_laser: true             # Add a laser scanner or not
        laser_range: 7              # Maximum range of the laser
        laser_resolution: 1081      # Maximum number of laser points
        laser_angle: 4.71239        # Angular spread of the laser
      - name: human1
        start_x: 8
        start_y: 2.1
        start_theta: 3.14
        radius: 0.3
        color: [150, 75, 0, 255]
        use_keyboard: false
        use_differential: false
        use_laser: true
        laser_range: 7
        laser_resolution: 1081
        laser_angle: 4.71239
    - name: human2
        start_x: 7.5
        start_y: 5.0
        start_theta: 1.57
        radius: 0.3
        color: [150, 75, 0, 255]
        use_keyboard: false
        use_differential: false
        use_laser: true
        laser_range: 7
        laser_resolution: 1081
        laser_angle: 4.71239

You can add more robots in a similar way. However, keyboard can only be attached to a single robot. Only the first one in the list gets the keyboard control, if multiple robots have it enabled.
