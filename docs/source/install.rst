Install
=======

Preparing things (Native)
-------------------------

1. Install ROS `Humble <https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html>`_ on Ubuntu 22.04. 
2. This installation assumes that the `ROS <https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html>`_ is already installed along with the `Nav2 <https://docs.nav2.org/development_guides/build_docs/index.html>`_. Otherwise, please install them before continuing to the next steps.

3. Clone the git repository::

       git clone https://github.com/sphanit/CoHAN-Nav2 -b main ~
       cd ~/CoHAN-Nav2

4. Install the dependencies using rosdep and pull the submodules::

       ./install-deps.sh

5. Build the package. Everything will be taken care of by the script. Just run compile.sh script::

       ./compile.sh

6. Once it is built, you need to source setup.bash file to use these packages along with other ROS packages inside the system. Then you can run the example launch:: 
       
       source install/setup.bash && roslaunch cohan_sim_navigation cohan_sim_pr2.launch

   If everything is installed correctly, you should see rviz opening and you can move the robot by giving it a goal (using 2D Nav Goal tool). 


Using Docker
------------

1. Clone the git repository::

       git clone https://github.com/sphanit/CoHAN-Nav2 -b main ~
       cd ~/CoHAN-Nav2

2. Navigate to docker directory in the folder and build the image. For example::

       cd ~/CoHAN-Nav2/docker/humble
       ./build-docker.sh

3. Activate the image and do the steps for building. For example::

       source ~/CoHAN-Nav2/docker/humble/run-docker.sh

4. Inside the docker, install deps and build::

       ./install-deps.sh && ./compile.sh

5. You can then source and run as previously::

       source install/setup.bash && roslaunch cohan_sim_navigation cohan_sim_pr2.launch





