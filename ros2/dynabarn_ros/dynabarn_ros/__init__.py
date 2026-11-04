"""ROS 2 interface for dynabarn-core.

* ``mujoco_node``: runs a scenario in MuJoCo (``DynaBarn``) and publishes it.
* ``gazebo.launch.py``: exports the scenario for Gazebo and starts gz-sim plus
  ros_gz_bridge, configured from the same interface description.

Both give the same topics, message types and TF frames (see
``dynabarn_core.interface``).
"""
