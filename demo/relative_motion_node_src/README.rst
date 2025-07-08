========
action
========

There is an action server to move the robot relative to its current position.

To launch the action server, you can use the following command:

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2_humble_src && ros2 run demo relative_motion_node

To use the action server, you can call it with the following command:

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2_humble_src && ros2 action send_goal /go_to_relative_position tm_msgs/action/GoToRelativePosition "{x: 0.00, y: 30., z: 0., rx_deg: 0., ry_deg: 0., rz_deg: 0., speed_percent: 20, time_to_top_speed_ms: 300, frame: LASER_ROS2}"

* If you set the frame as an empty string the robot will move relative to its current frame.
* If you set the frame as "LASER_ROS2" the robot will move relative to the frame LASER_ROS2.

The action server will return a result when the robot has reached the goal.
If the robot is not able to reach the goal, it will return a failure.
You can also cancel the goal by using the following command:

.. code-block:: bash

   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2_humble_src && ros2 action cancel /go_to_relative_position