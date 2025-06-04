.. code-block:: bash

   ros2_humble_src
   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 service call /go_to_relative_position  tm_msgs/srv/GoToRelativePosition   "{x: 0.00, y: 30., z: 0., rx_deg: 0., ry_deg: 0., rz_deg: 0., speed_percent: 20, timeout_s: 300.}"