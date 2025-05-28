.. code-block:: bash

   ros2_humble_src
   ros2 service call /go_to_relative_position  tm_msgs/srv/GoToRelativePosition   "{x: 0.00, y: 0.01, z: 0., rx_deg: 0., ry_deg: 0., rz_deg: 0., speed_percent: 1}"