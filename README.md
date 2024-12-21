# lla2local
ROS2 package to convert lat-lon to local X-Y coordinates which are offset from some local lat-lon. X is the distance east of the origin, Y is the distance north
of the origin. There is no sensor fusion - if you need that, use
robot_localization or fuse or other solutions.

The node subscribes to NavSatFix messages and extracts the coordinates and republishes 
them as X-Y coordinates in a PoseStamped message in the 'map' frame. It also publishes tf2 transforms between the 'map' frame and
the NavSatFix position.

# Use
open a terminal and run ros2 launch
```
ros2 run lla2local lla2local  # publishes coordinates on /utm
or
ros2 run lla_utm_local lla_utm_local --ros-args -p local_easting_origin:=692360.0 -p local_northing_origin:=13670670.0  # publishes local coordinates on /utm
```
The zero_e (easting reference) and zero_n (northing reference) are optional and if not provided
default to 0, resulting in UTM coordinates being published. If provided, they define the location
from which coordinates as offsets in meters will be published.
