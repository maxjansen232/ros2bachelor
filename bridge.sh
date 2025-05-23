# initialize bridges between ROS & gazebo
gnome-terminal --geometry=40x10+1000+0 --title="Bridge #1 (position - pose)" -- bash -c "cd ~/ && ros2 run ros_gz_bridge parameter_bridge /rov/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry; exec bash"

gnome-terminal --geometry=40x10+0+0 --title="Bridge #2 (force - wrench)" -- bash -c "cd ~/ && ros2 run ros_gz_bridge parameter_bridge /world/ROV_ENV_01/wrench@ros_gz_interfaces/msg/EntityWrench@gz.msgs.EntityWrench; exec bash"


# bridges for thruster plugin
gnome-terminal --geometry=40x10+0+0 --title="Thruster Bridge #1 (force - wrench)" -- bash -c "cd ~/ && ros2 run ros_gz_bridge parameter_bridge /rov_cmd/stern_thrust@std_msgs/msg/Float64@gz.msgs.Double; exec bash"

gnome-terminal --geometry=40x10+0+0 --title="Thruster Bridge #2 (force - wrench)" -- bash -c "cd ~/ && ros2 run ros_gz_bridge parameter_bridge /rov_cmd/bow_thrust@std_msgs/msg/Float64@gz.msgs.Double; exec bash"

gnome-terminal --geometry=40x10+0+0 --title="Thruster Bridge #3 (force - wrench)" -- bash -c "cd ~/ && ros2 run ros_gz_bridge parameter_bridge /rov_cmd/starboard_thrust@std_msgs/msg/Float64@gz.msgs.Double; exec bash"

gnome-terminal --geometry=40x10+0+0 --title="Thruster Bridge #4 (force - wrench)" -- bash -c "cd ~/ && ros2 run ros_gz_bridge parameter_bridge /rov_cmd/port_thrust@std_msgs/msg/Float64@gz.msgs.Double; exec bash"

gnome-terminal --geometry=40x10+0+0 --title="Thruster Bridge #5 (force - wrench)" -- bash -c "cd ~/ && ros2 run ros_gz_bridge parameter_bridge /rov_cmd/top_thrust@std_msgs/msg/Float64@gz.msgs.Double; exec bash"

gnome-terminal --geometry=40x10+0+0 --title="Thruster Bridge #6 (force - wrench)" -- bash -c "cd ~/ && ros2 run ros_gz_bridge parameter_bridge /rov_cmd/bottom_thrust@std_msgs/msg/Float64@gz.msgs.Double; exec bash"



