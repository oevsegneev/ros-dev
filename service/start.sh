#! /bin/bash

source /opt/ros/indigo/setup.bash
source /home/ubuntu/conf/ros_master_uri
source /home/ubuntu/catkin_ws/devel/setup.bash

#export ROS_WORKSPACE=/home/ubuntu/catkin_ws
#export ROSCONSOLE_CONFIG_FILE="/home/ubuntu/conf/rosconsole.config"
#echo "test message"
roslaunch /home/ubuntu/catkin_ws/my_map.launch &
sleep 30s
#echo "message test"
rosrun mavros gcs_bridge _gcs_url:='/dev/rfd900:57600' &
sleep 10s
#echo "poly_ros"
#rosrun poly_ros poly_ros_node &
roslaunch /home/ubuntu/catkin_ws/poly.launch &