
name=${1:-room}
source /opt/ros/humble/setup.bash
source ~/chapt10/chapt10_ws/install/setup.bash
mkdir -p ~/chapt10/chapt10_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/chapt10/chapt10_ws/maps/$name \
    --ros-args -p use_sim_time:=true -p save_map_timeout:=20.0
ls -la ~/chapt10/chapt10_ws/maps/
