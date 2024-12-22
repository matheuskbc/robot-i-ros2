SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
echo "Sourcing" $SCRIPT_DIR/../install/setup.bash
. $SCRIPT_DIR/../install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch roomba_navigation roomba_all.launch.py
