sudo chmod 777 /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
echo "Sourcing" $SCRIPT_DIR/../install/setup.bash
. $SCRIPT_DIR/../install/setup.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch roomba_bringup roomba_launch.py
