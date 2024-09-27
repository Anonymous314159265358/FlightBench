#include "mavros_bridge/mavros_bridge.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mavros_bridge");
  mavros_bridge::MAVROSBridge mavros_bridge;

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
