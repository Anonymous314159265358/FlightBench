#include "autopilot/autopilot.h"
#include "position_controller/position_controller.h"
#include "position_controller/position_controller_params.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "autopilot");
  autopilot::AutoPilot<position_controller::PositionController,
                       position_controller::PositionControllerParams>
      autopilot(true);// true means use pid

  ros::spin();

  return 0;
}
