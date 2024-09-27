#pragma once

#include <stdint.h>
#include <Eigen/Dense>
#include <quadrotor_msgs/ControlCommand.h>

#include "mavros_bridge/thrust_mapping_params.h"

namespace thrust_mapping {

class LinearMapping{
 public:
  LinearMapping();
  LinearMapping(const LinearMappingParameters& params);

  LinearMappingParameters param_;

  void resetThrustMapping(void);
  double calculateControl(const quadrotor_msgs::ControlCommand::ConstPtr& ctrl_msg);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Thrust-accel mapping params
  double thr2acc_;


  double computeDesiredCollectiveThrustSignal(const double desired_a);

};

}  // namespace thrust_mapping
