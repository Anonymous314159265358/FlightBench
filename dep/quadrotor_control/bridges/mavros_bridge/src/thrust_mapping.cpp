#include "mavros_bridge/thrust_mapping.h"

namespace thrust_mapping {

LinearMapping::LinearMapping() {
  param_ = LinearMappingParameters();
}

LinearMapping::LinearMapping(const LinearMappingParameters& params)
    : param_(params) {
  resetThrustMapping();
}

void LinearMapping::resetThrustMapping(void) {
  thr2acc_ = param_.gravity / param_.thrust_model.hover_percentage;
}

double LinearMapping::computeDesiredCollectiveThrustSignal(const double desired_a) {
  double throttle_percentage(0.0);

  throttle_percentage = desired_a / thr2acc_;

  return throttle_percentage;
}

double LinearMapping::calculateControl(
    const quadrotor_msgs::ControlCommand::ConstPtr& ctrl_msg) {
  
  // the collective thrust has already been normalized by the mass
  double desired_a = ctrl_msg->collective_thrust;
  // std::cout<<"hover percentage: "<<param_.thrust_model.hover_percentage<<std::endl;

  double desired_percentage = computeDesiredCollectiveThrustSignal(desired_a);
  if (desired_percentage > param_.thrust_model.clip_ratio * param_.thrust_model.hover_percentage) {
    return param_.thrust_model.clip_ratio * param_.thrust_model.hover_percentage;
  } else {
    return desired_percentage;
  }
}

}
