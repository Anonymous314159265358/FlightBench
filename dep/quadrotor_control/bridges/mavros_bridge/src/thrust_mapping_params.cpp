#include "mavros_bridge/thrust_mapping_params.h"

namespace thrust_mapping {

LinearMappingParameters::LinearMappingParameters() {}

void LinearMappingParameters::readConfigFromROSHandle(const ros::NodeHandle& nh) {
  read_essential_param(nh, "thrust_model/full_hover_percentage", thrust_model.hover_percentage);
  read_essential_param(nh, "gravity", gravity);
  read_essential_param(nh, "mass", mass);
  read_essential_param(nh, "thrust_model/clip_ratio", thrust_model.clip_ratio);
}

}
