#pragma once

#include <ros/ros.h>

namespace thrust_mapping {

struct ThrustLinearMapping {
  double hover_percentage;
  double clip_ratio;
};

class LinearMappingParameters {
 public:
  LinearMappingParameters();
  void readConfigFromROSHandle(const ros::NodeHandle& nh);

  double gravity;
  double mass;
  ThrustLinearMapping thrust_model;

 private:
  template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle& nh, const TName& name, TVal& val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};
};

} // namespace thrust_mapping
