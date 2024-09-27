#pragma once
#include <ros/ros.h>
#include <vector>
#include <quadrotor_msgs/AutopilotFeedback.h> 
#include <quadrotor_msgs/ControlCommand.h>
#include <mavros_bridge/thrust_mapping.h>
#include <sensor_msgs/BatteryState.h>

namespace vbat_thrust_calibration{
    class VbatThrustCalibrator {
        public:
        VbatThrustCalibrator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
        ~VbatThrustCalibrator();
        bool UpdateMappingParams(thrust_mapping::LinearMapping* thrust_mapping);

        private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Subscriber autopilot_feedback_sub_;
        ros::Subscriber vbat_sub_;
        ros::Subscriber control_command_sub_;

        // callback functions
        void autopilotCallback(const quadrotor_msgs::AutopilotFeedbackConstPtr &msg);
        void vbatCallback(const sensor_msgs::BatteryStateConstPtr &msg);
        void contolcommandCallback(const quadrotor_msgs::ControlCommandConstPtr &msg);

        bool autopilot_in_hover_;
        std::vector<float> hover_percentages_;
        int command_idx_;
        int buffer_size_;
        float vbat_now_;
        int cells_;
        float full_voltage_; // 4.2 normally
        float empty_voltage_;// 3.7 normally
        float full_hover_percentage_; // [0,1]
        float empty_hover_percentage_;// [0,1]
        float correction_factor_; // [0,1]
        float gravity_; //9.81
        float hover_percentage_now_;
        float percentage_lower_, percentage_upper_;
        bool loadParameters(const ros::NodeHandle& nh);
        template <typename TName, typename TVal>
	    inline void read_essential_param(const ros::NodeHandle& nh, const TName& name, TVal& val) {
		    if (!nh.getParam(name, val)) {
		    	ROS_ERROR_STREAM("Read param: " << name << " failed.");
		    	ROS_BREAK();
		    }
	    };
    };
} //namespace vbat_thrust_calibration
