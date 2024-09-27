#include "mavros_bridge/vbat_thrust_calibration.h"
#include <numeric>

namespace vbat_thrust_calibration {
    VbatThrustCalibrator::VbatThrustCalibrator(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) : nh_(nh),
        pnh_(pnh),
        autopilot_in_hover_(false),
        vbat_now_(0.0),
        command_idx_(0) {
        if (!loadParameters(pnh_)) {
            ROS_ERROR("[%s] Can not load all parameters!", pnh_.getNamespace().c_str());
            ros::shutdown();
        }
        vbat_now_ = cells_*(full_voltage_+empty_voltage_)/2;
        hover_percentage_now_ = (full_hover_percentage_+empty_hover_percentage_)/2;
        percentage_lower_ = hover_percentage_now_-0.02;
        percentage_upper_ = hover_percentage_now_+0.02;

        autopilot_feedback_sub_ = nh_.subscribe("autopilot/feedback", 1, &VbatThrustCalibrator::autopilotCallback, this); 
        vbat_sub_ = nh_.subscribe("mavros/battery", 1, &VbatThrustCalibrator::vbatCallback, this); 
        control_command_sub_ = nh_.subscribe("control_command", 1, &VbatThrustCalibrator::contolcommandCallback, this); 

        buffer_size_ = 20;
        hover_percentages_.resize(buffer_size_, full_hover_percentage_);
    }

    VbatThrustCalibrator::~VbatThrustCalibrator() {}

    bool VbatThrustCalibrator::loadParameters(const ros::NodeHandle& nh) {
        read_essential_param(nh, "thrust_model/full_hover_percentage", full_hover_percentage_);
        read_essential_param(nh, "thrust_model/empty_hover_percentage", empty_hover_percentage_);
        read_essential_param(nh, "thrust_model/empty_voltage", empty_voltage_);
        read_essential_param(nh, "thrust_model/full_voltage", full_voltage_);
        read_essential_param(nh, "thrust_model/correction_factor", correction_factor_);
        read_essential_param(nh, "thrust_model/battery_cells", cells_);
        read_essential_param(nh, "gravity", gravity_);
        if(cells_ < 2 || cells_ > 6) {
            ROS_ERROR("[%s] cells_ invalid!", pnh_.getNamespace().c_str());
            return false;
        }
        if(empty_voltage_ > full_voltage_) {
            ROS_ERROR("[%s] empty or full voltage invalid!", pnh_.getNamespace().c_str());
            return false;
        }
        if(empty_hover_percentage_ < full_hover_percentage_) {
            ROS_ERROR("[%s] empty or full hover_percentage invalid!", pnh_.getNamespace().c_str());
            return false;
        }
        if(correction_factor_ < 0 || correction_factor_ > 1) {
            ROS_ERROR("[%s] correction_factor invalid!", pnh_.getNamespace().c_str());
            return false;
        }
        return true;
    }

    void VbatThrustCalibrator::autopilotCallback(const quadrotor_msgs::AutopilotFeedbackConstPtr &msg) {
        autopilot_in_hover_ = (msg->autopilot_state == msg->HOVER);
    }

    void VbatThrustCalibrator::contolcommandCallback(const quadrotor_msgs::ControlCommandConstPtr &msg) {
        if(autopilot_in_hover_) {
            hover_percentages_[command_idx_] = (msg->collective_thrust)/gravity_ * hover_percentage_now_; 
            command_idx_ = (command_idx_+1) % buffer_size_;
        }
    }

    void VbatThrustCalibrator::vbatCallback(const sensor_msgs::BatteryStateConstPtr &msg) {
        if (msg->voltage < (cells_ * empty_voltage_ * 0.92) || msg->voltage > (cells_ * full_voltage_ * 1.03)) {
            // ROS_WARN("[%s] Battery voltage invalid, please check the parameters and the battery health", pnh_.getNamespace().c_str());
        } else if (msg->voltage < (cells_ * empty_voltage_)) {
            // ROS_WARN("[%s] Battery voltage LOW. Please Land Now!!!", pnh_.getNamespace().c_str());
        } else {
            vbat_now_ = msg->voltage;
        }
    }

    bool VbatThrustCalibrator::UpdateMappingParams(thrust_mapping::LinearMapping* thrust_mapping) {
        return false;
        if (vbat_now_ < cells_ * empty_voltage_ * 0.92 || vbat_now_ > cells_ * full_voltage_ * 1.03) {
            ROS_WARN("[%s] Battery voltage invalid, will not update hover_percentage", pnh_.getNamespace().c_str());
            return false;
        } else {
            float from_vbat = (full_hover_percentage_ + (empty_hover_percentage_ - full_hover_percentage_) / (full_voltage_ * cells_-empty_voltage_ * cells_) * (full_voltage_ * cells_ - vbat_now_));
            float from_hover = (std::accumulate(hover_percentages_.begin(), hover_percentages_.end(), 0.0) / hover_percentages_.size());
            hover_percentage_now_ = correction_factor_ * from_hover + (1-correction_factor_) * hover_percentage_now_;
            if(hover_percentage_now_>percentage_upper_) {
                hover_percentage_now_ = percentage_upper_;
            } else if(hover_percentage_now_<percentage_lower_) {
                hover_percentage_now_ = percentage_lower_;
            }
            thrust_mapping->param_.thrust_model.hover_percentage = hover_percentage_now_;
            thrust_mapping->resetThrustMapping();
            return true;
        }
    }
} // namespace vbat_thrust_calibration
