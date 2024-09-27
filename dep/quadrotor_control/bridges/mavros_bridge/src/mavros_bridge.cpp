#include "mavros_bridge/mavros_bridge.h"
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <quadrotor_common/math_common.h>
#include <quadrotor_common/parameter_helper.h>
#include <quadrotor_msgs/LowLevelFeedback.h>

namespace mavros_bridge {

MAVROSBridge::MAVROSBridge(const ros::NodeHandle& nh,
                           const ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      time_last_feedback_msg_received_(),
      time_last_mavros_msg_sent_(ros::Time::now()),
      time_last_active_control_command_received_(),
      time_last_control_command_received_(),
      control_mode_(ControlMode::NONE),
      bridge_state_(BridgeState::OFF),
      stop_watchdog_thread_(false),
      calibrator_(nh, pnh)  {
  if (!loadParameters()) {
    ROS_ERROR("[%s] Could not load parameters.", pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }

  // Publishers
  low_level_feedback_pub_ =
      nh_.advertise<quadrotor_msgs::LowLevelFeedback>("low_level_feedback", 1);
  // NOTICE: since /mavros is not under the mavros_bridge node,
  // we need to use pnh_ as nodehandle rather than nh_.
  // Same in other parts.
  mavros_msg_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
      "mavros/setpoint_raw/attitude", 10);

  // Subscribers
  arm_bridge_sub_ = nh_.subscribe("mavros_bridge/arm", 1,
                                  &MAVROSBridge::armBridgeCallback, this);
  control_command_sub_ = nh_.subscribe(
      "control_command", 1, &MAVROSBridge::controlCommandCallback, this);
  mavros_state_sub_ = nh_.subscribe("mavros/state", 10,
                                     &MAVROSBridge::mavrosStateCallback, this);

  low_level_feedback_pub_timer_ = nh_.createTimer(
      ros::Duration(1.0 / 100.0), &MAVROSBridge::publishLowLevelFeedback, this);

  arm_FCU_srv_ = nh_.serviceClient<mavros_msgs::CommandBool>(
      "mavros/cmd/arming");

  // watchdog thread
  try {
    watchdog_thread_ = std::thread(&MAVROSBridge::watchdogThread, this);
  } catch (...) {
    ROS_ERROR("[%s] Could not successfully start watchdog thread.",
              pnh_.getNamespace().c_str());
    ros::shutdown();
    return;
  }
}

MAVROSBridge::~MAVROSBridge() {
  ROS_WARN("Going to destruct the mavros bridge and switch it to OFF state");
  setBridgeState(BridgeState::OFF);
  stop_watchdog_thread_ = true;
  // FIXME: check which ones need to be closed
}

// state switching

void MAVROSBridge::setBridgeState(const BridgeState& desired_bridge_state) {
  if (bridge_state_ == BridgeState::OFF) {
    switch (desired_bridge_state) {
      case BridgeState::RC_FLIGHT:
        ROS_WARN("[%s] Cannot switch to RC directly from OFF",
                 pnh_.getNamespace().c_str());
        break;
      case BridgeState::OFF:
        ROS_INFO("[%s] Already OFF", pnh_.getNamespace().c_str());
        break;
      case BridgeState::AUTO:
        ROS_INFO("[%s] Switch from OFF to AUTO", pnh_.getNamespace().c_str());
        bridge_state_ = BridgeState::AUTO;
        break;
      default:
        ROS_WARN("[%s] Invalid bridge state", pnh_.getNamespace().c_str());
        break;
    }
    // bridge_state_ = BridgeState::AUTO;
  } else if (bridge_state_ == BridgeState::AUTO) {
    switch (desired_bridge_state) {
      case BridgeState::RC_FLIGHT:
        ROS_INFO("[%s] Switch from AUTO to RC_FLIGHT",
                 pnh_.getNamespace().c_str());
        bridge_state_ = BridgeState::RC_FLIGHT;
        break;
      case BridgeState::AUTO:
        ROS_INFO("[%s] Already AUTO", pnh_.getNamespace().c_str());
        break;
      case BridgeState::OFF:
        ROS_WARN("[%s] Switch from AUTO to OFF", pnh_.getNamespace().c_str());
        bridge_state_ = BridgeState::OFF;
        break;
      default:
        ROS_WARN("[%s] Invalid bridge state", pnh_.getNamespace().c_str());
        break;
    }
  } else {  // RC_FLIGHT
    switch (desired_bridge_state) {
      case BridgeState::RC_FLIGHT:
        ROS_INFO("[%s] Already RC_FLIGHT", pnh_.getNamespace().c_str());
        break;
      case BridgeState::AUTO:
        ROS_INFO("[%s] Switch from RC_FLIGHT to AUTO",
                 pnh_.getNamespace().c_str());
        bridge_state_ = BridgeState::AUTO;
        break;
      case BridgeState::OFF:
        ROS_WARN("[%s] Switch from RC_FLIGHT to OFF",
                 pnh_.getNamespace().c_str());
        bridge_state_ = BridgeState::OFF;
        break;
      default:
        ROS_WARN("[%s] Invalid bridge state", pnh_.getNamespace().c_str());
        break;
    }
  }
}

// callback functions

void MAVROSBridge::mavrosStateCallback(mavros_msgs::StateConstPtr msg) {
  // FIXME: check mutex
  std::lock_guard<std::mutex> fcu_state_lock(fcu_state_mutex_);
  fcu_state_data_.feed(msg);
  time_last_feedback_msg_received_ = ros::Time::now();
  if (fcu_state_data_.current_state.mode !=
          fcu_state_data_.current_state.MODE_PX4_OFFBOARD &&
      bridge_state_ == BridgeState::AUTO) {
    setBridgeState(BridgeState::RC_FLIGHT);
  }
  if (fcu_state_data_.current_state.mode ==
          fcu_state_data_.current_state.MODE_PX4_OFFBOARD &&
      bridge_state_ == BridgeState::RC_FLIGHT) {
    setBridgeState(BridgeState::AUTO);
  }
  return;
}

void MAVROSBridge::armBridgeCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    // check upstream and fb
    ros::Time now_time = ros::Time::now();
    if ((now_time - time_last_control_command_received_).toSec() >
        control_command_timeout_) {
      ROS_WARN("[%s] Control command timeout, cannot switch to AUTO",
               pnh_.getNamespace().c_str());
      return;
    } else if ((now_time - time_last_feedback_msg_received_).toSec() >
                   feedback_msg_timeout_ ||
               !fcu_state_data_.current_state.connected) {
      ROS_WARN("[%s] Connection to PX4 not available, cannot switch to AUTO",
               pnh_.getNamespace().c_str());
      return;
    } else {
      setBridgeState(BridgeState::AUTO);
    }
    return;
  } else {
    ROS_WARN("[%s] Manually set to OFF", pnh_.getNamespace().c_str());
    setBridgeState(BridgeState::OFF);
    return;
  }
}

void MAVROSBridge::controlCommandCallback(
    const quadrotor_msgs::ControlCommand::ConstPtr& msg) {
  std::lock_guard<std::mutex> ctrl_msg_lock(ctrl_msg_mutex_);

  ros::Time now_time = ros::Time::now();
  time_last_control_command_received_ = now_time;
  if (msg->armed) {
    if (bridge_state_ == BridgeState::OFF) {
        ROS_WARN("[%s] Bridge still locked, please turnoff the autopilot and arm the bridge first", pnh_.getNamespace().c_str());
        return;
    }
    time_last_active_control_command_received_ = now_time;
  }

  if (bridge_state_ == BridgeState::OFF) {
    return;
  }

  if (msg->armed && bridge_state_ == BridgeState::AUTO) {
    if (!fcu_state_data_.current_state.armed) {
        mavros_msgs::CommandBool arm_cmd;
	    arm_cmd.request.value = true;
        arm_FCU_srv_.call(arm_cmd);
    }
  }

  mavros_msgs::AttitudeTarget mavros_msg_to_send;
  mavros_msg_to_send = generateMAVROSMsgFromCTRL(msg, now_time);
  sendMAVROSMessage(mavros_msg_to_send);

  // Set control mode for low level feedback message to be published
  if (msg->control_mode == msg->ATTITUDE) {
    control_mode_ = ControlMode::ATTITUDE;
  } else if (msg->control_mode == msg->BODY_RATES) {
    control_mode_ = ControlMode::BODY_RATES;
  } else {
    control_mode_ = ControlMode::NONE;
  }
}

// timer callback functions
void MAVROSBridge::publishLowLevelFeedback(const ros::TimerEvent& time) const {
  quadrotor_msgs::LowLevelFeedback low_level_feedback_msg;

  {
    // any lock?
    // Publish a low level feedback message
    low_level_feedback_msg.header.stamp = ros::Time::now();

    if (bridge_state_ == BridgeState::RC_FLIGHT) {
      low_level_feedback_msg.control_mode = low_level_feedback_msg.RC_MANUAL;
    } else if (bridge_state_ == BridgeState::AUTO) {
      if (control_mode_ == ControlMode::ATTITUDE) {
        low_level_feedback_msg.control_mode = low_level_feedback_msg.ATTITUDE;
      } else if (control_mode_ == ControlMode::BODY_RATES) {
        low_level_feedback_msg.control_mode = low_level_feedback_msg.BODY_RATES;
      } else {
        low_level_feedback_msg.control_mode = low_level_feedback_msg.NONE;
      }
    } else {
      low_level_feedback_msg.control_mode = low_level_feedback_msg.NONE;
    }
    // Mutexes are unlocked here since they go out of scope
  }

  low_level_feedback_pub_.publish(low_level_feedback_msg);
}

// util functions
mavros_msgs::AttitudeTarget MAVROSBridge::generateMAVROSMsgFromCTRL(
    const quadrotor_msgs::ControlCommand::ConstPtr& ctrl_msg,
    const ros::Time& stamp) {
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = std::string("FCU");

  if (ctrl_msg->armed) {
    calibrator_.UpdateMappingParams(&thrust_model_);
    msg.thrust = thrust_model_.calculateControl(ctrl_msg);

    if (ctrl_msg->control_mode == ctrl_msg->BODY_RATES) {
      msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

      msg.body_rate.x = ctrl_msg->bodyrates.x;
      msg.body_rate.y = ctrl_msg->bodyrates.y;
      msg.body_rate.z = ctrl_msg->bodyrates.z;

    } else if (ctrl_msg->control_mode == ctrl_msg->ATTITUDE) {
      msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                      mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                      mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

      msg.orientation.x = ctrl_msg->orientation.x;
      msg.orientation.y = ctrl_msg->orientation.y;
      msg.orientation.z = ctrl_msg->orientation.z;
      msg.orientation.w = ctrl_msg->orientation.w;

    } else {
      ROS_ERROR("Invalid control mode, please use bodyrates or attitude!");
    }
  } else {  // send all-zero msg when disarmed
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    msg.body_rate.x = 0;
    msg.body_rate.y = 0;
    msg.body_rate.z = 0;
    msg.thrust = 0;
  }

  return msg;
}

bool MAVROSBridge::loadParameters() {
#define GET_PARAM(name) \
  if (!quadrotor_common::getParam(#name, name##_, pnh_)) return false

  GET_PARAM(control_command_timeout);
  GET_PARAM(feedback_msg_timeout);
  // GET_PARAM(rc_timeout);

  GET_PARAM(mass);

  GET_PARAM(max_roll_rate);
  GET_PARAM(max_pitch_rate);
  GET_PARAM(max_yaw_rate);
  max_roll_rate_ /= (180.0 / M_PI);
  max_pitch_rate_ /= (180.0 / M_PI);
  max_yaw_rate_ /= (180.0 / M_PI);

  GET_PARAM(max_roll_angle);
  GET_PARAM(max_pitch_angle);
  max_roll_angle_ /= (180.0 / M_PI);
  max_pitch_angle_ /= (180.0 / M_PI);

  thrust_model_.param_.readConfigFromROSHandle(pnh_);
  thrust_model_.resetThrustMapping();

  return true;

#undef GET_PARAM
}

void MAVROSBridge::sendMAVROSMessage(mavros_msgs::AttitudeTarget& msg) {
  mavros_msgs::AttitudeTarget mavros_msg_to_send = msg;

  ros::Time now_time = ros::Time::now();
  msg.header.stamp = now_time;
  mavros_msg_pub_.publish(msg);
  time_last_mavros_msg_sent_ = now_time;
}

// watchdog

void MAVROSBridge::watchdogThread() {
  ros::Rate watchdog_rate(110.0);
  while (ros::ok() && !stop_watchdog_thread_) {
    watchdog_rate.sleep();

    const ros::Time time_now = ros::Time::now();

    if (bridge_state_ == BridgeState::AUTO) {
      if (time_now - time_last_control_command_received_ >
          ros::Duration(control_command_timeout_)) {
        ROS_WARN("[%s] Control command timeout", pnh_.getNamespace().c_str());
        setBridgeState(BridgeState::OFF);
      }
      if (((time_now - time_last_feedback_msg_received_) >
           ros::Duration(feedback_msg_timeout_)) ||
          !fcu_state_data_.current_state.connected) {
        ROS_WARN("[%s] Connection to PX4 not available",
                 pnh_.getNamespace().c_str());
        setBridgeState(BridgeState::OFF);
      }
    }
  }
}

}  // namespace mavros_bridge

// low level fb
// watchdog
// controlcommand callback, check if armed (done)
// mavrosStateCallback, change to RC
