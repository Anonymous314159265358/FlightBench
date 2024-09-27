#include <thread>
#include <mutex>
#include <string>

#include <quadrotor_msgs/ControlCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>

#include "mavros_bridge/thrust_mapping.h"
#include "mavros_bridge/vbat_thrust_calibration.h"

#include <thread>

namespace mavros_bridge {

enum class ControlMode { NONE, ATTITUDE, BODY_RATES };
enum class BridgeState { OFF, AUTO, RC_FLIGHT};
enum class ThrustMappingMode { LINEAR, QUADRIC, ITERATIVE };

class FCUStateData {
  public:
    // FIXME: check initial state
    mavros_msgs::State current_state;
    mavros_msgs::State state_before_offboard;

    FCUStateData() {};
    void feed(mavros_msgs::StateConstPtr msg) {
        state_before_offboard = current_state;
        current_state = *msg;
    }
};
    
class MAVROSBridge {
  public:
    MAVROSBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    MAVROSBridge() : MAVROSBridge(ros::NodeHandle(), ros::NodeHandle("~")) {}

    virtual ~MAVROSBridge();

  private:
    // Paramters loader
    bool loadParameters();

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // mutex
    std::mutex fcu_state_mutex_;
    std::mutex ctrl_msg_mutex_;


    BridgeState bridge_state_;
    ControlMode control_mode_;
    FCUStateData fcu_state_data_;

    void setBridgeState(const BridgeState& desired_bridge_state);

    // Subscribers
    ros::Subscriber control_command_sub_;
    ros::Subscriber arm_bridge_sub_;
    ros::Subscriber mavros_state_sub_;

    void controlCommandCallback(const quadrotor_msgs::ControlCommand::ConstPtr& msg);
    void armBridgeCallback(const std_msgs::Bool::ConstPtr& msg);
    void mavrosStateCallback(mavros_msgs::StateConstPtr msg);

    //Publishers
    ros::Publisher low_level_feedback_pub_;
    ros::Publisher mavros_msg_pub_;

    void publishAttitudeCommand(const quadrotor_msgs::ControlCommand::ConstPtr& msg);
    void publishBodyRatesCommand(const quadrotor_msgs::ControlCommand::ConstPtr& msg);
    void publishLowLevelFeedback(const ros::TimerEvent& time) const;

    mavros_msgs::AttitudeTarget generateMAVROSMsgFromCTRL(
        const quadrotor_msgs::ControlCommand::ConstPtr& msg, const ros::Time &stamp);
    void sendMAVROSMessage(mavros_msgs::AttitudeTarget& msg);

    // Timer
    ros::Timer low_level_feedback_pub_timer_;
    ros::Timer watchdog_timer_;

    // watchdog
    void watchdogThread();
    std::thread watchdog_thread_;
    bool stop_watchdog_thread_;

    ros::Time time_last_feedback_msg_received_;
    ros::Time time_last_mavros_msg_sent_;
    ros::Time time_last_active_control_command_received_;
    ros::Time time_last_control_command_received_;

    // Parameters
    double control_command_timeout_;
    double feedback_msg_timeout_;

    // thrust mapping   
    thrust_mapping::LinearMapping thrust_model_;
    vbat_thrust_calibration::VbatThrustCalibrator calibrator_;
    
    double mass_;

    double max_roll_rate_;
    double max_pitch_rate_;
    double max_yaw_rate_;

    double max_roll_angle_;
    double max_pitch_angle_;

    //about FCU
    std::string quad_name_;
    ros::ServiceClient arm_FCU_srv_;
};


}//namespace mavros_bridge
