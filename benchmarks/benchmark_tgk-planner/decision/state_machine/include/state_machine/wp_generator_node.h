#include <ros/ros.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <vector>
#include <eigen3/Eigen/Eigen>

namespace tgk_planner {
    class WpIterator {
    public:
        WpIterator(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
        ~WpIterator();
    private:
        ros::NodeHandle nh_, pnh_;
        ros::Subscriber odom_sub_;
        ros::Publisher goal_pub_;
        ros::Timer checker_timer_;

        int point_num_, pub_times_, curr_wp_, delay_;
        bool have_odom_;
        std::vector<Eigen::Vector3d> waypoints_;
        Eigen::Vector3d odom_now_;
        void checkerCallback(const ros::TimerEvent& event);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_ptr);
        
    };
}// namespace tgk_planner