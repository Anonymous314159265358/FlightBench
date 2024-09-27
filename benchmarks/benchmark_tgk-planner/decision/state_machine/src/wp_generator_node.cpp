#include "state_machine/wp_generator_node.h"

namespace tgk_planner {
    WpIterator::WpIterator(const ros::NodeHandle &nh, const ros::NodeHandle &pnh) : 
    nh_(nh),
    pnh_(pnh) {
        odom_sub_ = nh_.subscribe("/odom", 10, &WpIterator::odomCallback, this);
        goal_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/goal", 10);

        pnh.param("wpt/point_num", point_num_, 0);
        waypoints_.resize(point_num_);
        double x,y,z;
        for(int i=0;i<point_num_;i++) {
            pnh.param("wpt/point"+std::to_string(i)+"_x", x, -1.0);
            pnh.param("wpt/point"+std::to_string(i)+"_y", y, -1.0);
            pnh.param("wpt/point"+std::to_string(i)+"_z", z, -1.0);
            waypoints_[i] = Eigen::Vector3d(x,y,z);
        }
        std::cout<<"points load down: "<<point_num_<<std::endl;
        curr_wp_ = 0;
        pub_times_ = 5;
        have_odom_ = false;
        delay_ = 30;
        checker_timer_ = nh_.createTimer(ros::Duration(0.05), &WpIterator::checkerCallback, this); // 20Hz
    }

    WpIterator::~WpIterator(){}

    void WpIterator::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_ptr) {
        odom_now_(0) = odom_ptr->pose.pose.position.x;
        odom_now_(1) = odom_ptr->pose.pose.position.y;
        odom_now_(2) = odom_ptr->pose.pose.position.z;
        have_odom_ = true;
    }

    void WpIterator::checkerCallback(const ros::TimerEvent& event) {
        if(!have_odom_) { 
            std::cout<<"no odom"<<std::endl;
            return;
        }
        if(delay_>0){
            delay_ --;
            return;
        }
        if((odom_now_-waypoints_[curr_wp_]).norm()<1.0){
            curr_wp_++;
            pub_times_ = 5;
        }
        if(curr_wp_>=point_num_) { 
            std::cout<<"reach target!"<<std::endl;
            return;
        }
        if(pub_times_>0) {
            quadrotor_msgs::PositionCommand goal_msg_;
            goal_msg_.position.x = waypoints_[curr_wp_](0);
            goal_msg_.position.y = waypoints_[curr_wp_](1);
            goal_msg_.position.z = waypoints_[curr_wp_](2);
            goal_pub_.publish(goal_msg_);
            pub_times_--;
        }
    }

}// namespace tgk_planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "wp_iterator_node");
    tgk_planner::WpIterator wp_iter(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();
    return 0;
}