/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_frame_tracker
 *
 * \author
 *   Author: Felix Messmer, email: Felix.Messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2014
 *
 * \brief
 *   This class provides a twist_generator for tracking a given tf-frame
 *
 ****************************************************************/
#ifndef COB_FRAME_TRACKER_H
#define COB_FRAME_TRACKER_H

#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <ros/ros.h>
#include <iostream>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <cob_srvs/SetString.h>

#include <actionlib/server/simple_action_server.h>
#include <cob_frame_tracker/FrameTrackerConfig.h>
#include <cob_frame_tracker/FrameTrackingAction.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

#include <control_toolbox/pid.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <acado/acado_toolkit.hpp>
#include <acado/acado_optimal_control.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>

#include <cob_frame_tracker/kinematic_calculations.h>

using namespace ACADO;

typedef actionlib::SimpleActionServer<cob_frame_tracker::FrameTrackingAction> SAS_FrameTrackingAction_t;

struct HoldTf
{
    tf::StampedTransform transform_tf;
    bool hold;
};


class CobFrameTracker
{
public:
    CobFrameTracker()
    {
        ht_.hold = false;
    }

    ~CobFrameTracker()
    {
        jntToCartSolver_vel_.reset();
        as_.reset();
        reconfigure_server_.reset();
    }

    bool initialize();
    void run(const ros::TimerEvent& event);

    void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    bool startTrackingCallback(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response);
    bool startLookatCallback(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response);
    bool stopCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

    bool getTransform(const std::string& from, const std::string& to, tf::StampedTransform& stamped_tf);

    void publishZeroTwist();
    void publishTwist(ros::Duration period, bool do_publish = true);
    void publishHoldTwist(const ros::Duration& period);

    ///ACADO optimization solver
    //void solver(tf::StampedTransform transform_tf, geometry_msgs::TwistStamped& controlled_twist);
    void solver(void);
    void hard_coded_solver();
    bool zeroTwist(void);

    void frameTrackerCallBack(const geometry_msgs::Pose::ConstPtr& msg);

    /// Action interface
    void goalCB();
    void preemptCB();
    void action_success();
    void action_abort();

private:
    HoldTf ht_;

    double update_rate_;
    ros::Timer timer_;

    bool tracking_;
    bool tracking_goal_;
    bool lookat_;
    std::string chain_base_link_;
    std::string chain_tip_link_;
    std::string lookat_focus_frame_;
    std::string tracking_frame_;    // the frame tracking the target (i.e. chain_tip or lookat_focus)
    std::string target_frame_;      // the frame to be tracked

    double max_vel_lin_;
    double max_vel_rot_;

    std::vector<std::string> joints_;
    unsigned int dof_;

    bool movable_trans_;
    bool movable_rot_;

    control_toolbox::Pid pid_controller_trans_x_;       /** < Internal PID controller. */
    control_toolbox::Pid pid_controller_trans_y_;
    control_toolbox::Pid pid_controller_trans_z_;

    control_toolbox::Pid pid_controller_rot_x_;         /** < Internal PID controller. */
    control_toolbox::Pid pid_controller_rot_y_;
    control_toolbox::Pid pid_controller_rot_z_;

    /// KDL Conversion
    KDL::Chain chain_;
    KDL::JntArray last_q_;
    KDL::JntArray lase_pose_;	// position and rpy of endeffector
    KDL::JntArray last_q_dot_;
    Eigen::Matrix<double, 6, 7>  J_Mat;
    boost::shared_ptr<KDL::ChainFkSolverVel_recursive> jntToCartSolver_vel_;
    boost::shared_ptr<KDL::ChainJntToJacSolver> jntToJacSolver_;
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jntToCartSolver_pos_;

    tf::TransformListener tf_listener_;

    ros::Subscriber jointstate_sub_, frame_tracker_sub_;
    ros::Publisher twist_pub_;
    ros::Publisher joint_vel_pub_;

    ros::ServiceServer start_tracking_server_;
    ros::ServiceServer start_lookat_server_;
    ros::ServiceServer stop_server_;
    ros::ServiceClient reconfigure_client_;

    /// Action interface
    std::string action_name_;
    boost::shared_ptr<SAS_FrameTrackingAction_t> as_;

    cob_frame_tracker::FrameTrackingFeedback action_feedback_;
    cob_frame_tracker::FrameTrackingResult action_result_;

    boost::recursive_mutex reconfig_mutex_;
    boost::shared_ptr< dynamic_reconfigure::Server<cob_frame_tracker::FrameTrackerConfig> > reconfigure_server_;
    void reconfigureCallback(cob_frame_tracker::FrameTrackerConfig& config, uint32_t level);

    /// ABORTION CRITERIA:
    int checkStatus();
    bool checkInfinitesimalTwist(const KDL::Twist current);
    bool checkCartDistanceViolation(const double dist, const double rot);
    bool checkTwistViolation(const KDL::Twist current, const KDL::Twist target);

    int checkServiceCallStatus();

    bool stop_on_goal_;
    double tracking_duration_;
    ros::Time tracking_start_time_;

    bool enable_abortion_checking_;
    double cart_min_dist_threshold_lin_;
    double cart_min_dist_threshold_rot_;
    double twist_dead_threshold_lin_;
    double twist_dead_threshold_rot_;
    double twist_deviation_threshold_lin_;
    double twist_deviation_threshold_rot_;

    KDL::Twist current_twist_;
    KDL::Twist target_twist_;

    double cart_distance_;
    double rot_distance_;

    unsigned int abortion_counter_;
    unsigned int max_abortions_;

    /// ACADO Variables
    //DifferentialState error; // Error, tracking and target frame
    //DifferentialState x;    // Use for end-effector velocity
    //Control q_dot;     // USe for joint velocity

    ///Kinematic solver
    boost::shared_ptr<Kinematic_calculations> kinematic_solver_;
    std_msgs::Float64MultiArray pub_data_joint_vel;

};

#endif
