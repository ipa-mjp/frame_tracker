
// Containts callback of services, acado solver, check joint violation, etc...

#ifndef FRAME_TRACKER_H
#define FRAME_TRACKER_H

// c++ includes
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <ros/ros.h>

// ros includes
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

#include <frame_tracker/GetFrameTrackingInfo.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// kdl includes
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

// boost include
#include <boost/shared_array.hpp>

class FrameTracker
{
public:
		FrameTracker(){};
		~FrameTracker(){};

		ros::NodeHandle nh;

		bool initialization();

		// Call function at time instance
		void run_node(const ros::TimerEvent& event);

		// Call back of service server
		bool startTrackingCallBack( frame_tracker::GetFrameTrackingInfo::Request& request, frame_tracker::GetFrameTrackingInfo::Response& response);
		bool stopTrackingCallBack( frame_tracker::GetFrameTrackingInfo::Request& request, frame_tracker::GetFrameTrackingInfo::Response& response);

		// joint state call back to get current position and velocity
		void jointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg);

		// checks whether the current twist is following the target twist
		bool checkTwistViolation(const KDL::Twist current, const KDL::Twist target);

		// checks whether the twist is infinitesimally small
		bool checkInfinitesimalTwist(const KDL::Twist current);

		// Check status
		int checkStatus();

		void publishZeroTwist();
private:

		double update_rate_;
		ros::Timer timer_;

		std::string base_link_;
		std::string tip_link_;
		std::string root_frame_;
		std::string target_frame_;
		std::string tracking_frame_;
		std::vector<std::string> joints_;
		unsigned int dof_;

		KDL::Twist current_twist_;
		KDL::Twist target_twist_;

		KDL::Chain chain_;
		KDL::JntArray last_q_;
		KDL::JntArray last_q_dot_;
		boost::shared_ptr<KDL::ChainFkSolverVel_recursive> jntToCartSolver_vel_;

		ros::ServiceServer start_tracking_server_;
		ros::ServiceServer stop_tracking_server_;

		// activation of tracking, output
		bool tracking_;
		bool activate_output_;

		// limit paramter
		double max_lin_vel_;
		double min_lin_vel_;
		double max_rot_vel_;
		double min_rot_vel_;
		double twist_deviation_threshold_lin_;
		double twist_deviation_threshold_rot_;
		double twist_dead_threshold_lin_;
		double twist_dead_threshold_rot_;

		//subscriber
		ros::Subscriber joint_state_sub_;
		ros::Publisher twist_pub_;

		tf::TransformListener tf_listener_;

};




#endif
