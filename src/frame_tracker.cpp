
#include <frame_tracker/frame_tracker.h>

bool FrameTracker::initialization()
{
	// Get chain_base and chain tip links, root frame, traget frame
	if (!nh.getParam ("/chain_base_link", base_link_) )
	{
		ROS_WARN(" Parameter 'Chain_base_link' not set on %s node " , ros::this_node::getName().c_str());
	}

	if (!nh.getParam ("/chain_tip_link", tip_link_) )
	{
		ROS_WARN(" Parameter 'Chain_tip_link' not set on %s node " , ros::this_node::getName().c_str());
	}

	if (!nh.getParam ("/root_frame", root_frame_) )
	{
		ROS_WARN(" Parameter 'Root_frame' not set on %s node " , ros::this_node::getName().c_str());
	}

	if (!nh.getParam ("/target_frame", target_frame_) )
	{
		ROS_WARN(" Parameter 'target_frame' not set on %s node " , ros::this_node::getName().c_str());
	}

	//Get joint names
	if (!nh.getParam ("/joint_names", joints_) )
	{
		ROS_WARN(" Parameter 'Joint names' not set on %s node " , ros::this_node::getName().c_str());
	}

	// Get debug info, using active_output
	nh.param("/activate_output", activate_output_, bool(false));
	nh.param("/max_lin_vel", max_lin_vel_, double(0.1));	// m/sec
	nh.param("/min_lin_vel_", min_lin_vel_, double(0.1));
	nh.param("/max_rot_vel_", max_rot_vel_, double(0.1));	// rad/sec
	nh.param("/min_rot_vel_", min_rot_vel_, double(0.1));
	nh.param("/update_rate", update_rate_, double(50.0));		//Hz

	// Generate KDL chain and urdf model by parse robot description
	KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromParam("/robot_description", kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    kdl_tree.getChain( base_link_, tip_link_, chain_ );
    if (chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kdl chain");
        return false;
    }

	dof_ = chain_.getNrOfJoints();
	last_q_ = KDL::JntArray(dof_);
	last_q_dot_ = KDL::JntArray(dof_);


	tracking_ = false;
	tracking_frame_ = tip_link_;
	twist_deviation_threshold_lin_ = 0.5;
	twist_deviation_threshold_rot_ = 0.5;
	twist_dead_threshold_lin_ = 0.05;
	twist_dead_threshold_rot_ = 0.05;

	joint_state_sub_ = nh.subscribe("joint_states", 1, &FrameTracker::jointStateCallBack, this);
	twist_pub_ = nh.advertise<geometry_msgs::TwistStamped> ("command_twist_stamped", 1);

    // initialize service client
    start_tracking_server_ = nh.advertiseService("start_tracking", &FrameTracker::startTrackingCallBack, this);
    stop_tracking_server_ = nh.advertiseService("stop_tracking", &FrameTracker::stopTrackingCallBack, this);

    timer_ = nh.createTimer(ros::Duration(1.0/update_rate_), &FrameTracker::run_node, this);
    timer_.start();

    ROS_INFO("FrameTracker initialized!!");

   return true;
}

void FrameTracker::run_node(const ros::TimerEvent& event)
{
;
}

void FrameTracker::publishZeroTwist()
{
	// publish zero Twist for stopping
	geometry_msgs::TwistStamped twist_msg;
	twist_msg.header.frame_id = tracking_frame_;
	twist_pub_.publish(twist_msg);
}

bool FrameTracker::startTrackingCallBack(frame_tracker::GetFrameTrackingInfo::Request& request, frame_tracker::GetFrameTrackingInfo::Response& response)
{
	if (tracking_)
	{
		std::string msg = "FrameTracker: Start Tracking is already activated";
        ROS_ERROR_STREAM(msg);
        response.success = false;
        response.message = msg;
	}

	else
	{
		// check whether given target frame exists
		if (!tf_listener_.frameExists( request.data))
		{
            std::string msg = "FrameTracker: StartTracking denied because target frame '" + request.data + "' does not exist";
            ROS_ERROR_STREAM(msg);
            response.success = false;
            response.message = msg;
		}
		else
		{
            std::string msg = "FrameTracker: StartTracking started";
            ROS_INFO_STREAM(msg);
            response.success = true;
            response.message = msg;

            tracking_ = true;
            tracking_frame_ = tip_link_;
            target_frame_ = request.data;
		}
	}
	return true;
}

bool FrameTracker::stopTrackingCallBack(frame_tracker::GetFrameTrackingInfo::Request& requrst, frame_tracker::GetFrameTrackingInfo::Response& response)
{
	if (!tracking_)
	{
		std::string msg = "FrameTracker: Stop Tracking is already activated";
        ROS_ERROR_STREAM(msg);
        response.success = false;
        response.message = msg;
	}
	else
	{
		std::string msg = "FrameTracker: Stop Tracking is activated";
        ROS_INFO_STREAM(msg);
        response.success = true;
        response.message = msg;
        publishZeroTwist();
	}
	return true;
}

void FrameTracker::jointStateCallBack(const sensor_msgs::JointState::ConstPtr& msg)
{
	int count = 0;

	ROS_DEBUG("Call joint_state_callBack function ... ");

	for (unsigned int i = 0; i < dof_; ++i)
	{
		for (unsigned int j = 0; j < msg->name.size(); ++j)
		{
			if ( std::strcmp( msg->name[j].c_str(), joints_[i].c_str()) == 0 )
			{
				//current_position[i] = msg->position[j];
				last_q_(i) =  msg->position[j] ;
				last_q_dot_(i) =  msg->velocity[j] ;
				count++;
				break;
			}
		}
	}

	ros::Duration(0.1).sleep();

	if (count != dof_)
	{
		ROS_WARN(" Joint names are mismatched, need to check yaml file or code ... joint_state_callBack ");
	}
	else
	{
        ///---------------------------------------------------------------------
        KDL::FrameVel FrameVel;
        KDL::JntArrayVel jntArrayVel = KDL::JntArrayVel(last_q_, last_q_dot_);
        jntToCartSolver_vel_.reset(new KDL::ChainFkSolverVel_recursive(chain_));
        int ret = jntToCartSolver_vel_->JntToCart(jntArrayVel, FrameVel, -1);
        if (ret >= 0)
        {
            KDL::Twist twist = FrameVel.GetTwist();
            current_twist_ = twist;
        }
        else
        {
            ROS_ERROR("ChainFkSolverVel failed!");
        }
	}
}

int FrameTracker::checkStatus()
{
	bool infinitesimal_twist = checkInfinitesimalTwist(current_twist_);
	bool twist_violation = checkTwistViolation(current_twist_, target_twist_);

	if (twist_violation)
	{
		ROS_ERROR_STREAM("twist_violation");
	}
	return 1;
}


bool FrameTracker::checkTwistViolation(const KDL::Twist current, const KDL::Twist target)
{
    if (fabs(current.vel.x() - target.vel.x()) > twist_deviation_threshold_lin_)
    {
        return true;
    }
    if (fabs(current.vel.y() - target.vel.y()) > twist_deviation_threshold_lin_)
    {
        return true;
    }
    if (fabs(current.vel.z() - target.vel.z()) > twist_deviation_threshold_lin_)
    {
        return true;
    }
    if (fabs(current.rot.x() - target.rot.x()) > twist_deviation_threshold_rot_)
    {
        return true;
    }
    if (fabs(current.rot.y() - target.rot.y()) > twist_deviation_threshold_rot_)
    {
        return true;
    }
    if (fabs(current.rot.z() - target.rot.z()) > twist_deviation_threshold_rot_)
    {
        return true;
    }

    return false;
}

bool FrameTracker::checkInfinitesimalTwist(const KDL::Twist current)
{
    if (fabs(current.vel.x()) > twist_dead_threshold_lin_)
    {
        return false;
    }
    if (fabs(current.vel.y()) > twist_dead_threshold_lin_)
    {
        return false;
    }
    if (fabs(current.vel.z()) > twist_dead_threshold_lin_)
    {
        return false;
    }
    if (fabs(current.rot.x()) > twist_dead_threshold_rot_)
    {
        return false;
    }
    if (fabs(current.rot.x()) > twist_dead_threshold_rot_)
    {
        return false;
    }
    if (fabs(current.rot.x()) > twist_dead_threshold_rot_)
    {
        return false;
    }

    return true;
}
