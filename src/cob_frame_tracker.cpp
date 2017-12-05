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
#include <string>
#include <algorithm>
#include <math.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <cob_frame_tracker/cob_frame_tracker.h>
#include <cob_frame_tracker/FrameTrackingAction.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

bool CobFrameTracker::initialize()
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_tracker("frame_tracker");
    ros::NodeHandle nh_twist("twist_controller");

    /// get params
    if (nh_tracker.hasParam("update_rate"))
    {    nh_tracker.getParam("update_rate", update_rate_);    }
    else
    {    update_rate_ = 50.0;    }    // hz

    if (nh_.hasParam("chain_base_link"))
    {
        nh_.getParam("chain_base_link", chain_base_link_);
    }
    else
    {
        ROS_ERROR("No chain_base_link specified. Aborting!");
        return false;
    }

    if (nh_.hasParam("chain_tip_link"))
    {
        nh_.getParam("chain_tip_link", chain_tip_link_);
    }
    else
    {
        ROS_ERROR("No chain_tip_link specified. Aborting!");
        return false;
    }

    if (!nh_.getParam("joint_names", joints_))
    {
        ROS_ERROR("Parameter 'joint_names' not set");
        return false;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromParam("/robot_description", tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    tree.getChain(chain_base_link_, chain_tip_link_, chain_);
    if (chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }

    // initialize variables and current joint values and velocities
    dof_ = chain_.getNrOfJoints();
    lase_pose_ = KDL::JntArray(7);
    last_q_ = KDL::JntArray(dof_);
    last_q_dot_ = KDL::JntArray(dof_);

    if (nh_tracker.hasParam("movable_trans"))
    {    nh_tracker.getParam("movable_trans", movable_trans_);    }
    else
    {    movable_trans_ = true;    }
    if (nh_tracker.hasParam("movable_rot"))
    {    nh_tracker.getParam("movable_rot", movable_rot_);    }
    else
    {    movable_rot_ = true;    }

    if (nh_tracker.hasParam("max_vel_lin"))
    {    nh_tracker.getParam("max_vel_lin", max_vel_lin_);    }
    else
    {    max_vel_lin_ = 0.1;    }    // m/sec

    if (nh_tracker.hasParam("max_vel_rot"))
    {    nh_tracker.getParam("max_vel_rot", max_vel_rot_);    }
    else
    {    max_vel_rot_ = 0.1;    }    // rad/sec

    // Load PID Controller using gains set on parameter server
    pid_controller_trans_x_.init(ros::NodeHandle(nh_tracker, "pid_trans_x"));
    pid_controller_trans_x_.reset();
    pid_controller_trans_y_.init(ros::NodeHandle(nh_tracker, "pid_trans_y"));
    pid_controller_trans_y_.reset();
    pid_controller_trans_z_.init(ros::NodeHandle(nh_tracker, "pid_trans_z"));
    pid_controller_trans_z_.reset();

    pid_controller_rot_x_.init(ros::NodeHandle(nh_tracker, "pid_rot_x"));
    pid_controller_rot_x_.reset();
    pid_controller_rot_y_.init(ros::NodeHandle(nh_tracker, "pid_rot_y"));
    pid_controller_rot_y_.reset();
    pid_controller_rot_z_.init(ros::NodeHandle(nh_tracker, "pid_rot_z"));
    pid_controller_rot_z_.reset();

    tracking_ = false;
    tracking_goal_ = false;
    lookat_ = false;
    tracking_frame_ = chain_tip_link_;
    target_frame_ = "arm_7_target";
    lookat_focus_frame_ = "lookat_focus_frame";

    // ABORTION CRITERIA:
    enable_abortion_checking_ = true;
    cart_min_dist_threshold_lin_ = 0.01;
    cart_min_dist_threshold_rot_ = 0.01;
    twist_dead_threshold_lin_ = 0.05;
    twist_dead_threshold_rot_ = 0.05;
    twist_deviation_threshold_lin_ = 0.5;
    twist_deviation_threshold_rot_ = 0.5;

    cart_distance_ = 0.0;
    rot_distance_ = 0.0;

    current_twist_.Zero();
    target_twist_.Zero();

    abortion_counter_ = 0;
    max_abortions_ = update_rate_;    // if tracking fails for 1 second

    reconfigure_server_.reset(new dynamic_reconfigure::Server<cob_frame_tracker::FrameTrackerConfig>(reconfig_mutex_, nh_tracker));
    reconfigure_server_->setCallback(boost::bind(&CobFrameTracker::reconfigureCallback,   this, _1, _2));

    reconfigure_client_ = nh_twist.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters");

    jointstate_sub_ = nh_.subscribe("joint_states", 1, &CobFrameTracker::jointstateCallback, this);
    twist_pub_ = nh_twist.advertise<geometry_msgs::TwistStamped> ("command_twist_stamped", 1);
    // Carefully use node handler, nh_twist
    joint_vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);


    start_tracking_server_ = nh_tracker.advertiseService("start_tracking", &CobFrameTracker::startTrackingCallback, this);
    start_lookat_server_ = nh_tracker.advertiseService("start_lookat", &CobFrameTracker::startLookatCallback, this);
    stop_server_ = nh_tracker.advertiseService("stop", &CobFrameTracker::stopCallback, this);

    action_name_ = "tracking_action";
    as_.reset(new SAS_FrameTrackingAction_t(nh_tracker, action_name_, false));
    as_->registerGoalCallback(boost::bind(&CobFrameTracker::goalCB, this));
    as_->registerPreemptCallback(boost::bind(&CobFrameTracker::preemptCB, this));
    as_->start();

    //this->hard_coded_solver();

    pub_data_joint_vel.data.resize(7, 1.0);
    pub_data_joint_vel.data[0] = (last_q_dot_.data(0));
    pub_data_joint_vel.data[1] = (last_q_dot_.data(1));
    pub_data_joint_vel.data[2] = (last_q_dot_.data(2));
    pub_data_joint_vel.data[3] = (last_q_dot_.data(3));
    pub_data_joint_vel.data[4] = (last_q_dot_.data(4));
    pub_data_joint_vel.data[5] = (last_q_dot_.data(5));
    pub_data_joint_vel.data[6] = (last_q_dot_.data(6));

    timer_ = nh_.createTimer(ros::Duration(1/update_rate_), &CobFrameTracker::run, this);
    timer_.start();


    //pub_data_joint_vel.data = last_q_dot_.data;

    //this->solver();

    ROS_INFO(" ======================= CobFrameTracker ... initialized! ==================== ");

    return true;
}

void CobFrameTracker::run(const ros::TimerEvent& event)
{
    ros::Duration period = event.current_real - event.last_real;

    if (tracking_ || tracking_goal_ || lookat_)
    {
        if (tracking_goal_)  // tracking on action goal.
        {
            int status = checkStatus();

            if (status > 0)
            {
                action_success();
            }
            else if (status < 0)
            {
                action_abort();
            }
            else
            {
                // action still active - publish feedback
                if (as_->isActive()) { as_->publishFeedback(action_feedback_); }
            }
        }
        else  // tracking/lookat on service call
        {
            int status = checkServiceCallStatus();
            if (status < 0)
            {
                this->publishHoldTwist(period);
            }

            ht_.hold = abortion_counter_ >= max_abortions_;  // only for service call in case of action ht_.hold = false. What to do with actions?
        }

        solver();
        //publishTwist(period, !ht_.hold);  // if not publishing then just update data!
        //publishTwist(period, true);
    }
}

bool CobFrameTracker::getTransform(const std::string& from, const std::string& to, tf::StampedTransform& stamped_tf)
{
    bool transform = false;

    try
    {
        tf_listener_.waitForTransform(from, to, ros::Time(0), ros::Duration(0.2));
        tf_listener_.lookupTransform(from, to, ros::Time(0), stamped_tf);
        transform = true;
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("CobFrameTracker::getTransform: \n%s", ex.what());
    }

    return transform;
}

void CobFrameTracker::publishZeroTwist()
{
    // publish zero Twist for stopping
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.frame_id = tracking_frame_;
    twist_pub_.publish(twist_msg);
}

void CobFrameTracker::publishTwist(ros::Duration period, bool do_publish)
{
    tf::StampedTransform transform_tf;
    bool success = this->getTransform(tracking_frame_, target_frame_, transform_tf);

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.frame_id = tracking_frame_;
    twist_msg.header.stamp = ros::Time::now();

    if (!success)
    {
        ROS_WARN("publishTwist: failed to getTransform");
        return;
    }

    if (movable_trans_)
    {
        twist_msg.twist.linear.x = pid_controller_trans_x_.computeCommand(transform_tf.getOrigin().x(), period);
        twist_msg.twist.linear.y = pid_controller_trans_y_.computeCommand(transform_tf.getOrigin().y(), period);
        twist_msg.twist.linear.z = pid_controller_trans_z_.computeCommand(transform_tf.getOrigin().z(), period);
    }

    if (movable_rot_)
    {
        /// ToDo: Consider angular error as RPY or Quaternion?
        /// ToDo: What to do about sign conversion (pi->-pi) in angular rotation?

        twist_msg.twist.angular.x = pid_controller_rot_x_.computeCommand(transform_tf.getRotation().x(), period);
        twist_msg.twist.angular.y = pid_controller_rot_y_.computeCommand(transform_tf.getRotation().y(), period);
        twist_msg.twist.angular.z = pid_controller_rot_z_.computeCommand(transform_tf.getRotation().z(), period);
    }

    ROS_WARN_STREAM("Twist_msg: "<< twist_msg.twist);

    // eukl distance
    cart_distance_ = sqrt(pow(transform_tf.getOrigin().x(), 2) + pow(transform_tf.getOrigin().y(), 2) + pow(transform_tf.getOrigin().z(), 2));

    // rot distance
    // // TODO: change to cartesian rot
    // rot_distance_ = 2* acos(transform_msg.transform.rotation.w);

    if (cart_distance_ >= 0.06)
    {
        solver();
        //do_publish = false;
    }

    // get target_twist
    target_twist_.vel.x(twist_msg.twist.linear.x);
    target_twist_.vel.y(twist_msg.twist.linear.y);
    target_twist_.vel.z(twist_msg.twist.linear.z);
    target_twist_.rot.x(twist_msg.twist.angular.x);
    target_twist_.rot.y(twist_msg.twist.angular.y);
    target_twist_.rot.z(twist_msg.twist.angular.z);

    if (do_publish)
    {
        twist_pub_.publish(twist_msg);
    }
}

void CobFrameTracker::hard_coded_solver()
{
	ROS_WARN("Hard coded solver");

	DifferentialState x; // position
	Control v;	// velocity

	DifferentialEquation f;	// dynamic model
	f << dot(x) == v;

	// todo: how it solve
	OCP ocp_problem(0.0, 1.0, 2);	// objective function want to minimize
	ocp_problem.minimizeMayerTerm( (x - 0.05) * (x - 0.05) );
	ocp_problem.subjectTo(f);

	OptimizationAlgorithm alg(ocp_problem);

	DVector c_init(1), s_init(1);
	c_init.setAll(0.0);
	s_init.setAll(0.0);

	alg.initializeControls(c_init);
	alg.initializeDifferentialStates(s_init);

    // set solver option
    //alg.set(INTEGRATOR_TYPE, INT_RK78);
	//alg.set(INTEGRATOR_TOLERANCE, 1.000000E-08);
	//alg.set(DISCRETIZATION_TYPE, SINGLE_SHOOTING);
	//alg.set(KKT_TOLERANCE, 1.000000E-6);
	alg.set(MAX_NUM_ITERATIONS, 10);
	//alg.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
	//alg.set( HESSIAN_PROJECTION_FACTOR, 2.0 );
	alg.set(LEVENBERG_MARQUARDT, 1e-5);
	//alg.set(GAUSS_NEWTON, 1e-5);

	alg.solve();

	VariablesGrid c_output;
	alg.getControls(c_output);
	c_output.print();
}


void CobFrameTracker::solver()
{

    ROS_WARN("CobFrameTracker: Start solving ocp using ACADO Toolkit");

    tf::StampedTransform transform_tf;
    bool success = this->getTransform(tracking_frame_, target_frame_, transform_tf);
    cart_distance_ = sqrt(pow(transform_tf.getOrigin().x(), 2) + pow(transform_tf.getOrigin().y(), 2) + pow(transform_tf.getOrigin().z(), 2));
    if (cart_distance_ == 0.00 || !success)
    	return;

    //J_Mat.setIdentity();
    std::cout << J_Mat << std::endl;

    // get Jacobian matrix
    DMatrix jac_mat = J_Mat;

   // const unsigned int m = jac_mat.rows();  // rows of jacobian matrix, 6 with 3 lin and 3 angular velocity
   // const unsigned int n = jac_mat.cols();  // columns of jacobian matrix, number of joints(DOF)

    const unsigned int m = 6;  // rows of jacobian matrix, 6 with 3 lin and 3 angular velocity
    const unsigned int n = 7;

    //-----------------------------------------------------------------------------
    DifferentialState x("",m,1);                // position
    Parameter e("",n,1);						// residual error
    Control q_dot("",n,1);                      // velocity

    x.clearStaticCounters();
    e.clearStaticCounters();
    q_dot.clearStaticCounters();

    DifferentialEquation f;             // Define differential equation
    f << dot(x) == jac_mat * q_dot;

    OCP ocp_problem(0.0, 1.0, 2);
    ocp_problem.minimizeMayerTerm( 10.0 * (e.transpose() * e) );
    ocp_problem.subjectTo(f);

    OptimizationAlgorithm alg(ocp_problem);

	DVector c_init(7), s_init(6), p_init(7);
	c_init.setAll(0.01);
	//c_init(0) = 1.0;
	//c_init = pub_data_joint_vel.data;//last_q_dot_.data;
	//c_init = last_q_dot_.data;

	std::cout << "********** control initialize **********************" << std::endl;
	std::cout << c_init << std::endl;
	std::cout << "********** read from parameter server **********************" << std::endl;
	std::cout << pub_data_joint_vel.data << std::endl;

	s_init.setAll(0.0);
	// todo: initalize end-effector pose using fk
	s_init = last_q_.data;
	//std::cout << "********** state initialize **********************" << std::endl;
	//std::cout << s_init << std::endl;

	p_init.setAll(0.0);
	//p_init(0) = 10.0;
	p_init(0) = transform_tf.getOrigin().x();
	p_init(1) = transform_tf.getOrigin().y();
	p_init(2) = transform_tf.getOrigin().z();
	p_init(3) = transform_tf.getRotation().x();
	p_init(4) = transform_tf.getRotation().y();
	p_init(5) = transform_tf.getRotation().z();
	p_init(6) = transform_tf.getRotation().w();
	std::cout << "********** Error initialize **********************" << std::endl;
	std::cout << p_init << std::endl;

	//alg.initializeControls(c_init);
	alg.initializeDifferentialStates(s_init);
	alg.initializeParameters(p_init);

    alg.set(MAX_NUM_ITERATIONS, 10);
    alg.set(LEVENBERG_MARQUARDT, 1e-5);

    alg.solve();

	VariablesGrid c_output, s_output, p_output;
	alg.getDifferentialStates(s_output);
	alg.getControls(c_output);
	alg.getParameters(p_output);
	std::cout << " ===============Control=========== " <<c_output.getDim()<<" ======================== " << std::endl;
	c_output.getLastVector().print();
	std::cout << " ===============State=========== " <<s_output.getDim()<<" ======================== " << std::endl;
	s_output.getLastVector().print();
	std::cout << " ===============Parameter=========== " <<p_output.getDim()<<" ======================== " << std::endl;
	p_output.getLastVector().print();

	DVector cnt_jnt_vel = c_output.getLastVector();
	pub_data_joint_vel.data.resize(7);

	pub_data_joint_vel.data[0] = ( double(cnt_jnt_vel(0)) );
    pub_data_joint_vel.data[1] = ( double(cnt_jnt_vel(1)) );
    pub_data_joint_vel.data[2] = ( double(cnt_jnt_vel(2)) );
	pub_data_joint_vel.data[3] = ( double(cnt_jnt_vel(3)) );
	pub_data_joint_vel.data[4] = ( double(cnt_jnt_vel(4)) );
	pub_data_joint_vel.data[5] = ( double(cnt_jnt_vel(5)) );
	pub_data_joint_vel.data[6] = ( double(cnt_jnt_vel(6)) );

/*
	std_msgs::Float64MultiArray pub_data_joint_vel;
	//pub_data_joint_vel.data.clear();
	pub_data_joint_vel.data.resize(7);

	if (!cnt_jnt_vel.isEmpty())
	{
************
    	pub_data_joint_vel.data.push_back( double(cnt_jnt_vel(0)) );
        pub_data_joint_vel.data.push_back( double(cnt_jnt_vel(1)) );
        pub_data_joint_vel.data.push_back( double(cnt_jnt_vel(2)) );
		pub_data_joint_vel.data.push_back( double(cnt_jnt_vel(3)) );
		pub_data_joint_vel.data.push_back( double(cnt_jnt_vel(4)) );
		pub_data_joint_vel.data.push_back( double(cnt_jnt_vel(5)) );
		pub_data_joint_vel.data.push_back( double(cnt_jnt_vel(6)) );**************

    	pub_data_joint_vel.data[0] = ( double(cnt_jnt_vel(0)) );
        pub_data_joint_vel.data[1] = ( double(cnt_jnt_vel(1)) );
        pub_data_joint_vel.data[2] = ( double(cnt_jnt_vel(2)) );
		pub_data_joint_vel.data[3] = ( double(cnt_jnt_vel(3)) );
		pub_data_joint_vel.data[4] = ( double(cnt_jnt_vel(4)) );
		pub_data_joint_vel.data[5] = ( double(cnt_jnt_vel(5)) );
		pub_data_joint_vel.data[6] = ( double(cnt_jnt_vel(6)) );

*************
        //pub_data_joint_vel.data.push_back(1.0);
      	pub_data_joint_vel.data.push_back(0.0);
      	pub_data_joint_vel.data.push_back(0.0);
      	pub_data_joint_vel.data.push_back(0.0);
      	pub_data_joint_vel.data.push_back(0.0);
      	pub_data_joint_vel.data.push_back(0.0);
    	pub_data_joint_vel.data.push_back(0.0);
*********
	}
    else
    {
    	std::cout << "Empty joint velocity" << std::endl;
    	pub_data_joint_vel.data[0] = (0.0);
        pub_data_joint_vel.data[1] = (0.0);
        pub_data_joint_vel.data[2] = (0.0);
		pub_data_joint_vel.data[3] = (0.0);
		pub_data_joint_vel.data[4] = (0.0);
		pub_data_joint_vel.data[5] = (0.0);
		pub_data_joint_vel.data[6] = (0.0);

    }

    //joint_vel_pub_.publish(pub_data_joint_vel);
*/
/*
    //-----------------------------------------------------------------------------
    // OCP problem definition
    ROS_INFO(" Define ocp problem ... ");
    OCP ocp_problem(0.0, 1.0, 10);
    //ocp_problem.minimizeMayerTerm( 0.5 * (pose_error.transpose() * pose_error) );
    //ocp_problem.minimizeLagrangeTerm(0.5 * (res_error.transpose() * res_error));
  ********  /ocp_problem.minimizeMayerTerm((  0.5 *	(transform_tf.getOrigin().x()*transform_tf.getOrigin().x()) +
    							   			(transform_tf.getOrigin().y()*transform_tf.getOrigin().y()) +
    							   			(transform_tf.getOrigin().z()*transform_tf.getOrigin().z()) +
    							   			(transform_tf.getRotation().x()*transform_tf.getRotation().x()) +
    							   			(transform_tf.getRotation().y()*transform_tf.getRotation().y()) +
    							   			(transform_tf.getRotation().z()*transform_tf.getRotation().z()) +
    							   			(transform_tf.getRotation().w()*transform_tf.getRotation().w()) )
    							   );**********

    ocp_problem.minimizeMayerTerm((  transform_tf.getOrigin().x() +
        							 transform_tf.getOrigin().y() +
        							 transform_tf.getOrigin().z() +
        							 transform_tf.getRotation().x() +
        							 transform_tf.getRotation().y() +
        							 transform_tf.getRotation().z() +
        							 transform_tf.getRotation().w())
        							   );
    ocp_problem.subjectTo(f);

    //-----------------------------------------------------------------------------
    // OCP Algorithm definition
    ROS_INFO(" Define ocp algorithm ... ");
    OptimizationAlgorithm ocp_solver(ocp_problem);

    // initialize pose error, controls and states
    DVector res_error_init(7), control_init(n);
    res_error_init.setAll(0.0);
    res_error_init(0) = transform_tf.getOrigin().x();
    res_error_init(1) = transform_tf.getOrigin().y();
    res_error_init(2) = transform_tf.getOrigin().z();
    res_error_init(3) = transform_tf.getRotation().x();
    res_error_init(4) = transform_tf.getRotation().y();
    res_error_init(5) = transform_tf.getRotation().z();
    res_error_init(6) = transform_tf.getRotation().w();
    //ROS_DEBUG_STREAM("Error Pose init: "<< pose_error_init);
    control_init.setAll(0.0);
    control_init = last_q_dot_.data;

    //ocp_solver.initializeDifferentialStates(res_error_init);
    ocp_solver.initializeControls(control_init);
    //ocp_solver.initializeParameters(res_error_init);

    // set solver option
    //ocp_solver.set(INTEGRATOR_TYPE, INT_RK78);
    ocp_solver.set(INTEGRATOR_TOLERANCE, 1.000000E-08);
    ocp_solver.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    ocp_solver.set(KKT_TOLERANCE, 1.000000E-15);
    ocp_solver.set(MAX_NUM_ITERATIONS, 10);
    ocp_solver.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    ocp_solver.set( HESSIAN_PROJECTION_FACTOR, 2.0 );
    ocp_solver.set(LEVENBERG_MARQUARDT, 1e-5);

    ocp_solver.solve();

    ROS_WARN(" Let's try to print states, controls ... ");
    VariablesGrid control_ouput, state_output;
    ocp_solver.getDifferentialStates(state_output);
    ocp_solver.getDifferentialStates("/home/bfb-ws/mpc_ws/src/frame_tracker/result/states.txt");
    ocp_solver.getControls("/home/bfb-ws/mpc_ws/src/frame_tracker/result/controls.txt");
    ocp_solver.getControls(control_ouput);

    control_ouput.getVector(5).print();

    ROS_WARN("OCP Solved!!");
    */
}


void CobFrameTracker::publishHoldTwist(const ros::Duration& period)
{
    tf::StampedTransform transform_tf;
    bool success = this->getTransform(chain_base_link_, tracking_frame_, transform_tf);

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.frame_id = tracking_frame_;

    if (!this->ht_.hold)
    {
        ROS_ERROR_STREAM_THROTTLE(1, "Abortion active: Publishing zero twist");
        ht_.hold = success;
        ht_.transform_tf = transform_tf;
    }
    else
    {
        ROS_ERROR_STREAM_THROTTLE(1, "Abortion active: Publishing hold posture twist");
        if (success)
        {
            twist_msg.twist.linear.x = pid_controller_trans_x_.computeCommand(ht_.transform_tf.getOrigin().x() - transform_tf.getOrigin().x(), period);
            twist_msg.twist.linear.y = pid_controller_trans_y_.computeCommand(ht_.transform_tf.getOrigin().y() - transform_tf.getOrigin().y(), period);
            twist_msg.twist.linear.z = pid_controller_trans_z_.computeCommand(ht_.transform_tf.getOrigin().z() - transform_tf.getOrigin().z(), period);

            twist_msg.twist.angular.x = pid_controller_rot_x_.computeCommand(ht_.transform_tf.getRotation().x() - transform_tf.getRotation().x(), period);
            twist_msg.twist.angular.y = pid_controller_rot_y_.computeCommand(ht_.transform_tf.getRotation().y() - transform_tf.getRotation().y(), period);
            twist_msg.twist.angular.z = pid_controller_rot_z_.computeCommand(ht_.transform_tf.getRotation().z() - transform_tf.getRotation().z(), period);
        }
    }

    twist_pub_.publish(twist_msg);
}

bool CobFrameTracker::startTrackingCallback(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response)
{
    if (tracking_)
    {
        std::string msg = "CobFrameTracker: StartTracking denied because Tracking already active";
        ROS_ERROR_STREAM(msg);
        response.success = false;
        response.message = msg;
    }
    else if (tracking_goal_)
    {
        std::string msg = "CobFrameTracker: StartTracking denied because TrackingAction is active";
        ROS_ERROR_STREAM(msg);
        response.success = false;
        response.message = msg;
    }
    else if (lookat_)
    {
        std::string msg = "CobFrameTracker: StartTracking denied because Lookat is active";
        ROS_ERROR_STREAM(msg);
        response.success = false;
        response.message = msg;
    }
    else
    {
        // check whether given target frame exists
        if (!tf_listener_.frameExists(request.data))
        {
            std::string msg = "CobFrameTracker: StartTracking denied because target frame '" + request.data + "' does not exist";
            ROS_ERROR_STREAM(msg);
            response.success = false;
            response.message = msg;
        }
        else
        {
            std::string msg = "CobFrameTracker: StartTracking started with CART_DIST_SECURITY MONITORING enabled";
            ROS_INFO_STREAM(msg);
            response.success = true;
            response.message = msg;

            tracking_ = true;
            tracking_goal_ = false;
            lookat_ = false;
            tracking_frame_ = chain_tip_link_;
            target_frame_ = request.data;
        }
    }
    return true;
}

bool CobFrameTracker::startLookatCallback(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response)
{
    if (tracking_)
    {
        std::string msg = "CobFrameTracker: StartLookat denied because Tracking active";
        ROS_ERROR_STREAM(msg);
        response.success = false;
        response.message = msg;
    }
    else if (tracking_goal_)
    {
        std::string msg = "CobFrameTracker: StartLookat denied because TrackingAction is active";
        ROS_ERROR_STREAM(msg);
        response.success = false;
        response.message = msg;
    }
    else if (lookat_)
    {
        std::string msg = "CobFrameTracker: StartLookat denied because Lookat is already active";
        ROS_ERROR_STREAM(msg);
        response.success = false;
        response.message = msg;
    }
    else
    {
        // check whether given target frame exists
        if (!tf_listener_.frameExists(request.data))
        {
            std::string msg = "CobFrameTracker: StartLookat denied because target frame '" + request.data + "' does not exist";
            ROS_ERROR_STREAM(msg);
            response.success = false;
            response.message = msg;
        }
        else
        {
            dynamic_reconfigure::Reconfigure srv;
            dynamic_reconfigure::IntParameter int_param;
            int_param.name = "kinematic_extension";
            int_param.value = 4;    // LOOKAT
            srv.request.config.ints.push_back(int_param);

            bool success = reconfigure_client_.call(srv);

            if (success)
            {
                std::string msg = "CobFrameTracker: StartLookat started with CART_DIST_SECURITY MONITORING enabled";
                ROS_INFO_STREAM(msg);
                response.success = true;
                response.message = msg;

                tracking_ = false;
                tracking_goal_ = false;
                lookat_ = true;
                tracking_frame_ = lookat_focus_frame_;
                target_frame_ = request.data;
            }
            else
            {
                std::string msg = "CobFrameTracker: StartLookat denied because DynamicReconfigure failed";
                ROS_ERROR_STREAM(msg);
                response.success = false;
                response.message = msg;
            }
        }
    }
    return true;
}

bool CobFrameTracker::stopCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
    if (tracking_goal_)
    {
        std::string msg = "CobFrameTracker: Stop denied because TrackingAction is active";
        ROS_ERROR_STREAM(msg);
        response.success = false;
        response.message = msg;
    }
    else if (tracking_ || lookat_)
    {
        if (lookat_)
        {
            // disable LOOKAT in dynamic_reconfigure server
            dynamic_reconfigure::Reconfigure srv;
            dynamic_reconfigure::IntParameter int_param;
            int_param.name = "kinematic_extension";
            int_param.value = 0;    // NO_EXTENSION
            srv.request.config.ints.push_back(int_param);

            if (!reconfigure_client_.call(srv))
            {
                std::string msg = "CobFrameTracker: Stop failed to disable LOOKAT_EXTENSION. Stopping anyway!";
                ROS_ERROR_STREAM(msg);
            }
        }

        std::string msg = "CobFrameTracker: Stop successful";
        ROS_INFO_STREAM(msg);
        response.success = true;
        response.message = msg;

        tracking_ = false;
        tracking_goal_ = false;
        lookat_ = false;
        tracking_frame_ = chain_tip_link_;
        target_frame_ = tracking_frame_;

        publishZeroTwist();
    }
    else
    {
        std::string msg = "CobFrameTracker: Stop failed because nothing was tracked";
        ROS_ERROR_STREAM(msg);
        response.success = false;
        response.message = msg;
    }
    return true;
}

void CobFrameTracker::goalCB()
{
    ROS_INFO("Received a new goal");
    if (as_->isNewGoalAvailable())
    {
        boost::shared_ptr<const cob_frame_tracker::FrameTrackingGoal> goal_ = as_->acceptNewGoal();

        if (tracking_ || lookat_)
        {
            // Goal should not be accepted
            ROS_ERROR_STREAM("CobFrameTracker: Received ActionGoal while tracking/lookat Service is active!");
        }
        else if (!tf_listener_.frameExists(goal_->tracking_frame))
        {
            // Goal should not be accepted
            ROS_ERROR_STREAM("CobFrameTracker: Received ActionGoal but target frame '" << goal_->tracking_frame << "' does not exist");
        }
        else
        {
            target_frame_ = goal_->tracking_frame;
            tracking_duration_ = goal_->tracking_duration;
            stop_on_goal_ = goal_->stop_on_goal;
            tracking_ = false;
            tracking_goal_ = true;
            lookat_ = false;
            abortion_counter_ = 0;
            tracking_start_time_ = ros::Time::now();
        }
    }
}

void CobFrameTracker::preemptCB()
{
    ROS_WARN("Received a preemption request");
    action_result_.success = true;
    action_result_.message = "Action has been preempted";
    as_->setPreempted(action_result_);
    tracking_ = false;
    tracking_goal_ = false;
    lookat_ = false;
    tracking_frame_ = chain_tip_link_;
    target_frame_ = tracking_frame_;

    publishZeroTwist();
}

void CobFrameTracker::action_success()
{
    ROS_INFO("Goal succeeded!");
    as_->setSucceeded(action_result_, action_result_.message);

    tracking_ = false;
    tracking_goal_ = false;
    lookat_ = false;
    tracking_frame_ = chain_tip_link_;
    target_frame_ = tracking_frame_;

    publishZeroTwist();
}

void CobFrameTracker::action_abort()
{
    ROS_WARN("Goal aborted");
    as_->setAborted(action_result_, action_result_.message);

    tracking_ = false;
    tracking_goal_ = false;
    lookat_ = false;
    tracking_frame_ = chain_tip_link_;
    target_frame_ = tracking_frame_;

    publishZeroTwist();
}

int CobFrameTracker::checkStatus()
{
    int status = 0;

    if (!enable_abortion_checking_)
    {
        abortion_counter_ = 0;
        return status;
    }

    if (ros::Time::now() > tracking_start_time_ + ros::Duration(tracking_duration_))
    {
        action_result_.success = true;
        action_result_.message = std::string("Successfully tracked goal for %f seconds", tracking_duration_);
        status = 1;
    }

    bool infinitesimal_twist = checkInfinitesimalTwist(current_twist_);
    bool distance_violation = checkCartDistanceViolation(cart_distance_, 0.0);
    bool twist_violation = checkTwistViolation(current_twist_, target_twist_);

    if (stop_on_goal_)
    {
        /// ToDo: better metric for when goal is reached
        if (infinitesimal_twist && !distance_violation && !twist_violation)
        {
            action_result_.success = true;
            action_result_.message = "Successfully reached goal";
            status = 2;
        }
    }

    if (distance_violation || twist_violation)
    {
        ROS_ERROR_STREAM("distance_violation || twist_violation");
        abortion_counter_++;
    }

    if (abortion_counter_ > max_abortions_)
    {
        action_result_.success = false;
        action_result_.message = "Constraints violated. Action aborted";
        status = -1;
    }

    return status;
}


int CobFrameTracker::checkServiceCallStatus()
{
    int status = 0;

    if (!enable_abortion_checking_)
    {
        abortion_counter_ = 0;
        return status;
    }

    bool distance_violation = checkCartDistanceViolation(cart_distance_, 0.0);

    if (distance_violation)
    {
        abortion_counter_++;
    }
    else
    {
        abortion_counter_ = abortion_counter_ > 0 ? abortion_counter_ - 1 : 0;
    }

    if (abortion_counter_ >= max_abortions_)
    {
        abortion_counter_ = max_abortions_;
        status = -1;
    }

    return status;
}


void CobFrameTracker::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    KDL::JntArray q_temp = last_q_;
    KDL::JntArray q_dot_temp = last_q_dot_;
    int count = 0;
    for (unsigned int j = 0; j < dof_; j++)
    {
        for (unsigned int i = 0; i < msg->name.size(); i++)
        {
            if (strcmp(msg->name[i].c_str(), joints_[j].c_str()) == 0)
            {
                q_temp(j) = msg->position[i];
                q_dot_temp(j) = msg->velocity[i];
                count++;
                break;
            }
        }
    }
    if (count == joints_.size())
    {
        last_q_ = q_temp;
        last_q_dot_ = q_dot_temp;
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
        ///--------------------------------------------------------------------
        /// Forward Kinematic
        jntToCartSolver_pos_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
        KDL::Frame FramePos;	//end-effector pos
        unsigned int pose_solver_state = jntToCartSolver_pos_->JntToCart(last_q_, FramePos);

        double qx, qy, qz, qw;
        FramePos.M.GetQuaternion(qx, qy, qz, qw);
        lase_pose_(0) = FramePos.p.x();	lase_pose_(1) = FramePos.p.y();	lase_pose_(2) = FramePos.p.z();	//position
        lase_pose_(3) = qx;	lase_pose_(4) = qy;	lase_pose_(5) = qz;	lase_pose_(6) = qw;	// Quaternion

        ///---------------------------------------------------------------------
        /// JAcobian calculation
        jntToJacSolver_.reset(new KDL::ChainJntToJacSolver(chain_));
        KDL::Jacobian j_kdl = KDL::Jacobian(dof_);

        unsigned int solver_state = jntToJacSolver_->JntToJac(last_q_, j_kdl );

		for (unsigned int i = 0; i < 6; ++i)
		{
			for (unsigned int j = 0; j < dof_; ++j)
			{
				J_Mat(i,j) = j_kdl(i,j);
			}
		}

    }
}

void CobFrameTracker::reconfigureCallback(cob_frame_tracker::FrameTrackerConfig& config, uint32_t level)
{
    enable_abortion_checking_ = config.enable_abortion_checking;
    cart_min_dist_threshold_lin_ = config.cart_min_dist_threshold_lin;
    cart_min_dist_threshold_rot_ = config.cart_min_dist_threshold_rot;
    twist_dead_threshold_lin_ = config.twist_dead_threshold_lin;
    twist_dead_threshold_rot_ = config.twist_dead_threshold_rot;
    twist_deviation_threshold_lin_ = config.twist_deviation_threshold_lin;
    twist_deviation_threshold_rot_ = config.twist_deviation_threshold_rot;
}

/** checks whether the twist is infinitesimally small **/
bool CobFrameTracker::checkInfinitesimalTwist(const KDL::Twist current)
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

    /// all twist velocities are <= dead_threshold -> twist is infinitesimal
    return true;
}

/** checks whether the Cartesian distance between tip and target frame is ok **/
bool CobFrameTracker::checkCartDistanceViolation(const double dist, const double rot)
{
    if (dist > cart_min_dist_threshold_lin_)
    {
        return true;
    }
    if (rot > cart_min_dist_threshold_rot_)
    {
        return true;
    }

    /// Cartesian distance is acceptable -> no violation
    return false;
}

/** checks whether the current twist is following the target twist "close enough" **/
bool CobFrameTracker::checkTwistViolation(const KDL::Twist current, const KDL::Twist target)
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

    /// Cartesian Twist distance is acceptable -> no violation
    return false;
}
