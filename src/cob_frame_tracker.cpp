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

#include <iostream>
#include <fstream>

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
    lase_pose_ = KDL::JntArray(6);
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
    //this->hard_coded_solver();

    //pub_data_joint_vel.data = last_q_dot_.data;

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
                //solver();
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
        //this->hard_coded_solver();
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

	ROS_WARN("***************** Last control vector: ****************** ");
	DVector cnt_jnt_vel = c_output.getLastVector();
	cnt_jnt_vel.print();
	ROS_ERROR_STREAM("Size of control vector: "<< c_output.getLastVector().getDim());

	pub_data_joint_vel.data.resize(7);
	pub_data_joint_vel.data[0] = cnt_jnt_vel(0);

	joint_vel_pub_.publish(pub_data_joint_vel);

}


void CobFrameTracker::solver()
{

    ROS_WARN("CobFrameTracker: Start solving ocp using ACADO Toolkit");

    tf::StampedTransform transform_tf, target_frame_TO_root_frame;
    bool success = this->getTransform(tracking_frame_, target_frame_, transform_tf); //target_frame_
    bool success1 = this->getTransform("/arm_podest_link", target_frame_, target_frame_TO_root_frame); //target_frame_



    //J_Mat.setIdentity();
    //std::cout << J_Mat << std::endl;

    // get Jacobian matrix
    DMatrix jac_mat = J_Mat;
    //Manipulability of Robotic Mechanisms by Tsuneo Yoshikawa
    Eigen::MatrixXd manipuble_matrix =  J_Mat * J_Mat.transpose();

    const unsigned int m = 6;  // rows of jacobian matrix, 6 with 3 lin and 3 angular velocity
    const unsigned int n = 7;

    //-----------------------------------------------------------------------------
    DifferentialState x("",m,1);                // position
    Control v("",n,1);                      // velocity

    x.clearStaticCounters();
    v.clearStaticCounters();

    DifferentialEquation f;             // Define differential equation
    f << dot(x) == jac_mat * v;
/*
    DVector e_init(7);
    e_init.setAll(0.0);
    e_init(0) = target_frame_TO_root_frame.getOrigin().x();
    e_init(1) = target_frame_TO_root_frame.getOrigin().y();
    e_init(2) = target_frame_TO_root_frame.getOrigin().z();
    e_init(3) = target_frame_TO_root_frame.getRotation().x();
    e_init(4) = target_frame_TO_root_frame.getRotation().y();
    e_init(5) = target_frame_TO_root_frame.getRotation().z();
    e_init(6) = target_frame_TO_root_frame.getRotation().w();
*/

	DVector c_init(7), s_init(6);
	c_init.setAll(0.00);
	s_init.setAll(0.00);

	std::cout << "*************** pose of end-effector: \n"<< lase_pose_.data << std::endl;
    ROS_WARN_STREAM("*************** target_frame_TO_root_frame: "<< target_frame_TO_root_frame.getOrigin().x()<<"  " << target_frame_TO_root_frame.getOrigin().y()
    																<<"  "<<target_frame_TO_root_frame.getOrigin().z());
	//std::cout << "*************** control of end-effector: \n"<< pub_data_joint_vel.data << std::endl;
	s_init = lase_pose_.data;
	//c_init = last_q_dot_.data;
	c_init = pub_data_joint_vel.data;

    OCP ocp_problem(0.0, 1.0, 4);
    ocp_problem.minimizeMayerTerm( (v.transpose()*v) + 10.0*(( (x(0)-target_frame_TO_root_frame.getOrigin().x())  * (x(0)-target_frame_TO_root_frame.getOrigin().x()) ) +
    							   ( (x(1)-target_frame_TO_root_frame.getOrigin().y())  * (x(1)-target_frame_TO_root_frame.getOrigin().y()) ) +
    							   ( (x(2)-target_frame_TO_root_frame.getOrigin().z())  * (x(2)-target_frame_TO_root_frame.getOrigin().z()) )  //+
    							   /*( (x(3)-target_frame_TO_root_frame.getRotation().getAxis().x())  * (x(3)-target_frame_TO_root_frame.getRotation().getAxis().x()) ) +
    							   ( (x(4)-target_frame_TO_root_frame.getRotation().getAxis().y())  * (x(4)-target_frame_TO_root_frame.getRotation().getAxis().y()) ) +
    							   ( (x(5)-target_frame_TO_root_frame.getRotation().getAxis().z())  * (x(5)-target_frame_TO_root_frame.getRotation().getAxis().z()) )*/
    							   ));
    //ocp_problem.minimizeMayerTerm(10.0 * v.transpose()*v);
    //ocp_problem.minimizeMayerTerm( 10.0 * (e.transpose() * e ) );
    //ocp_problem.minimizeMayerTerm( 100.0*((transform_tf.getOrigin().x()*transform_tf.getOrigin().x()) + (transform_tf.getOrigin().y()*transform_tf.getOrigin().y()) + (transform_tf.getOrigin().z()*transform_tf.getOrigin().z())) );
    //ocp_problem.minimizeMayerTerm( 10.0*( ( (x(0) - 0.39824) *(x(0)-0.39824) ) + ( (x(1) +0.30408) *(x(1)+0.30408) ) + ( (x(2) - 0.28162) *(x(2)-0.28162) )) );
    //ocp_problem.minimizeMayerTerm( 10.0*( ( (x(0) - 0) *(x(0)-0) ) + ( (x(1) +0.28298) *(x(1)+0.28298) ) + ( (x(2) - 0.0) *(x(2)-0.0) )) );
    //ocp_problem.minimizeMayerTerm( ( (x(2) - 0.01) *(x(2)-0.01) ) );

    ocp_problem.subjectTo(f);
    ocp_problem.subjectTo(-0.50 <= v <= 0.50);
    //ocp_problem.subjectTo(AT_START, v == 1.0);
    ocp_problem.subjectTo(AT_END , v == 0.0);

    //OptimizationAlgorithm alg(ocp_problem);
    RealTimeAlgorithm alg(ocp_problem, 0.025);

	//alg << window;

	//ROS_WARN_STREAM("*************** pose error of end-effector: "<< transform_tf.getOrigin().x()<<"  " << transform_tf.getOrigin().y()
																//<<"  "<<transform_tf.getOrigin().z()	<< "\n orirnt: \n" << transform_tf.getRotation());

	//std::cout << "*************** Error of end-effector: \n"<< e_init << std::endl;

	alg.initializeControls(c_init);
	alg.initializeDifferentialStates(s_init);

    alg.set(MAX_NUM_ITERATIONS, 10);
    alg.set(LEVENBERG_MARQUARDT, 1e-5);
    alg.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    //alg.set( DISCRETIZATION_TYPE, COLLOCATION);
    alg.set( DISCRETIZATION_TYPE, COLLOCATION);
    alg.set(KKT_TOLERANCE, 1.000000E-06);


/*    alg.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    alg.set( 'SPARSE_QP_SOLUTION',          FULL_CONDENSING_N2');
    alg.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
    alg.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );*/
	std::cout<<"\033[20;1m" // green console colour
			<< "***********************"<<std::endl
			<< "objective function: " <<  alg.getObjectiveValue()
			<< "***********************"
			<< "\033[0m\n" << std::endl;
	alg.getObjectiveValue("/home/bfb-ws/mpc_ws/src/frame_tracker/result/objective_func.txt");
    //alg.solve();
    DVector diff_control_State_init(6);
    diff_control_State_init.setAll(0.0);


    Controller controller(alg);

    controller.init(0.0, s_init);
    controller.step(0.0, s_init);

	VariablesGrid c_output;
	alg.getControls(c_output);
	alg.getControls("/home/bfb-ws/mpc_ws/src/frame_tracker/result/control_csv.csv");


	std::ofstream myfile;
	myfile.open("/home/bfb-ws/mpc_ws/src/frame_tracker/result/control_output.csv", std::ios::out | std::ios::app | std::ios::binary);
	myfile << c_output;
	myfile << "\n.\n";

	//c_output.print();

	ROS_WARN("***************** First control vector: ****************** ");
	//DVector cnt_jnt_vel = c_output.getFirstVector();
	DVector cnt_jnt_vel;
	controller.getU(cnt_jnt_vel);
	cnt_jnt_vel.print();
	//ROS_ERROR_STREAM("Size of control vector: "<< c_output.getFirstVector().getDim());
	//ROS_ERROR_STREAM("Size of last control vector: "<< c_output.getLastVector().getDim());

	double cart_distance = sqrt(transform_tf.getOrigin().x()*transform_tf.getOrigin().x() +
								transform_tf.getOrigin().y()*transform_tf.getOrigin().y() +
								transform_tf.getOrigin().z()*transform_tf.getOrigin().z()
								);


	double manipuability = sqrt(manipuble_matrix.determinant());

  //print data on to console
	std::cout<<"\033[36;1m" // green console colour
			<< "***********************"<<std::endl
			<< "cartesian distance: " << cart_distance
			<< "\t manipuabilty: " << manipuability
			<< "***********************"
			<< "\033[0m\n" << std::endl;


	if( cart_distance < 0.05 )
	{
		pub_data_joint_vel.data.resize(7);
		//pub_data_joint_vel.data[1] = cnt_jnt_vel(1);
		pub_data_joint_vel.data[0] = 0;
		pub_data_joint_vel.data[1] = 0;
		pub_data_joint_vel.data[2] = 0;
		pub_data_joint_vel.data[3] = 0;
		pub_data_joint_vel.data[4] = 0;
		pub_data_joint_vel.data[5] = 0;
		pub_data_joint_vel.data[6] = 0;
		joint_vel_pub_.publish(pub_data_joint_vel);

	}
	else
	{/*
		if (manipuble_matrix.determinant() <= 0.009)
		{
			pub_data_joint_vel.data.resize(7);
			//pub_data_joint_vel.data[1] = cnt_jnt_vel(1);
			pub_data_joint_vel.data[0] = 0;
			pub_data_joint_vel.data[1] = 0;
			pub_data_joint_vel.data[2] = 0;
			pub_data_joint_vel.data[3] = 0;
			pub_data_joint_vel.data[4] = 0;
			pub_data_joint_vel.data[5] = 0;
			pub_data_joint_vel.data[6] = 0;
			joint_vel_pub_.publish(pub_data_joint_vel);
		}
		else
		{*/
			pub_data_joint_vel.data.resize(7);
			//pub_data_joint_vel.data[1] = cnt_jnt_vel(1);
			pub_data_joint_vel.data[0] = cnt_jnt_vel(0);
			pub_data_joint_vel.data[1] = cnt_jnt_vel(1);
			pub_data_joint_vel.data[2] = cnt_jnt_vel(2);
			pub_data_joint_vel.data[3] = cnt_jnt_vel(3);
			pub_data_joint_vel.data[4] = cnt_jnt_vel(4);
			pub_data_joint_vel.data[5] = cnt_jnt_vel(5);
			pub_data_joint_vel.data[6] = cnt_jnt_vel(6);
			joint_vel_pub_.publish(pub_data_joint_vel);
		//}
	}
}
void CobFrameTracker::publishHoldJointState(const ros::Duration& period)
{
    tf::StampedTransform transform_tf;
    bool success = this->getTransform(tracking_frame_, target_frame_, transform_tf);

    double error = transform_tf.getOrigin().x() + transform_tf.getOrigin().y() + transform_tf.getOrigin().z() + transform_tf.getRotation().x() + transform_tf.getRotation().y() + transform_tf.getRotation().z() + transform_tf.getRotation().w();

  //print data on to console
	std::cout<<"\033[36;1m" << "***********************"<<std::endl << "residual error want to minimize: " << error<< "***********************"<< "\033[0m\n" << std::endl;


	pub_data_joint_vel.data.resize(7);
	//pub_data_joint_vel.data[1] = cnt_jnt_vel(1);
	pub_data_joint_vel.data[0] = 0;
	pub_data_joint_vel.data[1] = 0;
	pub_data_joint_vel.data[2] = 0;
	pub_data_joint_vel.data[3] = 0;
	pub_data_joint_vel.data[4] = 0;
	pub_data_joint_vel.data[5] = 0;
	pub_data_joint_vel.data[6] = 0;
	joint_vel_pub_.publish(pub_data_joint_vel);

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
	//ROS_ERROR("///////////////////////// JointStateCB -------------------------------------");
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

        double r,p,y;
		FramePos.M.GetRPY(r,p,y);
		lase_pose_(0) = FramePos.p.x();	lase_pose_(1) = FramePos.p.y();	lase_pose_(2) = FramePos.p.z();	//position
		lase_pose_(3) = r;	lase_pose_(4) = p;	lase_pose_(5) = y;	// Roll, pitch, yaw

        /*double qx, qy, qz, qw;
        FramePos.M.GetQuaternion(qx, qy, qz, qw);
        lase_pose_(0) = FramePos.p.x();	lase_pose_(1) = FramePos.p.y();	lase_pose_(2) = FramePos.p.z();	//position
        lase_pose_(3) = qx;	lase_pose_(4) = qy;	lase_pose_(5) = qz;	lase_pose_(6) = qw;	// Quaternion
*/
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
