#include "custom_controller.h"

CustomController::CustomController(DataContainer &dc, RobotData &rd) : dc_(dc), rd_(rd), wbc_(dc.wbc_)
{
	ControlVal_.setZero();
	walking_slider_command = dc.nh.subscribe("/tocabi/dg/walkingslidercommand", 100, &CustomController::WalkingSliderCommandCallback, this);
		
	upperbodymode_sub = dc.nh.subscribe("/tocabi/dg/upperbodymodecommand", 100, &CustomController::UpperbodyModeCallback, this);
	nextswingleg_sub = dc.nh.subscribe("/tocabi/dg/nextswinglegcommand", 100, &CustomController::NextSwinglegCallback, this);

	com_walking_pd_gain_sub = dc.nh.subscribe("/tocabi/dg/compospdgain", 100, &CustomController::ComPosGainCallback, this);
	pelv_ori_pd_gain_sub = dc.nh.subscribe("/tocabi/dg/pelvoripdgain", 100, &CustomController::PelvOriGainCallback, this);
	support_foot_damping_gain_sub = dc.nh.subscribe("/tocabi/dg/supportfootdampinggain", 100, &CustomController::SupportFootDampingGainCallback, this);
	dg_leg_pd_gain_sub = dc.nh.subscribe("/tocabi/dg/legpdgain", 100, &CustomController::LegJointGainCallback, this);
	alpha_x_sub = dc.nh.subscribe("/tocabi/dg/alpha_x", 100, &CustomController::AlphaXCallback, this);
	alpha_y_sub = dc.nh.subscribe("/tocabi/dg/alpha_y", 100, &CustomController::AlphaYCallback, this);
	step_width_sub = dc.nh.subscribe("/tocabi/dg/stepwidthcommand", 100, &CustomController::StepWidthCommandCallback, this);
	
	test1_sub = dc.nh.subscribe("/tocabi/dg/test1command", 100, &CustomController::Test1CommandCallback, this);
	test2_sub = dc.nh.subscribe("/tocabi/dg/test2command", 100, &CustomController::Test2CommandCallback, this);
	
	arm_pd_gain_sub = dc.nh.subscribe("/tocabi/dg/armpdgain", 100, &CustomController::ArmJointGainCallback, this);
	waist_pd_gain_sub = dc.nh.subscribe("/tocabi/dg/waistpdgain", 100, &CustomController::WaistJointGainCallback, this);

	// left_controller_posture_sub = dc.nh.subscribe("/LEFTCONTROLLER", 100, &CustomController::LeftControllerCallback, this);
	// right_controller_posture_sub = dc.nh.subscribe("/RIGHTCONTROLLER", 100, &CustomController::RightControllerCallback, this);
	hmd_posture_sub = dc.nh.subscribe("/HMD", 100, &CustomController::HmdCallback, this);
	lhand_tracker_posture_sub = dc.nh.subscribe("/TRACKER3", 100, &CustomController::LeftHandTrackerCallback, this);
	rhand_tracker_posture_sub = dc.nh.subscribe("/TRACKER5", 100, &CustomController::RightHandTrackerCallback, this);
	lelbow_tracker_posture_sub = dc.nh.subscribe("/TRACKER2", 100, &CustomController::LeftElbowTrackerCallback, this);
	relbow_tracker_posture_sub = dc.nh.subscribe("/TRACKER4", 100, &CustomController::RightElbowTrackerCallback, this);
	chest_tracker_posture_sub = dc.nh.subscribe("/TRACKER1", 100, &CustomController::ChestTrackerCallback, this);
	pelvis_tracker_posture_sub = dc.nh.subscribe("/TRACKER0", 100, &CustomController::PelvisTrackerCallback, this);
	tracker_status_sub = dc.nh.subscribe("/TRACKERSTATUS", 100, &CustomController::TrackerStatusCallback, this);

	vive_tracker_pose_calibration_sub = dc.nh.subscribe("/tocabi/dg/avatar/pose_calibration_flag", 100, &CustomController::PoseCalibrationCallback, this);

	exo_suit_sub = dc.nh.subscribe("/exosuit", 100, &CustomController::ExosuitCallback, this);
	azure_kinect_sub = dc.nh.subscribe("/body_tracking_data", 100, &CustomController::AzureKinectCallback, this);

	bool urdfmode;
	ros::param::get("/tocabi_controller/urdfAnkleRollDamping", urdfmode);
    std::string urdf_path, desc_package_path;

    ros::param::get("/tocabi_controller/urdf_path", desc_package_path);

    if (urdfmode)
    {
        urdf_path = desc_package_path + "/dyros_tocabi_ankleRollDamping.urdf";
    }
    else
    {
        urdf_path = desc_package_path + "/dyros_tocabi.urdf";
    }

	RigidBodyDynamics::Addons::URDFReadFromFile(desc_package_path.c_str(), &model_d_, true, false);

	for(int i = 0; i<FILE_CNT; i++)
    {
        file[i].open(FILE_NAMES[i]); 
    }
	calibration_log_file_[0].open("/home/dg/data/vive_tracker/calibration_log/kaleem/still_pose_.txt");
	calibration_log_file_[1].open("/home/dg/data/vive_tracker/calibration_log/kaleem/t_pose_pose_.txt");
	calibration_log_file_[2].open("/home/dg/data/vive_tracker/calibration_log/kaleem/forward_pose_.txt");
	calibration_log_file_[3].open("/home/dg/data/vive_tracker/calibration_log/kaleem/hmd_pose_init_.txt");

	setGains();
    first_loop_larm_ = true;
	first_loop_rarm_ = true;
	first_loop_upperbody_ = true;
	first_loop_hqpik_ = true;
}

void CustomController::setGains()
{
	//////////Control Gain///////////////////////////
	////sim
	// kp_compos_.setZero(); 		
	// kd_compos_.setZero();	 	

	// kp_compos_(0, 0) = 100;
	// kp_compos_(1, 1) = 100;
	// kp_compos_(2, 2) = 100;

	// kd_compos_(0, 0) = 20;
	// kd_compos_(1, 1) = 20;
	// kd_compos_(2, 2) = 20;

	// kp_pelv_ori_.setZero();
	// kd_pelv_ori_.setZero();
	// kp_pelv_ori_ = 1000*Eigen::Matrix3d::Identity(); 	//angle error gain (sim: 4900)(tune)
	// // kp_pelv_ori_(0, 0) = 1000;
	// // kp_pelv_ori_(1, 1) = 1000;
	// // kp_pelv_ori_(2, 2) = 0;

	// kd_pelv_ori_ = 100*Eigen::Matrix3d::Identity();		//angular velocity gain (sim: 140)(tune)

	// support_foot_damping_gain_.setZero();
	// support_foot_damping_gain_(0, 0) = 0.5;
	// support_foot_damping_gain_(1, 1) = 0.5;
	// support_foot_damping_gain_(2, 2) = 0;

	////real
	kp_compos_.setZero(); 		
	kd_compos_.setZero();	 	

	kp_compos_(0, 0) = 0.2;
	kp_compos_(1, 1) = 0.2;
	kp_compos_(2, 2) = 0.2;

	kd_compos_(0, 0) = 0.00;
	kd_compos_(1, 1) = 0.00;
	kd_compos_(2, 2) = 0.00;

	// kp_pelv_ori_ = 1000*Eigen::Matrix3d::Identity(); 	//angle error gain (sim: 4900)(tune)
	// kd_pelv_ori_ = 50*Eigen::Matrix3d::Identity();		//angular velocity gain (sim: 140)(tune)

	// support_foot_damping_gain_.setZero();
	// support_foot_damping_gain_(0, 0) = 0.01;
	// support_foot_damping_gain_(1, 1) = 0.01;
	// support_foot_damping_gain_(2, 2) = 0;

	////////////////////////////////////////////////////


	/////////Torque Limit///////////
	torque_task_min_(0) 	= -300;
	torque_task_min_(1) 	= -300;
	torque_task_min_(2) 	= -300;
	torque_task_min_(3) 	= -300;
	torque_task_min_(4) 	= -300;
	torque_task_min_(5) 	= -300;

	torque_task_min_(6) 	= -300;
	torque_task_min_(7) 	= -300;
	torque_task_min_(8) 	= -300;
	torque_task_min_(9) 	= -300;
	torque_task_min_(10) 	= -300;
	torque_task_min_(11) 	= -300;

	torque_task_min_(12) 	= -300;
	torque_task_min_(13) 	= -300;
	torque_task_min_(14) 	= -300;

	torque_task_min_(15) 	= -300;
	torque_task_min_(16) 	= -300;
	torque_task_min_(17) 	= -300;
	torque_task_min_(18) 	= -300;
	torque_task_min_(19) 	= -300;
	torque_task_min_(20) 	= -300;
	torque_task_min_(21) 	= -100;
	torque_task_min_(22) 	= -100;

	torque_task_min_(23) 	= -100;
	torque_task_min_(24) 	= -100;

	torque_task_min_(25) 	= -300;
	torque_task_min_(26) 	= -300;
	torque_task_min_(27) 	= -300;
	torque_task_min_(28) 	= -300;
	torque_task_min_(29) 	= -300;
	torque_task_min_(30) 	= -300;
	torque_task_min_(31) 	= -100;
	torque_task_min_(32) 	= -100;

	torque_task_max_(0) 	= 300;
	torque_task_max_(1) 	= 300;
	torque_task_max_(2) 	= 300;
	torque_task_max_(3) 	= 300;
	torque_task_max_(4) 	= 300;
	torque_task_max_(5) 	= 300;

	torque_task_max_(6) 	= 300;
	torque_task_max_(7) 	= 300;
	torque_task_max_(8) 	= 300;
	torque_task_max_(9) 	= 300;
	torque_task_max_(10) 	= 300;
	torque_task_max_(11) 	= 300;

	torque_task_max_(12) 	= 300;
	torque_task_max_(13) 	= 300;
	torque_task_max_(14) 	= 300;

	torque_task_max_(15) 	= 300;
	torque_task_max_(16) 	= 300;
	torque_task_max_(17) 	= 300;
	torque_task_max_(18) 	= 300;
	torque_task_max_(19) 	= 300;
	torque_task_max_(20) 	= 300;
	torque_task_max_(21) 	= 100;
	torque_task_max_(22) 	= 100;

	torque_task_max_(23) 	= 100;
	torque_task_max_(24) 	= 100;

	torque_task_max_(25) 	= 300;
	torque_task_max_(26) 	= 300;
	torque_task_max_(27) 	= 300;
	torque_task_max_(28) 	= 300;
	torque_task_max_(29) 	= 300;
	torque_task_max_(30) 	= 300;
	torque_task_max_(31) 	= 100;
	torque_task_max_(32) 	= 100;
	////////////////////////////////

	//////////Joint PD Gain/////////
	///For Simulation
	// for (int i = 0; i < MODEL_DOF; i++)
	// {
	// 	kp_joint_(i) = 100; 		//(tune)
	// 	kv_joint_(i) = 20;		//(tune)
	// }

	// //Waist Joint Gains
	// for (int i = 0; i < 3; i++) 
	// {
	// 	kp_joint_(12 + i) = 900;
	// 	kv_joint_(12 + i) = 60;
	// }
	// kp_joint_(12) = 2500;
	// kp_joint_(13) = 900;
	// kp_joint_(14) = 900;
	
	// kv_joint_(12) = 100;
	// kv_joint_(13) = 60;
	// kv_joint_(14) = 60;

	// kp_joint_(20) = 64;	//forearm
	// kp_joint_(21) = 64;	//wrist1
	// kp_joint_(22) = 64;	//wrist2
	// kv_joint_(20) = 10;
	// kv_joint_(21) = 10;
	// kv_joint_(22) = 10;

	// kp_joint_(30) = 64;
	// kp_joint_(31) = 64;
	// kp_joint_(32) = 64;
	// kv_joint_(30) = 10;
	// kv_joint_(31) = 10;
	// kv_joint_(32) = 10;

	// kp_joint_(23) = 49;	//head
	// kp_joint_(24) = 49;	
	// kv_joint_(23) = 14;	//head
	// kv_joint_(24) = 14;	

	// //stiff	//(tune)
	// kp_stiff_joint_(0) = 3600; //R hip yaw joint gain
	// kv_stiff_joint_(0) = 120;
	// kp_stiff_joint_(1) = 4900; //L hip roll joint gain
	// kv_stiff_joint_(1) = 140;
	// kp_stiff_joint_(2) = 4900; //L hip pitch joint gain
	// kv_stiff_joint_(2) = 140;

	// kp_stiff_joint_(3) = 1600; //L knee joint gain
	// kv_stiff_joint_(3) = 80;

	// kp_stiff_joint_(4) = 400; //L ankle pitch joint gain
	// kv_stiff_joint_(4) = 40;
	// kp_stiff_joint_(5) = 400; //L ankle roll joint gain
	// kv_stiff_joint_(5) = 40;

	// kp_stiff_joint_(6) = 3600; //R hip yaw joint gain
	// kv_stiff_joint_(6) = 120;
	// kp_stiff_joint_(7) = 4900; //R hip roll joint gain
	// kv_stiff_joint_(7) = 140;
	// kp_stiff_joint_(8) = 4900; //R hip pitch joint gain
	// kv_stiff_joint_(8) = 140;

	// kp_stiff_joint_(9) = 1600; //R knee joint gain
	// kv_stiff_joint_(9) = 80;

	// kp_stiff_joint_(10) = 400; //R ankle pitch joint gain
	// kv_stiff_joint_(10) = 40;
	// kp_stiff_joint_(11) = 400; //R ankle roll joint gain
	// kv_stiff_joint_(11) = 40;

	// //soft	//(tune)
	// kp_soft_joint_(0) = 3600; //L hip yaw joint gain
	// kv_soft_joint_(0) = 120;
	// kp_soft_joint_(1) = 400; //L hip roll joint gain
	// kv_soft_joint_(1) = 40;
	// kp_soft_joint_(2) = 400; //L hip pitch joint gain
	// kv_soft_joint_(2) = 40;

	// kp_soft_joint_(3) = 100; //L knee joint gain
	// kv_soft_joint_(3) = 20;

	// kp_soft_joint_(4) = 25; //L ankle pitch joint gain
	// kv_soft_joint_(4) = 10;
	// kp_soft_joint_(5) = 25; //L ankle roll joint gain
	// kv_soft_joint_(5) = 10;

	// kp_soft_joint_(6) = 3600; //R hip yaw joint gain
	// kv_soft_joint_(6) = 120;
	// kp_soft_joint_(7) = 400; //R hip roll joint gain
	// kv_soft_joint_(7) = 40;
	// kp_soft_joint_(8) = 400; //R hip pitch joint gain
	// kv_soft_joint_(8) = 40;

	// kp_soft_joint_(9) = 100; //R knee joint gain
	// kv_soft_joint_(9) = 20;

	// kp_soft_joint_(10) = 25; //R ankle pitch joint gain
	// kv_soft_joint_(10) = 10;
	// kp_soft_joint_(11) = 25; //R ankle roll joint gain
	// kv_soft_joint_(11) = 10;

	// for (int i = 0; i < 12; i++) //Leg
	// {
	// 	kp_joint_(i) = kp_stiff_joint_(i);
	// 	kv_joint_(i) = kv_stiff_joint_(i);
	// }
	/////////////////

	///For Real Robot
	kp_stiff_joint_(0) 			= 2000;		//right leg
	kp_stiff_joint_(1) 			= 5000;
	kp_stiff_joint_(2) 			= 4000;
	kp_stiff_joint_(3) 			= 3700;
	kp_stiff_joint_(4) 			= 3200;
	kp_stiff_joint_(5) 			= 3200;
	kp_stiff_joint_(6) 			= 2000;		//left leg
	kp_stiff_joint_(7) 			= 5000;
	kp_stiff_joint_(8) 			= 4000;
	kp_stiff_joint_(9) 			= 3700;
	kp_stiff_joint_(10) 		= 3200;
	kp_stiff_joint_(11) 		= 3200;
	kp_stiff_joint_(12) 		= 6000;		//waist
	kp_stiff_joint_(13) 		= 10000;
	kp_stiff_joint_(14) 		= 10000;
	kp_stiff_joint_(15) 		= 400;		//left arm
	kp_stiff_joint_(16) 		= 800;
	kp_stiff_joint_(17) 		= 400;
	kp_stiff_joint_(18) 		= 400;
	kp_stiff_joint_(19) 		= 250;
	kp_stiff_joint_(20) 		= 250;
	kp_stiff_joint_(21) 		= 50;
	kp_stiff_joint_(22) 		= 50;
	kp_stiff_joint_(23) 		= 50;		//head
	kp_stiff_joint_(24) 		= 50;
	kp_stiff_joint_(25) 		= 400;		//right arm
	kp_stiff_joint_(26) 		= 800;
	kp_stiff_joint_(27) 		= 400;
	kp_stiff_joint_(28) 		= 400;
	kp_stiff_joint_(29) 		= 250;
	kp_stiff_joint_(30) 		= 250;
	kp_stiff_joint_(31) 		= 50;
	kp_stiff_joint_(32) 		= 50;

	kv_stiff_joint_(0) 			= 15;		//right leg
	kv_stiff_joint_(1) 			= 50;
	kv_stiff_joint_(2) 			= 20;
	kv_stiff_joint_(3) 			= 25;
	kv_stiff_joint_(4) 			= 24;
	kv_stiff_joint_(5) 			= 24;
	kv_stiff_joint_(6) 			= 15;		//left leg
	kv_stiff_joint_(7) 			= 50;
	kv_stiff_joint_(8) 			= 20;
	kv_stiff_joint_(9) 			= 25;
	kv_stiff_joint_(10) 		= 24;
	kv_stiff_joint_(11) 		= 24;
	kv_stiff_joint_(12) 		= 200;		//waist
	kv_stiff_joint_(13) 		= 100;
	kv_stiff_joint_(14) 		= 100;
	kv_stiff_joint_(15) 		= 10;		//left arm
	kv_stiff_joint_(16) 		= 10;
	kv_stiff_joint_(17) 		= 10;
	kv_stiff_joint_(18) 		= 10;
	kv_stiff_joint_(19) 		= 2.5;
	kv_stiff_joint_(20) 		= 2;
	kv_stiff_joint_(21) 		= 2;
	kv_stiff_joint_(22) 		= 2;
	kv_stiff_joint_(23) 		= 2;		//head
	kv_stiff_joint_(24) 		= 2;		
	kv_stiff_joint_(25) 		= 10;		//right arm
	kv_stiff_joint_(26) 		= 10;
	kv_stiff_joint_(27) 		= 10;
	kv_stiff_joint_(28) 		= 10;
	kv_stiff_joint_(29) 		= 2.5;
	kv_stiff_joint_(30) 		= 2;
	kv_stiff_joint_(31) 		= 2;
	kv_stiff_joint_(32) 		= 2;

	for (int i = 0; i < MODEL_DOF; i++) 
	{
		kp_soft_joint_(i) = kp_stiff_joint_(i)/4;
		kp_soft_joint_(i) = kv_stiff_joint_(i)/2;
	}

	for (int i = 0; i < MODEL_DOF; i++) 
	{
		kp_joint_(i) = kp_stiff_joint_(i);
		kv_joint_(i) = kv_stiff_joint_(i);
	}
	///////////////

	///////////////////////////////
	
	//arm controller
	joint_limit_l_.resize(33);
    joint_limit_h_.resize(33);
    joint_vel_limit_l_.resize(33);
    joint_vel_limit_h_.resize(33);
    
    
	//LEG
	for (int i =0; i<12; i++)
	{
		joint_limit_l_(i) = -180*DEG2RAD;
		joint_limit_h_(i) = 180*DEG2RAD;
	}

	//WAIST
	joint_limit_l_(12) = -30	*DEG2RAD;	
	joint_limit_h_(12) =  30	*DEG2RAD;
	joint_limit_l_(13) = -20	*DEG2RAD;
	joint_limit_h_(13) =  20	*DEG2RAD;
	joint_limit_l_(14) = -30	*DEG2RAD;
	joint_limit_h_(14) =  30	*DEG2RAD;
	//LEFT ARM
	joint_limit_l_(15) = -30	*DEG2RAD;
	joint_limit_h_(15) =  30	*DEG2RAD;
	joint_limit_l_(16) = -170	*DEG2RAD;
	joint_limit_h_(16) =  70	*DEG2RAD;
	joint_limit_l_(17) = -95	*DEG2RAD;
	joint_limit_h_(17) =  95	*DEG2RAD;
	joint_limit_l_(18) = -200	*DEG2RAD;
	joint_limit_h_(18) =  200	*DEG2RAD;
	joint_limit_l_(19) = -150 	*DEG2RAD;
	joint_limit_h_(19) = -12 	*DEG2RAD;
	joint_limit_l_(20) = -180	*DEG2RAD;
	joint_limit_h_(20) =  180	*DEG2RAD;
	joint_limit_l_(21) = -90	*DEG2RAD;
	joint_limit_h_(21) =  90	*DEG2RAD;
	joint_limit_l_(22) = -60	*DEG2RAD;
	joint_limit_h_(22) =  60	*DEG2RAD;
	//HEAD
	joint_limit_l_(23) = -30	*DEG2RAD;
	joint_limit_h_(23) =  30	*DEG2RAD;
	joint_limit_l_(24) = -60	*DEG2RAD;
	joint_limit_h_(24) =  60	*DEG2RAD;
	//RIGHT ARM
	joint_limit_l_(25) = -30	*DEG2RAD;
	joint_limit_h_(25) =  30	*DEG2RAD;
	joint_limit_l_(26) = -70	*DEG2RAD;
	joint_limit_h_(26) =  170	*DEG2RAD;
	joint_limit_l_(27) = -95	*DEG2RAD;
	joint_limit_h_(27) =  95	*DEG2RAD;
	joint_limit_l_(28) = -200	*DEG2RAD;
	joint_limit_h_(28) =  200	*DEG2RAD;
	joint_limit_l_(29) =  12	*DEG2RAD;
	joint_limit_h_(29) =  150	*DEG2RAD;
	joint_limit_l_(30) = -180	*DEG2RAD;
	joint_limit_h_(30) =  180	*DEG2RAD;
	joint_limit_l_(31) = -90	*DEG2RAD;
	joint_limit_h_(31) =  90	*DEG2RAD;
	joint_limit_l_(32) = -60	*DEG2RAD;
	joint_limit_h_(32) =  60	*DEG2RAD;


	//LEG
	for (int i =0; i<12; i++)
	{
		joint_vel_limit_l_(i) = -2*M_PI;
		joint_vel_limit_h_(i) = 2*M_PI;
	}
	//UPPERBODY
	for (int i =12; i<33; i++)
	{
		joint_vel_limit_l_(i) = -M_PI;
		joint_vel_limit_h_(i) = M_PI;
	}
}

Eigen::VectorQd CustomController::getControl()
{
	return ControlVal_;
}

void CustomController::taskCommandToCC(TaskCommand tc_)
{
	tc = tc_;
}

void CustomController::computeSlow()
{
	if (tc.mode == 10)
	{
		//rd_.control_time_; current time
		//rd_.link_[Right_Foot].Jac : current rightfoot jac
		//rd_.q_dot_ : current q velocity

		//rd_.link_[Right_Foot]

		//ControlVal_=

		wbc_.set_contact(rd_, 1, 1);

		int task_number = 6;
		rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
		rd_.f_star.setZero(task_number);

		rd_.J_task = rd_.link_[Pelvis].Jac;

		if (tc.custom_taskgain)
		{
			rd_.link_[Pelvis].pos_p_gain = Vector3d::Ones() * tc.pos_p;
			rd_.link_[Pelvis].pos_d_gain = Vector3d::Ones() * tc.pos_d;
			rd_.link_[Pelvis].rot_p_gain = Vector3d::Ones() * tc.ang_p;
			rd_.link_[Pelvis].rot_d_gain = Vector3d::Ones() * tc.ang_d;
		}

		rd_.link_[Pelvis].x_desired = tc.ratio * rd_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos;
		rd_.link_[Pelvis].x_desired(2) = tc.height + tc.ratio * rd_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos(2);
		rd_.link_[Pelvis].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);

		rd_.f_star = wbc_.getfstar6d(rd_, Pelvis);
		ControlVal_ = wbc_.task_control_torque_QP(rd_, rd_.J_task, rd_.f_star);
	}
	else if (tc.mode == 11)
	{
        const int arm_task_number = 6;
        const int arm_dof = 8;
        ////////// Option 1: CoM Control //////////////////////

        // wbc_.set_contact(rd_, 1, 1);
        // int task_number = 6;
        // rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
        // rd_.f_star.setZero(task_number);

        // rd_.J_task = rd_.link_[COM_id].Jac;
        // rd_.J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac_COM_p;
        // rd_.J_task.block(0, 21, 3, arm_dof).setZero(); // Exclude Left Arm Jacobian
        // rd_.J_task.block(0, 31, 3, arm_dof).setZero(); // Exclude Right Arm Jacobian

        // rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
        // rd_.link_[COM_id].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);

        // rd_.f_star = wbc_.getfstar6d(rd_, COM_id);
        // ControlVal_ = wbc_.task_control_torque_QP2(rd_, rd_.J_task, rd_.f_star);


        ////////// Option 2: Without CoM Control, Only Gravty Compensation //////////////////////
        ControlVal_ = wbc_.gravity_compensation_torque(rd_, false, false);

        ///////// Jacobian based ik arm controller (Daegyu, Donghyeon)/////////////////
        Eigen::Matrix6d RotRF2Pelvis;
        RotRF2Pelvis.setZero();
        RotRF2Pelvis.block(0, 0, 3, 3) = rd_.link_[Right_Foot].Rotm.transpose() * rd_.link_[Pelvis].Rotm;
        RotRF2Pelvis.block(3, 3, 3, 3) = rd_.link_[Right_Foot].Rotm.transpose() * rd_.link_[Pelvis].Rotm;


        Eigen::Matrix<double, 2*arm_task_number, 2*arm_dof> J_task_Arm;
        J_task_Arm.setZero();
        J_task_Arm.block(0, 0, arm_task_number, arm_dof) = RotRF2Pelvis * rd_.link_[Left_Hand].Jac.block(0,21,arm_task_number,arm_dof);
        J_task_Arm.block(arm_task_number, arm_dof, arm_task_number, arm_dof) = RotRF2Pelvis * rd_.link_[Right_Hand].Jac.block(0,31,arm_task_number,arm_dof);
        Eigen::Matrix<double, 2*arm_dof, 2*arm_task_number> J_task_inv;
        J_task_inv = DyrosMath::pinv_SVD(J_task_Arm);

        rd_.link_[Left_Hand].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
        rd_.link_[Left_Hand].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

        rd_.link_[Right_Hand].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
        rd_.link_[Right_Hand].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
        
        Eigen::Vector12d x_dot_desired;
        Eigen::Vector6d error_v;
        Eigen::Vector6d error_w;                
        Eigen::Vector6d k_pos;
        Eigen::Vector6d k_rot;

        for (int i = 0; i<6; i++)
        {
            k_pos(i) = 10;
            k_rot(i) = 4;
        }

        error_v.segment<3>(0) = rd_.link_[Left_Hand].x_traj -  rd_.link_[Left_Hand].xpos;
        error_v.segment<3>(3) = rd_.link_[Right_Hand].x_traj -  rd_.link_[Right_Hand].xpos;

        error_w.segment<3>(0) = -DyrosMath::getPhi(rd_.link_[Left_Hand].Rotm, rd_.link_[Left_Hand].r_traj);
        error_w.segment<3>(3) = -DyrosMath::getPhi(rd_.link_[Right_Hand].Rotm, rd_.link_[Right_Hand].r_traj);

        for(int i = 0; i<3; i++)
        {
            x_dot_desired(i) = rd_.link_[Left_Hand].v_traj(i) + k_pos(i)*error_v(i); // linear velocity
            x_dot_desired(i+3) = rd_.link_[Left_Hand].w_traj(i) + k_rot(i)*error_w(i);
            x_dot_desired(i+6) = rd_.link_[Right_Hand].v_traj(i) + k_pos(i+3)*error_v(i+3); // linear velocity
            x_dot_desired(i+9) = rd_.link_[Right_Hand].w_traj(i) + k_rot(i+3)*error_w(i+3);
        }
        for(int i=0; i<2; i++)
        {
            x_dot_desired.segment<6>(6*i) = RotRF2Pelvis * x_dot_desired.segment<6>(6*i);
        }

        VectorXd q_dot_arm;
        q_dot_arm = J_task_inv*x_dot_desired;
        for (int i=0; i<arm_dof; i++)
        {
            rd_.q_dot_desired_(15+i) = q_dot_arm(i);
            rd_.q_dot_desired_(25+i) = q_dot_arm(i+arm_dof);
        }
        rd_.q_desired_.segment<8>(15) = rd_.q_.segment<8>(15) + rd_.q_dot_desired_.segment<8>(15)*(rd_.control_time_ - rd_.control_time_pre_);
        rd_.q_desired_.segment<8>(25) = rd_.q_.segment<8>(25) + rd_.q_dot_desired_.segment<8>(25)*(rd_.control_time_ - rd_.control_time_pre_);

        Eigen::MatrixXd kp(8,1);
        Eigen::MatrixXd kv(8,1);
        
        for(int i = 0; i<8; i++)
        {
            kp(i) = kp_joint_(15+i);
            kv(i) = kv_joint_(15+i);
        }

        for(int i = 0; i<8; i++)
        {
            ControlVal_(i+15) += kp(i)*(rd_.q_desired_(i+15) - rd_.q_(i+15)) + kv(i)*(rd_.q_dot_desired_(i+15) - rd_.q_dot_(i+15));
            ControlVal_(i+25) += kp(i)*(rd_.q_desired_(i+25) - rd_.q_(i+25)) + kv(i)*(rd_.q_dot_desired_(i+25) - rd_.q_dot_(i+25));
        }           
        rd_.control_time_pre_ = rd_.control_time_;
	}
	else if (tc.mode == 12)
	{
        ControlVal_ = wbc_.gravity_compensation_torque(rd_, false, false);
		
        Eigen::Matrix6d RotRF2Pelvis;
        RotRF2Pelvis.setZero();
        RotRF2Pelvis.block(0, 0, 3, 3) = rd_.link_[Right_Foot].Rotm.transpose() * rd_.link_[Pelvis].Rotm;
        RotRF2Pelvis.block(3, 3, 3, 3) = rd_.link_[Right_Foot].Rotm.transpose() * rd_.link_[Pelvis].Rotm;
        const int arm_dof = 8;
        const int waist_dof = 3;
        const int arm_task_number = 6;
        int variable_size = 2*8;
        int constraint_size = 2*6; // task + joint vel limit
        Eigen::Matrix<double, 2*arm_task_number, 2*arm_dof> J_task_Arm;
        J_task_Arm.setZero();
        J_task_Arm.block(0, 0, arm_task_number, arm_dof) = RotRF2Pelvis * rd_.link_[Left_Hand].Jac.block(0,21,arm_task_number,arm_dof);
        J_task_Arm.block(arm_task_number, arm_dof, arm_task_number, arm_dof) = RotRF2Pelvis * rd_.link_[Right_Hand].Jac.block(0,31,arm_task_number,arm_dof);

		rd_.link_[Left_Hand].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
        rd_.link_[Left_Hand].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

        rd_.link_[Right_Hand].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
        rd_.link_[Right_Hand].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
        Eigen::Vector12d x_dot_desired;
        Eigen::Vector6d error_v;
        Eigen::Vector6d error_w;                
        Eigen::Vector6d k_pos;
        Eigen::Vector6d k_rot;

        for (int i = 0; i<6; i++)
        {
            k_pos(i) = 10;
            k_rot(i) = 10;
        }
        
        if (first_loop_larm_)
        {
			rd_.control_time_pre_ = rd_.control_time_;
            q_virtual_clik_.setZero();
			q_virtual_clik_ = rd_.q_virtual_;
			// q_virtual_clik_.segment<MODEL_DOF>(6) = rd_.q_;
            left_x_traj_pre_ = rd_.link_[Left_Hand].x_traj;
            right_x_traj_pre_ = rd_.link_[Right_Hand].x_traj;
            left_rotm_pre_ = rd_.link_[Left_Hand].r_traj;
            right_rotm_pre_ = rd_.link_[Right_Hand].r_traj;
			QP_qdot.InitializeProblemSize(variable_size, constraint_size);
            first_loop_larm_ = false;
			integral.setZero();
        }

        error_v.segment<3>(0) = rd_.link_[Left_Hand].x_traj -  left_x_traj_pre_;
        error_v.segment<3>(3) = rd_.link_[Right_Hand].x_traj -  right_x_traj_pre_;

        error_w.segment<3>(0) = -DyrosMath::getPhi(left_rotm_pre_, rd_.link_[Left_Hand].r_traj);
        error_w.segment<3>(3) = -DyrosMath::getPhi(right_rotm_pre_, rd_.link_[Right_Hand].r_traj);

        for(int i = 0; i<3; i++)
        {
            x_dot_desired(i) = rd_.link_[Left_Hand].v_traj(i) + k_pos(i)*error_v(i); // linear velocity
            x_dot_desired(i+3) = rd_.link_[Left_Hand].w_traj(i) + k_rot(i)*error_w(i);
            x_dot_desired(i+6) = rd_.link_[Right_Hand].v_traj(i) + k_pos(i+3)*error_v(i+3); // linear velocity
            x_dot_desired(i+9) = rd_.link_[Right_Hand].w_traj(i) + k_rot(i+3)*error_w(i+3);
        }
        for(int i=0; i<2; i++)
        {
            x_dot_desired.segment<6>(6*i) = RotRF2Pelvis * x_dot_desired.segment<6>(6*i);
        }
        //QP initialize!
        VectorXd g, lb, ub;
        MatrixXd Weight_qdot;
        g.setZero(variable_size);
        lb.setZero(variable_size);
        ub.setZero(variable_size);
        Weight_qdot.setZero(variable_size,variable_size);

        for(int i=0; i<variable_size; i++)
        {
			Weight_qdot(i,i) = 1.0;
        }

        double speed_reduce_rate=18; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

        for (int i=0; i< arm_dof; i++)
        {
            lb(i) = max(speed_reduce_rate*(joint_limit_l_(i) - rd_.q_(i+15)), joint_vel_limit_l_(i));
            lb(i+arm_dof) = max(speed_reduce_rate*(joint_limit_l_(i+arm_dof) - rd_.q_(i+25)), joint_vel_limit_l_(i+arm_dof));
            ub(i) = min(speed_reduce_rate*(joint_limit_h_(i) - rd_.q_(i+15)), joint_vel_limit_h_(i));
            ub(i+arm_dof) = min(speed_reduce_rate*(joint_limit_h_(i+arm_dof) - rd_.q_(i+25)), joint_vel_limit_h_(i+arm_dof));
        }


        QP_qdot.EnableEqualityCondition(0.0001);
        QP_qdot.UpdateMinProblem(Weight_qdot, g);
        QP_qdot.UpdateSubjectToAx(J_task_Arm, x_dot_desired, x_dot_desired);
        QP_qdot.UpdateSubjectToX(lb, ub);
        
        VectorXd qpres;
        VectorXd q_dot_arm;
            
        if (QP_qdot.SolveQPoases(200, qpres))
        {
            q_dot_arm = qpres.segment(0, variable_size);
        }
        else
        {
            q_dot_arm.setZero(variable_size);
        }
        
        for (int i=0; i<arm_dof; i++)
        {
            rd_.q_dot_desired_(15+i) = q_dot_arm(i);
            rd_.q_dot_desired_(25+i) = q_dot_arm(i+arm_dof);
        }

		integral.segment<8>(0) = integral.segment<8>(0) + rd_.q_dot_desired_.segment<8>(25)*(rd_.control_time_ - rd_.control_time_pre_);
        q_virtual_clik_.segment<8>(21) = q_virtual_clik_.segment<8>(21) + rd_.q_dot_desired_.segment<8>(15)*(rd_.control_time_ - rd_.control_time_pre_);
        q_virtual_clik_.segment<8>(31) = q_virtual_clik_.segment<8>(31) + rd_.q_dot_desired_.segment<8>(25)*(rd_.control_time_ - rd_.control_time_pre_);

        Eigen::MatrixXd kp(8,1);
        Eigen::MatrixXd kv(8,1);
        
        for(int i = 0; i<8; i++)
        {
            kp(i) = kp_joint_(15+i);		//60
            kv(i) = kv_joint_(15+i);		//6
        }

		if( int(rd_.control_time_*10000)%1000 == 0)
		{
			cout<<"dT" << rd_.control_time_ - rd_.control_time_pre_<<endl;
			cout<<"right_x_traj_pre_" << right_x_traj_pre_<<endl;
			cout<<"rd_.link_[Right_Hand].x_traj" << rd_.link_[Right_Hand].x_traj(0)<<endl;
			cout<<"x_dot_desired(6)" << x_dot_desired(6)<<endl;	
			cout<<"q_virtual_clik_" << q_virtual_clik_.segment(31, 8)<<endl;
			cout<<"integral" << integral.segment(0, 8)<<endl;

			cout<<"\n"<<endl;	
		}
        for(int i = 0; i<8; i++)
        {
            ControlVal_(i+15) += kp(i)*(q_virtual_clik_(i+21) - rd_.q_(i+15)) + kv(i)*(rd_.q_dot_desired_(i+15) - rd_.q_dot_(i+15));
            ControlVal_(i+25) += kp(i)*(q_virtual_clik_(i+31) - rd_.q_(i+25)) + kv(i)*(rd_.q_dot_desired_(i+25) - rd_.q_dot_(i+25));
        }           

		left_x_traj_pre_ =  RigidBodyDynamics::CalcBodyToBaseCoordinates( model_d_, q_virtual_clik_, rd_.link_[Left_Hand].id, Eigen::Vector3d::Zero(), true);
		right_x_traj_pre_ =  RigidBodyDynamics::CalcBodyToBaseCoordinates( model_d_, q_virtual_clik_, rd_.link_[Right_Hand].id, Eigen::Vector3d::Zero(), true);
		left_rotm_pre_ = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_virtual_clik_, rd_.link_[Left_Hand].id, true)).transpose();
        right_rotm_pre_ = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_virtual_clik_, rd_.link_[Right_Hand].id, true)).transpose();

        rd_.control_time_pre_ = rd_.control_time_;
	}
	else if (tc.mode == 13)
	{

	}
	else if (tc.mode == 14) 
	{
		////////////////////////////////////////////////////////////////////////////
		/////////////////// Human-like Biped Walking Controller ////////////////////
		////////////////////////////////////////////////////////////////////////////

		torque_task_.setZero();
		torque_init_.setZero();
		if (tc.task_init == true)
		{
			initWalkingParameter();
			tc.task_init = false;
		}

		//data process//
		getRobotData(wbc_);
		walkingStateManager(); //avatar
		getProcessedRobotData(wbc_);	
		
		foot_swing_trigger_ = false;	//stay avatar

		//motion planing and control//
		motionGenerator();
		getZmpTrajectory();
		getComTrajectory_Preview();
		getCOMTrajectory();
		getSwingFootXYTrajectory(walking_phase_, com_pos_current_, com_vel_current_, com_vel_desired_);

		if( (current_time_ ) >=  program_start_time_ + program_ready_duration_ )
		{
			torque_task_ += comVelocityControlCompute(wbc_); 								//support chain control for COM velocity and pelvis orientation
			// if( int(current_time_*10000)%1000 == 0 )
				// cout<<"torque_task_(12) 1st: " << torque_task_(13) << endl;
			torque_task_ += swingFootControlCompute(wbc_);									//swing foot control
			// if( int(current_time_*10000)%1000 == 0 )
				// cout<<"torque_task_(12) 2nd: " << torque_task_(13) << endl;
			torque_task_ += jointTrajectoryPDControlCompute(wbc_); 							//upper body motion + joint damping control
			// if( int(current_time_*10000)%1000 == 0 )
				// cout<<"torque_task_(12) 3rd: " << torque_task_(13) << endl;
			// torque_task_ += dampingControlCompute(wbc_);									
			// if( int(current_time_*10000)%1000 == 0 )
			// 	cout<<"torque_task_(12) 4th: " << torque_task_(13) << endl;
			// torque_task_ += jointLimit();
			// if( int(current_time_*10000)%1000 == 0 )
			// 	cout<<"torque_task_(12) 5th: " << torque_task_(13) << endl;
		}
		torque_task_.segment(0, 12).setZero();
		torque_task_ += ikBalanceControlCompute(wbc_);

		// //CoM pos & pelv orientation control
		// wbc_.set_contact(rd_, 1, 1);

		// int task_number = 6;	//CoM is not controlled in z direction 
		// rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
		// rd_.f_star.setZero(task_number);

		// rd_.J_task = rd_.link_[COM_id].Jac;
		// //rd_.J_task.block(2, 0, 1, MODEL_DOF_VIRTUAL) = rd_.link_[Pelvis].Jac.block(2, 0, 1, MODEL_DOF_VIRTUAL);
		
		// // rd_.J_task.block(2, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac.block(3, 0, 3, MODEL_DOF_VIRTUAL);

		// rd_.link_[COM_id].x_desired = tc.ratio * rd_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos;
		// rd_.link_[COM_id].x_desired(2) = tc.height;
		// rd_.link_[COM_id].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + 3);

		// rd_.link_[COM_id].rot_desired = Matrix3d::Identity();
		// // rd_.link_[COM_id].rot_desired = pelv_yaw_rot_current_from_global_;
		// rd_.link_[COM_id].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + 3, false);
		
		// rd_.f_star = wbc_.getfstar6d(rd_, COM_id);
		// // rd_.f_star.segment(0, 2) = wbc_.getfstar6d(rd_, COM_id).segment(0, 2);
		// // rd_.f_star.segment(2, 3) = wbc_.getfstar6d(rd_, COM_id).segment(3, 3);
		// //tocabi_.f_star.segment(0, 2) = wbc_.fstar_regulation(tocabi_, tocabi_.f_star.segment(0, 3));
		// //torque_task = wbc_.task_control_torque(tocabi_, tocabi_.J_task, tocabi_.f_star, tc.solver);
		// Eigen::VectorQd torque_com_control;
		// torque_com_control = wbc_.task_control_torque_with_gravity(rd_, rd_.J_task, rd_.f_star);
		// rd_.contact_redistribution_mode = 0;

		// torque_task_.segment(0, 12) = torque_com_control.segment(0, 12);
		// torque_task_(3) = (kp_joint_(3) * (desired_q_(3) - current_q_(3)) + kv_joint_(3) * (desired_q_dot_(3) - current_q_dot_(3)));
		// torque_task_(9) = (kp_joint_(9) * (desired_q_(9) - current_q_(9)) + kv_joint_(9) * (desired_q_dot_(9) - current_q_dot_(9)));
		////////////////////////////////////////

		savePreData();

		////////////////////////////////TORQUE LIMIT//////// //////////////////////
		for (int i = 0; i < MODEL_DOF; i++)
		{
			torque_task_(i) = DyrosMath::minmax_cut(torque_task_(i), -300, 300);
		}
		///////////////////////////////////////////////////////////////////////////
        
		//////////////////////////////FALLING//////////////////////////////
		fallDetection();
		///////////////////////////////////////////////////////////////////

		ControlVal_ = torque_task_;
		if( int(current_time_*10000)%1000 == 0 )
		{
		// 	cout<<"torque_task_(12): "<<torque_task_(12)<<endl;
		// 	cout<<"torque_task_(13): "<<torque_task_(13)<<endl;
		// 	cout<<"torque_task_(14): "<<torque_task_(14)<<endl;
		// 	cout<<"desired_q_(12): "<<desired_q_(12)<<endl;
		// 	cout<<"desired_q_(13): "<<desired_q_(13)<<endl;
		// 	cout<<"desired_q_(14): "<<desired_q_(14)<<endl;
		// 	cout<<"azure_kinect_lshoulder_pos_raw_: "<<azure_kinect_lshoulder_pos_raw_<<endl;
		// 	cout<<"azure_kinect_lelbow_pos_raw_: "<<azure_kinect_lelbow_pos_raw_<<endl;
		// 	cout<<"azure_kinect_lwrist_pos_raw_: "<<azure_kinect_lwrist_pos_raw_<<endl;
		// 	cout<<"azure_kinect_lhand_pos_raw_: "<<azure_kinect_lhand_pos_raw_<<endl;
		// 	cout<<"azure_kinect_rshoulder_pos_raw_: "<<azure_kinect_rshoulder_pos_raw_<<endl;
		// 	cout<<"azure_kinect_relbow_pos_raw_: "<<azure_kinect_relbow_pos_raw_<<endl;
		// 	cout<<"azure_kinect_rwrist_pos_raw_: "<<azure_kinect_rwrist_pos_raw_<<endl;
		// 	cout<<"azure_kinect_rhand_pos_raw_: "<<azure_kinect_rhand_pos_raw_<<endl;
		// 	cout<<"azure_kinect_pelv_pos_raw_: "<<azure_kinect_pelv_pos_raw_<<endl;
		// 	cout<<"azure_kinect_upperbody_pos_raw_: "<<azure_kinect_upperbody_pos_raw_<<endl;
		// 	cout<<"azure_kinect_head_pos_raw_: "<<azure_kinect_head_pos_raw_<<endl;
		// 	cout<<"upperarm L: "<<(azure_kinect_lshoulder_pos_raw_ - azure_kinect_lelbow_pos_raw_).norm()<<endl;
		// 	cout<<"lowerarm L: "<<(azure_kinect_lwrist_pos_raw_ - azure_kinect_lelbow_pos_raw_).norm()<<endl;

		}


		// ControlVal_.setZero();
		// ControlVal_(23) = torque_task_(23);
		// ControlVal_(24) = torque_task_(24);
		// Vector3d temp_sh = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand-5].xpos - pelv_pos_current_);
		// Vector3d temp_elbow = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand-3].xpos - pelv_pos_current_);
		// Vector3d temp_hand = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand-1].xpos - pelv_pos_current_);
		
		// printOutTextFile();
	}
}

void CustomController::computeFast()
{
	if (tc.mode == 10)
	{
	}
	else if (tc.mode == 14)
	{

	}
}

void CustomController::computePlanner()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CustomController::initWalkingParameter()
{
	walking_mode_on_ = true;
	program_ready_duration_ = 0;
	walking_control_transition_duration_ = 0.1;		
	upper_body_mode_ = 1;
	stop_vel_threshold_ = 0.20;
	walking_duration_cmd_ = 0.6;
	walking_duration_start_delay_ = 0.3;
	turning_duration_ = walking_duration_cmd_*0.8;
	walking_phase_ = 0;
	turning_phase_ = 0;
	walking_speed_ = 0.0;
	walking_speed_side_ = 0.0;
	// step_width_ = 0.0915;  						//for linear velocity
	// knee_target_angle_ = 18*DEG2RAD;
	knee_target_angle_ = 0.6;                               //4.5degree
	swingfoot_highest_time_ = 0.35;
	ankle2footcenter_offset_ = 0.02;
	yaw_angular_vel_ = 0; //   rad/s
	swing_foot_height_ = 0.08;
	switching_phase_duration_ = 0.01;
	foot_contact_ = -1;
	foot_contact_pre_ = foot_contact_;
	step_width_ = 0.26;  						//for preview control
	alpha_x_ = 0.01;
	alpha_y_ = 0.18;
	alpha_x_command_ = alpha_x_;
	alpha_y_command_ = alpha_y_;

	start_walking_trigger_ = false;
	first_step_trigger_ = false;
	foot_swing_trigger_ = false;
	stop_walking_trigger_ = true;
	falling_detection_flag_ = false;

	upperbody_mode_recieved_ = true;

	preview_horizon_ = 1.6; //seconds
	preview_hz_ = 200;
	zmp_size_ = preview_horizon_*preview_hz_; 
	ref_zmp_.setZero(zmp_size_, 2);
	zmp_y_offset_ = 0.02;	//outward from com
	
	jac_rhand_.setZero(6, MODEL_DOF_VIRTUAL);
	jac_lhand_.setZero(6, MODEL_DOF_VIRTUAL);
	jac_rfoot_.setZero(6, MODEL_DOF_VIRTUAL);
	jac_lfoot_.setZero(6, MODEL_DOF_VIRTUAL);

	com_pos_error_.setZero();
	com_vel_error_.setZero();
	//set init pre data
	com_pos_desired_pre_ = dc_.link_[COM_id].xpos;
	com_vel_desired_pre_.setZero();
	com_acc_desired_pre_.setZero();
	
	pre_time_ = rd_.control_time_;
	pre_desired_q_ = rd_.q_;
	last_desired_q_ = rd_.q_;

	init_q_ = rd_.q_;
	zero_q_ = init_q_;
	desired_q_ = init_q_;
	desired_q_dot_.setZero();
	desired_q_ddot_.setZero();
	torque_task_.setZero();
	torque_task_pre_.setZero();

	motion_q_pre_ = init_q_;
	motion_q_dot_pre_.setZero();

	contact_force_lfoot_.setZero();
	contact_force_rfoot_.setZero();
	contact_force_lfoot_local_.setZero();
	contact_force_rfoot_local_.setZero();

	zmp_local_lfoot_.setZero();
	zmp_local_rfoot_.setZero();
	zmp_measured_.setZero();
	zmp_dot_measured_.setZero();

	f_star_l_.setZero();
	f_star_r_.setZero();
	f_star_l_pre_.setZero();
	f_star_r_pre_.setZero();

	swingfoot_f_star_l_.setZero();
	swingfoot_f_star_r_.setZero();
	swingfoot_f_star_l_pre_.setZero();
	swingfoot_f_star_r_pre_.setZero();


	f_lfoot_damping_.setZero();
	f_rfoot_damping_.setZero();
	f_lfoot_damping_pre_.setZero();
	f_rfoot_damping_pre_.setZero();

	foot_lift_count_ = 0;
	foot_landing_count_ = 0;

	first_loop_larm_ = true; // QP loop
	first_loop_rarm_ = true; // QP loop

	exo_shoulder_width_ = 0.41;
	robot_shoulder_width_ = 0.6;

	robot_upperarm_max_l_ = 0.3376*1.0;
	robot_lowerarm_max_l_ = 0.31967530867;
	// robot_arm_max_l_ = 0.98*sqrt(robot_upperarm_max_l_*robot_upperarm_max_l_ + robot_lowerarm_max_l_*robot_lowerarm_max_l_ + 2*robot_upperarm_max_l_*robot_lowerarm_max_l_*cos( -joint_limit_h_(19)) );
	robot_arm_max_l_ = (robot_upperarm_max_l_ + robot_lowerarm_max_l_)*0.95;

	hmd_check_pose_calibration_[0] = false;
	hmd_check_pose_calibration_[1] = false;
	hmd_check_pose_calibration_[2] = false;
	hmd_check_pose_calibration_[3] = false;
	hmd_check_pose_calibration_[4] = false;
	still_pose_cali_flag_ = false;
	t_pose_cali_flag_ = false;
	forward_pose_cali_flag_ = false;
	read_cali_log_flag_ = false;

	hmd_larm_max_l_ = 0.45;
	hmd_rarm_max_l_ = 0.45;
	hmd_shoulder_width_ = 0.5;

	hmd_pelv_pose_.setIdentity();
	hmd_lshoulder_pose_.setIdentity();
	hmd_lhand_pose_.setIdentity();
	hmd_rshoulder_pose_.setIdentity();
	hmd_rupperarm_pose_.setIdentity();
	hmd_rhand_pose_.setIdentity();
	hmd_chest_pose_.setIdentity();

	hmd_head_pose_pre_.setIdentity();
	hmd_lshoulder_pose_pre_.setIdentity();
	hmd_lupperarm_pose_pre_.setIdentity();
	hmd_lhand_pose_pre_.setIdentity();
	hmd_rshoulder_pose_pre_.setIdentity();
	hmd_rupperarm_pose_pre_.setIdentity();
	hmd_rhand_pose_pre_.setIdentity();
	hmd_chest_pose_pre_.setIdentity();
	hmd_pelv_pose_pre_.setIdentity();

	hmd_pelv_pose_init_.setIdentity();
	tracker_status_changed_time_ = current_time_;
	hmd_tracker_status_ = true;
	hmd_tracker_status_pre_ = true;
}

void CustomController::getRobotData(WholebodyController &wbc)
{
	current_time_ = rd_.control_time_;

	dt_ = current_time_ - pre_time_;

	current_q_ = rd_.q_;
	current_q_dot_ = rd_.q_dot_;
	current_q_ddot_ = rd_.q_ddot_virtual_.segment(6, MODEL_DOF);
	pelv_pos_current_ = rd_.link_[Pelvis].xpos;
	pelv_vel_current_ = rd_.link_[Pelvis].v;

	pelv_rot_current_ = rd_.link_[Pelvis].Rotm;
	pelv_rpy_current_ = DyrosMath::rot2Euler(pelv_rot_current_); //ZYX multiply
	// pelv_rpy_current_ = (pelv_rot_current_).eulerAngles(2, 1, 0);
	pelv_yaw_rot_current_from_global_ = DyrosMath::rotateWithZ(pelv_rpy_current_(2));
	pelv_rot_current_yaw_aline_ = pelv_yaw_rot_current_from_global_.transpose() * pelv_rot_current_;

	pelv_angvel_current_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Pelvis].w;

	com_pos_current_ = pelv_yaw_rot_current_from_global_.transpose() * (rd_.com_.pos - pelv_pos_current_);
	// com_vel_current_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[COM_id].v;
	com_vel_current_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.com_.vel;
	// com_acc_current_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[COM_id].a;
	// com_acc_current_ = ;

	/////////////////////////Feet Transformation and Velocity/////////////////////
	lfoot_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Foot].xpos - pelv_pos_current_);
	lfoot_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot].Rotm;
	rfoot_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Foot].xpos - pelv_pos_current_);
	rfoot_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot].Rotm;

	lfoot_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot].v;
	lfoot_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot].w;
	rfoot_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot].v;
	rfoot_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot].w;
	///////////////////////////////////////////////////////////////////////////////

	////////////////////////Knee Transformation ////////////////////////////////////
	lknee_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Foot-2].xpos - pelv_pos_current_);
	lknee_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot-2].Rotm;
	rknee_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Foot-2].xpos - pelv_pos_current_);
	rknee_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot-2].Rotm;
	////////////////////////////////////////////////////////////////////////////////

	/////////////////////////Hands Transformation and Velocity/////////////////////
	lhand_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand].xpos - pelv_pos_current_);
	lhand_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand].Rotm;
	rhand_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Hand].xpos - pelv_pos_current_);
	rhand_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand].Rotm;

	lhand_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand].v;
	lhand_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand].w;

	rhand_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand].v;
	rhand_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand].w;
	///////////////////////////////////////////////////////////////////////////////

	////////////////////////Elbow Trnasformation and Velocity///////////////////////
	lelbow_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand-3].xpos - pelv_pos_current_);
	lelbow_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand-3].Rotm;
	relbow_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Hand-3].xpos - pelv_pos_current_);
	relbow_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand-3].Rotm;

	lelbow_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand-3].v;
	lelbow_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand-3].w;
	relbow_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand-3].v;
	relbow_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand-3].w;
	////////////////////////////////////////////////////////////////////////////////
	
	////////////////////////Upper Arm Trnasformation and Velocity////////////////////
	lupperarm_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand-4].xpos - pelv_pos_current_);
	lupperarm_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand-4].Rotm;
	rupperarm_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Hand-4].xpos - pelv_pos_current_);
	rupperarm_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand-4].Rotm;

	lupperarm_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand-4].v;
	lupperarm_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand-4].w;
	rupperarm_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand-4].v;
	rupperarm_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand-4].w;
	////////////////////////////////////////////////////////////////////////////////

	////////////////////////Shoulder Trnasformation and Velocity////////////////////
	lshoulder_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand-5].xpos - pelv_pos_current_);
	lshoulder_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand-5].Rotm;
	rshoulder_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Hand-5].xpos - pelv_pos_current_);
	rshoulder_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand-5].Rotm;

	lshoulder_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand-5].v;
	lshoulder_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand-5].w;
	rshoulder_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand-5].v;
	rshoulder_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand-5].w;
	////////////////////////////////////////////////////////////////////////////////

	////////////////////////Acromion Trnasformation and Velocity////////////////////
	lacromion_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand-6].xpos - pelv_pos_current_);
	lacromion_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand-6].Rotm;
	racromion_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Hand-6].xpos - pelv_pos_current_);
	racromion_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand-6].Rotm;

	lacromion_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand-6].v;
	lacromion_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand-6].w;
	racromion_vel_current_from_global_.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand-6].v;
	racromion_vel_current_from_global_.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand-6].w;
	////////////////////////////////////////////////////////////////////////////////

	///////////////////////Armbase Trasformation and ///////////////////////////////
	larmbase_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand-7].xpos - pelv_pos_current_);
	larmbase_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand-7].Rotm;
	rarmbase_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Hand-7].xpos - pelv_pos_current_);
	rarmbase_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand-7].Rotm;
	////////////////////////////////////////////////////////////////////////////////

	////////////////////////Head & Upperbody Trnasformation ////////////////////////
	head_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Head].xpos - pelv_pos_current_);
	head_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Head].Rotm;

	upperbody_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Upper_Body].xpos - pelv_pos_current_);
	upperbody_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Upper_Body].Rotm;
	////////////////////////////////////////////////////////////////////////////////

	///////////////////////Rotation Euler Angles////////////////////////////////////
	lhand_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(lhand_transform_current_from_global_.linear());
	rhand_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(rhand_transform_current_from_global_.linear());
	lelbow_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(lelbow_transform_current_from_global_.linear());
	relbow_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(relbow_transform_current_from_global_.linear());
	lupperarm_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(lshoulder_transform_current_from_global_.linear());
	rupperarm_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(rupperarm_transform_current_from_global_.linear());
	lshoulder_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(lupperarm_transform_current_from_global_.linear());
	rshoulder_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(rshoulder_transform_current_from_global_.linear());
	lacromion_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(lacromion_transform_current_from_global_.linear());
	racromion_rpy_current_from_global_ = DyrosMath::rot2Euler_tf(racromion_transform_current_from_global_.linear());
	////////////////////////////////////////////////////////////////////////////////



	///////////////////////Variables Updated from desired joint position////////////////////////////
	VectorQVQd q_desired_pre;
	q_desired_pre.setZero();
	q_desired_pre(39) = 1;
	q_desired_pre.segment(6, MODEL_DOF) = pre_desired_q_;

	lhand_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Left_Hand].id, Eigen::Vector3d::Zero(), true);
	lhand_transform_pre_desired_from_.linear() = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Left_Hand].id, true)).transpose();
	
	lelbow_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Left_Hand-3].id, Eigen::Vector3d::Zero(), true);
	lelbow_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Left_Hand-3].id, true).transpose();

	lupperarm_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Left_Hand-4].id, Eigen::Vector3d::Zero(), true);
	lupperarm_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Left_Hand-4].id, true).transpose();

	lshoulder_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Left_Hand-5].id, Eigen::Vector3d::Zero(), true);
	lshoulder_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Left_Hand-5].id, true).transpose();

	lacromion_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Left_Hand-6].id, Eigen::Vector3d::Zero(), true);
	lacromion_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Left_Hand-6].id, true).transpose();

	larmbase_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Left_Hand-7].id, Eigen::Vector3d::Zero(), true);
	larmbase_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Left_Hand-7].id, true).transpose();

	rhand_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Right_Hand].id, Eigen::Vector3d::Zero(), true);
	rhand_transform_pre_desired_from_.linear() = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Right_Hand].id, true)).transpose();
	
	relbow_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Right_Hand-3].id, Eigen::Vector3d::Zero(), true);
	relbow_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Right_Hand-3].id, true).transpose();

	rupperarm_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Right_Hand-4].id, Eigen::Vector3d::Zero(), true);
	rupperarm_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Right_Hand-4].id, true).transpose();

	rshoulder_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Right_Hand-5].id, Eigen::Vector3d::Zero(), true);
	rshoulder_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Right_Hand-5].id, true).transpose();

	racromion_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Right_Hand-6].id, Eigen::Vector3d::Zero(), true);
	racromion_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Right_Hand-6].id, true).transpose();

	rarmbase_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Right_Hand-7].id, Eigen::Vector3d::Zero(), true);
	rarmbase_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Right_Hand-7].id, true).transpose();

	head_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Head].id, Eigen::Vector3d::Zero(), true);
	head_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Head].id, true).transpose();

	upperbody_transform_pre_desired_from_.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Upper_Body].id, Eigen::Vector3d::Zero(), true);
	upperbody_transform_pre_desired_from_.linear() = RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Upper_Body].id, true).transpose();
	///////////////////////////////////////////////////////////////////////////////////////////

	Matrix6d R_R;
	R_R.setZero();
	R_R.block(0, 0, 3, 3) = pelv_yaw_rot_current_from_global_.transpose();
	R_R.block(3, 3, 3, 3) = pelv_yaw_rot_current_from_global_.transpose();
	// R_R.setIdentity();

	jac_com_ = R_R * rd_.link_[COM_id].Jac;
	jac_com_pos_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[COM_id].Jac_COM_p;
	jac_rhand_ = R_R * rd_.link_[Right_Hand].Jac;
	jac_lhand_ = R_R * rd_.link_[Left_Hand].Jac;
	jac_rfoot_ = R_R * rd_.link_[Right_Foot].Jac;
	jac_lfoot_ = R_R * rd_.link_[Left_Foot].Jac;

	lfoot_to_com_jac_from_global_.setZero(6, MODEL_DOF_VIRTUAL);
	rfoot_to_com_jac_from_global_.setZero(6, MODEL_DOF_VIRTUAL);
	Matrix6d adjoint_pelv_to_ankle;
	adjoint_pelv_to_ankle.block(0, 0, 3, 3) = -Eigen::Matrix3d::Identity();
	adjoint_pelv_to_ankle.block(0, 3, 3, 3) = DyrosMath::skm(com_pos_current_ - lfoot_transform_current_from_global_.translation());
	adjoint_pelv_to_ankle.block(3, 3, 3, 3) = -Eigen::Matrix3d::Identity();

	lfoot_to_com_jac_from_global_.block(0, 0, 3, MODEL_DOF_VIRTUAL) = (adjoint_pelv_to_ankle * jac_lfoot_).block(0, 0, 3, MODEL_DOF_VIRTUAL) + jac_com_pos_;
	lfoot_to_com_jac_from_global_.block(3, 0, 3, MODEL_DOF_VIRTUAL) = (adjoint_pelv_to_ankle * jac_lfoot_).block(3, 0, 3, MODEL_DOF_VIRTUAL);

	adjoint_pelv_to_ankle.block(0, 0, 3, 3) = -Eigen::Matrix3d::Identity();
	adjoint_pelv_to_ankle.block(0, 3, 3, 3) = DyrosMath::skm(com_pos_current_ - rfoot_transform_current_from_global_.translation());
	adjoint_pelv_to_ankle.block(3, 3, 3, 3) = -Eigen::Matrix3d::Identity();

	rfoot_to_com_jac_from_global_.block(0, 0, 3, MODEL_DOF_VIRTUAL) = (adjoint_pelv_to_ankle * jac_rfoot_).block(0, 0, 3, MODEL_DOF_VIRTUAL) + jac_com_pos_;
	rfoot_to_com_jac_from_global_.block(3, 0, 3, MODEL_DOF_VIRTUAL) = (adjoint_pelv_to_ankle * jac_rfoot_).block(3, 0, 3, MODEL_DOF_VIRTUAL);

	A_mat_ = rd_.A_;
	// A_inv_mat_ = rd_.A_matrix_inverse;
	// motor_inertia_mat_ = rd_.Motor_inertia;
	// motor_inertia_inv_mat_ = rd_.Motor_inertia_inverse;


	Eigen::Vector3d zmp_local_both_foot;
	// rd_.ZMP_ft = wc_.GetZMPpos(rd_);

	// zmp_local_both_foot = wc.GetZMPpos_fromFT(rd_).segment(0, 2);  //get zmp using f/t sensors on both foot
	
	contact_force_lfoot_ = rd_.ContactForce_FT.segment(0, 6);
	contact_force_rfoot_ = rd_.ContactForce_FT.segment(6, 6);

	Matrix6d adt;
	adt.setZero();
	adt.block(0,0,3,3) = lfoot_transform_current_from_global_.linear();
	adt.block(3,3,3,3) = lfoot_transform_current_from_global_.linear();

	contact_force_lfoot_local_ = adt.inverse()*contact_force_lfoot_;
	
	adt.block(0,0,3,3) = rfoot_transform_current_from_global_.linear();
	adt.block(3,3,3,3) = rfoot_transform_current_from_global_.linear();

	contact_force_rfoot_local_ = adt.inverse()*contact_force_rfoot_;
	
	zmp_local_lfoot_(0) = -contact_force_lfoot_local_(4) / contact_force_lfoot_local_(2);
	zmp_local_lfoot_(1) = contact_force_lfoot_local_(3) / contact_force_lfoot_local_(2);
	zmp_local_rfoot_(0) = -contact_force_rfoot_local_(4) / contact_force_rfoot_local_(2);
	zmp_local_rfoot_(1) = contact_force_rfoot_local_(3) / contact_force_rfoot_local_(2);

	zmp_dot_local_lfoot_ = (zmp_local_lfoot_ - zmp_local_lfoot_pre_) / dt_;
	zmp_dot_local_rfoot_ = (zmp_local_rfoot_ - zmp_local_rfoot_pre_) / dt_;

	zmp_measured_lfoot_ = lfoot_transform_current_from_global_.linear() * zmp_local_lfoot_ + lfoot_transform_current_from_global_.translation(); //from global

	zmp_measured_rfoot_ = rfoot_transform_current_from_global_.linear() * zmp_local_lfoot_ + rfoot_transform_current_from_global_.translation();

	zmp_measured_ = (zmp_measured_lfoot_ * rd_.ContactForce_FT(2) + zmp_measured_rfoot_ * rd_.ContactForce_FT(8)) / (rd_.ContactForce_FT(2) + rd_.ContactForce_FT(8)); //from global
	zmp_dot_measured_ = (zmp_measured_ - zmp_measured_pre_) / dt_;

	l_ft_ = rd_.ContactForce_FT_raw.segment(0, 6);
	r_ft_ = rd_.ContactForce_FT_raw.segment(6, 6);

	first_torque_supplier_ = DyrosMath::cubic(current_time_, program_start_time_ + program_ready_duration_, program_start_time_ + program_ready_duration_ + walking_control_transition_duration_, 0, 1, 0, 0);
}

void CustomController::walkingStateManager()
{
	if (walking_phase_ < 1)
	{
		if (walking_speed_ == 0)
		{
			//first step start
			if (foot_swing_trigger_ == false)
			{
				if (stop_walking_trigger_ == true)
				{
					// if (balanceTrigger(com_pos_current_.segment<2>(0), com_vel_current_.segment<2>(0))) //gonna be fall
					// // if(false)
					// {
					// 	foot_swing_trigger_ = true;
					// 	first_step_trigger_ = true;
					// 	start_walking_trigger_ = false;
					// 	stop_walking_trigger_ = false;
					// 	start_time_ = current_time_;

					// 	if(com_vel_current_(1) >=0)
					// 	{
					// 		foot_contact_ = -1;
					// 	}
					// 	else
					// 	{
					// 		foot_contact_ = 1;
					// 	}
						
					// 	// foot_contact_ = -foot_contact_;  //support foot change
					// 	// std::cout << " ################################ Balancing Control ON! ################################" << std::endl;
					// }
					// else
					// {
						foot_swing_trigger_ = false;
						start_walking_trigger_ = false;
						first_step_trigger_ = false;
						start_time_ = current_time_;
					// }
				}
			}
		}
		else
		{
			stop_walking_trigger_ = false;

			if (foot_swing_trigger_ == false)
			{

				start_walking_trigger_ = true;
				
				if(current_time_ >= start_time_ + walking_duration_start_delay_) // swing foot starts to move
				{
					foot_swing_trigger_ = true;
					first_step_trigger_ = true;
					start_walking_trigger_ = false;
					start_time_ = current_time_;
					// std::cout << " ################################ First Step Triggered! ################################" << std::endl;
				}
			}
			else
			{

				if (foot_contact_ == 1)
				{
					Vector3d phi_swing_ankle;
					phi_swing_ankle = -DyrosMath::getPhi(rfoot_transform_current_from_global_.linear(), swing_foot_rot_trajectory_from_global_);
					
					if (-r_ft_(2) > rd_.com_.mass * GRAVITY / 2)
					{
						foot_landing_count_ += 1;
					}
					else
					{
						foot_landing_count_ = 0;
					}
					

					if ((walking_phase_ > 0.5) && (foot_landing_count_ > 100) )
					{
						
						// std::cout << " ################ Early Step Change Occured! ("<<walking_phase_<<", "<< -r_ft_(2) <<") #########################" << std::endl;
						walking_phase_ = 1;
						// start_time_ = current_time_ - walking_duration_; //make walking_phase_ 1
					}
				}
				else if (foot_contact_ == -1)
				{
					Vector3d phi_swing_ankle;
					phi_swing_ankle = -DyrosMath::getPhi(lfoot_transform_current_from_global_.linear(), swing_foot_rot_trajectory_from_global_);

					if (-l_ft_(2) > rd_.com_.mass * GRAVITY / 2)
					{
						foot_landing_count_ += 1;
					}
					else
					{
						foot_landing_count_ = 0;
					}

					if ((walking_phase_ > 0.5) && (foot_landing_count_ > 100))
					{
						// std::cout << " ################ Early Step Change Occured! ("<<walking_phase_<<", "<< -l_ft_(2) <<") #########################" << std::endl;
						// start_time_ = current_time_ - walking_duration_; //make walking_phase_ 1
						walking_phase_ = 1;
					}
				}
			}
		}
	}

	if (walking_phase_ == 1)
	{
		if (walking_speed_ == 0)
		{
			if (balanceTrigger(com_pos_current_.segment<2>(0), com_vel_current_.segment<2>(0))) //gonna be fall
			{
				foot_swing_trigger_ = true;
				foot_contact_ = -foot_contact_; //support foot change


				if (first_step_trigger_ == true)
				{
					first_step_trigger_ = false;
				}
				// std::cout << " ################################ Balancing Control ON! ################################" << std::endl;
			}
			else
			{
				foot_swing_trigger_ = false;
				stop_walking_trigger_ = true;  //robot stop
				first_step_trigger_ = false;
				start_walking_trigger_ = false;

				foot_contact_ = -foot_contact_;
				stance_start_time_ = current_time_;
				// std::cout << " ################################ Robot Stops Walking! ################################" << std::endl;
			}
		}
		else
		{
			foot_swing_trigger_ = true;
			stop_walking_trigger_ = false;
			first_step_trigger_ = false;
			start_walking_trigger_ = false;

			foot_contact_ = -foot_contact_;
			// std::cout << " ################################ Support Foot Changed! ################################" << std::endl;
		}
		start_time_ = current_time_;
	}

	if(start_walking_trigger_ == true)
	{
		walking_duration_ =  walking_duration_cmd_ + walking_duration_start_delay_;	
	}
	else
	{
		walking_duration_ = walking_duration_cmd_;
		walking_duration_ = DyrosMath::minmax_cut(walking_duration_, 0.2, 1);
	}


	turning_duration_ = walking_duration_*0.8;
	turning_duration_ = DyrosMath::minmax_cut(turning_duration_, 0.2, 0.8);

	walking_phase_ = (current_time_ - start_time_) / walking_duration_;
	walking_phase_ = DyrosMath::minmax_cut(walking_phase_, 0, 1);
	turning_phase_ = (walking_phase_-0.1)/0.8;
	turning_phase_ = DyrosMath::minmax_cut(turning_phase_, 0, 1);
	// walking_duration_ = walking_duration_cmd_  - 1.0*(abs(com_pos_error_(1)) + abs(com_vel_error_(1))*0.3) - 1.0*(abs(com_pos_error_(0)) + abs(com_vel_error_(0))*0.3);

}

bool CustomController::balanceTrigger(Eigen::Vector2d com_pos_2d, Eigen::Vector2d com_vel_2d)
{
	bool trigger = false;
	Vector2d capture_point_2d;
	Vector2d middle_point_of_foot_2d;
	double omega;
	omega = sqrt(GRAVITY / (com_pos_current_(2) - support_foot_transform_current_.translation()(2)));
	capture_point_2d = com_pos_2d + com_vel_2d / omega;
	middle_point_of_foot_2d = middle_of_both_foot_.segment(0, 2);

	
	// if(capture_point_2d.norm() > stop_vel_threshold_)
	// {
	//     trigger = true;
	//     cout<<"balance swing foot control activated"<<endl;
	// }
	if ((capture_point_2d(0) > middle_point_of_foot_2d(0) + 0.10) || (capture_point_2d(0) < middle_point_of_foot_2d(0) - 0.05))
	{
		trigger = true;
		// std::cout << "Catpure point in X axis is over the safety boundary! balance swing foot control activated" << std::endl;
	}

	if ((capture_point_2d(1) > lfoot_transform_current_from_global_.translation()(1) - 0.02 ) || (capture_point_2d(1) < rfoot_transform_current_from_global_.translation()(1) + 0.02))
	{
		trigger = true;
		// std::cout << "Catpure point in Y axis is over the safety boundary! balance swing foot control activated" << std::endl;
	}

	// if( com_vel_2d.norm() > stop_vel_threshold_)
	// {
	//     trigger = true;
	//     cout<<"com vel is over the limit ("<< com_vel_2d.norm()<<")"<<endl;
	// }

	if ( abs(com_vel_2d(0)) > 0.2 || abs(com_vel_2d(1)) > 0.15)
	{
		trigger = true;
		// std::cout << "com vel is over the limit (" << com_vel_2d(0) << "," << com_vel_2d(1) << ")" << std::endl;
	}

	if (abs(lfoot_transform_current_from_global_.translation()(0) - rfoot_transform_current_from_global_.translation()(0)) > 0.03 
	|| abs(lfoot_transform_current_from_global_.translation()(1) - rfoot_transform_current_from_global_.translation()(1)) > 0.25
	|| abs(lfoot_transform_current_from_global_.translation()(1) - rfoot_transform_current_from_global_.translation()(1)) < 0.18)
	{
		trigger = true;
		// std::cout << "Foot is not aligned" << std::endl;
	}

	// if (current_q_(3) > 0.2 || current_q_(9) > 0.2)
	// {
	// 	trigger = true;
	// 	cout << "Knee is bent" << endl;
	// }

	return trigger;
}

int CustomController::checkZMPinWhichFoot(Eigen::Vector2d zmp_measured)
{
	int flag;
	Eigen::Vector2d diff_zmp_lfoot;
	Eigen::Vector2d diff_zmp_rfoot;
	Eigen::Vector2d foot_size;
	double safe_region_ratio = 0.9;

	diff_zmp_lfoot(0) = abs(zmp_measured(0) - lfoot_transform_current_from_global_.translation()(0));
	diff_zmp_lfoot(1) = abs(zmp_measured(1) - lfoot_transform_current_from_global_.translation()(1));

	diff_zmp_rfoot(0) = abs(zmp_measured(0) - rfoot_transform_current_from_global_.translation()(0));
	diff_zmp_rfoot(1) = abs(zmp_measured(1) - rfoot_transform_current_from_global_.translation()(1));

	foot_size(0) = 0.15;
	foot_size(1) = 0.085;
	if ((diff_zmp_lfoot(0) < safe_region_ratio * foot_size(0)) && (diff_zmp_lfoot(1) < safe_region_ratio * foot_size(1)))
	{
		flag = 1; //zmp is in the left foot
	}
	else if ((diff_zmp_rfoot(0) < safe_region_ratio * foot_size(0)) && (diff_zmp_rfoot(1) < safe_region_ratio * foot_size(1)))
	{
		flag = -1; //zmp is in the right foot
	}
	else
	{
		flag = 0;
	}

	return flag;
}

void CustomController::getProcessedRobotData(WholebodyController &wbc)
{
	if (walking_mode_on_) //command on
	{
		stance_start_time_ = current_time_;
		start_time_ = current_time_;
		program_start_time_ = current_time_;

		init_q_ = current_q_;
		last_desired_q_ = current_q_;
		
		com_pos_desired_preview_ = com_pos_current_;
		com_vel_desired_preview_.setZero();
		com_acc_desired_preview_.setZero();

		com_vel_desired_preview_pre_ = com_pos_current_;
		com_vel_desired_preview_pre_.setZero();
		com_vel_desired_preview_pre_.setZero();

		com_pos_desired_ = com_pos_current_;
		com_vel_desired_.setZero();
		com_acc_desired_.setZero();

		com_pos_desired_last_ = com_pos_current_;
		com_vel_desired_last_.setZero();
		com_acc_desired_last_.setZero();

		xs_.setZero();
		ys_.setZero();

		xi_ = com_pos_current_(0);
		yi_ = com_pos_current_(1);

		xd_.setZero();
		yd_.setZero();
		xd_(0) = com_pos_current_(0);
		yd_(0) = com_pos_current_(1);

		xd_b.setZero();
		yd_b.setZero();
		xd_b(0) = com_pos_current_(0);
		yd_b(0) = com_pos_current_(1);
		
		walking_mode_on_ = false;

		pelv_transform_start_from_global_.translation()  = pelv_pos_current_;
		pelv_transform_start_from_global_.linear()  = pelv_rot_current_;
		lfoot_transform_start_from_global_ = lfoot_transform_current_from_global_;
		rfoot_transform_start_from_global_ = rfoot_transform_current_from_global_;

		lfoot_transform_init_from_global_ = lfoot_transform_current_from_global_;
		rfoot_transform_init_from_global_ = rfoot_transform_current_from_global_;

		lhand_transform_init_from_global_ = lhand_transform_current_from_global_;
		rhand_transform_init_from_global_ = rhand_transform_current_from_global_;

		lelbow_transform_init_from_global_ = lelbow_transform_current_from_global_;
		relbow_transform_init_from_global_ = relbow_transform_current_from_global_;

		lupperarm_transform_init_from_global_ = lupperarm_transform_current_from_global_;
		rupperarm_transform_init_from_global_ = rupperarm_transform_current_from_global_;

		lshoulder_transform_init_from_global_ = lshoulder_transform_current_from_global_;
		rshoulder_transform_init_from_global_ = rshoulder_transform_current_from_global_;
		
		lacromion_transform_init_from_global_ = lacromion_transform_current_from_global_;
		racromion_transform_init_from_global_ = racromion_transform_current_from_global_;
			
		larmbase_transform_init_from_global_ = larmbase_transform_current_from_global_;
		rarmbase_transform_init_from_global_ = rarmbase_transform_current_from_global_;

		head_transform_init_from_global_ = head_transform_current_from_global_;
		upperbody_transform_init_from_global_ = upperbody_transform_current_from_global_;

		master_lhand_pose_pre_ = lhand_transform_current_from_global_;
		master_rhand_pose_pre_ = rhand_transform_current_from_global_;

		master_lelbow_pose_pre_ = lelbow_transform_current_from_global_;
		master_relbow_pose_pre_ = relbow_transform_current_from_global_;

		master_lshoulder_pose_pre_ = lacromion_transform_current_from_global_;
		master_rshoulder_pose_pre_ = racromion_transform_current_from_global_;

		master_head_pose_pre_ = head_transform_current_from_global_;

		master_upperbody_pose_pre_ = upperbody_transform_current_from_global_;

		master_relative_lhand_pos_pre_ = lhand_transform_current_from_global_.translation() - rhand_transform_current_from_global_.translation();
		master_relative_rhand_pos_pre_ = rhand_transform_current_from_global_.translation() - lhand_transform_current_from_global_.translation();

		master_lhand_vel_.setZero();
		master_rhand_vel_.setZero();
		master_lelbow_vel_.setZero();
		master_relbow_vel_.setZero();
		master_lshoulder_vel_.setZero();
		master_rshoulder_vel_.setZero();

		master_lhand_rqy_.setZero();
		master_rhand_rqy_.setZero();
		master_lelbow_rqy_.setZero();
		master_relbow_rqy_.setZero();
		master_lshoulder_rqy_.setZero();
		master_rshoulder_rqy_.setZero();
		master_head_rqy_.setZero();
		
		lhand_vel_error_.setZero();
		rhand_vel_error_.setZero();
		lelbow_vel_error_.setZero();
		relbow_vel_error_.setZero();
		lacromion_vel_error_.setZero();
		racromion_vel_error_.setZero();

	}

	bool robot_goes_into_stance_phase = (current_time_ == stance_start_time_);
	bool robot_start_walking = ((start_walking_trigger_ == true) && (current_time_ == start_time_));
	bool robot_start_swing = ((foot_swing_trigger_ == true) && (current_time_ == start_time_));

	if (robot_goes_into_stance_phase || robot_start_walking || robot_start_swing)
	{
		com_pos_init_ = com_pos_current_;
		com_vel_init_ = com_vel_current_;
		com_acc_init_ = com_acc_current_;

		pelv_pos_init_ = pelv_pos_current_;
		pelv_vel_init_ = pelv_vel_current_;
		pelv_rot_init_ = pelv_rot_current_;
		pelv_rpy_init_ = pelv_rpy_current_;
		pelv_rot_init_yaw_aline_ = pelv_rot_current_yaw_aline_;

		lfoot_transform_init_from_global_ = lfoot_transform_current_from_global_;
		rfoot_transform_init_from_global_ = rfoot_transform_current_from_global_;
		if (foot_contact_ == 1) // left support foot
		{
			swing_foot_transform_init_ = rfoot_transform_current_from_global_;
			support_foot_transform_init_ = lfoot_transform_current_from_global_;
			swing_foot_vel_init_ = rfoot_vel_current_from_global_;
		}
		else if (foot_contact_ == -1) //right support foot
		{
			swing_foot_transform_init_ = lfoot_transform_current_from_global_;
			support_foot_transform_init_ = rfoot_transform_current_from_global_;
			swing_foot_vel_init_ = lfoot_vel_current_from_global_;
		}
		init_q_ = current_q_;
		last_desired_q_ = desired_q_;
		foot_lift_count_ = 0;

		com_pos_desired_last_ = com_pos_desired_;
		com_vel_desired_last_ = com_vel_desired_;
		com_acc_desired_last_ = com_acc_desired_;

		swingfoot_f_star_l_pre_.setZero();
		swingfoot_f_star_r_pre_.setZero();
	}

	if (foot_contact_ == 1) // left support foot
	{

		swing_foot_transform_current_ = rfoot_transform_current_from_global_;
		support_foot_transform_current_ = lfoot_transform_current_from_global_;
		swing_foot_vel_current_ = rfoot_vel_current_from_global_;
		support_foot_vel_current_.setZero();
		support_foot_vel_current_.segment(3, 3) = pelv_angvel_current_ - lfoot_to_com_jac_from_global_.block(3, 6, 3, MODEL_DOF)*current_q_dot_;

		com_vel_est1_ = lfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ ;
		com_vel_est2_ = lfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ + DyrosMath::skm(lfoot_transform_current_from_global_.translation())*(support_foot_vel_current_.segment(3, 3));
	}
	else if (foot_contact_ == -1) //right support foot
	{
		swing_foot_transform_current_ = lfoot_transform_current_from_global_;
		support_foot_transform_current_ = rfoot_transform_current_from_global_;
		swing_foot_vel_current_ = lfoot_vel_current_from_global_;
		support_foot_vel_current_.setZero();
		support_foot_vel_current_.segment(3, 3) = pelv_angvel_current_ - rfoot_to_com_jac_from_global_.block(3, 6, 3, MODEL_DOF)*current_q_dot_;

		com_vel_est1_ = rfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ ;
		com_vel_est2_ = rfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ + DyrosMath::skm(rfoot_transform_current_from_global_.translation())*(support_foot_vel_current_.segment(3, 3));
	}
	else if (foot_swing_trigger_ == false)
	{

	}


	zmp_measured_local_ = wbc.GetZMPpos_fromFT(rd_, true);

	middle_of_both_foot_ = (lfoot_transform_current_from_global_.translation() + rfoot_transform_current_from_global_.translation()) / 2;
	zc_ = com_pos_current_(2) - support_foot_transform_current_.translation()(2);

	if(current_time_ == program_start_time_)
	{
		support_foot_transform_pre_ = support_foot_transform_current_;
		swing_foot_transform_pre_ = swing_foot_transform_current_;

		com_pos_desired_preview_pre_ = com_pos_current_;
		com_vel_desired_preview_pre_.setZero();
		com_acc_desired_preview_pre_.setZero();

		//preview gain update
		previewParam_MJ(1/preview_hz_, zmp_size_, zc_, K_act_ , Gi_, Gd_, Gx_, A_, B_, C_, D_, A_bar_, B_bar_);
		last_preview_param_update_time_ = current_time_;
		preview_update_time_ = current_time_;
	}

	swingfoot_force_control_converter_ = DyrosMath::cubic(walking_phase_, 0.8, 0.9, 0, 1, 0, 0);
	// swingfoot_force_control_converter_ = 0;
}

void CustomController::motionGenerator()
{
	motion_q_dot_.setZero();
	motion_q_.setZero();
	pd_control_mask_.setZero();

	///////////////////////LEG/////////////////////////
	//////LEFT LEG///////0 0 0.02 0.15 -0.17 0
	motion_q_(0) = turning_duration_*yaw_angular_vel_;
	motion_q_(1) = 0;
	motion_q_(2) = 0.02;
	// motion_q_(3)   = DyrosMath::cubic(walking_phase_, 0.7, 1, knee_target_angle_, 2*knee_target_angle_, 0, 0); //0.1
	motion_q_(3) = knee_target_angle_;
	motion_q_(4) = -0.12;
	motion_q_(5) = 0;
	pd_control_mask_(0) = 1;
	pd_control_mask_(1) = 0;
	pd_control_mask_(2) = 0;
	pd_control_mask_(3) = 1;
	pd_control_mask_(4) = 1;
	pd_control_mask_(5) = 1;
	//////////////////////
	/////RIFHT LEG////////0 0 0.02 0.15 -0.17 0
	motion_q_(6) = turning_duration_*yaw_angular_vel_;
	motion_q_(7) = 0;
	motion_q_(8) = 0.02;
	// motion_q_(9)   = DyrosMath::cubic(walking_phase_, 0.7, 1, knee_target_angle_, 2*knee_target_angle_, 0, 0); //0.1
	motion_q_(9) = knee_target_angle_;
	motion_q_(10) = -0.12;
	motion_q_(11) = 0;
	pd_control_mask_(6) = 1;
	pd_control_mask_(7) = 0;
	pd_control_mask_(8) = 0;
	pd_control_mask_(9) = 1;
	pd_control_mask_(10) = 1;
	pd_control_mask_(11) = 1;
	//////////////////////

	poseCalibration();

	if(upper_body_mode_ == 1) // init pose 
	{
		if(upperbody_mode_recieved_ == true)
		{
			cout<<"Upperbody Mode is Changed to #1"<<endl;
			upperbody_mode_recieved_ = false;
			upperbody_command_time_ = current_time_;
			upperbody_mode_q_init_ = motion_q_pre_;
		}

		///////////////////////WAIST/////////////////////////
		if(yaw_angular_vel_ == 0)
		{
			motion_q_(12) = 0;
		}
		else
		{
			motion_q_(12) = (1-turning_phase_)*last_desired_q_(12) + turning_phase_*turning_duration_*(-yaw_angular_vel_)*1; //yaw
		}
		motion_q_(12) = 0;
		motion_q_(13) = 0; //pitch
		motion_q_(14) = 0; //roll
		pd_control_mask_(12) = 1;
		pd_control_mask_(13) = 1;
		pd_control_mask_(14) = 1;
		/////////////////////////////////////////////////////

		///////////////////////HEAD/////////////////////////
		if(yaw_angular_vel_ == 0)
		{
			motion_q_(23) = 0;
		}
		else
		{
			motion_q_(23) = (1-turning_phase_)*last_desired_q_(23) + turning_phase_*turning_duration_*(-yaw_angular_vel_)*1.2; //yaw
		}
		// motion_q_(23) = 0; //yaw
		motion_q_(24) = 0; //pitch
		pd_control_mask_(23) = 1;
		pd_control_mask_(24) = 1;
		/////////////////////////////////////////////////////

		///////////////////////ARM/////////////////////////
		//////LEFT ARM///////0.3 0.3 1.5 -1.27 -1 0 -1 0
		motion_q_(15) = 0.3;
		motion_q_(16) = 0.3; 
		motion_q_(17) = 1.5; 
		motion_q_(18) = -1.27;
		motion_q_(19) = -1.0;
		motion_q_(20) = 0.0; 
		motion_q_(21) = -1.0;
		motion_q_(22) = 0.0; 

		pd_control_mask_(15) = 1;
		pd_control_mask_(16) = 1;
		pd_control_mask_(17) = 1;
		pd_control_mask_(18) = 1;
		pd_control_mask_(19) = 1;
		pd_control_mask_(20) = 1;
		pd_control_mask_(21) = 1;
		pd_control_mask_(22) = 1;
		//////////////////////
		/////RIFHT ARM////////-0.3 -0.3 -1.5 1.27 1 0 1 0
		motion_q_(25) = -0.3;
		motion_q_(26) = -0.3;
		motion_q_(27) = -1.5;
		motion_q_(28) = 1.27;
		motion_q_(29) = 1.0 ;
		motion_q_(30) = 0.0 ;
		motion_q_(31) = 1.0 ;
		motion_q_(32) = 0.0 ;

		pd_control_mask_(25) = 1;
		pd_control_mask_(26) = 1;
		pd_control_mask_(27) = 1;
		pd_control_mask_(28) = 1;
		pd_control_mask_(29) = 1;
		pd_control_mask_(30) = 1;
		pd_control_mask_(31) = 1;
		pd_control_mask_(32) = 1;
		/////////////////////////////////////////////////////

		for(int i = 12; i<32; i++)
		{
			motion_q_(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_+4, upperbody_mode_q_init_(i), 0, 0, motion_q_(i), 0, 0)(0);
		}
	}
	else if(upper_body_mode_ == 2)		// Zero pose
	{
		if(upperbody_mode_recieved_ == true)
		{
			cout<<"Upperbody Mode is Changed to #2"<<endl;
			upperbody_mode_recieved_ = false;
			upperbody_command_time_ = current_time_;
			upperbody_mode_q_init_ = motion_q_pre_;
		}
		///////////////////////WAIST/////////////////////////
		if(yaw_angular_vel_ == 0)
		{
			motion_q_(12) = 0;
		}
		else
		{
			motion_q_(12) = (1-turning_phase_)*last_desired_q_(12) + turning_phase_*turning_duration_*(-yaw_angular_vel_)*1; //yaw
		}
		motion_q_(13) = 0; //pitch
		motion_q_(14) = 0; //roll
		pd_control_mask_(12) = 1;
		pd_control_mask_(13) = 1;
		pd_control_mask_(14) = 1;
		/////////////////////////////////////////////////////

		///////////////////////HEAD/////////////////////////
		if(yaw_angular_vel_ == 0)
		{
			motion_q_(23) = 0;
		}
		else
		{
			motion_q_(23) = (1-turning_phase_)*last_desired_q_(23) + turning_phase_*turning_duration_*(-yaw_angular_vel_)*1.2; //yaw
		}
		// motion_q_(23) = 0; //yaw
		motion_q_(24) = 0; //pitch
		pd_control_mask_(23) = 1;
		pd_control_mask_(24) = 1;
		/////////////////////////////////////////////////////

		///////////////////////ARM/////////////////////////
		//////LEFT ARM///////0.3 0.3 1.5 -1.27 -1 0 -1 0
		motion_q_(15) = 0.3;
		// motion_q_(16) = 0.12;
		motion_q_(16) = -1.5*desired_q_(8)-0.5;
		motion_q_(17) = 1.43;
		motion_q_(18) = -0.85; 
		motion_q_(19) = -0.45; //elbow
		motion_q_(20) = 1;
		motion_q_(21) = 0.0;
		motion_q_(22) = 0.0;
		pd_control_mask_(15) = 1;
		pd_control_mask_(16) = 1;
		pd_control_mask_(17) = 1;
		pd_control_mask_(18) = 1;
		pd_control_mask_(19) = 1;
		pd_control_mask_(20) = 1;
		pd_control_mask_(21) = 1;
		pd_control_mask_(22) = 1;
		//////////////////////
		/////RIFHT ARM////////-0.3 -0.3 -1.5 1.27 1 0 1 0
		motion_q_(25) = -0.3;
		// motion_q_(26) = -0.12;
		motion_q_(26) = 1.5*desired_q_(2)+0.5;
		motion_q_(27) = -1.43;
		motion_q_(28) = 0.85;
		motion_q_(29) = 0.45;	//elbow
		motion_q_(30) = -1;
		motion_q_(31) = 0.0;
		motion_q_(32) = 0.0;
		pd_control_mask_(25) = 1;
		pd_control_mask_(26) = 1;
		pd_control_mask_(27) = 1;
		pd_control_mask_(28) = 1;
		pd_control_mask_(29) = 1;
		pd_control_mask_(30) = 1;
		pd_control_mask_(31) = 1;
		pd_control_mask_(32) = 1;
		/////////////////////////////////////////////////////

		for(int i = 12; i<32; i++)
		{
			motion_q_(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_+4, upperbody_mode_q_init_(i), 0, 0, motion_q_(i), 0, 0)(0);
		}
	}
	else if(upper_body_mode_ == 3)		// upperbody qpik
	{
		if(hmd_check_pose_calibration_[3] == false)
		{
			cout<<" WARNING: Calibration is not completed! Upperbody returns to the init pose"<<endl;
			upper_body_mode_ = 1;
			motion_q_ = motion_q_pre_;
		}
		else
		{
			rawMasterPoseProcessing();
			motionRetargeting_HQPIK();
			// motionRetargeting_QPIK_upperbody();
		}
	}
	else if(upper_body_mode_ == 4)		// human motion retargetting: joint pd control with CLIK
	{
		if(hmd_check_pose_calibration_[3] == false)
		{
			cout<<" WARNING: Calibration is not completed! Upperbody returns to the init pose"<<endl;
			upper_body_mode_ = 1;
			motion_q_ = motion_q_pre_;
		}
		else
		{
			rawMasterPoseProcessing();
			// motionRetargeting2();
			motionRetargeting_QPIK_larm();
			motionRetargeting_QPIK_rarm();

			///////////////////////WAIST/////////////////////////
			Vector3d error_w_upperbody = -DyrosMath::getPhi(upperbody_transform_pre_desired_from_.linear(), master_upperbody_pose_.linear());
			Vector3d u_dot_upperbody = master_upperbody_vel_.segment(3, 3) + 100*error_w_upperbody;
			VectorQVQd q_desired_pre;
			q_desired_pre.setZero();
			q_desired_pre(39) = 1;
			q_desired_pre.segment(6, MODEL_DOF) = pre_desired_q_;
			MatrixXd J_temp, J_waist, J_head, J_inv_waist, J_inv_head, I3;
			J_temp.setZero(6, MODEL_DOF_VIRTUAL);
			J_waist.setZero(3, 3);
			I3.setIdentity(3, 3);

			RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Upper_Body].id, Eigen::Vector3d::Zero(), J_temp, false);
			J_waist.block(0, 0, 3, 3) = J_temp.block(0, 18, 3, 3);	//orientation
			J_inv_waist = J_waist.transpose()*(J_waist*J_waist.transpose()+I3*0.000001).inverse();

			for(int i=1; i<3; i++)
			{
				u_dot_upperbody(i) = DyrosMath::minmax_cut(u_dot_upperbody(i), - 1, 1);
			}

			motion_q_dot_.segment(12, 3) =  J_inv_waist*u_dot_upperbody;
			motion_q_.segment(12, 3) = motion_q_pre_.segment(12, 3) + motion_q_dot_.segment(12, 3)*dt_;
			motion_q_(12) = DyrosMath::minmax_cut(motion_q_(12), -30*DEG2RAD, 30*DEG2RAD);
			motion_q_(13) = DyrosMath::minmax_cut(motion_q_(13), -20*DEG2RAD, 20*DEG2RAD);
			motion_q_(14) = DyrosMath::minmax_cut(motion_q_(14), -30*DEG2RAD, 30*DEG2RAD);
			motion_q_dot_.segment(12,3).setZero();
			// motion_q_(12) = (1-turning_phase_)*init_q_(12) + turning_phase_*turning_duration_*(-yaw_angular_vel_)*1; //yaw
			// motion_q_(12) = 0;
			// motion_q_(13) = 0; //pitch
			// motion_q_(14) = 0; //roll
			pd_control_mask_(12) = 1;
			pd_control_mask_(13) = 1;
			pd_control_mask_(14) = 1;
			/////////////////////////////////////////////////////
			
			///////////////////////HEAD/////////////////////////
			Vector3d error_w_head = -DyrosMath::getPhi(head_transform_pre_desired_from_.linear(), master_head_pose_.linear());
			error_w_head = head_transform_pre_desired_from_.linear().transpose()*error_w_head;
			error_w_head(0) = 0;
			error_w_head = head_transform_pre_desired_from_.linear()*error_w_head;

			Vector3d u_dot_head = master_head_vel_.segment(3, 3) + 100*error_w_head;
			J_temp.setZero(6, MODEL_DOF_VIRTUAL);
			J_head.setZero(3, 2);
			I3.setIdentity(3, 3);

			RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Head].id, Eigen::Vector3d::Zero(), J_temp, false);
			J_head.block(0, 0, 3, 2) = J_temp.block(0, 29, 3, 2);	//orientation
			J_inv_head = J_head.transpose()*(J_head*J_head.transpose()+I3*0.000001).inverse();

			for(int i=1; i<3; i++)
			{
				u_dot_head(i) = DyrosMath::minmax_cut(u_dot_head(i), -1, 1);
			}

			motion_q_dot_.segment(23, 2) =  J_inv_head*u_dot_head;
			motion_q_.segment(23, 2) = motion_q_pre_.segment(23, 2) + motion_q_dot_.segment(23, 2)*dt_;
			motion_q_(23) = DyrosMath::minmax_cut(motion_q_(23), -30*DEG2RAD, 30*DEG2RAD);
			motion_q_(24) = DyrosMath::minmax_cut(motion_q_(24), -60*DEG2RAD, 60*DEG2RAD);
			motion_q_dot_.segment(23, 2).setZero();
			if( int(current_time_ * 10000)%1000 == 0)
			{
				// cout<< "head_transform_pre_desired_from_: \n" << head_transform_pre_desired_from_.linear() <<endl;
				// cout<< "master_head_pose_: \n" << master_head_pose_.linear() <<endl;
				// cout<< "master_head_pose_raw_: \n" << master_head_pose_raw_.linear() <<endl;
				// cout<< "error_w_head: " << error_w_head <<endl;
				// cout<< "u_dot_head: " << u_dot_head <<endl;
				// cout<< "J_head: \n" << J_head <<endl;
				// cout<<"motion_q_(23): "<<motion_q_(23)<<endl;
				// cout<<"motion_q_(24): "<<motion_q_(24)<<endl;	
				// cout<<"qpik_larm_time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;	
				// cout<<"qpik_rarm_time: "<< std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() <<endl;	
			}
			
			// motion_q_(23) = (1-turning_phase_)*init_q_(23) + turning_phase_*turning_duration_*(-yaw_angular_vel_)*1.2; //yaw
			// motion_q_(23) = 0; //yaw
			// motion_q_(24) = 0; //pitch
			pd_control_mask_(23) = 1;
			pd_control_mask_(24) = 1;
			/////////////////////////////////////////////////////

		}
	}
	else if(upper_body_mode_ == 5)
	{

	}

	/////////////////FOOT HEIGHT/////////////////////////

	if (foot_swing_trigger_ == true)
	{
		if (walking_phase_ < swingfoot_highest_time_)
		{
			// swing_foot_pos_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.00, 0.6, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0)(0);
			// swing_foot_vel_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.00, 0.6, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0)(1);
			swing_foot_pos_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 1 * switching_phase_duration_, swingfoot_highest_time_, support_foot_transform_current_.translation()(2)+ swing_foot_transform_init_.translation()(2) - support_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0)(0);
			swing_foot_vel_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 1 * switching_phase_duration_, swingfoot_highest_time_, support_foot_transform_current_.translation()(2)+ swing_foot_transform_init_.translation()(2) - support_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0)(1);
			swing_foot_acc_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 1 * switching_phase_duration_, swingfoot_highest_time_, support_foot_transform_current_.translation()(2)+ swing_foot_transform_init_.translation()(2) - support_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0)(2);
		}
		else
		{
			swing_foot_pos_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, swingfoot_highest_time_, 1, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2) - 0.005, 0, 0)(0);
			swing_foot_vel_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, swingfoot_highest_time_, 1, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2) - 0.005, 0, 0)(1);
			swing_foot_acc_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, swingfoot_highest_time_, 1, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2) - 0.005, 0, 0)(2);
		}

	}
}

void CustomController::motionRetargeting()
{

	///////////////////////WAIST/////////////////////////
	if(yaw_angular_vel_ == 0)
	{
		motion_q_(12) = 0;
	}
	else
	{
		motion_q_(12) = (1-turning_phase_)*last_desired_q_(12) + turning_phase_*turning_duration_*(-yaw_angular_vel_)*1.0; //yaw
	}
	motion_q_(13) = 0; //pitch
	motion_q_(14) = 0; //roll
	pd_control_mask_(12) = 1;
	pd_control_mask_(13) = 1;
	pd_control_mask_(14) = 1;
	/////////////////////////////////////////////////////

	///////////////////////HEAD/////////////////////////
	if(yaw_angular_vel_ == 0)
	{
		motion_q_(23) = 0;
	}
	else
	{
		motion_q_(23) = (1-turning_phase_)*last_desired_q_(23) + turning_phase_*turning_duration_*(-yaw_angular_vel_)*1.2; //yaw
	}
	// motion_q_(23) = 0; //yaw
	motion_q_(24) = 0; //pitch
	pd_control_mask_(23) = 1;
	pd_control_mask_(24) = 1;
	/////////////////////////////////////////////////////

	/////////////////////ARM/////////////////////////////////
	Vector7d qdot_d_larm;
	Vector7d qdot_d_rarm;

	Vector6d u_dot_lhand;
	Vector6d u_dot_rhand;
	
	Matrix3d kp_pos_lhand;
	Matrix3d kp_pos_rhand;

	Matrix3d kp_ori_lhand;
	Matrix3d kp_ori_rhand;

	Isometry3d lhand_transform_pre_desired_from;
	Isometry3d rhand_transform_pre_desired_from;

	VectorQVQd q_desired_pre;
	q_desired_pre.setZero();
	q_desired_pre(39) = 1;
	q_desired_pre.segment(6, MODEL_DOF) = motion_q_pre_;

	lhand_transform_pre_desired_from.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Left_Hand].id, Eigen::Vector3d::Zero(), true);
	rhand_transform_pre_desired_from.translation() = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_d_, q_desired_pre, rd_.link_[Right_Hand].id, Eigen::Vector3d::Zero(), true);

	lhand_transform_pre_desired_from.linear() = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Left_Hand].id, false)).transpose();
	rhand_transform_pre_desired_from.linear() = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_desired_pre, rd_.link_[Right_Hand].id, false)).transpose();

	u_dot_lhand.setZero();
	u_dot_rhand.setZero();
	kp_pos_lhand.setZero();
	kp_pos_rhand.setZero();
	kp_ori_lhand.setZero();
	kp_ori_rhand.setZero();

	kp_pos_lhand(0, 0) = 500;
	kp_pos_lhand(1, 1) = 500;
	kp_pos_lhand(2, 2) = 500;

	kp_pos_rhand(0, 0) = 500;
	kp_pos_rhand(1, 1) = 500;
	kp_pos_rhand(2, 2) = 500;

	kp_ori_lhand(0, 0) = 200;
	kp_ori_lhand(1, 1) = 200;
	kp_ori_lhand(2, 2) = 200;

	kp_ori_rhand(0, 0) = 200;
	kp_ori_rhand(1, 1) = 200;
	kp_ori_rhand(2, 2) = 200;

	
	// master_rhand_pose_.translation() 	= 	rd_.link_[Right_Hand].x_traj;
	// master_rhand_pose_.linear() 		= 	rd_.link_[Right_Hand].r_traj;
	// master_rhand_vel_.segment(0, 3) 	= 	rd_.link_[Right_Hand].v_traj;
	// master_rhand_vel_.segment(3, 3) 	= 	rd_.link_[Right_Hand].w_traj;

	// master_lhand_pose_.translation() 	= 	rd_.link_[Left_Hand].x_traj;
	// master_lhand_pose_.linear() 		= 	rd_.link_[Left_Hand].r_traj;
	// master_lhand_vel_.segment(0, 3) 	= 	rd_.link_[Left_Hand].v_traj;
	// master_lhand_vel_.segment(3, 3) 	= 	rd_.link_[Left_Hand].w_traj;
	
	// master_lhand_pose_.translation() 		= lhand_transform_init_from_global_.translation();
	// master_lhand_pose_.translation()(0) 	+= 	0.1*sin(current_time_*2*M_PI/1);
	// master_lhand_pose_.linear().Identity();
	// master_lhand_pose_.linear() = DyrosMath::rotateWithX(90*DEG2RAD)*DyrosMath::rotateWithZ(-90*DEG2RAD)*master_lhand_pose_.linear();
	// master_lhand_vel_.setZero();
	// master_lhand_vel_(0) 	= 	0.1*cos(current_time_*2*M_PI/1)*2*M_PI/1;

	// master_rhand_pose_.translation() 		= rhand_transform_init_from_global_.translation();
	// master_rhand_pose_.translation()(0) 	+= 	0.1*sin(current_time_*2*M_PI/1);
	// master_rhand_pose_.linear().Identity();
	// master_rhand_pose_.linear() = DyrosMath::rotateWithX(-90*DEG2RAD)*DyrosMath::rotateWithZ(90*DEG2RAD)*master_rhand_pose_.linear();
	// master_rhand_vel_.setZero();
	// master_rhand_vel_(0) 	= 	0.1*cos(current_time_*2*M_PI/1)*2*M_PI/1;
	
	u_dot_lhand.segment(0, 3) = master_lhand_vel_.segment(0, 3) + kp_pos_lhand*(master_lhand_pose_.translation()-lhand_transform_pre_desired_from.translation());
	u_dot_rhand.segment(0, 3) = master_rhand_vel_.segment(0, 3) + kp_pos_rhand*(master_rhand_pose_.translation()-rhand_transform_pre_desired_from.translation());

	for(int i =0; i<3; i++)
	{
		u_dot_lhand(i) = DyrosMath::minmax_cut(u_dot_lhand(i), -0.5, 0.5);
		u_dot_rhand(i) = DyrosMath::minmax_cut(u_dot_rhand(i), -0.5, 0.5);
	}

	Vector3d lhand_phi = -DyrosMath::getPhi(lhand_transform_pre_desired_from.linear(), master_lhand_pose_.linear());
	Vector3d rhand_phi = -DyrosMath::getPhi(rhand_transform_pre_desired_from.linear(), master_rhand_pose_.linear());
	u_dot_lhand.segment(3, 3) = master_lhand_vel_.segment(3, 3) + kp_ori_lhand*lhand_phi;
	u_dot_rhand.segment(3, 3) = master_rhand_vel_.segment(3, 3) + kp_ori_rhand*rhand_phi;


	qdot_d_larm = DyrosMath::pinv_SVD(jac_lhand_.block(0, 22, 6, 7), 0.0001)*u_dot_lhand;
	qdot_d_rarm = DyrosMath::pinv_SVD(jac_rhand_.block(0, 32, 6, 7), 0.0001)*u_dot_rhand;
	
	motion_q_dot_.segment(16, 7) = qdot_d_larm;
	motion_q_dot_.segment(26, 7) = qdot_d_rarm;

	for(int i =0; i<7; i++)
	{
		motion_q_(16 + i) += motion_q_pre_(16+i) + qdot_d_larm(i)*dt_;
		motion_q_(26 + i) += motion_q_pre_(26+i) + qdot_d_rarm(i)*dt_;
		pd_control_mask_(16+i) = 1;
		pd_control_mask_(26+i) = 1;
	}
	
	motion_q_(15) = 0.3;
	motion_q_(25) = -0.3;
	pd_control_mask_(15) = 1;
	pd_control_mask_(25) = 1;

	motion_q_(19) = DyrosMath::minmax_cut(motion_q_(19), joint_limit_l_(4), joint_limit_h_(4));		//elbow
	motion_q_(29) = DyrosMath::minmax_cut(motion_q_(29), joint_limit_l_(12), joint_limit_h_(12));
	///////////////////////////////////////////////////////////////////////////////////
}

void CustomController::motionRetargeting2()
{
	///////////////////////WAIST/////////////////////////
	motion_q_(12) = (1-turning_phase_)*init_q_(12) + turning_phase_*turning_duration_*(-yaw_angular_vel_)*1; //yaw
	motion_q_(13) = 0; //pitch
	motion_q_(14) = 0; //roll
	pd_control_mask_(12) = 1;
	pd_control_mask_(13) = 1;
	pd_control_mask_(14) = 1;
	/////////////////////////////////////////////////////

	///////////////////////HEAD/////////////////////////
	motion_q_(23) = (1-turning_phase_)*init_q_(23) + turning_phase_*turning_duration_*(-yaw_angular_vel_)*1.2; //yaw
	// motion_q_(23) = 0; //yaw
	motion_q_(24) = 0; //pitch
	pd_control_mask_(23) = 1;
	pd_control_mask_(24) = 1;
	/////////////////////////////////////////////////////

	/////////////////////ARM/////////////////////////////////
	for(int i =0; i<8; i++)
	{
		pd_control_mask_(15+i) = 0;
		pd_control_mask_(25+i) = 0;
	}

	motion_q_(15) = 0.3;
	motion_q_(25) = -0.3;
	pd_control_mask_(15) = 1;
	pd_control_mask_(25) = 1;
	
	motion_q_(18) = -1.2;
	motion_q_(28) = 1.2;
	pd_control_mask_(18) = 1;
	pd_control_mask_(28) = 1;

	motion_q_(19) = -0.3;
	motion_q_(29) = 0.3;
	pd_control_mask_(19) = 1;
	pd_control_mask_(29) = 1;

	motion_q_(20) = 0.0;
	motion_q_(21) = 0.0;
	motion_q_(22) = 0.0;
	pd_control_mask_(20) = 1;
	pd_control_mask_(21) = 1;
	pd_control_mask_(22) = 1;


	motion_q_(30) = 0.0;
	motion_q_(31) = 0.0;
	motion_q_(32) = 0.0;
	pd_control_mask_(30) = 1;
	pd_control_mask_(31) = 1;
	pd_control_mask_(32) = 1;
	VectorQd torque_d_larm;
	VectorQd torque_d_rarm;

	Vector6d f_d_lhand;
	Vector6d f_d_rhand;
	
	Matrix3d kp_pos_lhand;
	Matrix3d kp_pos_rhand;
	Matrix3d kd_pos_lhand;
	Matrix3d kd_pos_rhand;

	Matrix3d kp_ori_lhand;
	Matrix3d kp_ori_rhand;
	Matrix3d kd_ori_lhand;
	Matrix3d kd_ori_rhand;

	Isometry3d lhand_transform_pre_desired_from;
	Isometry3d rhand_transform_pre_desired_from;


	f_d_lhand.setZero();
	f_d_rhand.setZero();
	kp_ori_lhand.setZero();
	kp_ori_rhand.setZero();

	kp_pos_lhand(0, 0) = 2500;
	kp_pos_lhand(1, 1) = 2500;
	kp_pos_lhand(2, 2) = 0;

	kp_pos_rhand(0, 0) = 2500;
	kp_pos_rhand(1, 1) = 2500;
	kp_pos_rhand(2, 2) = 0;

	kd_pos_lhand(0, 0) = 100;
	kd_pos_lhand(1, 1) = 100;
	kd_pos_lhand(2, 2) = 0;

	kd_pos_rhand(0, 0) = 100;
	kd_pos_rhand(1, 1) = 100;
	kd_pos_rhand(2, 2) = 0;

	///////////////swing arm//////////////////
	master_lhand_pose_ = lhand_transform_init_from_global_;
	master_lhand_vel_.setZero();

	master_lhand_pose_.translation()(0) = 1.5*rknee_transform_current_from_global_.translation()(0)-0.15;
	master_lhand_pose_.translation()(1) = lhand_transform_init_from_global_.translation()(1);
	master_lhand_pose_.translation()(2) = lhand_transform_init_from_global_.translation()(2); 

	master_lhand_pose_.translation()(0) = DyrosMath::minmax_cut(master_lhand_pose_.translation()(0), -0.1, 0.6);

	master_lhand_pose_.linear() = Eigen::Matrix3d::Identity();

	master_rhand_pose_ = rhand_transform_init_from_global_;
	master_rhand_vel_.setZero();

	master_rhand_pose_.translation()(0) = 1.5*lknee_transform_current_from_global_.translation()(0)-0.15; 
	master_rhand_pose_.translation()(1) = rhand_transform_init_from_global_.translation()(1);
	master_rhand_pose_.translation()(2) = rhand_transform_init_from_global_.translation()(2);

	master_rhand_pose_.translation()(0) = DyrosMath::minmax_cut(master_rhand_pose_.translation()(0), -0.1, 0.6);

	master_rhand_pose_.linear() = Eigen::Matrix3d::Identity();
	
	/////////////////test////////////////////////
	// master_lhand_pose_.translation() 		= 	lhand_transform_init_from_global_.translation();
	// master_lhand_pose_.translation()(0) 	= 	0.1*sin(current_time_*2*M_PI/3);
	// master_lhand_pose_.linear().Identity();
	// master_lhand_vel_.setZero();
	// master_lhand_vel_(0) 	= 	0.1*cos(current_time_*2*M_PI/3)*2*M_PI/3;

	// master_rhand_pose_.translation() 		= 	rhand_transform_init_from_global_.translation();
	// master_rhand_pose_.translation()(0) 	= 	0.1*sin(current_time_*2*M_PI/3);
	// master_rhand_pose_.linear().Identity();
	// master_rhand_vel_.setZero();
	// master_rhand_vel_(0) 	= 	0.1*cos(current_time_*2*M_PI/3)*2*M_PI/3;

	// hand position
	f_d_lhand.segment(0, 3) = kp_pos_lhand*(master_lhand_pose_.translation()-lhand_transform_current_from_global_.translation()) + kd_pos_lhand*(master_lhand_vel_.segment(0,3) - lhand_vel_current_from_global_.segment(0,3));
	f_d_rhand.segment(0, 3) = kp_pos_rhand*(master_rhand_pose_.translation()-rhand_transform_current_from_global_.translation()) + kd_pos_rhand*(master_rhand_vel_.segment(0,3) - rhand_vel_current_from_global_.segment(0,3));

	f_d_lhand(2) -= 0;
	f_d_rhand(2) -= 0;
	// hand orientation
	Vector3d phi_lhand;
	Vector3d phi_rhand;
	phi_lhand = -DyrosMath::getPhi(lhand_transform_current_from_global_.linear(), master_lhand_pose_.linear());
	phi_rhand = -DyrosMath::getPhi(rhand_transform_current_from_global_.linear(), master_rhand_pose_.linear());

	double kpa_hand = 000; //angle error gain
	double kva_hand = 00;	//angular velocity gain

	f_d_lhand.segment(3, 3) = kpa_hand * phi_lhand + kva_hand * (master_lhand_vel_.segment(3,3) - lhand_vel_current_from_global_.segment(3,3));
	f_d_rhand.segment(3, 3) = kpa_hand * phi_rhand + kva_hand * (master_rhand_vel_.segment(3,3) - rhand_vel_current_from_global_.segment(3,3));
	
	torque_d_larm = (jac_lhand_.transpose()*f_d_lhand).segment(6, MODEL_DOF);
	torque_d_rarm = (jac_rhand_.transpose()*f_d_rhand).segment(6, MODEL_DOF);
	
	torque_d_larm.segment(12, 3).setZero(); // penalize pelvi torques
	torque_d_rarm.segment(12, 3).setZero();
	
	torque_task_ += (torque_d_larm + torque_d_rarm);
	// for(int i =0; i<8; i++)
	// {
	// 	torque_task_(15 + i) += torque_d_larm(i);
	// 	torque_task_(25 + i) += torque_d_rarm(i);
	// }
	///////////////////////////////////////////////////////////////////////////////////
}

void CustomController::motionRetargeting_QPIK_larm()
{
	
	// std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	/////////////////////////////////ARM///////////////////////////////////////////
	const int variable_size = 8;
	const int constraint_size1 = 8;	//[lb <=	x	<= 	ub] form constraints
	const int constraint_size2 = 6;	//[lb <=	Ax 	<=	ub] from constraints
	const int control_size_hand = 6;
	const int control_size_upperarm = 3; 
	const int control_size_shoulder = 3; 

	if(first_loop_larm_)
    {	
		QP_qdot_larm.InitializeProblemSize(variable_size, constraint_size2);

		first_loop_larm_ = false;
	}
	
	double w1 = 2500;		//hand tracking
	double w2 = 0.002;			//joint acc
	double w3 = 1;			//task space vel
	double w4 = 30;			//joint vel
	double w5 = 50;			//upperarm oriention tracking
	double w6 = 1;			//shoulder oriention tracking
	
	MatrixXd J_l_arm, J_l_upperarm, J_l_shoulder, J_temp1, J_temp2, J_temp3;
	J_l_arm.setZero(control_size_hand, variable_size);
	J_l_upperarm.setZero(control_size_upperarm, variable_size);
	J_l_shoulder.setZero(control_size_shoulder, variable_size);

	VectorXd u_dot_lhand, u_dot_lupperarm, u_dot_lshoulder; 
	u_dot_lhand.setZero(control_size_hand);
	u_dot_lupperarm.setZero(control_size_upperarm);
	u_dot_lshoulder.setZero(control_size_shoulder);

	MatrixXd H, H1, H2, H3, H4, H5, H6, A;
	VectorXd g, g1, g2, g5, g6, ub, lb, ubA, lbA;

	MatrixXd I8;
	I8.setIdentity(variable_size,variable_size);

    H.setZero(variable_size,variable_size);
	H1.setZero(variable_size,variable_size);
	H2.setZero(variable_size,variable_size);
	H3.setZero(variable_size,variable_size);
	H4.setZero(variable_size,variable_size);
	H5.setZero(variable_size,variable_size);
	H6.setZero(variable_size,variable_size);

	A.setZero(constraint_size2,variable_size);

	g.setZero(variable_size);
	g1.setZero(variable_size);
	g2.setZero(variable_size);
	g5.setZero(variable_size);
	g6.setZero(variable_size);

	ub.setZero(constraint_size1);
	lb.setZero(constraint_size1);

	ubA.setZero(constraint_size2);
	lbA.setZero(constraint_size2);
	
	VectorXd qpres;

	
	VectorQVQd q_desired_pre;
	q_desired_pre.setZero();
	q_desired_pre(39) = 1;
	q_desired_pre.segment(6, MODEL_DOF) = pre_desired_q_;


	Vector3d error_v_lhand = master_lhand_pose_.translation() - lhand_transform_pre_desired_from_.translation();
	Vector3d error_w_lhand = -DyrosMath::getPhi(lhand_transform_pre_desired_from_.linear(), master_lhand_pose_.linear());
	
	Vector3d error_w_lupperarm = -DyrosMath::getPhi(lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_.linear());
	error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear().transpose()*error_w_lupperarm;
	error_w_lupperarm(0) = 0;
	error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear()*error_w_lupperarm;

	Vector3d error_w_lshoulder = -DyrosMath::getPhi(lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_.linear());
	error_w_lshoulder = lacromion_transform_pre_desired_from_.linear().transpose()*error_w_lshoulder;
	error_w_lshoulder(0) = 0;
	error_w_lshoulder = lacromion_transform_pre_desired_from_.linear()*error_w_lshoulder;

	Vector3d error_v_lhand_rel = master_relative_lhand_pos_ + rhand_transform_pre_desired_from_.translation() - lhand_transform_pre_desired_from_.translation();


	double hand_relative_p_gain = DyrosMath::cubic( master_relative_lhand_pos_.norm(), 0.4, robot_shoulder_width_, 1, 0, 0, 0);

	for(int i = 0; i<3; i++)
	{
		u_dot_lhand(i) = master_lhand_vel_(i) + 200*error_v_lhand(i);	
		u_dot_lhand(i+3) = master_lhand_vel_(i+3) + 100*error_w_lhand(i);

		u_dot_lupperarm(i) = master_lelbow_vel_(i+3) + 100*error_w_lupperarm(i);
		u_dot_lshoulder(i) = master_lshoulder_vel_(i+3) + 100*error_w_lshoulder(i);
	}

	Vector3d zero3;
	zero3.setZero();
	J_temp1.setZero(6, MODEL_DOF_VIRTUAL);
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Left_Hand].id, zero3, J_temp1, false);
	J_l_arm.block(0, 0, 3, 8) = J_temp1.block(3, 21, 3, 8);	//position
	J_l_arm.block(3, 0, 3, 8) = J_temp1.block(0, 21, 3, 8);	//orientation
	// J_l_arm.block(0, 0, 6, 1).setZero(); //penalize 1st joint for hand control

	J_temp2.setZero(6, MODEL_DOF_VIRTUAL);
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Left_Hand-4].id, zero3, J_temp2, false);
	J_l_upperarm.block(0, 0, 3, 8) = J_temp2.block(0, 21, 3, 8);	//orientation
	// J_l_upperarm.block(0, 0, 3, 1).setZero(); //penalize 1st joint for hand control

	J_temp3.setZero(6, MODEL_DOF_VIRTUAL);
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Left_Hand-6].id, zero3, J_temp3, false);
	J_l_shoulder.block(0, 0, 3, 8) = J_temp3.block(0, 21, 3, 8);	//orientation

	//// null space projection of hand and elbow////
	MatrixXd J1, J2, J3, N1, N2_aug, J2_aug, J2_aug_pinv, J1_pinv, J2_pinv, J3_pinv, J2N1, J2N1_pinv, J3N2_aug, J3N2_aug_pinv, I3, I6, I9;
	MatrixXd q1_dot_star, q2_dot_star;
	J1.setZero(control_size_hand, variable_size);
	J2.setZero(control_size_upperarm, variable_size);
	J3.setZero(control_size_shoulder, variable_size);

	N1.setZero(variable_size, variable_size);
	N2_aug.setZero(variable_size, variable_size);
	J2_aug.setZero(control_size_hand+control_size_upperarm, variable_size);
	J2_aug_pinv.setZero(variable_size, control_size_hand+control_size_upperarm);
	J2N1.setZero(control_size_upperarm, variable_size);
	J2N1_pinv.setZero(variable_size, control_size_upperarm);
	J3N2_aug.setZero(control_size_shoulder, variable_size);
	J3N2_aug_pinv.setZero(variable_size, control_size_shoulder);
	q1_dot_star.setZero(variable_size, 1);
	q2_dot_star.setZero(variable_size, 1);
	I3.setIdentity(3, 3);
	I6.setIdentity(6, 6);
	I9.setIdentity(9, 9);
	
	J1 = J_l_arm;
	J2 = J_l_upperarm;
	J3 = J_l_shoulder;
	J1_pinv = J1.transpose()*(J1*J1.transpose()+I6*0.0001).inverse();
	N1 = I8 - (J1_pinv)*J1;
	// q1_dot_star = J1_pinv*u_dot_lhand;

	J2_pinv = J2.transpose()*(J2*J2.transpose()+I3*0.0001).inverse();
	J2_aug.block(0, 0, control_size_hand, variable_size) = J1;
	J2_aug.block(control_size_hand, 0, control_size_upperarm, variable_size) = J2;

	// std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();	
	J2_aug_pinv = J2_aug.transpose()*(J2_aug*J2_aug.transpose()+I9*0.0001).inverse();
	// std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();	
	N2_aug = I8 - (J2_aug_pinv)*J2_aug;

	J2N1 = J2*N1;
	J2N1_pinv = J2N1.transpose()*(J2N1*J2N1.transpose()+I3*0.0001).inverse();
	// q2_dot_star = q1_dot_star + J2N1_pinv*(u_dot_lupperarm - J2*q1_dot_star);

	J3N2_aug = J3*N2_aug;

	H1 = J1.transpose()*J1;
	H2 = I8*(1/dt_)*(1/dt_);
	H3 = J1.transpose()*J1;
	H4 = I8;
	H5 = (J2N1).transpose()*(J2N1);
	H6 = (J3N2_aug).transpose()*(J3N2_aug);

	g1 = -J1.transpose()*u_dot_lhand;
	g2 = -motion_q_dot_pre_.segment(15, 8)*(1/dt_)*(1/dt_);
	g5 = -(J2N1).transpose()*(u_dot_lupperarm - J2*motion_q_dot_pre_.segment(15, variable_size));
	g6 = -(J3N2_aug).transpose()*( u_dot_lshoulder - J3*motion_q_dot_pre_.segment(15, variable_size));
	////////////////////////////////////////////////////////////

	H = w1*H1 + w2*H2 + w3*H3 + w4*H4 + w5*H5 + w6*H6;
	g = w1*g1 + w2*g2 + w5*g5 + w6*g6;

 	if( int(current_time_*1e4)%int(1e3) == 0)
	{
		// cout<<"g1: \n"<<g1<<endl;
		// cout<<"g2: \n"<<g2<<endl;
		// cout<<"g5: \n"<<g5<<endl;

		// cout<<"u_dot_lhand: \n"<<u_dot_lhand<<endl;
		// cout<<"u_dot_lelbow: \n"<<u_dot_lelbow<<endl;	

		// cout<<"lelbow_transform_pre_desired_from: \n"<<lelbow_transform_pre_desired_from.translation()<<endl;

		// // cout<<"N1: \n"<<N1<<endl;
		// cout<<"J_l_arm: \n"<<J_l_arm<<endl;
	}

	double speed_reduce_rate= 20; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

	for (int i=0; i< constraint_size1; i++)
	{
		lb(i) = max(speed_reduce_rate*(joint_limit_l_(i+15) - current_q_(i+15)), joint_vel_limit_l_(i+15));
		ub(i) = min(speed_reduce_rate*(joint_limit_h_(i+15) - current_q_(i+15)), joint_vel_limit_h_(i+15));
		// lb(i) = joint_vel_limit_l_(i+1);
		// ub(i) = joint_vel_limit_h_(i+1);
	}

	A = J_l_arm;

	for(int i = 0; i<constraint_size2; i++)
	{
		for(int j=0 ;j<variable_size; j++)
		{
			if( abs(A(i, j)) < 1e-3)
			{	
				if(A(i,j) > 0)
				{
					A(i, j) = 1e-3;
				}
				else
				{
					A(i, j) = -1e-3;
				}
			}
		}
	}

	for (int i=0; i< 3; i++) //position velocity limit
	{
		lbA(i) = -1;
		ubA(i) = 1;
	}

	for (int i=3; i< 6; i++)	//angular velocity limit
	{
		lbA(i) = -3;
		ubA(i) = 3;
	}

	QP_qdot_larm.EnableEqualityCondition(0.0000001);
	QP_qdot_larm.UpdateMinProblem(H, g);
	QP_qdot_larm.UpdateSubjectToAx(A, lbA, ubA);
	QP_qdot_larm.UpdateSubjectToX(lb, ub);

	VectorXd q_dot_larm;
	// std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();	
	// std::chrono::steady_clock::time_point t5;	

	if(QP_qdot_larm.SolveQPoases(100, qpres))
	{
		q_dot_larm = qpres.segment(0, variable_size);
		// t5 = std::chrono::steady_clock::now();
	}
	else
	{
		q_dot_larm.setZero(variable_size);
	}

	for(int i =0; i<variable_size; i++)
	{
		motion_q_dot_(15+i) = q_dot_larm(i);
		motion_q_(15 + i) = motion_q_pre_(15+i) + motion_q_dot_(15+i)*dt_;
		pd_control_mask_(15+i) = 1;
	}


	if(int(current_time_*1e4)%int(1e3) == 0)
	{
		// cout<<"J1: \n"<<J_l_arm<<endl;
		// cout<<"N1: \n"<<N1<<endl;
		// cout<<"N2: \n"<<N_2<<endl;
		// cout<<"J2N1: \n"<<J2N1<<endl;
		// cout<<"J2N1_pinv: \n"<<J2N1_pinv<<endl;
		// cout<<"J3N1N2: \n"<<J3N1N2<<endl;
		// // cout<<"J3N1N2_pinv: \n"<<J3N1N2_pinv<<endl;
		// cout<<"left 1st task error: \n"<< J_l_arm*q_dot_larm - u_dot_lhand<<endl;
		// cout<<"left 2nd task error: \n"<< J_l_upperarm*q_dot_larm - u_dot_lupperarm<<endl;
		// cout<<"left 3rd task error: \n"<< J_l_shoulder*q_dot_larm - u_dot_lshoulder<<endl;
		// cout<<"qdot2: \n"<<(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
		// cout<<"N1 X qdot2: \n"<<N1*(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
		// cout<<"J2 X N1 X qdot2: \n"<<J_l_upperarm*N1*(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
		// cout<<"J1 X N1 X qdot2: \n"<<J_l_arm*N1*(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;

		// cout<<"t2 - t1: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;
		// cout<<"t3 - t2: "<< std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() <<endl;
		// cout<<"t4 - t3: "<< std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() <<endl;
		// cout<<"t5 - t4: "<< std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() <<endl;
		// cout<<"t5 - t1: "<< std::chrono::duration_cast<std::chrono::microseconds>(t5 - t1).count() <<endl;
		// cout<<"q1_dot_star: "<< q1_dot_star <<endl;
		// cout<<"q2_dot_star: "<< q2_dot_star <<endl;
	}

	lhand_vel_error_ = J_l_arm*q_dot_larm - u_dot_lhand;
	lelbow_vel_error_ = J_l_upperarm*q_dot_larm - u_dot_lupperarm;
	lacromion_vel_error_ = J_l_shoulder*q_dot_larm - u_dot_lshoulder;

	// motion_q_(15) = 0.3;
	// pd_control_mask_(15) = 1;

	// motion_q_(19) = DyrosMath::minmax_cut(motion_q_(19), joint_limit_l_(4), joint_limit_h_(4));		//elbow
	// motion_q_(29) = DyrosMath::minmax_cut(motion_q_(29), joint_limit_l_(12), joint_limit_h_(12));
	/////////////////////////////////////////////////////////////////////////////////////////////
}

void CustomController::motionRetargeting_QPIK_rarm()
{
	/////////////////////////////////ARM///////////////////////////////////////////
	const int variable_size = 8;
	const int constraint_size1 = 8;	//[lb <=	x	<= 	ub] form constraints
	const int constraint_size2 = 6;	//[lb <=	Ax 	<=	ub] from constraints
	const int control_size_hand = 6;
	const int control_size_upperarm = 3;
	const int control_size_shoulder = 3; 

	if(first_loop_rarm_)
    {	
		QP_qdot_rarm.InitializeProblemSize(variable_size, constraint_size2);

		first_loop_rarm_ = false;
	}
	
	double w1 = 2500;		//hand tracking
	double w2 = 0.002;		//joint acc
	double w3 = 1;			//task space vel
	double w4 = 30;			//joint vel
	double w5 = 50;		//elbow position tracking
	double w6 = 1;			//shoulder oriention tracking

	MatrixXd J_r_arm, J_r_upperarm, J_r_shoulder, J_temp1, J_temp2, J_temp3;
	J_r_arm.setZero(control_size_hand, variable_size);
	J_r_upperarm.setZero(control_size_upperarm, variable_size);
	J_r_shoulder.setZero(control_size_shoulder, variable_size);

	VectorXd u_dot_rhand, u_dot_rupperarm, u_dot_rshoulder; 
	u_dot_rhand.setZero(control_size_hand);
	u_dot_rupperarm.setZero(control_size_upperarm);
	u_dot_rshoulder.setZero(control_size_shoulder);

	MatrixXd H, H1, H2, H3, H4, H5, H6, A;
	VectorXd g, g1, g2, g5, g6, ub, lb, ubA, lbA;

	MatrixXd I8;
	I8.setIdentity(variable_size,variable_size);

    H.setZero(variable_size,variable_size);
	H1.setZero(variable_size,variable_size);
	H2.setZero(variable_size,variable_size);
	H3.setZero(variable_size,variable_size);
	H4.setZero(variable_size,variable_size);
	H5.setZero(variable_size,variable_size);
	H6.setZero(variable_size,variable_size);

	A.setZero(constraint_size2,variable_size);

	g.setZero(variable_size);
	g1.setZero(variable_size);
	g2.setZero(variable_size);
	g5.setZero(variable_size);
	g6.setZero(variable_size);

	ub.setZero(constraint_size1);
	lb.setZero(constraint_size1);

	ubA.setZero(constraint_size2);
	lbA.setZero(constraint_size2);
	
	VectorXd qpres;

	
	VectorQVQd q_desired_pre;
	q_desired_pre.setZero();
	q_desired_pre(39) = 1;
	q_desired_pre.segment(6, MODEL_DOF) = pre_desired_q_;

	
	Vector3d error_v_rhand = master_rhand_pose_.translation() - rhand_transform_pre_desired_from_.translation();
	Vector3d error_w_rhand = -DyrosMath::getPhi(rhand_transform_pre_desired_from_.linear(), master_rhand_pose_.linear());
	

	Vector3d error_w_rupperarm = -DyrosMath::getPhi(rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_.linear());
	error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear().transpose()*error_w_rupperarm;
	error_w_rupperarm(0) = 0;
	error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear()*error_w_rupperarm;

	Vector3d error_w_rshoulder = -DyrosMath::getPhi(racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_.linear());
	error_w_rshoulder = racromion_transform_pre_desired_from_.linear().transpose()*error_w_rshoulder;
	error_w_rshoulder(0) = 0;
	error_w_rshoulder = racromion_transform_pre_desired_from_.linear()*error_w_rshoulder;


	double hand_relative_p_gain = DyrosMath::cubic( master_relative_rhand_pos_.norm(), 0.4, robot_shoulder_width_, 1, 0, 0, 0);

	for(int i = 0; i<3; i++)
	{
		u_dot_rhand(i) = master_rhand_vel_(i) + 200*error_v_rhand(i);	
		u_dot_rhand(i+3) = master_rhand_vel_(i+3) + 100*error_w_rhand(i);

		u_dot_rupperarm(i) = master_relbow_vel_(i+3) + 100*error_w_rupperarm(i);
		u_dot_rshoulder(i) = master_rshoulder_vel_(i+3) + 100*error_w_rshoulder(i);
	}

	Vector3d zero3;
	zero3.setZero();
	J_temp1.setZero(6, MODEL_DOF_VIRTUAL);
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Right_Hand].id, zero3, J_temp1, false);
	J_r_arm.block(0, 0, 3, 8) = J_temp1.block(3, 31, 3, 8);	//position
	J_r_arm.block(3, 0, 3, 8) = J_temp1.block(0, 31, 3, 8);	//orientation
	// J_r_arm.block(0, 0, 6, 1).setZero(); //penalize 1st joint for hand control

	J_temp2.setZero(6, MODEL_DOF_VIRTUAL);
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Right_Hand-4].id, zero3, J_temp2, false);
	J_r_upperarm.block(0, 0, 3, 8) = J_temp2.block(0, 31, 3, 8);	//orientation
	// J_r_upperarm.block(0, 0, 3, 1).setZero(); //penalize 1st joint for hand control

	J_temp3.setZero(6, MODEL_DOF_VIRTUAL);
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Right_Hand-6].id, zero3, J_temp3, false);
	J_r_shoulder.block(0, 0, 3, 8) = J_temp3.block(0, 31, 3, 8);	//orientation


	//// null space projection of hand and elbow////
	MatrixXd J1, J2, J3, N1, N2_aug, J2_aug, J2_aug_pinv, J1_pinv, J2_pinv, J3_pinv, J2N1, J2N1_pinv, J3N2_aug, J3N2_aug_pinv, I3, I6, I9;
	MatrixXd q1_dot_star, q2_dot_star;
	J1.setZero(control_size_hand, variable_size);
	J2.setZero(control_size_upperarm, variable_size);
	J3.setZero(control_size_shoulder, variable_size);

	N1.setZero(variable_size, variable_size);
	N2_aug.setZero(variable_size, variable_size);
	J2_aug.setZero(control_size_hand+control_size_upperarm, variable_size);
	J2_aug_pinv.setZero(variable_size, control_size_hand+control_size_upperarm);
	J2N1.setZero(control_size_upperarm, variable_size);
	J2N1_pinv.setZero(variable_size, control_size_upperarm);
	J3N2_aug.setZero(control_size_shoulder, variable_size);
	J3N2_aug_pinv.setZero(variable_size, control_size_shoulder);
	q1_dot_star.setZero(variable_size, 1);
	q2_dot_star.setZero(variable_size, 1);
	I3.setIdentity(3, 3);
	I6.setIdentity(6, 6);
	I9.setIdentity(9, 9);

	J1 = J_r_arm;
	J2 = J_r_upperarm;
	J3 = J_r_shoulder;
	J1_pinv = J1.transpose()*(J1*J1.transpose()+I6*0.0001).inverse();
	N1 = I8 - (J1_pinv)*J1;
	// q1_dot_star = J1_pinv*u_dot_rhand;

	J2_pinv = J2.transpose()*(J2*J2.transpose()+I3*0.0001).inverse();
	J2_aug.block(0, 0, control_size_hand, variable_size) = J1;
	J2_aug.block(control_size_hand, 0, control_size_upperarm, variable_size) = J2;

	J2_aug_pinv = J2_aug.transpose()*(J2_aug*J2_aug.transpose()+I9*0.0001).inverse();

	N2_aug = I8 - (J2_aug_pinv)*J2_aug;

	J2N1 = J2*N1;
	J2N1_pinv = J2N1.transpose()*(J2N1*J2N1.transpose()+I3*0.0001).inverse();
	// q2_dot_star = q1_dot_star + J2N1_pinv*(u_dot_rupperarm - J2*q1_dot_star);
	
	J3N2_aug = J3*N2_aug;

	H1 = J1.transpose()*J1;
	H2 = I8*(1/dt_)*(1/dt_);
	H3 = J1.transpose()*J1;
	H4 = I8;
	H5 = (J2N1).transpose()*(J2N1);
	H6 = (J3N2_aug).transpose()*(J3N2_aug);

	g1 = -J1.transpose()*u_dot_rhand;
	g2 = -motion_q_dot_pre_.segment(25, 8)*(1/dt_)*(1/dt_);
	g5 = -(J2N1).transpose()*(u_dot_rupperarm - J2*motion_q_dot_pre_.segment(25, variable_size));
	g6 = -(J3N2_aug).transpose()*(u_dot_rshoulder - J3*motion_q_dot_pre_.segment(25, variable_size));
	////////////////////////////////////////////////////////////                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       

	H = w1*H1 + w2*H2 + w3*H3 + w4*H4 + w5*H5 + w6*H6;
	g = w1*g1 + w2*g2 + w5*g5 + w6*g6;



	if( int(current_time_*1e4)%int(1e3) == 0)
	{
		// cout<<"g1: \n"<<g1<<endl;
		// cout<<"g2: \n"<<g2<<endl;
		// cout<<"g5: \n"<<g5<<endl;

		// cout<<"u_dot_rhand: \n"<<u_dot_rhand<<endl;
		// cout<<"u_dot_rupperarm: \n"<<u_dot_rupperarm<<endl;	

		// cout<<"racromion_transform_pre_desired_from: \n"<<racromion_transform_pre_desired_from_.translation()<<endl;
		// cout<<"rarmbase_transform_pre_desired_from: \n"<<rarmbase_transform_pre_desired_from_.translation()<<endl;

		// cout<<"N1: \n"<<N1<<endl;
		// cout<<"J_r_arm: \n"<<J_r_arm<<endl;
	}

	double speed_reduce_rate= 20; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

	for (int i=0; i< constraint_size1; i++)
	{
		lb(i) = max(speed_reduce_rate*(joint_limit_l_(i+25) - current_q_(i+25)), joint_vel_limit_l_(i+25));
		ub(i) = min(speed_reduce_rate*(joint_limit_h_(i+25) - current_q_(i+25)), joint_vel_limit_h_(i+25));
	}

	A = J_r_arm;

	for(int i = 0; i<constraint_size2; i++)
	{
		for(int j=0 ;j<variable_size; j++)
		{
			if( abs(A(i, j)) < 1e-3)
			{	
				if(A(i,j) > 0)
				{
					A(i, j) = 1e-3;
				}
				else
				{
					A(i, j) = -1e-3;
				}
			}
		}
	}
	
	for (int i=0; i< 3; i++) //position velocity limit
	{
		lbA(i) = -1;
		ubA(i) = 1;
	}

	for (int i=3; i< 6; i++)	//angular velocity limit
	{
		lbA(i) = -3;
		ubA(i) = 3;
	}

	QP_qdot_rarm.EnableEqualityCondition(0.0000001);
	QP_qdot_rarm.UpdateMinProblem(H, g);
	QP_qdot_rarm.UpdateSubjectToAx(A, lbA, ubA);
	QP_qdot_rarm.UpdateSubjectToX(lb, ub);

	VectorXd q_dot_rarm;

	if(QP_qdot_rarm.SolveQPoases(100, qpres))
	{
		q_dot_rarm = qpres.segment(0, variable_size);
	}
	else
	{
		q_dot_rarm.setZero(variable_size);
	}

	for(int i =0; i<variable_size; i++)
	{
		motion_q_dot_(25+i) = q_dot_rarm(i);
		motion_q_(25 + i) = motion_q_pre_(25+i) + motion_q_dot_(25+i)*dt_;
		pd_control_mask_(25+i) = 1;
	}
	
	if(int(current_time_*1e4)%int(1e3) == 0)
	{
		// cout<<"J1: \n"<<J_l_arm<<endl;
		// cout<<"N1: \n"<<N1<<endl;
		// cout<<"N2: \n"<<N_2<<endl;
		// cout<<"J2N1: \n"<<J2N1<<endl;
		// cout<<"J2N1_pinv: \n"<<J2N1_pinv<<endl;
		// cout<<"J3N1N2: \n"<<J3N1N2<<endl;
		// cout<<"J3N1N2_pinv: \n"<<J3N1N2_pinv<<endl;
		// cout<<"right 1st task error: \n"<< J_r_arm*q_dot_rarm - u_dot_rhand<<endl;
		// cout<<"right 2nd task error: \n"<< J_r_upperarm*q_dot_rarm - u_dot_rupperarm<<endl;
		// cout<<"right 3rd task error: \n"<< J_r_shoulder*q_dot_rarm - u_dot_rshoulder<<endl;
		// cout<<"qdot2: \n"<<(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
		// cout<<"N1 X qdot2: \n"<<N1*(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
		// cout<<"J2 X N1 X qdot2: \n"<<J_l_upperarm*N1*(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
		// cout<<"J1 X N1 X qdot2: \n"<<J_l_arm*N1*(J_l_upperarm.completeOrthogonalDecomposition().pseudoInverse())*u_dot_lupperarm<<endl;
	}

	rhand_vel_error_ = J_r_arm*q_dot_rarm - u_dot_rhand;
	relbow_vel_error_ = J_r_upperarm*q_dot_rarm - u_dot_rupperarm;
	racromion_vel_error_ = J_r_shoulder*q_dot_rarm - u_dot_rshoulder;
	
	// motion_q_(25) = -0.3;
	// pd_control_mask_(25) = 1;

	// motion_q_(19) = DyrosMath::minmax_cut(motion_q_(19), joint_limit_l_(4), joint_limit_h_(4));		//elbow
	// motion_q_(29) = DyrosMath::minmax_cut(motion_q_(29), joint_limit_l_(12), joint_limit_h_(12));
	/////////////////////////////////////////////////////////////////////////////////////////////
}

void CustomController::motionRetargeting_QPIK_upperbody()
{
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

	const int variable_size = 21;
	const int constraint_size1 = 21;	//[lb <=	x	<= 	ub] form constraints
	const int constraint_size2 = 12;	//[lb <=	Ax 	<=	ub] from constraints
	const int control_size_hand = 12;		//1
	const int control_size_upperbody = 3;	//1
	const int control_size_head = 2;		//2
	const int control_size_upperarm = 4; 	//3
	const int control_size_shoulder = 4;	//4


	if(first_loop_rarm_)
    {	
		QP_qdot_upperbody.InitializeProblemSize(variable_size, constraint_size2);

		first_loop_rarm_ = false;
	}

	double w1 = 5000;		//upperbody tracking
	double w2 = 000;			//hand & head
	double w3 = 00;			//upperarm
	double w4 = 0;			//shoulder
	double w5 = 50;			//kinematic energy
	double w6 = 0.000;		//acceleration

	MatrixXd J1, J2, J3, J4, J_temp;
	J1.setZero(control_size_upperbody, variable_size);
	J2.setZero(control_size_hand + control_size_head, variable_size);
	J3.setZero(control_size_upperarm, variable_size);
	J4.setZero(control_size_shoulder, variable_size);
	
	VectorXd u_dot_1, u_dot_2, u_dot_3, u_dot_4; 
	u_dot_1.setZero(control_size_upperbody);						//upperbody(3)
	u_dot_2.setZero(control_size_hand + control_size_head);			//lhand(6); rhand(6); head(2)
	u_dot_3.setZero(control_size_upperarm);							//lupperarm(2); rupperarm(2);
	u_dot_4.setZero(control_size_shoulder);							//lshoulder(2); rshoulder(2);

	MatrixXd H, H1, H2, H3, H4, H5, H6, A;
	VectorXd g, g1, g2, g3, g4, g5, g6, ub, lb, ubA, lbA;

	H.setZero(variable_size,variable_size);
	H1.setZero(variable_size,variable_size);
	H2.setZero(variable_size,variable_size);
	H3.setZero(variable_size,variable_size);
	H4.setZero(variable_size,variable_size);
	H5.setZero(variable_size,variable_size);
	H6.setZero(variable_size,variable_size);

	A.setZero(constraint_size2,variable_size);

	g.setZero(variable_size);
	g1.setZero(variable_size);
	g2.setZero(variable_size);
	g3.setZero(variable_size);
	g4.setZero(variable_size);
	g5.setZero(variable_size);
	g6.setZero(variable_size);

	ub.setZero(constraint_size1);
	lb.setZero(constraint_size1);

	ubA.setZero(constraint_size2);
	lbA.setZero(constraint_size2);
	
	VectorXd qpres;

	VectorQVQd q_desired_pre;
	q_desired_pre.setZero();
	q_desired_pre(39) = 1;
	q_desired_pre.segment(6, MODEL_DOF) = pre_desired_q_;

	//Upperbody
	Vector3d error_w_upperbody = -DyrosMath::getPhi(upperbody_transform_pre_desired_from_.linear(), master_upperbody_pose_.linear());

	//Hand error
	Vector3d error_v_lhand = master_lhand_pose_.translation() - lhand_transform_pre_desired_from_.translation();
	Vector3d error_w_lhand = -DyrosMath::getPhi(lhand_transform_pre_desired_from_.linear(), master_lhand_pose_.linear());
	Vector3d error_v_rhand = master_rhand_pose_.translation() - rhand_transform_pre_desired_from_.translation();
	Vector3d error_w_rhand = -DyrosMath::getPhi(rhand_transform_pre_desired_from_.linear(), master_rhand_pose_.linear());
	
	//Head error
	Vector3d error_w_head = -DyrosMath::getPhi(head_transform_pre_desired_from_.linear(), master_head_pose_.linear());
	error_w_head = head_transform_pre_desired_from_.linear().transpose()*error_w_head;
	error_w_head(0) = 0;
	// error_w_head = head_transform_pre_desired_from_.linear()*error_w_head;

	//Upperarm error
	Vector3d error_w_lupperarm = -DyrosMath::getPhi(lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_.linear());
	error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear().transpose()*error_w_lupperarm;
	error_w_lupperarm(0) = 0;
	// error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear()*error_w_lupperarm;
	Vector3d error_w_rupperarm = -DyrosMath::getPhi(rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_.linear());
	error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear().transpose()*error_w_rupperarm;
	error_w_rupperarm(0) = 0;
	// error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear()*error_w_rupperarm;

	//Shoulder error
	Vector3d error_w_lshoulder = -DyrosMath::getPhi(lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_.linear());
	error_w_lshoulder = lacromion_transform_pre_desired_from_.linear().transpose()*error_w_lshoulder;
	error_w_lshoulder(0) = 0;
	// error_w_lshoulder = lacromion_transform_pre_desired_from_.linear()*error_w_lshoulder;
	Vector3d error_w_rshoulder = -DyrosMath::getPhi(racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_.linear());
	error_w_rshoulder = racromion_transform_pre_desired_from_.linear().transpose()*error_w_rshoulder;
	error_w_rshoulder(0) = 0;
	// error_w_rshoulder = racromion_transform_pre_desired_from_.linear()*error_w_rshoulder;

	u_dot_1 = 50*error_w_upperbody;

	u_dot_2.segment(0, 3) = 200*error_v_lhand;
	u_dot_2.segment(3, 3) = 100*error_w_lhand;
	u_dot_2.segment(6, 3) = 200*error_v_rhand;
	u_dot_2.segment(9, 3) = 100*error_w_rhand;
	u_dot_2.segment(12, 2) = 50*error_w_head.segment(1, 2);

	u_dot_3.segment(0, 2) = 50*error_w_lupperarm.segment(1, 2);
	u_dot_3.segment(2, 2) = 50*error_w_rupperarm.segment(1, 2);

	u_dot_4.segment(0, 2) = 50*error_w_lshoulder.segment(1, 2);
	u_dot_4.segment(2, 2) = 50*error_w_rshoulder.segment(1, 2);

	Vector3d zero3;
	zero3.setZero();
	J_temp.setZero(6, MODEL_DOF_VIRTUAL);
		
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Upper_Body].id, zero3, J_temp, false);
	J1.block(0, 0, 3, variable_size) = 	J_temp.block(0, 18, 3, variable_size);	//orientation

	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Left_Hand].id, zero3, J_temp, false);
	J2.block(0, 0, 3, variable_size) = J_temp.block(3, 18, 3, variable_size); 	//position
	J2.block(3, 0, 3, variable_size) = J_temp.block(0, 18, 3, variable_size);	//orientation
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Right_Hand].id, zero3, J_temp, false);
	J2.block(6, 0, 3, variable_size) = J_temp.block(3, 18, 3, variable_size); 	//position
	J2.block(9, 0, 3, variable_size) = J_temp.block(0, 18, 3, variable_size);	//orientation
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Head].id, zero3, J_temp, false);
	J2.block(12, 0, 2, variable_size) = (head_transform_pre_desired_from_.linear().transpose()*J_temp.block(0, 18, 3, variable_size)).block(1, 0, 2, variable_size);	//orientation

	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Left_Hand-4].id, zero3, J_temp, false);
	J3.block(0, 0, 2, variable_size) = (lupperarm_transform_pre_desired_from_.linear().transpose()*J_temp.block(0, 18, 3, variable_size)).block(1, 0, 2, variable_size);	//orientation
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Right_Hand-4].id, zero3, J_temp, false);
	J3.block(2, 0, 2, variable_size) = (rupperarm_transform_pre_desired_from_.linear().transpose()*J_temp.block(0, 18, 3, variable_size)).block(1, 0, 2, variable_size);	//orientation
	
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Left_Hand-6].id, zero3, J_temp, false);
	J4.block(0, 0, 2, variable_size) = (lacromion_transform_pre_desired_from_.linear().transpose()*J_temp.block(0, 18, 3, variable_size)).block(1, 0, 2, variable_size);	//orientation
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Right_Hand-6].id, zero3, J_temp, false);
	J4.block(2, 0, 2, variable_size) = (racromion_transform_pre_desired_from_.linear().transpose()*J_temp.block(0, 18, 3, variable_size)).block(1, 0, 2, variable_size);	//orientation
	
	//// null space projection of tasks for strict hierarchy////
	MatrixXd N1, N2_aug, N3_aug, J2_aug, J2_aug_pinv, J3_aug, J3_aug_pinv, J1_pinv,  J2N1, J3N2_aug, J4N3_aug, I3, I6, I15, I21;

	N1.setZero(variable_size, variable_size);
	N2_aug.setZero(variable_size, variable_size);
	N3_aug.setZero(variable_size, variable_size);
	J2_aug.setZero(control_size_hand+control_size_head+control_size_upperbody, variable_size);
	J2_aug_pinv.setZero(variable_size, control_size_hand+control_size_head+control_size_upperbody);
	J3_aug.setZero(control_size_hand+control_size_head+control_size_upperbody+control_size_upperarm, variable_size);
	J3_aug_pinv.setZero(variable_size, control_size_hand+control_size_head+control_size_upperbody+control_size_upperarm);
	J2N1.setZero(control_size_head, variable_size);
	J3N2_aug.setZero(control_size_upperarm, variable_size);
	J4N3_aug.setZero(control_size_shoulder, variable_size);
	
	I3.setIdentity(3, 3);
	I6.setIdentity(6, 6);
	I15.setIdentity(15, 15);
	I21.setIdentity(21, 21);
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	J1_pinv = J1.transpose()*(J1*J1.transpose()+I3*0.00001).inverse();
	std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
	N1 = I21 - (J1_pinv)*J1;

	J2_aug.block(0, 0, control_size_upperbody, variable_size) = J1;
	J2_aug.block(control_size_upperbody, 0, control_size_hand + control_size_head, variable_size) = J2;
	
	std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
	J2_aug_pinv = J2_aug.transpose()*(J2_aug*J2_aug.transpose()+Eigen::MatrixXd::Identity(17, 17)*0.00001).inverse();
	std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
	N2_aug = I21 - (J2_aug_pinv)*J2_aug;
	
	J3_aug.block(0, 0, control_size_hand+control_size_head+control_size_upperbody, variable_size) = J2_aug;
	J3_aug.block(control_size_hand+control_size_head+control_size_upperbody, 0, control_size_upperarm, variable_size) = J3;
	std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
	J3_aug_pinv = J3_aug.transpose()*(J3_aug*J3_aug.transpose()+Eigen::MatrixXd::Identity(21, 21)*0.00001).inverse();
	std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();
	N3_aug = I21 - (J3_aug_pinv)*J3_aug;
	
	J2N1 = J2*N1;
	J3N2_aug = J3*N2_aug;
	J4N3_aug = J4*N3_aug;

	H1 = J1.transpose()*J1;
	H2 = (J2N1).transpose()*(J2N1);
	H3 = (J3N2_aug).transpose()*(J3N2_aug);
	H4 = (J4N3_aug).transpose()*(J4N3_aug);
	H5 = I21; //A_mat_.block(18, 19, variable_size, variable_size);
	H6 = I21*(1/dt_)*(1/dt_);
	
	g1 = -J1.transpose()*u_dot_1;
	g2 = -(J2N1).transpose()*(u_dot_2 - J2*motion_q_dot_pre_.segment(12, variable_size));
	g3 = -(J3N2_aug).transpose()*(u_dot_3 - J3*motion_q_dot_pre_.segment(12, variable_size));
	g4 = -(J4N3_aug).transpose()*(u_dot_4 - J4*motion_q_dot_pre_.segment(12, variable_size));
	g5.setZero();
	g6 = -motion_q_dot_pre_.segment(12, variable_size)*(1/dt_)*(1/dt_);

	H = w1*H1 + w2*H2 + w3*H3 + w4*H4 + w5*H5 + w6*H6;
	g = w1*g1 + w2*g2 + w3*g3 + w4*g4 + w5*g5 + w6*g6;
	
	double speed_reduce_rate= 20; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

	for (int i=0; i< constraint_size1; i++)
	{
		lb(i) = max(speed_reduce_rate*(joint_limit_l_(i+12) - current_q_(i+12)), joint_vel_limit_l_(i+12));
		ub(i) = min(speed_reduce_rate*(joint_limit_h_(i+12) - current_q_(i+12)), joint_vel_limit_h_(i+12));
	}

	A = J2.block(0, 0, constraint_size2, variable_size);

	for(int i = 0; i<constraint_size2; i++)
	{
		for(int j=0 ;j<variable_size; j++)
		{
			if( abs(A(i, j)) < 1e-3)
			{	
				if(A(i,j) > 0)
				{
					A(i, j) = 1e-3;
				}
				else
				{
					A(i, j) = -1e-3;
				}
			}
		}
	}

	for (int i=0; i< 3; i++) //linear velocity limit
	{
		lbA(i) = -1;
		ubA(i) = 1;
		lbA(i+6) = -1;
		ubA(i+6) = 1;
	}

	for (int i=0; i< 3; i++)	//angular velocity limit
	{
		lbA(i+3) = -3;
		ubA(i+3) = 3;
		lbA(i+9) = -3;
		ubA(i+9) = 3;
	}
	
	QP_qdot_upperbody.EnableEqualityCondition(0.0000001);
	QP_qdot_upperbody.UpdateMinProblem(H, g);
	QP_qdot_upperbody.UpdateSubjectToAx(A, lbA, ubA);
	QP_qdot_upperbody.UpdateSubjectToX(lb, ub);

	VectorXd q_dot_upperbody;
	std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();
	if(QP_qdot_upperbody.SolveQPoases(100, qpres))
	{
		q_dot_upperbody = qpres.segment(0, variable_size);
	}
	else
	{
		q_dot_upperbody.setZero(variable_size);
	}
	std::chrono::steady_clock::time_point t9 = std::chrono::steady_clock::now();
	for(int i =0; i<variable_size; i++)
	{
		motion_q_dot_(12+i) = q_dot_upperbody(i);
		motion_q_(12 + i) = motion_q_pre_(12+i) + motion_q_dot_(12+i)*dt_;
		pd_control_mask_(12+i) = 1;
	}

	std::chrono::steady_clock::time_point t10 = std::chrono::steady_clock::now();

	if(int(current_time_*1e4)%int(1e3) == 0)
	{
		cout<<"t2 - t1: "<< std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() <<endl;
		cout<<"t3 - t2: "<< std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() <<endl;
		cout<<"t4 - t3: "<< std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count() <<endl;
		cout<<"t5 - t4: "<< std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count() <<endl;
		cout<<"t6 - t5: "<< std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5).count() <<endl;
		cout<<"t7 - t6: "<< std::chrono::duration_cast<std::chrono::microseconds>(t7 - t6).count() <<endl;
		cout<<"t8 - t7: "<< std::chrono::duration_cast<std::chrono::microseconds>(t8 - t7).count() <<endl;
		cout<<"t9 - t8: "<< std::chrono::duration_cast<std::chrono::microseconds>(t9 - t8).count() <<endl;
		cout<<"t10 - t1: "<< std::chrono::duration_cast<std::chrono::microseconds>(t10 - t1).count() <<endl;
	}

}

void CustomController::motionRetargeting_QPIK_wholebody()
{
	const int variable_size = 33;
	const int constraint_size1 = 33;
	

}

void CustomController::motionRetargeting_HQPIK()
{
	// const int variable_size_ = 21;
	// const int constraint_size1_ = 21;	//[lb <=	x	<= 	ub] form constraints
	// const int constraint_size2_[4] = {12, 3+12, 3+14+0, 3+14+4+0};	//[lb <=	Ax 	<=	ub] or [Ax = b]
	// const int control_size_[4] = {3, 14, 4, 4};		//1: upperbody, 2: head + hand, 3: upperarm, 4: shoulder
	// cout<<"test1"<<endl;
	if(first_loop_hqpik_)
    {	
		for (int i = 0; i<hierarchy_num_; i++)
		{
			QP_qdot_hqpik.resize(hierarchy_num_);
			QP_qdot_hqpik[i].InitializeProblemSize(variable_size_, constraint_size2_[i]);
			J_hqpik_[i].setZero(control_size_[i], variable_size_);
			u_dot_[i].setZero(control_size_[i]);

			ubA_[i].setZero(constraint_size2_[i]);
			lbA_[i].setZero(constraint_size2_[i]);
					
			H_[i].setZero(variable_size_, variable_size_);
			g_[i].setZero(variable_size_);

			ub_[i].setZero(constraint_size1_);
			lb_[i].setZero(constraint_size1_);

			q_dot_hqpik_[i].setZero(variable_size_);
		}

		w1_ = 100;		//upperbody tracking
		w2_ = 50;			//kinematic energy
		w3_ = 0;		//acceleration

		equality_condition_eps_ = 1E-6;
		damped_puedoinverse_eps_ = 1E-4;

		first_loop_hqpik_ = false;
	}
	
	VectorQVQd q_desired_pre;
	q_desired_pre.setZero();
	q_desired_pre(39) = 1;
	q_desired_pre.segment(6, MODEL_DOF) = pre_desired_q_;
	Vector3d zero3;
	zero3.setZero();
	J_temp_.setZero(6, MODEL_DOF_VIRTUAL);

	////1st Task
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Upper_Body].id, zero3, J_temp_, false);
	J_hqpik_[0].block(0, 0, 3, variable_size_) = J_temp_.block(0, 18, 3, variable_size_);	//orientation

	Vector3d error_w_upperbody = -DyrosMath::getPhi(upperbody_transform_pre_desired_from_.linear(), master_upperbody_pose_.linear());
	u_dot_[0] = 100*error_w_upperbody;


	////2nd Task
	J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Left_Hand].id, zero3, J_temp_, false);
	J_hqpik_[1].block(0, 0, 3, variable_size_) = J_temp_.block(3, 18, 3, variable_size_); 	//position
	J_hqpik_[1].block(3, 0, 3, variable_size_) = J_temp_.block(0, 18, 3, variable_size_);	//orientation
	J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Right_Hand].id, zero3, J_temp_, false);
	J_hqpik_[1].block(6, 0, 3, variable_size_) = J_temp_.block(3, 18, 3, variable_size_); 	//position
	J_hqpik_[1].block(9, 0, 3, variable_size_) = J_temp_.block(0, 18, 3, variable_size_);	//orientation
	J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Head].id, zero3, J_temp_, false);
	J_hqpik_[1].block(12, 0, 2, variable_size_) = (head_transform_pre_desired_from_.linear().transpose()*J_temp_.block(0, 18, 3, variable_size_)).block(1, 0, 2, variable_size_);	//orientation

		//Hand error
	Vector3d error_v_lhand = master_lhand_pose_.translation() - lhand_transform_pre_desired_from_.translation();
	Vector3d error_w_lhand = -DyrosMath::getPhi(lhand_transform_pre_desired_from_.linear(), master_lhand_pose_.linear());
	Vector3d error_v_rhand = master_rhand_pose_.translation() - rhand_transform_pre_desired_from_.translation();
	Vector3d error_w_rhand = -DyrosMath::getPhi(rhand_transform_pre_desired_from_.linear(), master_rhand_pose_.linear());
	
		//Head error
	Vector3d error_w_head = -DyrosMath::getPhi(head_transform_pre_desired_from_.linear(), master_head_pose_.linear());
	error_w_head = head_transform_pre_desired_from_.linear().transpose()*error_w_head;
	error_w_head(0) = 0;

	u_dot_[1].segment(0, 3) = 200*error_v_lhand;
	u_dot_[1].segment(3, 3) = 100*error_w_lhand;
	u_dot_[1].segment(6, 3) = 200*error_v_rhand;
	u_dot_[1].segment(9, 3) = 100*error_w_rhand;
	u_dot_[1].segment(12, 2) = 100*error_w_head.segment(1, 2);

	////3rd Task
	J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Left_Hand-4].id, zero3, J_temp_, false);
	J_hqpik_[2].block(0, 0, 2, variable_size_) = (lupperarm_transform_pre_desired_from_.linear().transpose()*J_temp_.block(0, 18, 3, variable_size_)).block(1, 0, 2, variable_size_);	//orientation
	J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Right_Hand-4].id, zero3, J_temp_, false);
	J_hqpik_[2].block(2, 0, 2, variable_size_) = (rupperarm_transform_pre_desired_from_.linear().transpose()*J_temp_.block(0, 18, 3, variable_size_)).block(1, 0, 2, variable_size_);	//orientation

		//Upperarm error
	Vector3d error_w_lupperarm = -DyrosMath::getPhi(lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_.linear());
	error_w_lupperarm = lupperarm_transform_pre_desired_from_.linear().transpose()*error_w_lupperarm;
	error_w_lupperarm(0) = 0;
		
	Vector3d error_w_rupperarm = -DyrosMath::getPhi(rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_.linear());
	error_w_rupperarm = rupperarm_transform_pre_desired_from_.linear().transpose()*error_w_rupperarm;
	error_w_rupperarm(0) = 0;

	u_dot_[2].segment(0, 2) = 100*error_w_lupperarm.segment(1, 2);
	u_dot_[2].segment(2, 2) = 100*error_w_rupperarm.segment(1, 2);
	// cout<<"test2"<<endl;
	//4th Task
	J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Left_Hand-6].id, zero3, J_temp_, false);
	J_hqpik_[3].block(0, 0, 2, variable_size_) = (lacromion_transform_pre_desired_from_.linear().transpose()*J_temp_.block(0, 18, 3, variable_size_)).block(1, 0, 2, variable_size_);	//orientation
	J_temp_.setZero(6, MODEL_DOF_VIRTUAL);
	RigidBodyDynamics::CalcPointJacobian6D(model_d_, q_desired_pre, rd_.link_[Right_Hand-6].id, zero3, J_temp_, false);
	J_hqpik_[3].block(2, 0, 2, variable_size_) = (racromion_transform_pre_desired_from_.linear().transpose()*J_temp_.block(0, 18, 3, variable_size_)).block(1, 0, 2, variable_size_);	//orientation

		//Shoulder error
	Vector3d error_w_lshoulder = -DyrosMath::getPhi(lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_.linear());
	error_w_lshoulder = lacromion_transform_pre_desired_from_.linear().transpose()*error_w_lshoulder;
	error_w_lshoulder(0) = 0;
	Vector3d error_w_rshoulder = -DyrosMath::getPhi(racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_.linear());
	error_w_rshoulder = racromion_transform_pre_desired_from_.linear().transpose()*error_w_rshoulder;
	error_w_rshoulder(0) = 0;
	u_dot_[3].segment(0, 2) = 50*error_w_lshoulder.segment(1, 2);
	u_dot_[3].segment(2, 2) = 50*error_w_rshoulder.segment(1, 2);


	// cout<<"test3"<<endl;

	for( int i=0; i < hierarchy_num_; i ++ )
	{

		MatrixXd H1, H2, H3;
		VectorXd g1, g2, g3;

		H1 = J_hqpik_[i].transpose()*J_hqpik_[i];
		H2 = Eigen::MatrixXd::Identity(variable_size_, variable_size_);
		H3 = Eigen::MatrixXd::Identity(variable_size_, variable_size_)*(1/dt_)*(1/dt_);

		g1 = -J_hqpik_[i].transpose()*u_dot_[i];
		g2.setZero(variable_size_);
		g3 = -motion_q_dot_pre_.segment(12, variable_size_)*(1/dt_)*(1/dt_);

		H_[i] = w1_*H1 + w2_*H2 + w3_*H3;
		g_[i] = w1_*g1 + w2_*g2 + w3_*g3;

		double speed_reduce_rate= 20; // when the current joint position is near joint limit (10 degree), joint limit condition is activated.

		for (int j=0; j< constraint_size1_; j++)
		{
			lb_[i](j) = max(speed_reduce_rate*(joint_limit_l_(j+12) - current_q_(j+12)), joint_vel_limit_l_(j+12));
			ub_[i](j) = min(speed_reduce_rate*(joint_limit_h_(j+12) - current_q_(j+12)), joint_vel_limit_h_(j+12));
		}

		A_hqpik_[i].setZero(constraint_size2_[i], variable_size_);
		// cout<<"test4"<<endl;
		int higher_task_equality_num = 0;
		for(int h = 0; h< i; h++ )
		{
			A_hqpik_[i].block(higher_task_equality_num, 0, control_size_[h], variable_size_) = J_hqpik_[h];
			ubA_[i].segment(higher_task_equality_num, control_size_[h]) = J_hqpik_[h]*q_dot_hqpik_[h];
			lbA_[i].segment(higher_task_equality_num, control_size_[h]) = J_hqpik_[h]*q_dot_hqpik_[h];
			
			higher_task_equality_num += control_size_[h];
		}  
		// cout<<"test5"<<endl;
		// hand velocity constraints
		if( i < 2)
		{
			A_hqpik_[i].block(higher_task_equality_num, 0, 12, variable_size_) = J_hqpik_[1].block(0, 0, 12, variable_size_);
			
			for (int j=0; j< 3; j++) 
			{
				//linear velocity limit
				lbA_[i](higher_task_equality_num+j) = -2;
				ubA_[i](higher_task_equality_num+j) = 2;
				lbA_[i](higher_task_equality_num+j+6) = -2;
				ubA_[i](higher_task_equality_num+j+6) = 2;

				//angular velocity limit
				lbA_[i](higher_task_equality_num+j+3) = -3;
				ubA_[i](higher_task_equality_num+j+3) = 3;
				lbA_[i](higher_task_equality_num+j+9) = -3;
				ubA_[i](higher_task_equality_num+j+9) = 3;
			}
		}
		
		// cout<<"test6"<<endl;
		for(int m = 0; m<constraint_size2_[i]; m++)
		{
			for(int n=0 ;n<variable_size_; n++)
			{
				if( abs(A_hqpik_[i](m, n)) < 1e-3)
				{	
					if(A_hqpik_[i](m, n) > 0)
					{
						A_hqpik_[i](m, n) = 1e-3;
					}
					else
					{
						A_hqpik_[i](m, n) = -1e-3;
					}
				}
			}
		}
		// cout<<"test7"<<endl;

		QP_qdot_hqpik[i].EnableEqualityCondition(equality_condition_eps_);
		QP_qdot_hqpik[i].UpdateMinProblem(H_[i], g_[i]);
		QP_qdot_hqpik[i].UpdateSubjectToAx(A_hqpik_[i], lbA_[i], ubA_[i]);
		QP_qdot_hqpik[i].UpdateSubjectToX(lb_[i], ub_[i]);
		
		// cout<<"test8"<<endl;
		if(QP_qdot_hqpik[i].SolveQPoases(100, qpres_))
		{
			q_dot_hqpik_[i] = qpres_.segment(0, variable_size_);

			last_solved_hierarchy_num_ = i;
		}
		else
		{	
			if(i == 0)
			{
				q_dot_hqpik_[i] = 0.5*motion_q_dot_pre_.segment(12, variable_size_);
			}
				
			last_solved_hierarchy_num_ = max(i-1, 0);
			cout<<"Error hierarchy: "<< i << endl;
			// cout<<"Error qpres_: \n"<< qpres_ << endl;
			continue;
		}
		// cout<<"ubA_[0]: " << ubA_[0]<<endl;
	}
	if( int(current_time_*10000)%200 == 0)
	{
		cout<<"u_dot_[0]- J_hqpik_[0]*q_dot_hqpik_[0]: \n" << u_dot_[0] - J_hqpik_[0]*q_dot_hqpik_[0] << endl;
		cout<<"u_dot_[0]- J_hqpik_[0]*q_dot_hqpik_[1]: \n" << u_dot_[0] - J_hqpik_[0]*q_dot_hqpik_[1] << endl;
		cout<<"u_dot_[0]- J_hqpik_[0]*q_dot_hqpik_[2]: \n" << u_dot_[0] - J_hqpik_[0]*q_dot_hqpik_[2] << endl;
		cout<<"u_dot_[1]- J_hqpik_[1]*q_dot_hqpik_[1]: \n" << u_dot_[1] - J_hqpik_[1]*q_dot_hqpik_[1] << endl;
		cout<<"u_dot_[2]- J_hqpik_[2]*q_dot_hqpik_[2]: \n" << u_dot_[2] - J_hqpik_[2]*q_dot_hqpik_[2] << endl;

		cout<<"J_hqpik_[1]: \n" << J_hqpik_[1]<<endl;
		cout<<"u_dot_[0]: \n" << u_dot_[0]<<endl;
		cout<<"u_dot_[1]: \n" << u_dot_[1]<<endl;
		cout<<"u_dot_[2]: \n" << u_dot_[2]<<endl;
	}

	// cout<<"J_hqpik_[0]: \n"<< J_hqpik_[0]<<endl;
	for(int i =0; i<variable_size_; i++)
	{
		motion_q_dot_(12+i) = q_dot_hqpik_[last_solved_hierarchy_num_](i);
		motion_q_(12 + i) = motion_q_pre_(12+i) + motion_q_dot_(12+i)*dt_;
		pd_control_mask_(12+i) = 1;
	}
	
}

void CustomController::poseCalibration()
{
	hmd_tracker_status_ = hmd_tracker_status_raw_;

	if( hmd_tracker_status_ == true)
	{
		if(hmd_tracker_status_pre_ == false)
		{
			tracker_status_changed_time_ = current_time_;
			cout<<"tracker is attatched"<<endl;

			hmd_head_pose_raw_last_			=hmd_head_pose_;
			hmd_lupperarm_pose_raw_last_	=hmd_lupperarm_pose_;
			hmd_lhand_pose_raw_last_		=hmd_lhand_pose_;
			hmd_rupperarm_pose_raw_last_	=hmd_rupperarm_pose_;
			hmd_rhand_pose_raw_last_		=hmd_rhand_pose_;
			hmd_chest_pose_raw_last_		=hmd_chest_pose_;
			hmd_pelv_pose_raw_last_			=hmd_pelv_pose_;
		}
		
		if( current_time_ - tracker_status_changed_time_ <= 5)
		{
			double w = DyrosMath::cubic(current_time_, tracker_status_changed_time_, tracker_status_changed_time_+5, 0, 1, 0, 0);
			// double w = (current_time_ - tracker_status_changed_time_)/5;
			// w = DyrosMath::minmax_cut(w, 0, 1);

			hmd_head_pose_.translation()		=	w*hmd_head_pose_raw_.translation() + (1-w)*hmd_head_pose_raw_last_.translation();
			hmd_lupperarm_pose_.translation()	=	w*hmd_lupperarm_pose_raw_.translation() + (1-w)*hmd_lupperarm_pose_raw_last_.translation();
			hmd_lhand_pose_.translation()		=	w*hmd_lhand_pose_raw_.translation() + (1-w)*hmd_lhand_pose_raw_last_.translation();
			hmd_rupperarm_pose_.translation()	=	w*hmd_rupperarm_pose_raw_.translation() + (1-w)*hmd_rupperarm_pose_raw_last_.translation();
			hmd_rhand_pose_.translation()		=	w*hmd_rhand_pose_raw_.translation() + (1-w)*hmd_rhand_pose_raw_last_.translation();
			hmd_chest_pose_.translation()		=	w*hmd_chest_pose_raw_.translation() + (1-w)*hmd_chest_pose_raw_last_.translation();
			hmd_pelv_pose_.translation()		=	w*hmd_pelv_pose_raw_.translation() + (1-w)*hmd_pelv_pose_raw_last_.translation();

			Eigen::AngleAxisd head_ang_diff(hmd_head_pose_raw_.linear()*hmd_head_pose_raw_last_.linear().transpose());
			Eigen::AngleAxisd lelbow_ang_diff(hmd_lupperarm_pose_raw_.linear()*hmd_lupperarm_pose_raw_last_.linear().transpose());
			Eigen::AngleAxisd lhand_ang_diff(hmd_lhand_pose_raw_.linear()*hmd_lhand_pose_raw_last_.linear().transpose());
			Eigen::AngleAxisd relbow_ang_diff(hmd_rupperarm_pose_raw_.linear()*hmd_rupperarm_pose_raw_last_.linear().transpose());
			Eigen::AngleAxisd rhand_ang_diff(hmd_rhand_pose_raw_.linear()*hmd_rhand_pose_raw_last_.linear().transpose());
			Eigen::AngleAxisd upperbody_ang_diff(hmd_chest_pose_raw_.linear()*hmd_chest_pose_raw_last_.linear().transpose());
			Eigen::AngleAxisd pelv_ang_diff(hmd_pelv_pose_raw_.linear()*hmd_pelv_pose_raw_last_.linear().transpose());

			Eigen::Matrix3d lhand_diff_m, rhand_diff_m, lelbow_diff_m, relbow_diff_m,  head_diff_m, upperbody_diff_m, pelv_diff_m;
			lhand_diff_m = Eigen::AngleAxisd(lhand_ang_diff.angle()*w, lhand_ang_diff.axis());
			rhand_diff_m = Eigen::AngleAxisd(rhand_ang_diff.angle()*w, rhand_ang_diff.axis());
			lelbow_diff_m = Eigen::AngleAxisd(lelbow_ang_diff.angle()*w, lelbow_ang_diff.axis());
			relbow_diff_m = Eigen::AngleAxisd(relbow_ang_diff.angle()*w, relbow_ang_diff.axis());
			head_diff_m = Eigen::AngleAxisd(head_ang_diff.angle()*w, head_ang_diff.axis());
			upperbody_diff_m = Eigen::AngleAxisd(upperbody_ang_diff.angle()*w, upperbody_ang_diff.axis());
			pelv_diff_m = Eigen::AngleAxisd(pelv_ang_diff.angle()*w, pelv_ang_diff.axis());

			hmd_lupperarm_pose_.linear() = lelbow_diff_m*hmd_lupperarm_pose_raw_last_.linear();
			hmd_lhand_pose_.linear() = lhand_diff_m*hmd_lhand_pose_raw_last_.linear();
			hmd_rupperarm_pose_.linear() = relbow_diff_m*hmd_rupperarm_pose_raw_last_.linear();
			hmd_rhand_pose_.linear() = rhand_diff_m*hmd_rhand_pose_raw_last_.linear();
			hmd_head_pose_.linear() = head_diff_m*hmd_head_pose_raw_last_.linear();
			hmd_chest_pose_.linear() = upperbody_diff_m*hmd_chest_pose_raw_last_.linear();
			hmd_pelv_pose_.linear() = pelv_diff_m*hmd_pelv_pose_raw_last_.linear();
		}
		else
		{
			hmd_head_pose_			=	hmd_head_pose_raw_;
			hmd_lshoulder_pose_		=	hmd_lshoulder_pose_raw_;
			hmd_lupperarm_pose_		=	hmd_lupperarm_pose_raw_;
			hmd_lhand_pose_			=	hmd_lhand_pose_raw_;
			hmd_rshoulder_pose_		=	hmd_rshoulder_pose_raw_;
			hmd_rupperarm_pose_		=	hmd_rupperarm_pose_raw_;
			hmd_rhand_pose_			=	hmd_rhand_pose_raw_;
			hmd_chest_pose_			=	hmd_chest_pose_raw_;
			hmd_pelv_pose_			=	hmd_pelv_pose_raw_;
		}

		//Check Abrupt Movement of trackers
		hmd_head_vel_.segment(0, 3) = (hmd_head_pose_.translation() - hmd_head_pose_pre_.translation())/dt_;
		hmd_lshoulder_vel_.segment(0, 3) = (hmd_lshoulder_pose_.translation() - hmd_lshoulder_pose_pre_.translation())/dt_;
		hmd_lupperarm_vel_.segment(0, 3) = (hmd_lupperarm_pose_.translation() - hmd_lupperarm_pose_pre_.translation())/dt_;
		hmd_lhand_vel_.segment(0, 3) = (hmd_lhand_pose_.translation() - hmd_lhand_pose_pre_.translation())/dt_;
		hmd_rshoulder_vel_.segment(0, 3) = (hmd_rshoulder_pose_.translation() - hmd_rshoulder_pose_pre_.translation())/dt_;
		hmd_rupperarm_vel_.segment(0, 3) = (hmd_rupperarm_pose_.translation() - hmd_rupperarm_pose_pre_.translation())/dt_;
		hmd_rhand_vel_.segment(0, 3) = (hmd_rhand_pose_.translation() - hmd_rhand_pose_pre_.translation())/dt_;
		hmd_chest_vel_.segment(0, 3) = (hmd_chest_pose_.translation() - hmd_chest_pose_pre_.translation())/dt_;
		hmd_pelv_vel_.segment(0, 3) = (hmd_pelv_pose_.translation() - hmd_pelv_pose_pre_.translation())/dt_;

		Eigen::AngleAxisd ang_temp_1(hmd_head_pose_.linear()*hmd_head_pose_pre_.linear().transpose());
		Eigen::AngleAxisd ang_temp_2(hmd_lshoulder_pose_.linear()*hmd_lshoulder_pose_pre_.linear().transpose());
		Eigen::AngleAxisd ang_temp_3(hmd_lupperarm_pose_.linear()*hmd_lupperarm_pose_pre_.linear().transpose());
		Eigen::AngleAxisd ang_temp_4(hmd_lhand_pose_.linear()*hmd_lhand_pose_pre_.linear().transpose());
		Eigen::AngleAxisd ang_temp_5(hmd_rshoulder_pose_.linear()*hmd_rshoulder_pose_pre_.linear().transpose());
		Eigen::AngleAxisd ang_temp_6(hmd_rupperarm_pose_.linear()*hmd_rupperarm_pose_pre_.linear().transpose());
		Eigen::AngleAxisd ang_temp_7(hmd_rhand_pose_.linear()*hmd_rhand_pose_pre_.linear().transpose());
		Eigen::AngleAxisd ang_temp_8(hmd_chest_pose_.linear()*hmd_chest_pose_pre_.linear().transpose());
		Eigen::AngleAxisd ang_temp_9(hmd_pelv_pose_.linear()*hmd_pelv_pose_pre_.linear().transpose());

		hmd_head_vel_.segment(3, 3) = ang_temp_1.axis()*ang_temp_1.angle()/dt_;
		hmd_lshoulder_vel_.segment(3, 3) = ang_temp_2.axis()*ang_temp_2.angle()/dt_;
		hmd_lupperarm_vel_.segment(3, 3) = ang_temp_3.axis()*ang_temp_3.angle()/dt_;
		hmd_lhand_vel_.segment(3, 3) = ang_temp_4.axis()*ang_temp_4.angle()/dt_;
		hmd_rshoulder_vel_.segment(3, 3) = ang_temp_5.axis()*ang_temp_5.angle()/dt_;
		hmd_rupperarm_vel_.segment(3, 3) = ang_temp_6.axis()*ang_temp_6.angle()/dt_;
		hmd_rhand_vel_.segment(3, 3) = ang_temp_7.axis()*ang_temp_7.angle()/dt_;
		hmd_chest_vel_.segment(3, 3) = ang_temp_8.axis()*ang_temp_8.angle()/dt_;
		hmd_pelv_vel_.segment(3, 3) = ang_temp_9.axis()*ang_temp_9.angle()/dt_;

		// if(hmd_head_vel_.norm() > 10)
		// { 
		// 	cout<<"abrupt movement is detected at the head tracker!"<<endl;
		// 	hmd_head_pose_ =  hmd_head_pose_pre_;
		// }
		// if(hmd_lshoulder_vel_.norm() > 10)
		// { 
		// 	cout<<"abrupt movement is detected at the lshoulder!"<<endl;
		// 	hmd_lshoulder_pose_ =  hmd_lshoulder_pose_pre_;
		// }
		// if(hmd_lupperarm_vel_.norm() > 10)
		// { 
		// 	cout<<"abrupt movement is detected at the lupperarm tracker!"<<endl;
		// 	hmd_lupperarm_pose_ =  hmd_lupperarm_pose_pre_;
		// }
		// if(hmd_lhand_vel_.norm() > 10)
		// { 
		// 	cout<<"abrupt movement is detected at the lhand tracker!"<<endl;
		// 	hmd_lhand_pose_ =  hmd_lhand_pose_pre_;
		// }
		// if(hmd_rshoulder_vel_.norm() > 10)
		// { 
		// 	cout<<"abrupt movement is detected at the rshoulder!"<<endl;
		// 	hmd_rshoulder_pose_ =  hmd_rshoulder_pose_pre_;
		// }
		// if(hmd_rupperarm_vel_.norm() > 10)
		// { 
		// 	cout<<"abrupt movement is detected at the rupperarm tracker!"<<endl;
		// 	hmd_rupperarm_pose_ =  hmd_rupperarm_pose_pre_;
		// }
		// if(hmd_rhand_vel_.norm() > 10)
		// { 
		// 	cout<<"abrupt movement is detected at the rhand tracker!"<<endl;
		// 	hmd_rhand_pose_ =  hmd_rhand_pose_pre_;
		// }
		// if(hmd_chest_vel_.norm() > 10)
		// { 
		// 	cout<<"abrupt movement is detected at the chest tracker!"<<endl;
		// 	hmd_chest_pose_ =  hmd_chest_pose_pre_;
		// }
		// if(hmd_pelv_vel_.norm() > 10)
		// { 
		// 	cout<<"abrupt movement is detected at the pelv tracker!"<<endl;
		// 	hmd_pelv_pose_ =  hmd_pelv_pose_pre_;
		// }

	}
	else	//false
	{
		if(hmd_tracker_status_pre_ == true)
		{
			tracker_status_changed_time_ = current_time_;
			cout<<"tracker is detatched"<<endl;

			hmd_head_pose_raw_last_			=hmd_head_pose_;
			hmd_lupperarm_pose_raw_last_	=hmd_lupperarm_pose_;
			hmd_lhand_pose_raw_last_		=hmd_lhand_pose_;
			hmd_rupperarm_pose_raw_last_	=hmd_rupperarm_pose_;
			hmd_rhand_pose_raw_last_		=hmd_rhand_pose_;
			hmd_chest_pose_raw_last_		=hmd_chest_pose_;
			hmd_pelv_pose_raw_last_			=hmd_pelv_pose_;
		}

		double w = DyrosMath::cubic(current_time_, tracker_status_changed_time_, tracker_status_changed_time_+5, 0, 1, 0, 0);
		// double w = (current_time_ - tracker_status_changed_time_)/5;
		// w = DyrosMath::minmax_cut(w, 0, 1);

		hmd_head_pose_.translation()		=	w*hmd_head_pose_init_.translation() + (1-w)*hmd_head_pose_raw_last_.translation();
		hmd_lupperarm_pose_.translation()	=	w*hmd_lupperarm_pose_init_.translation() + (1-w)*hmd_lupperarm_pose_raw_last_.translation();
		hmd_lhand_pose_.translation()		=	w*hmd_lhand_pose_init_.translation() + (1-w)*hmd_lhand_pose_raw_last_.translation();
		hmd_rupperarm_pose_.translation()	=	w*hmd_rupperarm_pose_init_.translation() + (1-w)*hmd_rupperarm_pose_raw_last_.translation();
		hmd_rhand_pose_.translation()		=	w*hmd_rhand_pose_init_.translation() + (1-w)*hmd_rhand_pose_raw_last_.translation();
		hmd_chest_pose_.translation()		=	w*hmd_chest_pose_init_.translation() + (1-w)*hmd_chest_pose_raw_last_.translation();
		hmd_pelv_pose_.translation()		=	w*hmd_pelv_pose_init_.translation() + (1-w)*hmd_pelv_pose_raw_last_.translation();

		Eigen::AngleAxisd head_ang_diff(hmd_head_pose_init_.linear()*hmd_head_pose_raw_last_.linear().transpose());
		Eigen::AngleAxisd lelbow_ang_diff(hmd_lupperarm_pose_init_.linear()*hmd_lupperarm_pose_raw_last_.linear().transpose());
		Eigen::AngleAxisd lhand_ang_diff(hmd_lhand_pose_init_.linear()*hmd_lhand_pose_raw_last_.linear().transpose());
		Eigen::AngleAxisd relbow_ang_diff(hmd_rupperarm_pose_init_.linear()*hmd_rupperarm_pose_raw_last_.linear().transpose());
		Eigen::AngleAxisd rhand_ang_diff(hmd_rhand_pose_init_.linear()*hmd_rhand_pose_raw_last_.linear().transpose());
		Eigen::AngleAxisd upperbody_ang_diff(hmd_chest_pose_init_.linear()*hmd_chest_pose_raw_last_.linear().transpose());
		Eigen::AngleAxisd pelv_ang_diff(hmd_pelv_pose_init_.linear()*hmd_pelv_pose_raw_last_.linear().transpose());

		Eigen::Matrix3d lhand_diff_m, rhand_diff_m, lelbow_diff_m, relbow_diff_m,  head_diff_m, upperbody_diff_m, pelv_diff_m;
		lhand_diff_m = Eigen::AngleAxisd(lhand_ang_diff.angle()*w, lhand_ang_diff.axis());
		rhand_diff_m = Eigen::AngleAxisd(rhand_ang_diff.angle()*w, rhand_ang_diff.axis());
		lelbow_diff_m = Eigen::AngleAxisd(lelbow_ang_diff.angle()*w, lelbow_ang_diff.axis());
		relbow_diff_m = Eigen::AngleAxisd(relbow_ang_diff.angle()*w, relbow_ang_diff.axis());
		head_diff_m = Eigen::AngleAxisd(head_ang_diff.angle()*w, head_ang_diff.axis());
		upperbody_diff_m = Eigen::AngleAxisd(upperbody_ang_diff.angle()*w, upperbody_ang_diff.axis());
		pelv_diff_m = Eigen::AngleAxisd(pelv_ang_diff.angle()*w, pelv_ang_diff.axis());

		hmd_lupperarm_pose_.linear() = lelbow_diff_m*hmd_lupperarm_pose_raw_last_.linear();
		hmd_lhand_pose_.linear() = lhand_diff_m*hmd_lhand_pose_raw_last_.linear();
		hmd_rupperarm_pose_.linear() = relbow_diff_m*hmd_rupperarm_pose_raw_last_.linear();
		hmd_rhand_pose_.linear() = rhand_diff_m*hmd_rhand_pose_raw_last_.linear();
		hmd_head_pose_.linear() = head_diff_m*hmd_head_pose_raw_last_.linear();
		hmd_chest_pose_.linear() = upperbody_diff_m*hmd_chest_pose_raw_last_.linear();
		hmd_pelv_pose_.linear() = pelv_diff_m*hmd_pelv_pose_raw_last_.linear();

		// hmd_head_pose_		= hmd_head_pose_raw_last_;
		// hmd_lupperarm_pose_	= hmd_lupperarm_pose_raw_last_;
		// hmd_lhand_pose_		= hmd_lhand_pose_raw_last_;	
		// hmd_rupperarm_pose_	= hmd_rupperarm_pose_raw_last_;
		// hmd_rhand_pose_		= hmd_rhand_pose_raw_last_;	
		// hmd_chest_pose_		= hmd_chest_pose_raw_last_;	
		// hmd_pelv_pose_		= hmd_pelv_pose_raw_last_;		
	}
	


	Eigen::Vector3d hmd_pelv_rpy;
	Eigen::Matrix3d hmd_pelv_yaw_rot;
	hmd_pelv_rpy = DyrosMath::rot2Euler(hmd_pelv_pose_.linear());
	hmd_pelv_yaw_rot = DyrosMath::rotateWithZ(hmd_pelv_rpy(2));
	hmd_pelv_pose_.linear() = hmd_pelv_yaw_rot;

	//coordinate conversion
	hmd_head_pose_ = hmd_pelv_pose_.inverse()*hmd_head_pose_;
	hmd_lupperarm_pose_ = hmd_pelv_pose_.inverse()*hmd_lupperarm_pose_;
	hmd_lhand_pose_ = hmd_pelv_pose_.inverse()*hmd_lhand_pose_;
	hmd_rupperarm_pose_ = hmd_pelv_pose_.inverse()*hmd_rupperarm_pose_;
	hmd_rhand_pose_ = hmd_pelv_pose_.inverse()*hmd_rhand_pose_;
	hmd_chest_pose_ = hmd_pelv_pose_.inverse()*hmd_chest_pose_;
	// hmd_pelv_pose_.linear().setIdentity();

	//Shoulder Data
	hmd_lshoulder_pose_.translation() = hmd_chest_pose_.linear()*hmd_chest_pose_init_.linear().transpose()*hmd_chest_2_lshoulder_center_pos_ + hmd_chest_pose_.translation();
	hmd_lshoulder_pose_.linear() = hmd_chest_pose_.linear();
	hmd_rshoulder_pose_.translation() = hmd_chest_pose_.linear()*hmd_chest_pose_init_.linear().transpose()*hmd_chest_2_rshoulder_center_pos_ + hmd_chest_pose_.translation();
	hmd_rshoulder_pose_.linear() = hmd_chest_pose_.linear();

	Eigen::Vector3d tracker_offset;
	tracker_offset << -0.00, 0, 0;

	hmd_lhand_pose_.translation() += hmd_lhand_pose_.linear()*tracker_offset;
	hmd_rhand_pose_.translation() += hmd_rhand_pose_.linear()*tracker_offset;

	if( (hmd_check_pose_calibration_[0] == true) && (still_pose_cali_flag_ == false) )
	{
		// hmd_still_cali_lhand_pos_ = hmd_lhand_pose_.translation() - hmd_chest_pose_.translation();
		// hmd_still_cali_rhand_pos_ = hmd_rhand_pose_.translation() - hmd_chest_pose_.translation();
		hmd_still_cali_lhand_pos_ = hmd_lhand_pose_.translation();
		hmd_still_cali_rhand_pos_ = hmd_rhand_pose_.translation();

		// hmd_still_cali_lhand_pos_ <<  -0.13517, 0.289845, -0.259223;
		// hmd_still_cali_rhand_pos_ <<   -0.0650479, -0.324795, -0.255538; 

		cout<<"hmd_still_cali_lhand_pos_: "<<hmd_still_cali_lhand_pos_<<endl;
		cout<<"hmd_still_cali_rhand_pos_: "<<hmd_still_cali_rhand_pos_<<endl;

		calibration_log_file_[0] << hmd_still_cali_lhand_pos_ <<endl;
		calibration_log_file_[0] << hmd_still_cali_rhand_pos_ <<endl;

		hmd_head_pose_init_ = hmd_head_pose_;   
		hmd_lupperarm_pose_init_ = hmd_lupperarm_pose_;
    	hmd_lhand_pose_init_ = hmd_lhand_pose_;
    	hmd_rshoulder_pose_init_ = hmd_rshoulder_pose_;
		hmd_rupperarm_pose_init_ = hmd_rupperarm_pose_;
    	hmd_rhand_pose_init_ = hmd_rhand_pose_;
    	hmd_pelv_pose_init_ = hmd_pelv_pose_;
		hmd_chest_pose_init_ = hmd_chest_pose_;


		// hmd_head_pose_init_ 	<<  -0.13517, 0.289845, -0.259223;
		// hmd_lupperarm_pose_init_<<  -0.13517, 0.289845, -0.259223;
		// hmd_lhand_pose_init_ 	<<  -0.13517, 0.289845, -0.259223;
		// hmd_rshoulder_pose_init_<<  -0.13517, 0.289845, -0.259223;
		// hmd_rupperarm_pose_init_<<  -0.13517, 0.289845, -0.259223;
		// hmd_rhand_pose_init_ 	<<  -0.13517, 0.289845, -0.259223;
		// hmd_pelv_pose_init_ 	<<  -0.13517, 0.289845, -0.259223;
		// hmd_chest_pose_init_ 	<<  -0.13517, 0.289845, -0.259223;

		cout<<"hmd_head_pose_init_: " <<hmd_head_pose_init_.translation()<<endl;
		cout<<"hmd_lupperarm_pose_init_: "<<hmd_lupperarm_pose_init_.translation()<<endl;
		cout<<"hmd_lhand_pose_init_: "<<hmd_lhand_pose_init_.translation()<<endl;
		cout<<"hmd_rshoulder_pose_init_: "<<hmd_rshoulder_pose_init_.translation()<<endl;
		cout<<"hmd_rupperarm_pose_init_: "<<hmd_rupperarm_pose_init_.translation()<<endl;
		cout<<"hmd_rhand_pose_init_: "<<hmd_rhand_pose_init_.translation()<<endl;
		cout<<"hmd_pelv_pose_init_: " <<hmd_pelv_pose_init_.translation()<<endl;
		cout<<"hmd_chest_pose_init_: " <<hmd_chest_pose_init_.translation()<<endl;

		calibration_log_file_[3] << hmd_head_pose_init_.translation()<< "\n" << hmd_head_pose_init_.linear() << endl;
		calibration_log_file_[3] << hmd_lupperarm_pose_init_.translation()<< "\n" << hmd_lupperarm_pose_init_.linear() << endl;
		calibration_log_file_[3] << hmd_lhand_pose_init_.translation()<< "\n" << hmd_lhand_pose_init_.linear() << endl;
		calibration_log_file_[3] << hmd_rshoulder_pose_init_.translation()<< "\n" << hmd_rshoulder_pose_init_.linear() << endl;
		calibration_log_file_[3] << hmd_rupperarm_pose_init_.translation()<< "\n" << hmd_rupperarm_pose_init_.linear() << endl;
		calibration_log_file_[3] << hmd_rhand_pose_init_.translation()<< "\n" << hmd_rhand_pose_init_.linear() << endl;
		calibration_log_file_[3] << hmd_pelv_pose_init_.translation()<< "\n" << hmd_pelv_pose_init_.linear() << endl;
		calibration_log_file_[3] << hmd_chest_pose_init_.translation()<< "\n" << hmd_chest_pose_init_.linear() << endl;
		calibration_log_file_[3].close();
		still_pose_cali_flag_ = true;
	}
	
	if( (hmd_check_pose_calibration_[1] == true) && (t_pose_cali_flag_ == false) )
	{
		// hmd_tpose_cali_lhand_pos_ = hmd_lhand_pose_.translation() - hmd_chest_pose_.translation();
		// hmd_tpose_cali_rhand_pos_ = hmd_rhand_pose_.translation() - hmd_chest_pose_.translation();
		hmd_tpose_cali_lhand_pos_ = hmd_lhand_pose_.translation();
		hmd_tpose_cali_rhand_pos_ = hmd_rhand_pose_.translation();
		//20210209
		// 		hmd_tpose_cali_lhand_pos_: -0.284813
		//  0.803326
		//  0.356484
		// hmd_tpose_cali_rhand_pos_: -0.146704
		// -0.806712
		//  0.357469
		hmd_tpose_cali_lhand_pos_ <<  -0.284813, 0.803326, 0.356484;
		hmd_tpose_cali_rhand_pos_ <<  -0.146704, -0.806712, 0.357469;
		
		cout<<"hmd_tpose_cali_lhand_pos_: "<<hmd_tpose_cali_lhand_pos_<<endl;
		cout<<"hmd_tpose_cali_rhand_pos_: "<<hmd_tpose_cali_rhand_pos_<<endl;

		calibration_log_file_[1] << hmd_tpose_cali_lhand_pos_ <<endl;
		calibration_log_file_[1] << hmd_tpose_cali_rhand_pos_ <<endl;
		calibration_log_file_[1].close();
		t_pose_cali_flag_ = true;
	}

	if( (hmd_check_pose_calibration_[2] == true) && (forward_pose_cali_flag_ == false) )
	{
		// hmd_forward_cali_lhand_pos_ = hmd_lhand_pose_.translation() - hmd_chest_pose_.translation();
		// hmd_forward_cali_rhand_pos_ = hmd_rhand_pose_.translation() - hmd_chest_pose_.translation();
		hmd_forward_cali_lhand_pos_ = hmd_lhand_pose_.translation();
		hmd_forward_cali_rhand_pos_ = hmd_rhand_pose_.translation();

		//20210209

		// 0.430654
		// 0.263043
		//  0.36878
		// hmd_forward_cali_rhand_pos_:  0.463099
		// -0.198173
		//  0.373117

		hmd_forward_cali_lhand_pos_ <<  0.430654, 0.263043, 0.36878;
		hmd_forward_cali_rhand_pos_ <<  0.463099, -0.198173, 0.373117;
		
		cout<<"hmd_forward_cali_lhand_pos_: "<<hmd_forward_cali_lhand_pos_<<endl;
		cout<<"hmd_forward_cali_rhand_pos_: "<<hmd_forward_cali_rhand_pos_<<endl;

		calibration_log_file_[2] << hmd_forward_cali_lhand_pos_ <<endl;
		calibration_log_file_[2] << hmd_forward_cali_rhand_pos_ <<endl;
		calibration_log_file_[2].close();
		forward_pose_cali_flag_ = true;
	}

	if( (hmd_check_pose_calibration_[4] == true) && (read_cali_log_flag_ == false) )
	{
		//////////read calibration log file/////////////////
		calibration_log_file_[0].open("/home/dg/data/vive_tracker/calibration_log/kaleem/still_pose_.txt");
		calibration_log_file_[1].open("/home/dg/data/vive_tracker/calibration_log/kaleem/t_pose_pose_.txt");
		calibration_log_file_[2].open("/home/dg/data/vive_tracker/calibration_log/kaleem/forward_pose_.txt");		
		calibration_log_file_[3].open("/home/dg/data/vive_tracker/calibration_log/kaleem/hmd_pose_init_.txt");		

		if(calibration_log_file_[0].is_open())
		{
			getTranslationDataFromText(calibration_log_file_[0], hmd_still_cali_lhand_pos_);
			getTranslationDataFromText(calibration_log_file_[0], hmd_still_cali_lhand_pos_);
			cout<<"Still Pose is Uploaded: ("<< hmd_still_cali_lhand_pos_<<"), ("<<hmd_still_cali_rhand_pos_<<")"<<endl;
			calibration_log_file_[0].close();
		}

		if(calibration_log_file_[1].is_open())
		{
			getTranslationDataFromText(calibration_log_file_[1], hmd_tpose_cali_lhand_pos_);
			getTranslationDataFromText(calibration_log_file_[1], hmd_tpose_cali_rhand_pos_);
			cout<<"T Pose is Uploaded: ("<< hmd_tpose_cali_lhand_pos_<<"), ("<<hmd_tpose_cali_rhand_pos_<<")"<<endl;
			calibration_log_file_[1].close();
		}

		if(calibration_log_file_[2].is_open())
		{
			getTranslationDataFromText(calibration_log_file_[2], hmd_forward_cali_lhand_pos_);
			getTranslationDataFromText(calibration_log_file_[2], hmd_forward_cali_rhand_pos_);			
			cout<<"Forward Pose is Uploaded: ("<< hmd_forward_cali_lhand_pos_<<"), ("<<hmd_forward_cali_rhand_pos_<<")"<<endl;
			calibration_log_file_[2].close();
		}

		if(calibration_log_file_[3].is_open())
		{
			getIsometry3dDataFromText(calibration_log_file_[3], hmd_head_pose_init_);
			getIsometry3dDataFromText(calibration_log_file_[3], hmd_lupperarm_pose_init_);
			getIsometry3dDataFromText(calibration_log_file_[3], hmd_lhand_pose_init_);
			getIsometry3dDataFromText(calibration_log_file_[3], hmd_rshoulder_pose_init_);
			getIsometry3dDataFromText(calibration_log_file_[3], hmd_rupperarm_pose_init_);
			getIsometry3dDataFromText(calibration_log_file_[3], hmd_rhand_pose_init_);
			getIsometry3dDataFromText(calibration_log_file_[3], hmd_pelv_pose_init_);
			getIsometry3dDataFromText(calibration_log_file_[3], hmd_chest_pose_init_);

			cout<<"hmd_head_pose_init_: " <<hmd_head_pose_init_.translation()<<endl;
			cout<<"hmd_lupperarm_pose_init_: "<<hmd_lupperarm_pose_init_.translation()<<endl;
			cout<<"hmd_lhand_pose_init_: "<<hmd_lhand_pose_init_.translation()<<endl;
			cout<<"hmd_rshoulder_pose_init_: "<<hmd_rshoulder_pose_init_.translation()<<endl;
			cout<<"hmd_rupperarm_pose_init_: "<<hmd_rupperarm_pose_init_.translation()<<endl;
			cout<<"hmd_rhand_pose_init_: "<<hmd_rhand_pose_init_.translation()<<endl;
			cout<<"hmd_pelv_pose_init_: " <<hmd_pelv_pose_init_.translation()<<endl;
			cout<<"hmd_chest_pose_init_: " <<hmd_chest_pose_init_.translation()<<endl;
		}

		// hmd_still_cali_lhand_pos_
		// hmd_still_cali_rhand_pos_
		
		// hmd_head_pose_init_ 	
		// hmd_lupperarm_pose_init_
		// hmd_lhand_pose_init_ 	
		// hmd_rshoulder_pose_init_
		// hmd_rupperarm_pose_init_
		// hmd_rhand_pose_init_ 	
		// hmd_pelv_pose_init_ 	
		// hmd_chest_pose_init_ 	

		// hmd_tpose_cali_lhand_pos_
		// hmd_tpose_cali_rhand_pos_

		// hmd_forward_cali_lhand_pos_
		// hmd_forward_cali_rhand_pos_	

		read_cali_log_flag_ = true;
	}

	if( (hmd_check_pose_calibration_[3] == false) && (still_pose_cali_flag_*t_pose_cali_flag_*forward_pose_cali_flag_ == true) )
	{
		hmd_lshoulder_center_pos_(0) = (hmd_still_cali_lhand_pos_(0) + hmd_tpose_cali_lhand_pos_(0))/2;
		// hmd_lshoulder_center_pos_(1) = (hmd_still_cali_lhand_pos_(1) + hmd_forward_cali_lhand_pos_(1))/2;
		hmd_lshoulder_center_pos_(1) = hmd_still_cali_lhand_pos_(1);
		hmd_lshoulder_center_pos_(2) = (hmd_tpose_cali_lhand_pos_(2) + hmd_forward_cali_lhand_pos_(2))/2;
		
		hmd_rshoulder_center_pos_(0) = (hmd_still_cali_rhand_pos_(0) + hmd_tpose_cali_rhand_pos_(0))/2;
		// hmd_rshoulder_center_pos_(1) = (hmd_still_cali_rhand_pos_(1) + hmd_forward_cali_rhand_pos_(1))/2;
		hmd_rshoulder_center_pos_(1) = hmd_still_cali_rhand_pos_(1);
		hmd_rshoulder_center_pos_(2) = (hmd_tpose_cali_rhand_pos_(2) + hmd_forward_cali_rhand_pos_(2))/2;

		hmd_larm_max_l_ = 0;
		hmd_larm_max_l_ += (hmd_lshoulder_center_pos_ - hmd_still_cali_lhand_pos_).norm();
		hmd_larm_max_l_ += (hmd_lshoulder_center_pos_ - hmd_tpose_cali_lhand_pos_).norm();
		hmd_larm_max_l_ += (hmd_lshoulder_center_pos_ - hmd_forward_cali_lhand_pos_).norm();
		hmd_larm_max_l_ /= 3;
		// hmd_larm_max_l_ = 0.54;

		hmd_rarm_max_l_ = 0;
		hmd_rarm_max_l_ += (hmd_rshoulder_center_pos_ - hmd_still_cali_rhand_pos_).norm();
		hmd_rarm_max_l_ += (hmd_rshoulder_center_pos_ - hmd_tpose_cali_rhand_pos_).norm();
		hmd_rarm_max_l_ += (hmd_rshoulder_center_pos_ - hmd_forward_cali_rhand_pos_).norm();
		hmd_rarm_max_l_ /= 3;

		hmd_shoulder_width_ = (hmd_lshoulder_center_pos_ - hmd_rshoulder_center_pos_).norm();
		// hmd_shoulder_width_ = 0.521045;

		// hmd_rarm_max_l_ = 0.54;

		cout<<"hmd_lshoulder_center_pos_: "<<hmd_lshoulder_center_pos_<<endl;
		cout<<"hmd_rshoulder_center_pos_: "<<hmd_rshoulder_center_pos_<<endl;
		cout<<"hmd_larm_max_l_: "<<hmd_larm_max_l_<<endl;
		cout<<"hmd_rarm_max_l_: "<<hmd_rarm_max_l_<<endl;
		cout<<"hmd_shoulder_width_: "<<hmd_shoulder_width_<<endl;
		hmd_check_pose_calibration_[3] = true;

		hmd_chest_2_lshoulder_center_pos_ = hmd_lshoulder_center_pos_ - hmd_chest_pose_.translation();
		hmd_chest_2_rshoulder_center_pos_ = hmd_rshoulder_center_pos_ - hmd_chest_pose_.translation();

    	hmd_lshoulder_pose_init_.translation() = hmd_chest_pose_.linear()*hmd_chest_pose_init_.linear().transpose()*hmd_chest_2_lshoulder_center_pos_ + hmd_chest_pose_.translation();
		hmd_lshoulder_pose_init_.linear() = hmd_chest_pose_init_.linear();
    	hmd_rshoulder_pose_init_.translation() = hmd_chest_pose_.linear()*hmd_chest_pose_init_.linear().transpose()*hmd_chest_2_rshoulder_center_pos_ + hmd_chest_pose_.translation();
		hmd_rshoulder_pose_init_.linear() = hmd_chest_pose_init_.linear();
	}
}

void CustomController::getTranslationDataFromText(std::fstream &text_file, Eigen::Vector3d &trans)
{
	for(int i = 0; i<3; i++)
	{
		string data;  
    	text_file >> data;
    	trans(i) = atof(data.c_str()); 
	}
}
void CustomController::getMatrix3dDataFromText(std::fstream &text_file, Eigen::Matrix3d &mat)
{
	for(int i = 0; i<3; i++)
	{
		for(int j = 0; j<3; j++)
		{
			string data;  
    		text_file >> data;
    		mat(i, j) = atof(data.c_str()); 
		}
	}
}
void CustomController::getIsometry3dDataFromText(std::fstream &text_file, Eigen::Isometry3d &isom)
{
	Vector3d trans;
	Matrix3d mat;
	getTranslationDataFromText(text_file, trans);
	getMatrix3dDataFromText(text_file, mat);
	isom.translation() = trans;
	isom.linear() = mat;
}

void CustomController::rawMasterPoseProcessing()
{
	if(upperbody_mode_recieved_ == true)
	{
		upperbody_command_time_ = current_time_;
		master_lhand_pose_ = lhand_transform_current_from_global_;
		master_lhand_pose_pre_ = lhand_transform_current_from_global_;
		master_rhand_pose_ = rhand_transform_current_from_global_;
		master_rhand_pose_pre_ = rhand_transform_current_from_global_;

		master_lelbow_pose_pre_ = lelbow_transform_current_from_global_;
		master_relbow_pose_pre_ = relbow_transform_current_from_global_;

		master_lshoulder_pose_pre_ = lshoulder_transform_current_from_global_;
		master_rshoulder_pose_pre_ = rshoulder_transform_current_from_global_;
		master_head_pose_pre_ 	= head_transform_current_from_global_;
		master_upperbody_pose_pre_ 	= upperbody_transform_current_from_global_;

		master_relative_lhand_pos_pre_ = lhand_transform_current_from_global_.translation() - rhand_transform_current_from_global_.translation();
		master_relative_rhand_pos_pre_ = rhand_transform_current_from_global_.translation() - lhand_transform_current_from_global_.translation();

		upperbody_mode_recieved_ = false;

		// exo_suit_init_pose_calibration_ = true;
		// azure_kinect_init_pose_calibration_ = true;
		hmd_init_pose_calibration_ = true;

		// exo_larm_max_l_ = (exo_suit_lshoulder_pos_raw_ - exo_suit_lhand_pos_raw_).norm();
		// exo_rarm_max_l_ = (exo_suit_rshoulder_pos_raw_ - exo_suit_rhand_pos_raw_).norm();
		// exo_lelbow_max_l_ = (exo_suit_lshoulder_pos_raw_ - exo_suit_lupperarm_pos_raw_).norm();
		// exo_relbow_max_l_ = (exo_suit_rshoulder_pos_raw_ - exo_suit_rupperarm_pos_raw_).norm();		

		// kinect_larm_max_l_ = 	(azure_kinect_lshoulder_pos_raw_ - azure_kinect_lhand_pos_raw_).norm();
		// kinect_rarm_max_l_ = 	(azure_kinect_rshoulder_pos_raw_ - azure_kinect_rhand_pos_raw_).norm();
		// kinect_lelbow_max_l_ = 	(azure_kinect_lshoulder_pos_raw_ - azure_kinect_lelbow_pos_raw_).norm();
		// kinect_relbow_max_l_ = 	(azure_kinect_rshoulder_pos_raw_ - azure_kinect_relbow_pos_raw_).norm();
		// kinect_shoulder_width_ =(azure_kinect_lshoulder_pos_raw_ - azure_kinect_rshoulder_pos_raw_).norm();

		// hmd_shoulder_width_ = (hmd_lupperarm_pose_.translation() - hmd_rupperarm_pose_.translation()).norm();
	}

	// exoSuitRawDataProcessing();
	// azureKinectRawDataProcessing();
	hmdRawDataProcessing();
	double fc_filter = 3; //hz

	for(int i=0; i<3; i++)
	{
		master_lhand_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lhand_transform_pre_desired_from_.translation()(i), 0, 0, master_lhand_pose_raw_.translation()(i), 0, 0)(0);
		master_rhand_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, rhand_transform_pre_desired_from_.translation()(i), 0, 0, master_rhand_pose_raw_.translation()(i), 0, 0)(0);

		master_lelbow_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lupperarm_transform_pre_desired_from_.translation()(i), 0, 0, master_lelbow_pose_raw_.translation()(i), 0, 0)(0);
		master_relbow_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, rupperarm_transform_pre_desired_from_.translation()(i), 0, 0, master_relbow_pose_raw_.translation()(i), 0, 0)(0);

		master_lshoulder_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lacromion_transform_pre_desired_from_.translation()(i), 0, 0, master_lshoulder_pose_raw_.translation()(i), 0, 0)(0);
		master_rshoulder_pose_raw_.translation()(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, racromion_transform_pre_desired_from_.translation()(i), 0, 0, master_rshoulder_pose_raw_.translation()(i), 0, 0)(0);

		master_relative_lhand_pos_raw_(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lhand_transform_pre_desired_from_.translation()(i) - rhand_transform_pre_desired_from_.translation()(i), 0, 0, master_relative_lhand_pos_raw_(i), 0, 0)(0);
		master_relative_rhand_pos_raw_(i) = DyrosMath::QuinticSpline(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, rhand_transform_pre_desired_from_.translation()(i) - lhand_transform_pre_desired_from_.translation()(i), 0, 0, master_relative_rhand_pos_raw_(i), 0, 0)(0);
	}
	
	master_lhand_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lhand_transform_pre_desired_from_.linear(), master_lhand_pose_raw_.linear());
	master_rhand_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, rhand_transform_pre_desired_from_.linear(), master_rhand_pose_raw_.linear());
	master_lelbow_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lupperarm_transform_pre_desired_from_.linear(), master_lelbow_pose_raw_.linear());
	master_relbow_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, rupperarm_transform_pre_desired_from_.linear(), master_relbow_pose_raw_.linear());
	master_lshoulder_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, lacromion_transform_pre_desired_from_.linear(), master_lshoulder_pose_raw_.linear());
	master_rshoulder_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, racromion_transform_pre_desired_from_.linear(), master_rshoulder_pose_raw_.linear());
	master_head_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, head_transform_pre_desired_from_.linear(), master_head_pose_raw_.linear());
	master_upperbody_pose_raw_.linear() = DyrosMath::rotationCubic(current_time_, upperbody_command_time_, upperbody_command_time_ + 5, upperbody_transform_pre_desired_from_.linear(), master_upperbody_pose_raw_.linear());

	master_lhand_pose_.translation() = DyrosMath::lpf<3>(master_lhand_pose_raw_.translation(), master_lhand_pose_pre_.translation(), 2000, fc_filter);
	master_rhand_pose_.translation() = DyrosMath::lpf<3>(master_rhand_pose_raw_.translation(), master_rhand_pose_pre_.translation(), 2000, fc_filter);
	master_lelbow_pose_.translation() = DyrosMath::lpf<3>(master_lelbow_pose_raw_.translation(), master_lelbow_pose_pre_.translation(), 2000, fc_filter);
	master_relbow_pose_.translation() = DyrosMath::lpf<3>(master_relbow_pose_raw_.translation(), master_relbow_pose_pre_.translation(), 2000, fc_filter);
	master_lshoulder_pose_.translation() = DyrosMath::lpf<3>(master_lshoulder_pose_raw_.translation(), master_lshoulder_pose_pre_.translation(), 2000, fc_filter);
	master_rshoulder_pose_.translation() = DyrosMath::lpf<3>(master_rshoulder_pose_raw_.translation(), master_rshoulder_pose_pre_.translation(), 2000, fc_filter);
	master_head_pose_.translation() = DyrosMath::lpf<3>(master_head_pose_raw_.translation(), master_head_pose_pre_.translation(), 2000, fc_filter);
	master_upperbody_pose_.translation() = DyrosMath::lpf<3>(master_upperbody_pose_raw_.translation(), master_upperbody_pose_pre_.translation(), 2000, fc_filter);

	master_relative_lhand_pos_ = DyrosMath::lpf<3>(master_relative_lhand_pos_raw_, master_relative_lhand_pos_pre_, 2000, fc_filter);
 	master_relative_rhand_pos_ = DyrosMath::lpf<3>(master_relative_rhand_pos_raw_, master_relative_rhand_pos_pre_, 2000, fc_filter);

	Eigen::AngleAxisd lhand_ang_diff(master_lhand_pose_raw_.linear()*master_lhand_pose_pre_.linear().transpose());
	Eigen::AngleAxisd rhand_ang_diff(master_rhand_pose_raw_.linear()*master_rhand_pose_pre_.linear().transpose());
	Eigen::AngleAxisd lelbow_ang_diff(master_lelbow_pose_raw_.linear()*master_lelbow_pose_pre_.linear().transpose());
	Eigen::AngleAxisd relbow_ang_diff(master_relbow_pose_raw_.linear()*master_relbow_pose_pre_.linear().transpose());
	Eigen::AngleAxisd lshoulder_ang_diff(master_lshoulder_pose_raw_.linear()*master_lshoulder_pose_pre_.linear().transpose());
	Eigen::AngleAxisd rshoulder_ang_diff(master_rshoulder_pose_raw_.linear()*master_rshoulder_pose_pre_.linear().transpose());
	Eigen::AngleAxisd head_ang_diff(master_head_pose_raw_.linear()*master_head_pose_pre_.linear().transpose());
	Eigen::AngleAxisd upperbody_ang_diff(master_upperbody_pose_raw_.linear()*master_upperbody_pose_pre_.linear().transpose());

	Eigen::Matrix3d lhand_diff_m, rhand_diff_m, lelbow_diff_m, relbow_diff_m, lshoulder_diff_m, rshoulder_diff_m, head_diff_m, upperbody_diff_m;
	double temp_ang_diff_filtered;
	// lhand_ang_diff.angle() = DyrosMath::lpf(lhand_ang_diff.angle(), 0, 2000, fc_filter);
	// lhand_diff_m = Eigen::AngleAxisd( temp_ang_diff_filtered, fc_filter), lhand_ang_diff.axis());
	lhand_diff_m = Eigen::AngleAxisd( DyrosMath::lpf(lhand_ang_diff.angle(), 0, 2000, fc_filter), lhand_ang_diff.axis()); 
	rhand_diff_m = Eigen::AngleAxisd( DyrosMath::lpf(rhand_ang_diff.angle(), 0, 2000, fc_filter), rhand_ang_diff.axis());
	lelbow_diff_m = Eigen::AngleAxisd( DyrosMath::lpf(lelbow_ang_diff.angle(), 0, 2000, fc_filter), lelbow_ang_diff.axis());
	relbow_diff_m = Eigen::AngleAxisd( DyrosMath::lpf(relbow_ang_diff.angle(), 0, 2000, fc_filter), relbow_ang_diff.axis());
	lshoulder_diff_m = Eigen::AngleAxisd( DyrosMath::lpf(lshoulder_ang_diff.angle(), 0, 2000, fc_filter), lshoulder_ang_diff.axis());
	rshoulder_diff_m = Eigen::AngleAxisd( DyrosMath::lpf(rshoulder_ang_diff.angle(), 0, 2000, fc_filter), rshoulder_ang_diff.axis());
	head_diff_m = Eigen::AngleAxisd( DyrosMath::lpf(head_ang_diff.angle(), 0, 2000, fc_filter), head_ang_diff.axis());
	upperbody_diff_m = Eigen::AngleAxisd( DyrosMath::lpf(upperbody_ang_diff.angle(), 0, 2000, fc_filter), upperbody_ang_diff.axis());

	master_lhand_pose_.linear() = lhand_diff_m*master_lhand_pose_pre_.linear();
	master_rhand_pose_.linear() = rhand_diff_m*master_rhand_pose_pre_.linear();
	master_lelbow_pose_.linear() = lelbow_diff_m*master_lelbow_pose_pre_.linear();
	master_relbow_pose_.linear() = relbow_diff_m*master_relbow_pose_pre_.linear();
	master_lshoulder_pose_.linear() = lshoulder_diff_m*master_lshoulder_pose_pre_.linear();
	master_rshoulder_pose_.linear() = rshoulder_diff_m*master_rshoulder_pose_pre_.linear();
	master_head_pose_.linear() = head_diff_m*master_head_pose_pre_.linear();
	master_upperbody_pose_.linear() = upperbody_diff_m*master_upperbody_pose_pre_.linear();
	
	// for print
	master_lhand_rqy_ = DyrosMath::rot2Euler_tf(master_lhand_pose_.linear());
	master_rhand_rqy_ = DyrosMath::rot2Euler_tf(master_rhand_pose_.linear());

	master_lelbow_rqy_ = DyrosMath::rot2Euler_tf(master_lelbow_pose_.linear());
	master_relbow_rqy_ = DyrosMath::rot2Euler_tf(master_relbow_pose_.linear());

	master_lshoulder_rqy_ = DyrosMath::rot2Euler_tf(master_lshoulder_pose_.linear());
	master_rshoulder_rqy_ = DyrosMath::rot2Euler_tf(master_rshoulder_pose_.linear());

	master_head_rqy_ = DyrosMath::rot2Euler_tf(master_head_pose_.linear());

	// if( int(current_time_*10000)%1000 == 0)
	// {
	// 	cout<<"master_lelbow_rqy_: "<<master_lelbow_rqy_<<endl;
	// 	cout<<"master_relbow_rqy_: "<<master_relbow_rqy_<<endl;
	// }
	// master_lhand_pose_.linear() = lhand_transform_init_from_global_.linear();
	// master_rhand_pose_.linear() = rhand_transform_init_from_global_.linear();

	// double arm_len_max = 0.95;

	// if( master_lhand_pose_.translation().norm() > arm_len_max)
	// {
	// 	master_lhand_pose_.translation() = master_lhand_pose_.translation().normalized() * arm_len_max;
	// }

	// if( master_rhand_pose_.translation().norm() > arm_len_max)
	// {
	// 	master_rhand_pose_.translation() = master_rhand_pose_.translation().normalized() * arm_len_max;
	// }

	// for(int i = 0; i<3; i++)
	// {
	// 	master_lhand_vel_(i) = (master_lhand_pose_.translation()(i) - master_lhand_pose_pre_.translation()(i))/dt_;
	// 	master_rhand_vel_(i) = (master_rhand_pose_.translation()(i) - master_rhand_pose_pre_.translation()(i))/dt_;
	// }
	

	master_lhand_vel_.setZero();
	master_rhand_vel_.setZero();

	master_lelbow_vel_.setZero();
	master_relbow_vel_.setZero();

	master_lshoulder_vel_.setZero();
	master_rshoulder_vel_.setZero();

	master_head_vel_.setZero();
	master_upperbody_vel_.setZero();
}

void CustomController::exoSuitRawDataProcessing()
{
	// position
	exo_suit_head_pose_.translation() = exo_suit_head_pos_raw_;
	exo_suit_lshoulder_pose_.translation() = exo_suit_lshoulder_pos_raw_;
	exo_suit_lupperarm_pose_.translation() = exo_suit_lupperarm_pos_raw_;
	exo_suit_llowerarm_pose_.translation() = exo_suit_llowerarm_pos_raw_;
	exo_suit_lhand_pose_.translation() = exo_suit_lhand_pos_raw_;
	exo_suit_rshoulder_pose_.translation() = exo_suit_rshoulder_pos_raw_;
	exo_suit_rupperarm_pose_.translation() = exo_suit_rupperarm_pos_raw_;
	exo_suit_rlowerarm_pose_.translation() = exo_suit_rlowerarm_pos_raw_;
	exo_suit_rhand_pose_.translation() = exo_suit_rhand_pos_raw_;
	exo_suit_pelv_pose_.translation() = exo_suit_pelv_pos_raw_;
	exo_suit_upperbody_pose_.translation() = exo_suit_upperbody_pos_raw_;
	// //orienation


	exo_suit_head_pose_.linear() = 	exo_suit_head_q_raw_.normalized().toRotationMatrix();

	exo_suit_lshoulder_pose_.linear() = 	exo_suit_lshoulder_q_raw_.normalized().toRotationMatrix();
	exo_suit_lupperarm_pose_.linear() = 	exo_suit_lupperarm_q_raw_.normalized().toRotationMatrix();
	exo_suit_llowerarm_pose_.linear() = 	exo_suit_llowerarm_q_raw_.normalized().toRotationMatrix();
	exo_suit_lhand_pose_.linear() = 	exo_suit_lhand_q_raw_.normalized().toRotationMatrix();

	exo_suit_rshoulder_pose_.linear() = 	exo_suit_rshoulder_q_raw_.normalized().toRotationMatrix();
	exo_suit_rupperarm_pose_.linear() = 	exo_suit_rupperarm_q_raw_.normalized().toRotationMatrix();	// exo_suit_rlowerarm_pose_.linear().block(0, 0, 3, 1) = 	exo_suit_rlowerarm_q_raw_.normalized().toRotationMatrix().block(0, 2, 3, 1);	// exo_suit_rlowerarm_pose_.linear().block(0, 1, 3, 1) = -	exo_suit_rlowerarm_q_raw_.normalized().toRotationMatrix().block(0, 0, 3, 1);
	exo_suit_rlowerarm_pose_.linear() = 	exo_suit_rlowerarm_q_raw_.normalized().toRotationMatrix();
	exo_suit_rhand_pose_.linear() = 	exo_suit_rhand_q_raw_.normalized().toRotationMatrix();

	exo_suit_upperbody_pose_.linear() = 	exo_suit_upperbody_q_raw_.normalized().toRotationMatrix();

	exo_suit_pelv_pose_.linear().setIdentity();

	// if( int(current_time_*1e4)%int(1e3) == 0)
	// {
	// 	cout<<"lhand ori mat before conversion: \n"<<exo_suit_lhand_pose_.linear()<<endl;
	// 	cout<<"lupperarm ori mat before conversion: \n"<<exo_suit_lupperarm_pose_.linear()<<endl;
	// 	cout<<"lelbow_transform_current_from_global_.linear(): \n"<<lelbow_transform_current_from_global_.linear()<<endl;
	// 	cout<<"master_lhand_pose_.linear(): \n"<<master_lhand_pose_.linear()<<endl;
	// 	cout<<"master_lelbow_pose_.linear(): \n"<<master_lelbow_pose_.linear()<<endl;

	// 	cout<<"rhand ori mat before conversion: \n"<<exo_suit_rhand_pose_.linear()<<endl;
	// 	cout<<"rupperarm ori mat before conversion: \n"<<exo_suit_rupperarm_pose_.linear()<<endl;
	// 	cout<<"relbow_transform_current_from_global_.linear(): \n"<<relbow_transform_current_from_global_.linear()<<endl;

	// 	cout<<"pelvis ori mat before conversion: \n"<<exo_suit_pelv_pose_.linear()<<endl;
	// }

	//coordinate conversion
	exo_suit_head_pose_ = exo_suit_pelv_pose_.inverse()*exo_suit_head_pose_;
	exo_suit_lshoulder_pose_ = exo_suit_pelv_pose_.inverse()*exo_suit_lshoulder_pose_;
	exo_suit_lupperarm_pose_ = exo_suit_pelv_pose_.inverse()*exo_suit_lupperarm_pose_;
	exo_suit_llowerarm_pose_ = exo_suit_pelv_pose_.inverse()*exo_suit_llowerarm_pose_;
	exo_suit_lhand_pose_ = exo_suit_pelv_pose_.inverse()*exo_suit_lhand_pose_;
	exo_suit_rshoulder_pose_ = exo_suit_pelv_pose_.inverse()*exo_suit_rshoulder_pose_;
	exo_suit_rupperarm_pose_ = exo_suit_pelv_pose_.inverse()*exo_suit_rupperarm_pose_;
	exo_suit_rlowerarm_pose_ = exo_suit_pelv_pose_.inverse()*exo_suit_rlowerarm_pose_;
	exo_suit_rhand_pose_ = exo_suit_pelv_pose_.inverse()*exo_suit_rhand_pose_;
	exo_suit_upperbody_pose_ = exo_suit_pelv_pose_.inverse()*exo_suit_upperbody_pose_;

	// get upperbody pose from pelv and shoulder
	// exo_suit_pelv_pose_.linear() = exo_suit_pelv_q_raw_.normalized().toRotationMatrix();
	// Matrix3d lshoulder_rot_m_local = exo_suit_pelv_pose_.linear().inverse()*exo_suit_lshoulder_pose_.linear();
	// Matrix3d rshoulder_rot_m_local = exo_suit_pelv_pose_.linear().inverse()*exo_suit_rshoulder_pose_.linear();
	// AngleAxisd aa_lsh(lshoulder_rot_m_local);
	// AngleAxisd aa_rsh(rshoulder_rot_m_local);

	// Vector3d lshoulder_aa_local = aa_lsh.axis()*aa_lsh.angle();
	// Vector3d rshoulder_aa_local = aa_rsh.axis()*aa_rsh.angle();
	// Vector3d upperbody_aa_local = (lshoulder_aa_local + rshoulder_aa_local)/2;
	// Matrix3d upperbody_rob_m;
	// upperbody_rob_m = AngleAxisd(upperbody_aa_local.norm(), upperbody_aa_local.normalized());
	// exo_suit_upperbody_pose_.linear() = upperbody_rob_m;

	if( int(current_time_*1e4)%int(1e3) == 0)
	{
		// cout<<"left hand ori mat: \n"<<exo_suit_lhand_pose_.linear()<<endl;
		// cout<<"left hand pos: \n"<<exo_suit_lhand_pose_.translation()<<endl;
		// cout<<"right hand ori mat: \n"<<exo_suit_rhand_pose_.linear()<<endl;
		// cout<<"right hand pos: \n"<<exo_suit_rhand_pose_.translation()<<endl;
		// cout<<"pelvis ori mat: \n"<<exo_suit_pelv_pose_.linear()<<endl;
		// cout<<"pelvis pos: \n"<<exo_suit_pelv_pose_.translation()<<endl;
		// cout<<"pelvis pos: \n"<<exo_suit_pelv_pose_.translation()<<endl;
		// cout<<"right hand ori mat: \n"<<rhand_transform_current_from_global_.linear()<<endl;

		// cout<<"lower arm length: \n"<< (exo_suit_llowerarm_pose_.translation() - exo_suit_lhand_pose_.translation()).norm() <<endl;
		// cout<<"upper arm length: \n"<< (exo_suit_lshoulder_pose_.translation() - exo_suit_lupperarm_pose_.translation()).norm() <<endl;
		// cout<<"elbow length: \n"<< (exo_suit_lupperarm_pose_.translation() - exo_suit_llowerarm_pose_.translation()).norm() <<endl;
	}

	// exo2robot_lhand_pos_mapping_ = exo_suit_lhand_pose_.translation() - exo_suit_lshoulder_pose_.translation();
	// exo2robot_lhand_pos_mapping_ = (robot_arm_max_l_)/(exo_larm_max_l_)*exo2robot_lhand_pos_mapping_;
		
	// exo2robot_rhand_pos_mapping_ = exo_suit_rhand_pose_.translation() - exo_suit_rshoulder_pose_.translation();
	// exo2robot_rhand_pos_mapping_ = (robot_arm_max_l_)/(exo_rarm_max_l_)*exo2robot_rhand_pos_mapping_;

	exo2robot_lelbow_pos_mapping_ = exo_suit_lupperarm_pose_.translation() - exo_suit_lshoulder_pose_.translation();
	exo2robot_lelbow_pos_mapping_ = (robot_upperarm_max_l_)/(exo_lelbow_max_l_)*exo2robot_lelbow_pos_mapping_;
		
	exo2robot_relbow_pos_mapping_ = exo_suit_rupperarm_pose_.translation() - exo_suit_rshoulder_pose_.translation();
	exo2robot_relbow_pos_mapping_ = (robot_upperarm_max_l_)/(exo_relbow_max_l_)*exo2robot_relbow_pos_mapping_;

	double dist_btw_hands = (exo_suit_lhand_pose_.translation() - exo_suit_rhand_pose_.translation()).norm();
	Eigen::Vector3d unit_vec_shoulder = (exo_suit_lshoulder_pose_.translation() - exo_suit_rshoulder_pose_.translation()).normalized();

	
	// left hand
	if( unit_vec_shoulder.transpose() * exo_suit_lhand_pose_.translation() >= exo_shoulder_width_/2 )
	{
		exo2robot_lhand_pos_mapping_ = exo_suit_lhand_pose_.translation() - exo_suit_lshoulder_pose_.translation();	
		exo2robot_lhand_pos_mapping_ = (robot_arm_max_l_)/(exo_larm_max_l_)*exo2robot_lhand_pos_mapping_;
		exo2robot_lhand_pos_mapping_ -= double(unit_vec_shoulder.transpose() * exo2robot_lhand_pos_mapping_)*unit_vec_shoulder;
		exo2robot_lhand_pos_mapping_ += ( (robot_arm_max_l_)/(exo_larm_max_l_)*(unit_vec_shoulder.transpose()*exo_suit_lhand_pose_.translation() - exo_shoulder_width_/2) + robot_shoulder_width_/2 )*unit_vec_shoulder;
	
	}
	else if ( (unit_vec_shoulder.transpose() * exo_suit_lhand_pose_.translation() >= -exo_shoulder_width_/2)&&(unit_vec_shoulder.transpose() * exo_suit_lhand_pose_.translation() < exo_shoulder_width_/2))
	{
		exo2robot_lhand_pos_mapping_ = exo_suit_lhand_pose_.translation() - exo_suit_lshoulder_pose_.translation();	
		exo2robot_lhand_pos_mapping_ = (robot_arm_max_l_)/(exo_larm_max_l_)*exo2robot_lhand_pos_mapping_;
		exo2robot_lhand_pos_mapping_ -= double(unit_vec_shoulder.transpose() * exo2robot_lhand_pos_mapping_)*unit_vec_shoulder;
		exo2robot_lhand_pos_mapping_ += ( (robot_shoulder_width_)/(exo_shoulder_width_)*( unit_vec_shoulder.transpose()*exo_suit_lhand_pose_.translation()+0) )*unit_vec_shoulder;
	}
	else
	{
		exo2robot_lhand_pos_mapping_ = exo_suit_lhand_pose_.translation() - exo_suit_lshoulder_pose_.translation();	
		exo2robot_lhand_pos_mapping_ = (robot_arm_max_l_)/(exo_larm_max_l_)*exo2robot_lhand_pos_mapping_;
		exo2robot_lhand_pos_mapping_ -= double(unit_vec_shoulder.transpose() * exo2robot_lhand_pos_mapping_)*unit_vec_shoulder;
		exo2robot_lhand_pos_mapping_ += ( (robot_arm_max_l_)/(exo_larm_max_l_)*(unit_vec_shoulder.transpose()*exo_suit_lhand_pose_.translation() + exo_shoulder_width_/2) - robot_shoulder_width_/2 )*unit_vec_shoulder;
	}

	// right hand
	if( unit_vec_shoulder.transpose() * exo_suit_rhand_pose_.translation() >= exo_shoulder_width_/2 )
	{
		exo2robot_rhand_pos_mapping_ = exo_suit_rhand_pose_.translation() - exo_suit_rshoulder_pose_.translation();	
		exo2robot_rhand_pos_mapping_ = (robot_arm_max_l_)/(exo_rarm_max_l_)*exo2robot_rhand_pos_mapping_;
		exo2robot_rhand_pos_mapping_ -= double(unit_vec_shoulder.transpose() * exo2robot_rhand_pos_mapping_)*unit_vec_shoulder;
		exo2robot_rhand_pos_mapping_ += ( (robot_arm_max_l_)/(exo_rarm_max_l_)*(unit_vec_shoulder.transpose()*exo_suit_rhand_pose_.translation() - exo_shoulder_width_/2) + robot_shoulder_width_/2 )*unit_vec_shoulder;
	}
	else if ( (unit_vec_shoulder.transpose() * exo_suit_rhand_pose_.translation() >= -exo_shoulder_width_/2)&&(unit_vec_shoulder.transpose() * exo_suit_rhand_pose_.translation() < exo_shoulder_width_/2))
	{
		exo2robot_rhand_pos_mapping_ = exo_suit_rhand_pose_.translation() - exo_suit_rshoulder_pose_.translation();	
		exo2robot_rhand_pos_mapping_ = (robot_arm_max_l_)/(exo_rarm_max_l_)*exo2robot_rhand_pos_mapping_;
		exo2robot_rhand_pos_mapping_ -= double(unit_vec_shoulder.transpose() * exo2robot_rhand_pos_mapping_)*unit_vec_shoulder;
		exo2robot_rhand_pos_mapping_ += ( (robot_shoulder_width_)/(exo_shoulder_width_)*(unit_vec_shoulder.transpose()*exo_suit_rhand_pose_.translation()+0) )*unit_vec_shoulder;
	}
	else
	{
		exo2robot_rhand_pos_mapping_ = exo_suit_rhand_pose_.translation() - exo_suit_rshoulder_pose_.translation();	
		exo2robot_rhand_pos_mapping_ = (robot_arm_max_l_)/(exo_rarm_max_l_)*exo2robot_rhand_pos_mapping_;
		exo2robot_rhand_pos_mapping_ -= double(unit_vec_shoulder.transpose() * exo2robot_rhand_pos_mapping_)*unit_vec_shoulder;
		exo2robot_rhand_pos_mapping_ += ( (robot_arm_max_l_)/(exo_rarm_max_l_)*(unit_vec_shoulder.transpose()*exo_suit_rhand_pose_.translation() + exo_shoulder_width_/2) - robot_shoulder_width_/2 )*unit_vec_shoulder;
	}

	if(exo_suit_init_pose_calibration_ == true)
	{
		exo_suit_init_pose_calibration_ = false;
		exo_suit_init_pose_cali_time_ = current_time_;

		exo_suit_head_pose_init_ = exo_suit_head_pose_;   
    	exo_suit_lshoulder_pose_init_ = exo_suit_lshoulder_pose_;
		exo_suit_lupperarm_pose_init_ = exo_suit_lupperarm_pose_;
    	exo_suit_llowerarm_pose_init_ = exo_suit_llowerarm_pose_;
    	exo_suit_lhand_pose_init_ = exo_suit_lhand_pose_;
    	exo_suit_rshoulder_pose_init_ = exo_suit_rshoulder_pose_;
		exo_suit_rupperarm_pose_init_ = exo_suit_rupperarm_pose_;
    	exo_suit_rlowerarm_pose_init_ = exo_suit_rlowerarm_pose_;
    	exo_suit_rhand_pose_init_ = exo_suit_rhand_pose_;
    	exo_suit_pelv_pose_init_ = exo_suit_pelv_pose_;
		exo_suit_upperbody_pose_init_ = exo_suit_upperbody_pose_;

		exo2robot_lhand_pos_mapping_init_ = exo2robot_lhand_pos_mapping_;
		exo2robot_rhand_pos_mapping_init_ = exo2robot_rhand_pos_mapping_;
		exo2robot_lelbow_pos_mapping_init_ = exo2robot_lelbow_pos_mapping_;
		exo2robot_relbow_pos_mapping_init_ = exo2robot_relbow_pos_mapping_;
	}

	Vector3d robot_init_hand_pos, robot_init_lshoulder_pos, robot_init_rshoulder_pos, robot_init_elbow_pos, delta_exo2robot_lhand_pos_maping, delta_exo2robot_rhand_pos_maping, delta_exo2robot_lelbow_pos_maping, delta_exo2robot_relbow_pos_maping;
	robot_init_hand_pos << 0, 0, -(robot_arm_max_l_);
	robot_init_lshoulder_pos << 0, 0.1491, 0.065; 
	robot_init_rshoulder_pos << 0, -0.1491, 0.065;
	robot_init_elbow_pos<<0, 0, -(robot_upperarm_max_l_);
	Matrix3d robot_lhand_ori_init, robot_rhand_ori_init, robot_lelbow_ori_init, robot_relbow_ori_init, robot_lshoulder_ori_init, robot_rshoulder_ori_init, robot_head_ori_init, robot_upperbody_ori_init;
	robot_lhand_ori_init = DyrosMath::rotateWithZ(-90*DEG2RAD);
	robot_rhand_ori_init = DyrosMath::rotateWithZ(90*DEG2RAD);
	// robot_lshoulder_ori_init = DyrosMath::rotateWithZ(-0.3);
	robot_lshoulder_ori_init.setIdentity();
	// robot_rshoulder_ori_init = DyrosMath::rotateWithZ(0.3);
	robot_rshoulder_ori_init.setIdentity();
	robot_head_ori_init.setIdentity();
	robot_upperbody_ori_init.setIdentity();

	// robot_lelbow_ori_init << 0, 0, -1, 1, 0, 0, 0, -1, 0;
	robot_lelbow_ori_init.setZero();
	robot_lelbow_ori_init(0, 2) = -1;
	robot_lelbow_ori_init(1, 0) = 1;
	robot_lelbow_ori_init(2, 1) = -1;

	robot_relbow_ori_init.setZero();
	robot_relbow_ori_init(0, 2) = -1;
	robot_relbow_ori_init(1, 0) = -1;
	robot_relbow_ori_init(2, 1) = 1;

	delta_exo2robot_lhand_pos_maping = exo2robot_lhand_pos_mapping_ - exo2robot_lhand_pos_mapping_init_;
	delta_exo2robot_rhand_pos_maping = exo2robot_rhand_pos_mapping_ - exo2robot_rhand_pos_mapping_init_;

	delta_exo2robot_lelbow_pos_maping = exo2robot_lelbow_pos_mapping_ - exo2robot_lelbow_pos_mapping_init_;
	delta_exo2robot_relbow_pos_maping = exo2robot_relbow_pos_mapping_ - exo2robot_relbow_pos_mapping_init_;

	exo2robot_lhand_pos_mapping_ = robot_init_hand_pos + delta_exo2robot_lhand_pos_maping;
	exo2robot_rhand_pos_mapping_ = robot_init_hand_pos + delta_exo2robot_rhand_pos_maping;

	exo2robot_lelbow_pos_mapping_ = robot_init_elbow_pos + delta_exo2robot_lelbow_pos_maping;
	exo2robot_relbow_pos_mapping_ = robot_init_elbow_pos + delta_exo2robot_relbow_pos_maping;

	if( exo2robot_lhand_pos_mapping_.norm() >  robot_arm_max_l_)
	{
		exo2robot_lhand_pos_mapping_ = exo2robot_lhand_pos_mapping_.normalized()*robot_arm_max_l_;
	}

	if( exo2robot_rhand_pos_mapping_.norm() >  robot_arm_max_l_)
	{
		exo2robot_rhand_pos_mapping_ = exo2robot_rhand_pos_mapping_.normalized()*robot_arm_max_l_;
	}

	if( exo2robot_lelbow_pos_mapping_.norm() >  robot_upperarm_max_l_)
	{
		exo2robot_lelbow_pos_mapping_ = exo2robot_lelbow_pos_mapping_.normalized()*robot_upperarm_max_l_;
	}

	if( exo2robot_relbow_pos_mapping_.norm() >  robot_upperarm_max_l_)
	{
		exo2robot_relbow_pos_mapping_ = exo2robot_relbow_pos_mapping_.normalized()*robot_upperarm_max_l_;
	}

	master_lhand_pose_raw_.translation() = larmbase_transform_pre_desired_from_.translation() + robot_init_lshoulder_pos + exo2robot_lhand_pos_mapping_;
	master_lhand_pose_raw_.linear() = exo_suit_lhand_pose_.linear()*exo_suit_lhand_pose_init_.linear().transpose()*robot_lhand_ori_init;

	master_rhand_pose_raw_.translation() = rarmbase_transform_pre_desired_from_.translation() + robot_init_rshoulder_pos + exo2robot_rhand_pos_mapping_;
	master_rhand_pose_raw_.linear() = exo_suit_rhand_pose_.linear()*exo_suit_rhand_pose_init_.linear().transpose()*robot_rhand_ori_init;

	master_lelbow_pose_raw_.translation() = lshoulder_transform_pre_desired_from_.translation() + exo2robot_lelbow_pos_mapping_;
	master_lelbow_pose_raw_.linear() = exo_suit_lupperarm_pose_.linear()*exo_suit_lupperarm_pose_init_.linear().transpose()*robot_lelbow_ori_init;

	master_relbow_pose_raw_.translation() = rshoulder_transform_pre_desired_from_.translation() + exo2robot_relbow_pos_mapping_;
	master_relbow_pose_raw_.linear() = exo_suit_rupperarm_pose_.linear()*exo_suit_rupperarm_pose_init_.linear().transpose()*robot_relbow_ori_init;

	master_lshoulder_pose_raw_.translation().setZero();
	master_lshoulder_pose_raw_.linear() = exo_suit_lshoulder_pose_.linear()*exo_suit_lshoulder_pose_init_.linear().transpose()*robot_lshoulder_ori_init;

	master_rshoulder_pose_raw_.translation().setZero();
	master_rshoulder_pose_raw_.linear() = exo_suit_rshoulder_pose_.linear()*exo_suit_rshoulder_pose_init_.linear().transpose()*robot_rshoulder_ori_init;

	master_head_pose_raw_.translation().setZero();
	master_head_pose_raw_.linear() = exo_suit_head_pose_.linear()*exo_suit_head_pose_init_.linear().transpose()*robot_head_ori_init;

	master_upperbody_pose_raw_.translation().setZero();
	master_upperbody_pose_raw_.linear() = exo_suit_upperbody_pose_.linear()*exo_suit_upperbody_pose_init_.linear().transpose()*robot_upperbody_ori_init;
	
	master_relative_lhand_pos_raw_ = exo_suit_lhand_pose_.translation() - exo_suit_rhand_pose_.translation();
	master_relative_rhand_pos_raw_ = exo_suit_rhand_pose_.translation() - exo_suit_lhand_pose_.translation();
	master_relative_lhand_pos_raw_ *= (robot_shoulder_width_)/(exo_shoulder_width_);
	master_relative_rhand_pos_raw_ *= (robot_shoulder_width_)/(exo_shoulder_width_);

	if( int(current_time_*1e4)%int(1e3) == 0)
	{
		// cout<<"exo_suit_lhand_pose_init_.linear(): \n"<<exo_suit_lhand_pose_init_.linear()<<endl;
		// cout<<"master_lhand_pose_raw_.linear(): \n"<<master_lhand_pose_raw_.linear()<<endl;
		// cout<<"master_lhand_pose_.linear(): \n"<<master_lhand_pose_.linear()<<endl;
		// cout<<"master_lhand_pose_raw_.translation(): \n"<<master_lhand_pose_raw_.translation()<<endl;
		// cout<<"exo delta linear matrix: \n"<<exo_suit_lhand_pose_init_.linear().transpose()*exo_suit_lhand_pose_.linear()<<endl;

		// cout<<"master_lelbow_pose_raw_.translation(): \n"<<master_lelbow_pose_raw_.translation()<<endl;
		// cout<<"exo2robot_lhand_pos_mapping \n"<<exo2robot_lhand_pos_mapping_<<endl;
		// cout<<"left hand ori mat: \n"<<lhand_transform_current_from_global_.linear()<<endl;
		// cout<<"command arm length: "<<(lshoulder_transform_current_from_global_.translation() - master_lhand_pose_.translation()).norm() <<endl;
		// cout<<"exosuit arm length: "<<(exo_suit_lshoulder_pose_.translation() - exo_suit_lhand_pose_.translation()).norm() <<endl;
	}
}

void CustomController::azureKinectRawDataProcessing()
{
	// position
	azure_kinect_head_pose_.translation() = azure_kinect_head_pos_raw_;
	azure_kinect_lshoulder_pose_.translation() = azure_kinect_lshoulder_pos_raw_;
	azure_kinect_lelbow_pose_.translation() = azure_kinect_lelbow_pos_raw_;
	azure_kinect_lwrist_pose_.translation() = azure_kinect_lwrist_pos_raw_;
	azure_kinect_lhand_pose_.translation() = azure_kinect_lhand_pos_raw_;
	azure_kinect_rshoulder_pose_.translation() = azure_kinect_rshoulder_pos_raw_;
	azure_kinect_relbow_pose_.translation() = azure_kinect_relbow_pos_raw_;
	azure_kinect_rwrist_pose_.translation() = azure_kinect_rwrist_pos_raw_;
	azure_kinect_rhand_pose_.translation() = azure_kinect_rhand_pos_raw_;
	azure_kinect_pelv_pose_.translation() = azure_kinect_pelv_pos_raw_;
	azure_kinect_upperbody_pose_.translation() = azure_kinect_upperbody_pos_raw_;
	
	// //orienation
	azure_kinect_head_pose_.linear() = 		azure_kinect_head_q_raw_.normalized().toRotationMatrix();
	azure_kinect_lshoulder_pose_.linear() = azure_kinect_lshoulder_q_raw_.normalized().toRotationMatrix();
	azure_kinect_lelbow_pose_.linear() = 	azure_kinect_lelbow_q_raw_.normalized().toRotationMatrix();
	azure_kinect_lwrist_pose_.linear() = 	azure_kinect_lwrist_q_raw_.normalized().toRotationMatrix();
	azure_kinect_lhand_pose_.linear() = 	azure_kinect_lhand_q_raw_.normalized().toRotationMatrix();
	azure_kinect_rshoulder_pose_.linear() = azure_kinect_rshoulder_q_raw_.normalized().toRotationMatrix();
	azure_kinect_relbow_pose_.linear() = 	azure_kinect_relbow_q_raw_.normalized().toRotationMatrix();	
	azure_kinect_rwrist_pose_.linear() = 	azure_kinect_rwrist_q_raw_.normalized().toRotationMatrix();
	azure_kinect_rhand_pose_.linear() = 	azure_kinect_rhand_q_raw_.normalized().toRotationMatrix();
	azure_kinect_upperbody_pose_.linear() = azure_kinect_upperbody_q_raw_.normalized().toRotationMatrix();
	azure_kinect_pelv_pose_.linear() = 		azure_kinect_pelv_q_raw_.normalized().toRotationMatrix();

	azure_kinect_pelv_pose_.linear().setIdentity();

	//coordinate conversion
	azure_kinect_head_pose_ = 		azure_kinect_pelv_pose_.inverse()*azure_kinect_head_pose_;
	azure_kinect_upperbody_pose_ = 	azure_kinect_pelv_pose_.inverse()*azure_kinect_upperbody_pose_;
	azure_kinect_lshoulder_pose_ = 	azure_kinect_pelv_pose_.inverse()*azure_kinect_lshoulder_pose_;
	azure_kinect_lelbow_pose_ = 	azure_kinect_pelv_pose_.inverse()*azure_kinect_lelbow_pose_;
	azure_kinect_lwrist_pose_ = 	azure_kinect_pelv_pose_.inverse()*azure_kinect_lwrist_pose_;
	azure_kinect_lhand_pose_ = 		azure_kinect_pelv_pose_.inverse()*azure_kinect_lhand_pose_;
	azure_kinect_rshoulder_pose_ = 	azure_kinect_pelv_pose_.inverse()*azure_kinect_rshoulder_pose_;
	azure_kinect_relbow_pose_ = 	azure_kinect_pelv_pose_.inverse()*azure_kinect_relbow_pose_;
	azure_kinect_rwrist_pose_ = 	azure_kinect_pelv_pose_.inverse()*azure_kinect_rwrist_pose_;
	azure_kinect_rhand_pose_ = 		azure_kinect_pelv_pose_.inverse()*azure_kinect_rhand_pose_;

	kinect2robot_lelbow_pos_mapping_ = azure_kinect_lelbow_pose_.translation() - azure_kinect_lshoulder_pose_.translation();
	kinect2robot_lelbow_pos_mapping_ = (robot_upperarm_max_l_)/(kinect_lelbow_max_l_)*kinect2robot_lelbow_pos_mapping_;
		
	kinect2robot_relbow_pos_mapping_ = azure_kinect_relbow_pose_.translation() - azure_kinect_rshoulder_pose_.translation();
	kinect2robot_relbow_pos_mapping_ = (robot_upperarm_max_l_)/(kinect_relbow_max_l_)*kinect2robot_relbow_pos_mapping_;

	double dist_btw_hands = (azure_kinect_lhand_pose_.translation() - azure_kinect_rhand_pose_.translation()).norm();
	Vector3d unit_vec_shoulder = (azure_kinect_lshoulder_pose_.translation() - azure_kinect_rshoulder_pose_.translation()).normalized();

	
	// left hand
	if( unit_vec_shoulder.transpose() * azure_kinect_lhand_pose_.translation() >= kinect_shoulder_width_/2 )
	{
		kinect2robot_lhand_pos_mapping_ = azure_kinect_lhand_pose_.translation() - azure_kinect_lshoulder_pose_.translation();	
		kinect2robot_lhand_pos_mapping_ = (robot_arm_max_l_)/(kinect_larm_max_l_)*kinect2robot_lhand_pos_mapping_;
		kinect2robot_lhand_pos_mapping_ -= double(unit_vec_shoulder.transpose() * kinect2robot_lhand_pos_mapping_)*unit_vec_shoulder;
		kinect2robot_lhand_pos_mapping_ += ( (robot_arm_max_l_)/(kinect_larm_max_l_)*(unit_vec_shoulder.transpose()*azure_kinect_lhand_pose_.translation() - kinect_shoulder_width_/2) + robot_shoulder_width_/2 )*unit_vec_shoulder;
	
	}
	else if ( (unit_vec_shoulder.transpose() * azure_kinect_lhand_pose_.translation() >= -kinect_shoulder_width_/2)&&(unit_vec_shoulder.transpose() * azure_kinect_lhand_pose_.translation() < kinect_shoulder_width_/2))
	{
		kinect2robot_lhand_pos_mapping_ = azure_kinect_lhand_pose_.translation() - azure_kinect_lshoulder_pose_.translation();	
		kinect2robot_lhand_pos_mapping_ = (robot_arm_max_l_)/(kinect_larm_max_l_)*kinect2robot_lhand_pos_mapping_;
		kinect2robot_lhand_pos_mapping_ -= double(unit_vec_shoulder.transpose() * kinect2robot_lhand_pos_mapping_)*unit_vec_shoulder;
		kinect2robot_lhand_pos_mapping_ += ( (robot_shoulder_width_)/(kinect_shoulder_width_)*( unit_vec_shoulder.transpose()*azure_kinect_lhand_pose_.translation()+0) )*unit_vec_shoulder;
	}
	else
	{
		kinect2robot_lhand_pos_mapping_ = azure_kinect_lhand_pose_.translation() - azure_kinect_lshoulder_pose_.translation();	
		kinect2robot_lhand_pos_mapping_ = (robot_arm_max_l_)/(kinect_larm_max_l_)*kinect2robot_lhand_pos_mapping_;
		kinect2robot_lhand_pos_mapping_ -= double(unit_vec_shoulder.transpose() * kinect2robot_lhand_pos_mapping_)*unit_vec_shoulder;
		kinect2robot_lhand_pos_mapping_ += ( (robot_arm_max_l_)/(kinect_larm_max_l_)*(unit_vec_shoulder.transpose()*azure_kinect_lhand_pose_.translation() + kinect_shoulder_width_/2) - robot_shoulder_width_/2 )*unit_vec_shoulder;
	}

	// right hand
	if( unit_vec_shoulder.transpose() * azure_kinect_rhand_pose_.translation() >= kinect_shoulder_width_/2 )
	{
		kinect2robot_rhand_pos_mapping_ = azure_kinect_rhand_pose_.translation() - azure_kinect_rshoulder_pose_.translation();	
		kinect2robot_rhand_pos_mapping_ = (robot_arm_max_l_)/(kinect_rarm_max_l_)*kinect2robot_rhand_pos_mapping_;
		kinect2robot_rhand_pos_mapping_ -= double(unit_vec_shoulder.transpose() * kinect2robot_rhand_pos_mapping_)*unit_vec_shoulder;
		kinect2robot_rhand_pos_mapping_ += ( (robot_arm_max_l_)/(kinect_rarm_max_l_)*(unit_vec_shoulder.transpose()*azure_kinect_rhand_pose_.translation() - kinect_shoulder_width_/2) + robot_shoulder_width_/2 )*unit_vec_shoulder;
	}
	else if ( (unit_vec_shoulder.transpose() * azure_kinect_rhand_pose_.translation() >= -kinect_shoulder_width_/2)&&(unit_vec_shoulder.transpose() * azure_kinect_rhand_pose_.translation() < kinect_shoulder_width_/2))
	{
		kinect2robot_rhand_pos_mapping_ = azure_kinect_rhand_pose_.translation() - azure_kinect_rshoulder_pose_.translation();	
		kinect2robot_rhand_pos_mapping_ = (robot_arm_max_l_)/(kinect_rarm_max_l_)*kinect2robot_rhand_pos_mapping_;
		kinect2robot_rhand_pos_mapping_ -= double(unit_vec_shoulder.transpose() * kinect2robot_rhand_pos_mapping_)*unit_vec_shoulder;
		kinect2robot_rhand_pos_mapping_ += ( (robot_shoulder_width_)/(kinect_shoulder_width_)*(unit_vec_shoulder.transpose()*azure_kinect_rhand_pose_.translation()+0) )*unit_vec_shoulder;
	}
	else
	{
		kinect2robot_rhand_pos_mapping_ = azure_kinect_rhand_pose_.translation() - azure_kinect_rshoulder_pose_.translation();	
		kinect2robot_rhand_pos_mapping_ = (robot_arm_max_l_)/(kinect_rarm_max_l_)*kinect2robot_rhand_pos_mapping_;
		kinect2robot_rhand_pos_mapping_ -= double(unit_vec_shoulder.transpose() * kinect2robot_rhand_pos_mapping_)*unit_vec_shoulder;
		kinect2robot_rhand_pos_mapping_ += ( (robot_arm_max_l_)/(kinect_rarm_max_l_)*(unit_vec_shoulder.transpose()*azure_kinect_rhand_pose_.translation() + kinect_shoulder_width_/2) - robot_shoulder_width_/2 )*unit_vec_shoulder;
	}

	if(azure_kinect_init_pose_calibration_ == true)
	{
		azure_kinect_init_pose_calibration_ = false;
		azure_kinect_init_pose_cali_time_ = current_time_;

		azure_kinect_head_pose_init_ = 		azure_kinect_head_pose_;   
    	azure_kinect_lshoulder_pose_init_ = azure_kinect_lshoulder_pose_;
		azure_kinect_lelbow_pose_init_ = 	azure_kinect_lelbow_pose_;
    	azure_kinect_lwrist_pose_init_ = 	azure_kinect_lwrist_pose_;
    	azure_kinect_lhand_pose_init_ = 	azure_kinect_lhand_pose_;
    	azure_kinect_rshoulder_pose_init_ = azure_kinect_rshoulder_pose_;
		azure_kinect_relbow_pose_init_ = 	azure_kinect_relbow_pose_;
    	azure_kinect_rwrist_pose_init_ = 	azure_kinect_rwrist_pose_;
    	azure_kinect_rhand_pose_init_ = 	azure_kinect_rhand_pose_;
    	azure_kinect_pelv_pose_init_ = 		azure_kinect_pelv_pose_;
		azure_kinect_upperbody_pose_init_ = azure_kinect_upperbody_pose_;

		kinect2robot_lhand_pos_mapping_init_ = 	kinect2robot_lhand_pos_mapping_;
		kinect2robot_rhand_pos_mapping_init_ = 	kinect2robot_rhand_pos_mapping_;
		kinect2robot_lelbow_pos_mapping_init_ = 	kinect2robot_lelbow_pos_mapping_;
		kinect2robot_relbow_pos_mapping_init_ = 	kinect2robot_relbow_pos_mapping_;
	}

	Vector3d robot_init_hand_pos, robot_init_lshoulder_pos, robot_init_rshoulder_pos, robot_init_elbow_pos, delta_kinect2robot_lhand_pos_maping, delta_kinect2robot_rhand_pos_maping, delta_kinect2robot_lelbow_pos_maping, delta_kinect2robot_relbow_pos_maping;
	robot_init_hand_pos << 0, 0, -(robot_arm_max_l_);
	robot_init_lshoulder_pos << 0, 0.1491, 0.065; 
	robot_init_rshoulder_pos << 0, -0.1491, 0.065;
	robot_init_elbow_pos<<0, 0, -(robot_upperarm_max_l_);
	Matrix3d robot_lhand_ori_init, robot_rhand_ori_init, robot_lelbow_ori_init, robot_relbow_ori_init, robot_lshoulder_ori_init, robot_rshoulder_ori_init, robot_head_ori_init, robot_upperbody_ori_init;
	robot_lhand_ori_init = DyrosMath::rotateWithZ(-90*DEG2RAD);
	robot_rhand_ori_init = DyrosMath::rotateWithZ(90*DEG2RAD);
	// robot_lshoulder_ori_init = DyrosMath::rotateWithZ(-0.3);
	robot_lshoulder_ori_init.setIdentity();
	// robot_rshoulder_ori_init = DyrosMath::rotateWithZ(0.3);
	robot_rshoulder_ori_init.setIdentity();
	robot_head_ori_init.setIdentity();
	robot_upperbody_ori_init.setIdentity();

	// robot_lelbow_ori_init << 0, 0, -1, 1, 0, 0, 0, -1, 0;
	robot_lelbow_ori_init.setZero();
	robot_lelbow_ori_init(0, 2) = -1;
	robot_lelbow_ori_init(1, 0) = 1;
	robot_lelbow_ori_init(2, 1) = -1;

	robot_relbow_ori_init.setZero();
	robot_relbow_ori_init(0, 2) = -1;
	robot_relbow_ori_init(1, 0) = -1;
	robot_relbow_ori_init(2, 1) = 1;

	delta_kinect2robot_lhand_pos_maping = kinect2robot_lhand_pos_mapping_ - kinect2robot_lhand_pos_mapping_init_;
	delta_kinect2robot_rhand_pos_maping = kinect2robot_rhand_pos_mapping_ - kinect2robot_rhand_pos_mapping_init_;

	delta_kinect2robot_lelbow_pos_maping = kinect2robot_lelbow_pos_mapping_ - kinect2robot_lelbow_pos_mapping_init_;
	delta_kinect2robot_relbow_pos_maping = kinect2robot_relbow_pos_mapping_ - kinect2robot_relbow_pos_mapping_init_;

	kinect2robot_lhand_pos_mapping_ = robot_init_hand_pos + delta_kinect2robot_lhand_pos_maping;
	kinect2robot_rhand_pos_mapping_ = robot_init_hand_pos + delta_kinect2robot_rhand_pos_maping;

	kinect2robot_lelbow_pos_mapping_ = robot_init_elbow_pos + delta_kinect2robot_lelbow_pos_maping;
	kinect2robot_relbow_pos_mapping_ = robot_init_elbow_pos + delta_kinect2robot_relbow_pos_maping;

	if( kinect2robot_lhand_pos_mapping_.norm() >  robot_arm_max_l_)
	{
		kinect2robot_lhand_pos_mapping_ = kinect2robot_lhand_pos_mapping_.normalized()*robot_arm_max_l_;
	}

	if( kinect2robot_rhand_pos_mapping_.norm() >  robot_arm_max_l_)
	{
		kinect2robot_rhand_pos_mapping_ = kinect2robot_rhand_pos_mapping_.normalized()*robot_arm_max_l_;
	}

	if( kinect2robot_lelbow_pos_mapping_.norm() >  robot_upperarm_max_l_)
	{
		kinect2robot_lelbow_pos_mapping_ = kinect2robot_lelbow_pos_mapping_.normalized()*robot_upperarm_max_l_;
	}

	if( kinect2robot_relbow_pos_mapping_.norm() >  robot_upperarm_max_l_)
	{
		kinect2robot_relbow_pos_mapping_ = kinect2robot_relbow_pos_mapping_.normalized()*robot_upperarm_max_l_;
	}

	master_lhand_pose_raw_.translation() = larmbase_transform_pre_desired_from_.translation() + robot_init_lshoulder_pos + kinect2robot_lhand_pos_mapping_;
	master_lhand_pose_raw_.linear() = azure_kinect_lhand_pose_.linear()*azure_kinect_lhand_pose_init_.linear().transpose()*robot_lhand_ori_init;

	master_rhand_pose_raw_.translation() = rarmbase_transform_pre_desired_from_.translation() + robot_init_rshoulder_pos + kinect2robot_rhand_pos_mapping_;
	master_rhand_pose_raw_.linear() = azure_kinect_rhand_pose_.linear()*azure_kinect_rhand_pose_init_.linear().transpose()*robot_rhand_ori_init;

	master_lelbow_pose_raw_.translation() = lshoulder_transform_pre_desired_from_.translation() + kinect2robot_lelbow_pos_mapping_;
	master_lelbow_pose_raw_.linear() = azure_kinect_lelbow_pose_.linear()*azure_kinect_lelbow_pose_init_.linear().transpose()*robot_lelbow_ori_init;

	master_relbow_pose_raw_.translation() = rshoulder_transform_pre_desired_from_.translation() + kinect2robot_relbow_pos_mapping_;
	master_relbow_pose_raw_.linear() = azure_kinect_relbow_pose_.linear()*azure_kinect_relbow_pose_init_.linear().transpose()*robot_relbow_ori_init;

	master_lshoulder_pose_raw_.translation().setZero();
	master_lshoulder_pose_raw_.linear() = azure_kinect_lshoulder_pose_.linear()*azure_kinect_lshoulder_pose_init_.linear().transpose()*robot_lshoulder_ori_init;

	master_rshoulder_pose_raw_.translation().setZero();
	master_rshoulder_pose_raw_.linear() = azure_kinect_rshoulder_pose_.linear()*azure_kinect_rshoulder_pose_init_.linear().transpose()*robot_rshoulder_ori_init;

	master_head_pose_raw_.translation().setZero();
	master_head_pose_raw_.linear() = azure_kinect_head_pose_.linear()*azure_kinect_head_pose_init_.linear().transpose()*robot_head_ori_init;

	master_upperbody_pose_raw_.translation().setZero();
	master_upperbody_pose_raw_.linear() = azure_kinect_upperbody_pose_.linear()*azure_kinect_upperbody_pose_init_.linear().transpose()*robot_upperbody_ori_init;
	
	master_relative_lhand_pos_raw_ = azure_kinect_lhand_pose_.translation() - azure_kinect_rhand_pose_.translation();
	master_relative_rhand_pos_raw_ = azure_kinect_rhand_pose_.translation() - azure_kinect_lhand_pose_.translation();
	master_relative_lhand_pos_raw_ *= (robot_shoulder_width_)/(kinect_shoulder_width_);
	master_relative_rhand_pos_raw_ *= (robot_shoulder_width_)/(kinect_shoulder_width_);

	if( int(current_time_*1e4)%int(1e3) == 0)
	{
		// cout<<"azure_kinect_lhand_pose_.translation(): \n"<<azure_kinect_lhand_pose_.translation()<<endl;
		// cout<<"azure_kinect_lhand_pose_.linear(): \n"<<azure_kinect_lhand_pose_.linear()<<endl;
		// cout<<"azure_kinect_rhand_pose_.translation(): \n"<<azure_kinect_rhand_pose_.translation()<<endl;
		// cout<<"azure_kinect_rhand_pose_.linear(): \n"<<azure_kinect_rhand_pose_.linear()<<endl;

		// cout<<"master_lhand_pose_raw_.translation(): \n"<<master_lhand_pose_raw_.translation()<<endl;
		// cout<<"master_lhand_pose_raw_.linear(): \n"<<master_lhand_pose_raw_.linear()<<endl;
		// cout<<"master_rhand_pose_raw_.translation(): \n"<<master_rhand_pose_raw_.translation()<<endl;
		// cout<<"master_rhand_pose_raw_.linear(): \n"<<master_rhand_pose_raw_.linear()<<endl;
		// cout<<"master_lhand_pose_raw_.linear(): \n"<<master_lhand_pose_raw_.linear()<<endl;
		// cout<<"master_lhand_pose_.linear(): \n"<<master_lhand_pose_.linear()<<endl;
		// cout<<"master_lhand_pose_raw_.translation(): \n"<<master_lhand_pose_raw_.translation()<<endl;
		// cout<<"exo delta linear matrix: \n"<<exo_suit_lhand_pose_init_.linear().transpose()*exo_suit_lhand_pose_.linear()<<endl;

		// cout<<"master_lelbow_pose_raw_.translation(): \n"<<master_lelbow_pose_raw_.translation()<<endl;
		// cout<<"exo2robot_lhand_pos_mapping \n"<<exo2robot_lhand_pos_mapping_<<endl;
		// cout<<"left hand ori mat: \n"<<lhand_transform_current_from_global_.linear()<<endl;
		// cout<<"command arm length: "<<(lshoulder_transform_current_from_global_.translation() - master_lhand_pose_.translation()).norm() <<endl;
		// cout<<"exosuit arm length: "<<(exo_suit_lshoulder_pose_.translation() - exo_suit_lhand_pose_.translation()).norm() <<endl;
	}
}

void CustomController::hmdRawDataProcessing()
{

	if( int(current_time_*1e4)%int(1e3) == 0)
	{
		// cout<<"left hand ori mat: \n"<<exo_suit_lhand_pose_.linear()<<endl;
	}


	///////////////////////////////////MAPPING///////////////////////////////
	Vector3d robot_still_pose_lhand, robot_t_pose_lhand, robot_forward_pose_lhand, robot_still_pose_rhand, robot_t_pose_rhand, robot_forward_pose_rhand;
	Vector4d lhand_mapping_vector, rhand_mapping_vector;
	MatrixXd lhand_master_ref_stack, lhand_robot_ref_stack, rhand_master_ref_stack, rhand_robot_ref_stack, lhand_master_ref_stack_pinverse, rhand_master_ref_stack_pinverse;
	lhand_master_ref_stack.setZero(3,4);
	lhand_robot_ref_stack.setZero(3,4);
	lhand_master_ref_stack_pinverse.setZero(4,3);
	rhand_master_ref_stack.setZero(3,4);
	rhand_robot_ref_stack.setZero(3,4);
	rhand_master_ref_stack_pinverse.setZero(4,3);

	robot_still_pose_lhand.setZero();
	robot_t_pose_lhand.setZero();
	robot_forward_pose_lhand.setZero();
	robot_still_pose_rhand.setZero();
	robot_t_pose_rhand.setZero();
	robot_forward_pose_rhand.setZero();
	
	robot_still_pose_lhand(2) += -(robot_arm_max_l_);
	robot_t_pose_lhand(1) += (robot_arm_max_l_);
	robot_forward_pose_lhand(0) += (robot_arm_max_l_);

	robot_still_pose_rhand(2) += -(robot_arm_max_l_);
	robot_t_pose_rhand(1) += -(robot_arm_max_l_);
	robot_forward_pose_rhand(0) += (robot_arm_max_l_);

	lhand_master_ref_stack.block(0, 0, 3, 1) = hmd_still_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
	lhand_master_ref_stack.block(0, 1, 3, 1) = hmd_tpose_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
	lhand_master_ref_stack.block(0, 2, 3, 1) = hmd_forward_cali_lhand_pos_ - hmd_lshoulder_center_pos_;
	lhand_master_ref_stack.block(0, 3, 3, 1) = hmd_rshoulder_center_pos_ - hmd_lshoulder_center_pos_;

	lhand_robot_ref_stack.block(0, 0, 3, 1) = robot_still_pose_lhand;
	lhand_robot_ref_stack.block(0, 1, 3, 1) = robot_t_pose_lhand;
	lhand_robot_ref_stack.block(0, 2, 3, 1) = robot_forward_pose_lhand;
	lhand_robot_ref_stack(1, 3) = -robot_shoulder_width_;

	lhand_master_ref_stack_pinverse = lhand_master_ref_stack.transpose()*(lhand_master_ref_stack*lhand_master_ref_stack.transpose() + 0.000001*Eigen::Matrix3d::Identity() ).inverse();
	lhand_mapping_vector = lhand_master_ref_stack_pinverse*(hmd_lhand_pose_.translation() - hmd_lshoulder_pose_.translation());

	hmd2robot_lhand_pos_mapping_ = lhand_robot_ref_stack*lhand_mapping_vector;

	rhand_master_ref_stack.block(0, 0, 3, 1) = hmd_still_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
	rhand_master_ref_stack.block(0, 1, 3, 1) = hmd_tpose_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
	rhand_master_ref_stack.block(0, 2, 3, 1) = hmd_forward_cali_rhand_pos_ - hmd_rshoulder_center_pos_;
	rhand_master_ref_stack.block(0, 3, 3, 1) = hmd_lshoulder_center_pos_ - hmd_rshoulder_center_pos_;

	rhand_robot_ref_stack.block(0, 0, 3, 1) = robot_still_pose_rhand;
	rhand_robot_ref_stack.block(0, 1, 3, 1) = robot_t_pose_rhand;
	rhand_robot_ref_stack.block(0, 2, 3, 1) = robot_forward_pose_rhand;
	rhand_robot_ref_stack(1, 3) = robot_shoulder_width_;
	
	rhand_master_ref_stack_pinverse = rhand_master_ref_stack.transpose()*(rhand_master_ref_stack*rhand_master_ref_stack.transpose() + 0.000001*Eigen::Matrix3d::Identity() ).inverse();

	rhand_mapping_vector = rhand_master_ref_stack_pinverse*(hmd_rhand_pose_.translation() - hmd_rshoulder_pose_.translation());

	hmd2robot_rhand_pos_mapping_ = rhand_robot_ref_stack*rhand_mapping_vector;
	///////////////////////////////////////////////////////////////////////////////

	if( hmd2robot_lhand_pos_mapping_.norm() > robot_arm_max_l_)
	{
		hmd2robot_lhand_pos_mapping_ = hmd2robot_lhand_pos_mapping_.normalized()*robot_arm_max_l_;
	}

	if( hmd2robot_rhand_pos_mapping_.norm() > robot_arm_max_l_)
	{
		hmd2robot_rhand_pos_mapping_ = hmd2robot_rhand_pos_mapping_.normalized()*robot_arm_max_l_;
	}

	if( hmd2robot_lhand_pos_mapping_.norm() < 0.1)
	{
		hmd2robot_lhand_pos_mapping_ = hmd2robot_lhand_pos_mapping_.normalized()*0.1;
	}

	if( hmd2robot_rhand_pos_mapping_.norm() < 0.1)
	{
		hmd2robot_rhand_pos_mapping_ = hmd2robot_rhand_pos_mapping_.normalized()*0.1;
	}

	if( (hmd_init_pose_calibration_ == true) && (hmd_check_pose_calibration_[3] == true))	//still cali로 옮기기
	{
		cout<<"Motion Retargeting Parameter Initialized"<<endl;
		hmd_init_pose_calibration_ = false;
		// hmd_init_pose_cali_time_ = current_time_;

		// hmd_head_pose_init_ = hmd_head_pose_;   
    	// hmd_lshoulder_pose_init_ = hmd_lshoulder_pose_;
		// hmd_lupperarm_pose_init_ = hmd_lupperarm_pose_;
    	// hmd_lhand_pose_init_ = hmd_lhand_pose_;
    	// hmd_rshoulder_pose_init_ = hmd_rshoulder_pose_;
		// hmd_rupperarm_pose_init_ = hmd_rupperarm_pose_;
    	// hmd_rhand_pose_init_ = hmd_rhand_pose_;
    	// hmd_pelv_pose_init_ = hmd_pelv_pose_;
		// hmd_chest_pose_init_ = hmd_chest_pose_;

		// hmd2robot_lhand_pos_mapping_init_ = hmd2robot_lhand_pos_mapping_;
		// hmd2robot_rhand_pos_mapping_init_ = hmd2robot_rhand_pos_mapping_;
	}

	Vector3d robot_init_hand_pos, robot_init_lshoulder_pos, robot_init_rshoulder_pos, delta_hmd2robot_lhand_pos_maping, delta_hmd2robot_rhand_pos_maping, delta_hmd2robot_lelbow_pos_maping, delta_hmd2robot_relbow_pos_maping;
	robot_init_hand_pos << 0, 0, -(robot_arm_max_l_);
	robot_init_lshoulder_pos << 0, 0.1491, 0.065; 
	robot_init_rshoulder_pos << 0, -0.1491, 0.065;
	Matrix3d robot_lhand_ori_init, robot_rhand_ori_init, robot_lelbow_ori_init, robot_relbow_ori_init, robot_lshoulder_ori_init, robot_rshoulder_ori_init, robot_head_ori_init, robot_upperbody_ori_init;
	robot_lhand_ori_init = DyrosMath::rotateWithZ(-90*DEG2RAD);
	robot_rhand_ori_init = DyrosMath::rotateWithZ(90*DEG2RAD);
	// robot_lshoulder_ori_init = DyrosMath::rotateWithZ(-0.3);
	robot_lshoulder_ori_init.setIdentity();
	// robot_rshoulder_ori_init = DyrosMath::rotateWithZ(0.3);
	robot_rshoulder_ori_init.setIdentity();
	robot_head_ori_init.setIdentity();
	robot_upperbody_ori_init.setIdentity();

	// robot_lelbow_ori_init << 0, 0, -1, 1, 0, 0, 0, -1, 0;
	robot_lelbow_ori_init.setZero();
	robot_lelbow_ori_init(0, 2) = -1;
	robot_lelbow_ori_init(1, 0) = 1;
	robot_lelbow_ori_init(2, 1) = -1;

	robot_relbow_ori_init.setZero();
	robot_relbow_ori_init(0, 2) = -1;
	robot_relbow_ori_init(1, 0) = -1;
	robot_relbow_ori_init(2, 1) = 1;

	// delta_hmd2robot_lhand_pos_maping = hmd2robot_lhand_pos_mapping_ - hmd2robot_lhand_pos_mapping_init_;
	// delta_hmd2robot_rhand_pos_maping = hmd2robot_rhand_pos_mapping_ - hmd2robot_rhand_pos_mapping_init_;

	// hmd2robot_lhand_pos_mapping_ = robot_init_hand_pos + delta_hmd2robot_lhand_pos_maping;
	// hmd2robot_rhand_pos_mapping_ = robot_init_hand_pos + delta_hmd2robot_rhand_pos_maping;


	master_lhand_pose_raw_.translation() = larmbase_transform_pre_desired_from_.translation() + upperbody_transform_pre_desired_from_.linear()*robot_init_lshoulder_pos + hmd2robot_lhand_pos_mapping_;
	// master_lhand_pose_raw_.linear() = hmd_lhand_pose_.linear()*hmd_lhand_pose_init_.linear().transpose()*robot_lhand_ori_init;	//relative orientation
	master_lhand_pose_raw_.linear() = hmd_lhand_pose_.linear()*DyrosMath::rotateWithZ(M_PI);	//absolute orientation

	master_rhand_pose_raw_.translation() = rarmbase_transform_pre_desired_from_.translation() + upperbody_transform_pre_desired_from_.linear()*robot_init_rshoulder_pos + hmd2robot_rhand_pos_mapping_;
	// master_rhand_pose_raw_.linear() = hmd_rhand_pose_.linear()*hmd_rhand_pose_init_.linear().transpose()*robot_rhand_ori_init;	//relative orientation
	master_rhand_pose_raw_.linear() = hmd_rhand_pose_.linear()*DyrosMath::rotateWithZ(M_PI);	//absolute orientation

	// master_lelbow_pose_raw_.translation() = lshoulder_transform_pre_desired_from_.translation() + hmd2robot_lelbow_pos_mapping_;
	master_lelbow_pose_raw_.translation().setZero();
	master_lelbow_pose_raw_.linear() = hmd_lupperarm_pose_.linear()*hmd_lupperarm_pose_init_.linear().transpose()*robot_lelbow_ori_init;

	// master_relbow_pose_raw_.translation() = rshoulder_transform_pre_desired_from_.translation() + hmd2robot_relbow_pos_mapping_;
	master_relbow_pose_raw_.translation().setZero();
	master_relbow_pose_raw_.linear() = hmd_rupperarm_pose_.linear()*hmd_rupperarm_pose_init_.linear().transpose()*robot_relbow_ori_init;

	master_lshoulder_pose_raw_.translation().setZero();
	master_lshoulder_pose_raw_.linear() = hmd_lshoulder_pose_.linear()*hmd_lshoulder_pose_init_.linear().transpose()*robot_lshoulder_ori_init;

	master_rshoulder_pose_raw_.translation().setZero();
	master_rshoulder_pose_raw_.linear() = hmd_rshoulder_pose_.linear()*hmd_rshoulder_pose_init_.linear().transpose()*robot_rshoulder_ori_init;

	master_head_pose_raw_.translation().setZero();
	master_head_pose_raw_.linear() = hmd_head_pose_.linear()*hmd_head_pose_init_.linear().transpose()*robot_head_ori_init;
	// master_head_pose_raw_.linear() = hmd_head_pose_.linear();

	master_upperbody_pose_raw_.translation().setZero();
	Eigen::AngleAxisd chest_ang_diff(hmd_chest_pose_.linear()*hmd_chest_pose_init_.linear().transpose());
	Eigen::Matrix3d chest_diff_m, shoulder_diff_m;
	chest_diff_m = Eigen::AngleAxisd(chest_ang_diff.angle()*0.5, chest_ang_diff.axis());
	master_upperbody_pose_raw_.linear() = chest_diff_m*robot_upperbody_ori_init;
	// master_upperbody_pose_raw_.linear() = hmd_chest_pose_.linear()*hmd_chest_pose_init_.linear().transpose()*robot_upperbody_ori_init;
	
	shoulder_diff_m = Eigen::AngleAxisd(chest_ang_diff.angle()*0.5, chest_ang_diff.axis());
	master_lshoulder_pose_raw_.linear() = shoulder_diff_m*robot_upperbody_ori_init;
	master_rshoulder_pose_raw_.linear() = shoulder_diff_m*robot_upperbody_ori_init;

	master_relative_lhand_pos_raw_ = hmd_lhand_pose_.translation() - hmd_rhand_pose_.translation();
	master_relative_rhand_pos_raw_ = hmd_rhand_pose_.translation() - hmd_lhand_pose_.translation();
	master_relative_lhand_pos_raw_ *= (robot_shoulder_width_)/(hmd_shoulder_width_);
	master_relative_rhand_pos_raw_ *= (robot_shoulder_width_)/(hmd_shoulder_width_);

	if( int(current_time_*1e4)%int(1e3) == 0)
	{
		// cout<<"exo_suit_lhand_pose_init_.linear(): \n"<<exo_suit_lhand_pose_init_.linear()<<endl;
		// cout<<"master_lelbow_pose_raw_.linear(): \n"<<master_lelbow_pose_raw_.linear()<<endl;
		// cout<<"master_lelbow_pose_.linear(): \n"<<master_lelbow_pose_.linear()<<endl;
		// cout<<"master_relbow_pose_raw_.linear(): \n"<<master_relbow_pose_raw_.linear()<<endl;
		// cout<<"master_relbow_pose_.linear(): \n"<<master_relbow_pose_.linear()<<endl;
		// cout<<"master_lhand_pose_raw_.translation(): \n"<<master_lhand_pose_raw_.translation()<<endl;
		// cout<<"exo delta linear matrix: \n"<<exo_suit_lhand_pose_init_.linear().transpose()*exo_suit_lhand_pose_.linear()<<endl;

		// cout<<"master_lelbow_pose_raw_.linear(): \n"<<master_lelbow_pose_raw_.linear()<<endl;
		// cout<<"master_relbow_pose_raw_.linear(): \n"<<master_relbow_pose_raw_.linear()<<endl;
		// cout<<"exo2robot_lhand_pos_mapping \n"<<exo2robot_lhand_pos_mapping_<<endl;
		// cout<<"left hand ori mat: \n"<<lhand_transform_current_from_global_.linear()<<endl;
		// cout<<"command arm length: "<<(lshoulder_transform_current_from_global_.translation() - master_lhand_pose_.translation()).norm() <<endl;
		// cout<<"exosuit arm length: "<<(exo_suit_lshoulder_pose_.translation() - exo_suit_lhand_pose_.translation()).norm() <<endl;
		// cout<<"lhand_mapping_vector: "<<lhand_mapping_vector<<endl;
		// cout<<"rhand_mapping_vector: "<<rhand_mapping_vector<<endl;
	}
}

void CustomController::getCOMTrajectory()
{
	double desired_step_position_in_y;
	double desired_step_velocity_in_y;
	double d_temp_y;
	com_pos_desired_.setZero();
	com_vel_desired_.setZero();
	com_acc_desired_.setZero();



	// com_pos_desired_(1) = support_foot_transform_current_.translation()(1) + com_pos_init_(1) - support_foot_transform_init_.translation()(1);  // from preview	

	if ((walking_speed_ != 0)) // when the robot want to move
	{
		// com_vel_desired_(0) = walking_speed_;
		// com_pos_desired_(0) = com_pos_current_(0) + com_vel_desired_(0)*dt_;

		if (start_walking_trigger_ == true)
		{
			// com_acc_desired_(2) = GRAVITY/2;
			// com_vel_desired_(0) = walking_speed_;
			// com_pos_desired_(0) = com_pos_current_(0) + com_vel_desired_(0)*dt_;

			// desired_step_position_in_y = -(step_width_)*foot_contact_;

			double w = (current_time_ - start_time_)/walking_duration_start_delay_;
			w = DyrosMath::minmax_cut(w, 0, 1);

			// com_pos_desired_(0) = (1-w)*middle_of_both_foot_(0) + w*com_pos_current_(0);
			com_pos_desired_(0) = (1-w)*(com_pos_desired_last_(0)) + w*(com_pos_current_(0) + com_vel_desired_(0)*dt_);
			com_vel_desired_(0) = w*walking_speed_;
		}
		else
		{
			///////////////X DIRECTIOIN///////////
			// com_pos_desired_(0) = com_pos_current_(0);
			com_vel_desired_(0) = walking_speed_;
			com_pos_desired_(0) = com_pos_current_(0) + com_vel_desired_(0)*dt_;

			///////////////Y DIRECTIOIN///////////
			desired_step_position_in_y = -(step_width_)*foot_contact_;
			double target_com_y_speed = (support_foot_transform_init_.translation()(1) + desired_step_position_in_y - com_pos_init_(1)) / (walking_duration_);

			// com_pos_desired_(1) = support_foot_transform_current_.translation()(1) + (1 - walking_phase_) * (com_pos_init_(1) - support_foot_transform_init_.translation()(1)) + walking_phase_ * (desired_step_position_in_y);
			// com_vel_desired_(1) = target_com_y_speed;

		}
	}
	else
	{
		if ((foot_swing_trigger_ == true))
		{
			// com_acc_desired_(2) = GRAVITY;		//divide 2 because each legs apply same acc
			// double traj_duraiton = 3.0;
			///////////////X DIRECTIOIN///////////
			// com_pos_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(0), 0, 0, (middle_of_both_foot_)(0), 0, 0)(0);
			// com_vel_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(0), 0, 0, (middle_of_both_foot_)(0), 0, 0)(1);

			// com_pos_desired_(0) = com_pos_current_(0);
			com_vel_desired_(0) = 0.9*com_vel_desired_pre_(0);
			com_pos_desired_(0) = com_pos_current_(0) + com_vel_desired_(0)*dt_;
			// com_pos_desired_(0) = middle_of_both_foot_(0);


		}
		else if (program_start_time_ == stance_start_time_)
		{
			// com_pos_desired_(0) = com_pos_init_(0) - support_foot_transform_init_.translation()(0) + support_foot_transform_current_.translation()(0);
			// com_vel_desired_(2) = 0;
			// com_acc_desired_(2) = GRAVITY/2;
			// com_pos_desired_(0) = middle_of_both_foot_(0);
			// com_vel_desired_(0) = 0;

			// com_pos_desired_(1) = middle_of_both_foot_(1);
			// com_vel_desired_(1) = 0;
			
			com_pos_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_ + program_ready_duration_ + walking_control_transition_duration_, stance_start_time_ + program_ready_duration_ + walking_control_transition_duration_+1, support_foot_transform_current_.translation()(0) + (com_pos_init_)(0) - support_foot_transform_init_.translation()(0), 0, 0, (middle_of_both_foot_)(0)+ankle2footcenter_offset_, 0, 0)(0);
			com_vel_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_ + program_ready_duration_ + walking_control_transition_duration_, stance_start_time_ + program_ready_duration_ + walking_control_transition_duration_+1, support_foot_transform_current_.translation()(0) + (com_pos_init_)(0) - support_foot_transform_init_.translation()(0), 0, 0, (middle_of_both_foot_)(0)+ankle2footcenter_offset_, 0, 0)(1);
			com_acc_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_ + program_ready_duration_ + walking_control_transition_duration_, stance_start_time_ + program_ready_duration_ + walking_control_transition_duration_+1, support_foot_transform_current_.translation()(0) + (com_pos_init_)(0) - support_foot_transform_init_.translation()(0), 0, 0, (middle_of_both_foot_)(0)+ankle2footcenter_offset_, 0, 0)(2);

		}
		else if (stop_walking_trigger_ == true)
		{
			// com_pos_desired_(2) = com_pos_init_(2) - support_foot_transform_init_.translation()(2) + support_foot_transform_current_.translation()(2);
			// com_vel_desired_(2) = 0;
			// com_acc_desired_(2) = GRAVITY/2;		//divide 2 because each legs apply same acc
			
			com_vel_desired_(0) = 0.9*com_vel_desired_pre_(0);

			double traj_duraiton = walking_duration_*1;

			com_pos_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, support_foot_transform_current_.translation()(0) + (com_pos_desired_last_)(0) - support_foot_transform_init_.translation()(0), 0, 0, (middle_of_both_foot_)(0)+ankle2footcenter_offset_, 0, 0)(0);
			// com_vel_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, support_foot_transform_current_.translation()(0) + (com_pos_init_)(0) - support_foot_transform_init_.translation()(0), com_vel_init_(0), 0, (middle_of_both_foot_)(0)+0.02, 0, 0)(1);
			// com_acc_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, support_foot_transform_current_.translation()(0) + (com_pos_init_)(0) - support_foot_transform_init_.translation()(0), com_vel_init_(0), 0, (middle_of_both_foot_)(0)+0.02, 0, 0)(2);
			// com_vel_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(0), com_vel_init_(0), 0, (middle_of_both_foot_)(0), 0, 0)(1);
			// com_pos_desired_(0) = com_pos_init_(0);
			// com_pos_desired_(0) = middle_of_both_foot_(0);

		}
	}

	com_pos_desired_(2) = com_pos_current_(2);
	// com_vel_desired_(2) = com_vel_current_(2);
	// com_pos_desired_(2) = support_foot_transform_current_.translation()(2) + 0.75;
	com_vel_desired_(2) = 0;
	com_acc_desired_(2) = GRAVITY;

	com_pos_desired_(1) = yd_(0);  // from preview
	com_vel_desired_(1) = yd_(1);
	com_acc_desired_(1) = yd_(2);

	// com_pos_desired_(0) = xd_(0);  // avatar
	com_pos_desired_(0) = DyrosMath::QuinticSpline(current_time_, program_start_time_ , program_start_time_ + 3, support_foot_transform_current_.translation()(0) + (com_pos_init_)(0) - support_foot_transform_init_.translation()(0), 0, 0, (middle_of_both_foot_)(0), 0, 0)(0);
	com_pos_desired_(1) = DyrosMath::QuinticSpline(current_time_, program_start_time_ , program_start_time_ + 3, support_foot_transform_current_.translation()(1) + (com_pos_init_)(1) - support_foot_transform_init_.translation()(1), 0, 0, (middle_of_both_foot_)(1), 0, 0)(0);

	com_vel_desired_.setZero();
	com_acc_desired_.setZero();
}

void CustomController::getSwingFootXYTrajectory(double phase, Eigen::Vector3d com_pos_current, Eigen::Vector3d com_vel_current, Eigen::Vector3d com_vel_desired)
{
	Eigen::Vector2d d;
	Eigen::Vector2d d_prime;
	double d_temp_x;
	double d_temp_y;
	double ipm_calc_end_phsse = 0.9;

	if(walking_speed_ == 0)
	{
		// alpha_y_ = 0.15;
		alpha_x_ = 0.0;
	}
	else
	{
		alpha_x_ = alpha_x_command_;
		alpha_y_ = alpha_y_command_;
	}
	

	if (foot_swing_trigger_ == true)
	{ 
		// x axis
		d_temp_x = (com_pos_current(2) - support_foot_transform_current_.translation()(2)) / GRAVITY + (com_vel_current(0) / (2 * GRAVITY)) * (com_vel_current(0) / (2 * GRAVITY));

		if (d_temp_x<0) d_temp_x = 0;

		d_temp_x = sqrt(d_temp_x); 

		d(0) = com_vel_current(0) * d_temp_x;

		// y axis
		d_temp_y = (com_pos_current(2) - support_foot_transform_current_.translation()(2)) / GRAVITY + (com_vel_current(1) / (2 * GRAVITY)) * (com_vel_current(1) / (2 * GRAVITY));

		if (d_temp_y<0) d_temp_y = 0;
		
		d_temp_y = sqrt(d_temp_y);

		d(1) = com_vel_current(1) * d_temp_y;

		if (com_vel_current(0) * walking_speed_ < 0)
		{
			alpha_x_ = 0.05;
		}

		d_prime(0) = d(0) - alpha_x_ * com_vel_desired(0);
		d_prime(1) = d(1) - alpha_y_*step_width_/walking_duration_*foot_contact_;
		// d_prime(1) = d(1) - alpha_y * com_vel_desired(1) * foot_contact_;


		if (walking_phase_ < ipm_calc_end_phsse) 
		{
			target_foot_landing_from_pelv_ = com_pos_current.segment<2>(0) + d_prime; 
		}

		Vector2d landing_point;
		Vector2d desired_landing_point;
		double swingfoot_target_weight;

		// swingfoot_target_weight = (walking_phase_ - 0.1)/0.5;
		swingfoot_target_weight = 1;
		swingfoot_target_weight = DyrosMath::minmax_cut(swingfoot_target_weight, 0, 1);

		desired_landing_point(0) = walking_speed_*walking_duration_ + support_foot_transform_current_.translation()(0);
		desired_landing_point(1) = step_width_*(-foot_contact_) + support_foot_transform_current_.translation()(1);

		landing_point = swingfoot_target_weight*target_foot_landing_from_pelv_ + (1-swingfoot_target_weight)*desired_landing_point;

		double dsp_coeff = 2;
		if (walking_phase_ < dsp_coeff * switching_phase_duration_)
		{
			swing_foot_pos_trajectory_from_global_.segment(0, 2) = support_foot_transform_current_.translation().segment<2>(0) + swing_foot_transform_init_.translation().segment<2>(0) - support_foot_transform_init_.translation().segment<2>(0);
			swing_foot_vel_trajectory_from_global_.segment(0, 2).setZero();
			swing_foot_vel_trajectory_from_global_.segment(3, 3).setZero();
			// swing_foot_rot_trajectory_from_global_ = swing_foot_transform_init_.linear();
			swing_foot_rot_trajectory_from_global_.setIdentity();
		}
		else
		{
			double ssp = (walking_phase_ - dsp_coeff * switching_phase_duration_) / (ipm_calc_end_phsse - dsp_coeff * switching_phase_duration_);
			ssp = DyrosMath::minmax_cut(ssp, 0, 1);

			swing_foot_pos_trajectory_from_global_.segment(0, 2) = support_foot_transform_current_.translation().segment<2>(0) + 
			(1 - ssp) * (swing_foot_transform_init_.translation().segment<2>(0) - support_foot_transform_init_.translation().segment<2>(0)) +
			(ssp) * (target_foot_landing_from_pelv_ - support_foot_transform_current_.translation().segment<2>(0));

			swing_foot_vel_trajectory_from_global_.segment(0, 2) = (target_foot_landing_from_pelv_ - swing_foot_transform_init_.translation().segment<2>(0)) / (ipm_calc_end_phsse - dsp_coeff * switching_phase_duration_);
			swing_foot_rot_trajectory_from_global_.setIdentity(); 
		}

	}
	else
	{
		swing_foot_pos_trajectory_from_global_ = support_foot_transform_current_.translation() + swing_foot_transform_init_.translation() - support_foot_transform_init_.translation();
		swing_foot_vel_trajectory_from_global_.setZero();
		// swing_foot_rot_trajectory_from_global_ = swing_foot_transform_init_.linear();
		swing_foot_rot_trajectory_from_global_.setIdentity();
	}

	if (foot_contact_ == 1) //left support
	{
		swing_foot_pos_trajectory_from_global_(0) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(0), -0.6, 1.2);
		swing_foot_pos_trajectory_from_global_(1) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(1), -0.7, lfoot_transform_current_from_global_.translation()(1) - 0.21);
	}
	else if (foot_contact_ == -1) // right support
	{
		swing_foot_pos_trajectory_from_global_(0) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(0), - 0.6, 1.2);
		swing_foot_pos_trajectory_from_global_(1) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(1), rfoot_transform_current_from_global_.translation()(1) + 0.21, 0.7);
	}
	
}

Eigen::VectorQd CustomController::comVelocityControlCompute(WholebodyController &wbc)
{
	Eigen::VectorQd torque;

	Eigen::VectorXd f_star;
	Eigen::MatrixXd J_task;
	VectorQd torque_r_vel_tun;
	VectorQd torque_l_vel_tun;
	const int task_dof = 6;
	
	Vector3d alpha_unit;
	Vector3d beta_unit;

	double f_star_mag_alpha;
	double f_star_mag_beta;
	double f_star_mag_z;

	double f_star_mag_alpha_lfoot;
	double f_star_mag_beta_lfoot;
	double f_star_mag_z_lfoot;
	double f_star_mag_alpha_rfoot;
	double f_star_mag_beta_rfoot;
	double f_star_mag_z_rfoot;
	
	double d_l;				//distance from CoM_alpha to lfoot contact point
	double d_r;				//distance from CoM_alpha to rfoot contact point
	double w_l;				//weighting factor to how the CoM is closer to the lfoot
	double w_r;				//weighting factor to how the CoM is closer to the rfoot
	double h_l;				//the height of CoM with respect to the left foot
	double h_r;				//the height of CoM with respect to the right foot
	double gamma_l;			//tangent value of the angle from CoM to the lfoot;
	double gamma_r;			//tangent value of the angle from CoM to the rfoot;
	
	double gamma_l_min;		//minimum tangent value of the angle from CoM to the lfoot;
	double gamma_l_max;		//maximum tangent value of the angle from CoM to the lfoot;
	double gamma_r_min;		//minimum tangent value of the angle from CoM to the rfoot;
	double gamma_r_max;		//maximum tangent value of the angle from CoM to the rfoot;

	torque.setZero();
	phi_pelv_.setZero();
	torque_pelv_.setZero();
	J_task.setZero(task_dof, MODEL_DOF_VIRTUAL);
	f_star.setZero(task_dof);


	// wbc.set_contact(rd_, 0, 0); // for graviti torque calc
	// if(foot_swing_trigger_ == true)
	// {
	//     if( (foot_contact_ == 1) )
	//     {
	//         wc.set_contact(rd_, 1, 0);

	//     }
	//     else if ( (foot_contact_ == -1) )
	//     {
	//         wc.set_contact(rd_, 0, 1);
	//     }
	// }
	// wbc_.set_contact(rd_, 1, 1);
	torque_g_.setZero();
	torque_g_ = wbc.gravity_compensation_torque(rd_, true);

	////////////// Set f_start  ////////////////


	// com_pos_desired_(1) = com_pos_init_(0) + sin(2*M_PI/8*current_time_-tc.command_time);
	// com_vel_desired_(1) = M_PI/2*cos(2*M_PI/8*current_time_-tc.command_time);
	com_pos_error_ = com_pos_desired_ - com_pos_current_;
	com_vel_error_ = com_vel_desired_ - com_vel_current_;

	f_star(0) = kd_compos_(0, 0) * (com_vel_error_(0)) + kp_compos_(0, 0) * (com_pos_error_(0)) + com_acc_desired_(0);		//X axis PD control
	f_star(1) = kd_compos_(1, 1) * (com_vel_error_(1)) + kp_compos_(1, 1) * (com_pos_error_(1)) + com_acc_desired_(1);		//Y axis PD control
	f_star(2) = kd_compos_(2, 2) * (com_vel_error_(2)) + kp_compos_(2, 2) * (com_pos_error_(2)) + com_acc_desired_(2);

	f_star(0) *= rd_.com_.mass;				//cancle out mass effect
	f_star(1) *= rd_.com_.mass;
	f_star(2) *= rd_.com_.mass;
	
	// f_star.segment(0, 3).setZero();
	
	phi_pelv_ = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());
	torque_pelv_ = kp_pelv_ori_ * phi_pelv_ - kd_pelv_ori_ * pelv_angvel_current_;
	torque_pelv_(2) = 0;
	f_star.segment(3,3) = torque_pelv_*3;
	/////////////////////////////////////////////

	/////////////////////JACOBIAN///////////////////////////////////////
	lfoot_to_com_jac_from_global_.block(0, 6, 6, 1).setZero(); 		// 	left yaw
	lfoot_to_com_jac_from_global_.block(0, 12, 6, 6).setZero(); 	//	right leg
	lfoot_to_com_jac_from_global_.block(0, 21, 6, 8).setZero();		//	left arm
	lfoot_to_com_jac_from_global_.block(0, 29, 6, 2).setZero();		//	head
	lfoot_to_com_jac_from_global_.block(0, 31, 6, 8).setZero();		// 	right arm
	
	lfoot_to_com_jac_from_global_.block(3, 9, 3, 3).setZero();		// 	left leg roational component: knee pitch, ankle pitch, ankle roll
	

	rfoot_to_com_jac_from_global_.block(0, 6, 6, 6).setZero();		//	left leg
	rfoot_to_com_jac_from_global_.block(0, 12, 6, 1).setZero(); 	// 	right yaw
	rfoot_to_com_jac_from_global_.block(0, 21, 6, 8).setZero();		//	left arm
	rfoot_to_com_jac_from_global_.block(0, 29, 6, 2).setZero();		//	head
	rfoot_to_com_jac_from_global_.block(0, 31, 6, 8).setZero();		//	right arm

	rfoot_to_com_jac_from_global_.block(3, 15, 3, 3).setZero();		// 	right leg roational component: knee pitch, ankle pitch, ankle roll

	// std::cout<<"rfoot_to_com_jac_from_global_.block(3, 18, 3, 3): \n"<<rfoot_to_com_jac_from_global_.block(3, 18, 3, 3)<<std::endl;
	
	
	if(foot_swing_trigger_ == true)
	{
		if(foot_contact_ == 1)			//left support
		{	
			h_l = (com_pos_current_ - lfoot_transform_current_from_global_.translation())(2)+0.12;
			h_l = DyrosMath::minmax_cut(h_l, 0.1, 1.2);

			// x axis
			gamma_l_min = (com_pos_current_(0) - (lfoot_transform_current_from_global_.translation()(0)+0.20))/h_l;
			gamma_l_max = (com_pos_current_(0) - (lfoot_transform_current_from_global_.translation()(0)-0.15))/h_l;

			f_star(0) = DyrosMath::minmax_cut( f_star(0), f_star(2)*gamma_l_min, f_star(2)*gamma_l_max);


			// y axis
			gamma_l_min = (com_pos_current_(1) - (lfoot_transform_current_from_global_.translation()(1)+0.07))/h_l;
			gamma_l_max = (com_pos_current_(1) - (lfoot_transform_current_from_global_.translation()(1)-0.07))/h_l;

			f_star(1) = DyrosMath::minmax_cut( f_star(1), f_star(2)*gamma_l_min, f_star(2)*gamma_l_max);


			f_star_l_ = 0.5*f_star + 0.5*f_star_l_pre_;	//lpf
			f_star_r_ = 0.5*f_star_r_pre_;


		}
		else if(foot_contact_ == -1)	//right support
		{
			h_r = (com_pos_current_ - rfoot_transform_current_from_global_.translation())(2)+0.12;
			h_r = DyrosMath::minmax_cut(h_r, 0.1, 1.2);

			// x axis
			gamma_r_min = (com_pos_current_(0) - (rfoot_transform_current_from_global_.translation()(0)+0.20))/h_r;
			gamma_r_max = (com_pos_current_(0) - (rfoot_transform_current_from_global_.translation()(0)-0.15))/h_r;

			f_star(0) = DyrosMath::minmax_cut( f_star(0), f_star(2)*gamma_r_min, f_star(2)*gamma_r_max);

			// y axis
			gamma_r_min = (com_pos_current_(1) - rfoot_transform_current_from_global_.translation()(1)-0.07)/h_r;
			gamma_r_max = (com_pos_current_(1) - rfoot_transform_current_from_global_.translation()(1)+0.07)/h_r;

			f_star(1) = DyrosMath::minmax_cut( f_star(1), f_star(2)*gamma_r_min, f_star(2)*gamma_r_max);

			f_star_r_ = 0.5*f_star + 0.5*f_star_r_pre_;	//lpf
			f_star_l_ = 0.5*f_star_l_pre_;
		}
	}
	else
	{ 


		alpha_unit = (lfoot_transform_current_from_global_.translation() - rfoot_transform_current_from_global_.translation());
		alpha_unit(2) = 0;
		alpha_unit.normalize();

		beta_unit = DyrosMath::rotateWithZ( -M_PI/2) * alpha_unit;

		d_l = (com_pos_current_ - lfoot_transform_current_from_global_.translation()).transpose()*alpha_unit;
		d_l = DyrosMath::minmax_cut(d_l, -1, 0);
		d_l = abs(d_l);
		d_r = (com_pos_current_ - rfoot_transform_current_from_global_.translation()).transpose()*alpha_unit;
		d_r = DyrosMath::minmax_cut(d_r, 0, 1);
		d_r = abs(d_r);

		w_l = d_r/(d_l + d_r);
		w_r = d_l/(d_l + d_r);

		h_l = (com_pos_current_ - lfoot_transform_current_from_global_.translation())(2)+0.12;
		h_r = (com_pos_current_ - rfoot_transform_current_from_global_.translation())(2)+0.12;
		h_l = DyrosMath::minmax_cut(h_l, 0.1, 1.2);
		h_r = DyrosMath::minmax_cut(h_r, 0.1, 1.2);

		gamma_l = d_l/h_l;
		gamma_r = d_r/h_r;

		f_star_mag_alpha = (f_star.segment(0,3).transpose()*alpha_unit);
		f_star_mag_beta = (f_star.segment(0,3).transpose()*beta_unit);
		f_star_mag_z = f_star(2);

		f_star_mag_alpha_lfoot = (gamma_l*f_star_mag_alpha - gamma_l*gamma_r*f_star_mag_z)/(gamma_l + gamma_r);
		f_star_mag_z_lfoot = (-f_star_mag_alpha + gamma_r*f_star_mag_z)/(gamma_l + gamma_r);

		f_star_mag_alpha_rfoot = (gamma_r*f_star_mag_alpha + gamma_l*gamma_r*f_star_mag_z)/(gamma_l + gamma_r);
		f_star_mag_z_rfoot = (f_star_mag_alpha + gamma_l*f_star_mag_z)/(gamma_l + gamma_r);

		f_star_mag_beta_lfoot =  w_l*f_star_mag_beta;
		f_star_mag_beta_rfoot =  w_r*f_star_mag_beta;

		f_star_l_.segment(0, 3) = f_star_mag_alpha_lfoot*alpha_unit + f_star_mag_beta_lfoot*beta_unit;
		f_star_l_(2) += f_star_mag_z_lfoot;
		f_star_l_.segment(3, 3) = w_l*f_star.segment(3, 3);


		f_star_r_.segment(0, 3) = f_star_mag_alpha_rfoot*alpha_unit + f_star_mag_beta_rfoot*beta_unit;
		f_star_r_(2) += f_star_mag_z_rfoot;
		f_star_r_.segment(3, 3) = w_r*f_star.segment(3, 3);

		f_star_l_ = 0.5*f_star_l_ +0.5*f_star_l_pre_;
		f_star_r_ = 0.5*f_star_r_ +0.5*f_star_r_pre_;
	}
	
	torque_l_vel_tun = (lfoot_to_com_jac_from_global_.transpose() * (f_star_l_ )).segment(6, MODEL_DOF);
	torque_r_vel_tun = (rfoot_to_com_jac_from_global_.transpose() * (f_star_r_ )).segment(6, MODEL_DOF);


	////////////TORQUE CLACULATION/////////////////////////////////
	double lfoot_torque_g_switch;
	double rfoot_torque_g_switch;
	double lfoot_task_torque_switch;
	double rfoot_task_torque_switch;

	if (foot_swing_trigger_ == true)
	{
		double vel_tune_switching = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);

		if (foot_contact_ == 1) //left support
		{
			if (first_step_trigger_ == true)
			{
				lfoot_task_torque_switch = 1;
				rfoot_task_torque_switch = 1 - vel_tune_switching;

				lfoot_torque_g_switch = 0;
				rfoot_torque_g_switch = vel_tune_switching;
			}
			else
			{
				lfoot_task_torque_switch = vel_tune_switching;
				rfoot_task_torque_switch = 1 - vel_tune_switching;

				lfoot_torque_g_switch = 1 - vel_tune_switching;
				rfoot_torque_g_switch = vel_tune_switching;
			}
		}
		else if (foot_contact_ == -1)
		{
			if (first_step_trigger_ == true)
			{
				lfoot_task_torque_switch = 1 - vel_tune_switching;
				rfoot_task_torque_switch = 1;

				lfoot_torque_g_switch = vel_tune_switching;
				rfoot_torque_g_switch = 0;
			}
			else
			{
				lfoot_task_torque_switch = 1 - vel_tune_switching;
				rfoot_task_torque_switch = vel_tune_switching;

				lfoot_torque_g_switch = vel_tune_switching;
				rfoot_torque_g_switch = 1 - vel_tune_switching;
			}
		}
	}
	else
	{
		if(stance_start_time_ == program_start_time_)
		{
			
			lfoot_task_torque_switch = first_torque_supplier_;
			rfoot_task_torque_switch = first_torque_supplier_;

			lfoot_torque_g_switch = 0;
			rfoot_torque_g_switch = 0;

		}
		else if(stop_walking_trigger_ == true)
		{
			if (foot_contact_ == -1) // right support, left support previously
			{

				lfoot_task_torque_switch = 1;
				rfoot_task_torque_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + walking_duration_ * switching_phase_duration_, 0, 1, 0, 0);

				lfoot_torque_g_switch = 0;
				rfoot_torque_g_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + walking_duration_ * switching_phase_duration_, 1, 0, 0, 0);
			}
			else if (foot_contact_ == 1)
			{

				lfoot_task_torque_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + walking_duration_ * switching_phase_duration_, 0, 1, 0, 0);
				rfoot_task_torque_switch = 1;

				lfoot_torque_g_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + walking_duration_ * switching_phase_duration_, 1, 0, 0, 0);
				rfoot_torque_g_switch = 0;
			}
		}
		else if(start_walking_trigger_ == true)
		{
			if (foot_contact_ == -1) // right support, left support previously
			{
				// torque_l_vel_tun.segment(12, 3).setZero(); // waist torque
				// torque_l_vel_tun.segment(23, 2).setZero(); // head torque

				lfoot_task_torque_switch = 1;
				rfoot_task_torque_switch = 1;

				lfoot_torque_g_switch = 0;
				rfoot_torque_g_switch = 0;
			}
			else if (foot_contact_ == 1)
			{
				// torque_r_vel_tun.segment(12, 3).setZero(); // waist torque
				// torque_r_vel_tun.segment(23, 2).setZero(); // head torque

				lfoot_task_torque_switch = 1;
				rfoot_task_torque_switch = 1;

				lfoot_torque_g_switch = 0;
				rfoot_torque_g_switch = 0;
			}
		}
	}

	torque_g_.segment(0, 6) = torque_g_.segment(0, 6) * lfoot_torque_g_switch;
	torque_g_.segment(6, 6) = torque_g_.segment(6, 6) * rfoot_torque_g_switch;
	
	// torque_g_.segment(12, 3).setZero();
	// if( int(walking_duration_*100)%10 == 0 )
	// cout<<"walking_phase_: \n"<<walking_phase_<<endl;
	// torque += torque_l_vel_tun * lfoot_task_torque_switch + torque_r_vel_tun * rfoot_task_torque_switch;
	// torque += torque_l_vel_tun + torque_r_vel_tun;
	// torque(0) =0;
	// torque(6) =0;
	/////////////////////////////////////////////////////////////////

	// torque = wc.task_control_torque_QP2(rd_, jac_com_xy, f_com);  //jacobian control + gravity torque

	// if(int(control_time_) %2 ==0)
	// {
	//     cout<<"Com Vel torque: \n"<< torque <<endl;
	//     cout<<"f_com: \n"<< f_com <<endl;
	// }

	torque += torque_g_;

	// if( walking_phase_< 0.1)
	// {
	// 	cout<<"pelv_rot_current_: \n"<<pelv_rot_current_<<endl;
	// 	cout<<"pelv_rpy_current_: \n"<<pelv_rpy_current_<<endl;
	// 	cout<<"f_star: \n"<<f_star<<endl;
	// 	cout<<"torque: \n"<<torque<<endl;
	// 	cout<<"torque_g: \n"<<torque_g_<<endl;
	// 	cout<<"lfoot_task_torque_switch: \n"<<lfoot_task_torque_switch<<endl;
	// 	cout<<"rfoot_task_torque_switch: \n"<<rfoot_task_torque_switch<<endl;
	// 	cout<<"torque_l_vel_tun: \n"<<torque_l_vel_tun<<endl;
	// 	cout<<"torque_r_vel_tun: \n"<<torque_r_vel_tun<<endl;
	// 	cout<<"lfoot_to_com_jac_from_global_: \n"<<lfoot_to_com_jac_from_global_<<endl;
	// }

	// if( (walking_phase_ > 0.2) && (walking_phase_ < 0.7))
	// torque = tuneTorqueForZMPSafety(torque); //turn off velocity tuning if the zmp is outside of the foot

	return torque;
}

Eigen::VectorQd CustomController::swingFootControlCompute(WholebodyController &wbc)
{	
	VectorQd torque; 
	torque.setZero();

	if(foot_swing_trigger_ == true)
	{
		Vector3d lhip_joint_position;
		lhip_joint_position << 0.11, 0.1025, -0.1025;
		lhip_joint_position = pelv_rot_current_yaw_aline_*lhip_joint_position;
		Vector3d rhip_joint_position;
		rhip_joint_position << 0.11, -0.1025, -0.1025;
		rhip_joint_position = pelv_rot_current_yaw_aline_*rhip_joint_position;

		Vector3d landing_3d_point;
		Vector2d desired_landing_xy_point;
		double swingfoot_target_weight;

		// swingfoot_target_weight = (walking_phase_ - 0.3)/0.2;
		swingfoot_target_weight = 1;
		swingfoot_target_weight = DyrosMath::minmax_cut(swingfoot_target_weight, 0, 1);

		desired_landing_xy_point(0) = walking_speed_*walking_duration_ + support_foot_transform_current_.translation()(0);
		desired_landing_xy_point(1) = step_width_*(-foot_contact_) + support_foot_transform_current_.translation()(1);

		landing_3d_point.segment(0, 2) = swingfoot_target_weight*swing_foot_pos_trajectory_from_global_.segment(0, 2) + (1-swingfoot_target_weight)*desired_landing_xy_point;
		landing_3d_point(2) = swing_foot_pos_trajectory_from_global_(2);


		
		if(foot_contact_ == -1)
		{
			swingfoot_f_star_l_ = landing_3d_point - lhip_joint_position;
			swingfoot_f_star_l_.normalize();
			swingfoot_f_star_l_ *= 1*rd_.com_.mass*GRAVITY;
			swingfoot_f_star_l_ = 0.7*swingfoot_f_star_l_ + 0.3*swingfoot_f_star_l_pre_;
			
			swingfoot_f_star_r_ = 0.3*swingfoot_f_star_r_pre_;
		}
		else if(foot_contact_ == 1)
		{
			swingfoot_f_star_r_ = landing_3d_point - rhip_joint_position;
			swingfoot_f_star_r_.normalize();
			swingfoot_f_star_r_ *= 1*rd_.com_.mass*GRAVITY;
			swingfoot_f_star_r_ = 0.7*swingfoot_f_star_r_ + 0.3*swingfoot_f_star_r_pre_;
			
			swingfoot_f_star_l_ = 0.3*swingfoot_f_star_l_pre_;
		}
	}
	else
	{
		swingfoot_f_star_l_ = 0.3*swingfoot_f_star_l_pre_;
		swingfoot_f_star_r_ = 0.3*swingfoot_f_star_r_pre_;
	}
	

	torque += ((jac_lfoot_.block(0, 6, 3, MODEL_DOF)).transpose()*swingfoot_f_star_l_ + (jac_rfoot_.block(0, 6, 3, MODEL_DOF)).transpose()*swingfoot_f_star_r_)*swingfoot_force_control_converter_;

	return torque;
}

Eigen::VectorQd CustomController::jointTrajectoryPDControlCompute(WholebodyController &wbc)
{
	Eigen::VectorQd torque;
	Eigen::Vector12d desired_q_leg;
	Eigen::Isometry3d pelv_transform_from_global;
	Eigen::Isometry3d lleg_transform_from_global;
	Eigen::Isometry3d rleg_transform_from_global;
	Eigen::Isometry3d lleg_transform_target;
	Eigen::Isometry3d rleg_transform_target;
	Eigen::Matrix3d pevl_target_rot;
	Eigen::Vector3d torque_pelv_for_ankle;
 
	// double default_stance_foot_z_from_pelv = -0.349 * (cos(0.02) + cos(0.12)) - 0.1025;
	// lleg_transform_target.translation()(0) = -0.015;
	// lleg_transform_target.translation()(1) = 0.1025;
	// lleg_transform_target.translation()(2) = default_stance_foot_z_from_pelv;
	lleg_transform_target.linear().setIdentity();
	// rleg_transform_target.translation()(0) = -0.015;
	// rleg_transform_target.translation()(1) = -0.1025;
	// rleg_transform_target.translation()(2) = default_stance_foot_z_from_pelv;
	rleg_transform_target.linear().setIdentity();
	lleg_transform_target.translation() = lfoot_transform_current_from_global_.translation();
	rleg_transform_target.translation() = rfoot_transform_current_from_global_.translation();

	// cout<<"lfoot_transform_init_from_global_.translation(): \n"<<lfoot_transform_init_from_global_.translation()<<endl;
	// cout<<"lfoot_transform_init_from_global_.linear(): \n"<<lfoot_transform_init_from_global_.linear()<<endl;
	// cout<<"rfoot_transform_init_from_global_.translation(): \n"<<rfoot_transform_init_from_global_.translation()<<endl;
	// cout<<"rfoot_transform_init_from_global_.linear(): \n"<<rfoot_transform_init_from_global_.linear()<<endl;
	/////////////////////////////////PELVIS/////////////////////////////////////

	pelv_transform_from_global.translation().setZero();
	pelv_transform_from_global.linear() = pelv_rot_current_yaw_aline_; //
	// pelv_transform_from_global.linear() = pelv_yaw_rot_current_from_global_;
	// pelv_transform_from_global.linear().setIdentity();


	torque_pelv_.setZero();
	torque_pelv_for_ankle.setZero();
	torque_swing_assist_.setZero();

	//////////////////////////////////////////////////////////////////////////////////////////


	double swing_pd_switch;


	torque.setZero();

	//////////////////////////SWING FOOT & STANCE FOOT ankle and knee joints///////////////////////
	if (foot_swing_trigger_ == true)
	{
		if (foot_contact_ == -1) //right support
		{
			lleg_transform_from_global.translation() = swing_foot_pos_trajectory_from_global_;
			lleg_transform_from_global.linear() = swing_foot_rot_trajectory_from_global_;

			rleg_transform_from_global.translation()(0) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(0), 0, 0, rleg_transform_target.translation()(0), 0, 0)(0);
			rleg_transform_from_global.translation()(1) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(1), 0, 0, rleg_transform_target.translation()(1), 0, 0)(0);
			rleg_transform_from_global.translation()(2) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(2), 0, 0, rleg_transform_target.translation()(2), 0, 0)(0);

			rleg_transform_from_global.linear() = rleg_transform_target.linear();
			// rleg_transform_from_global = rleg_transform_target;
			computeIk(pelv_transform_from_global, lleg_transform_from_global, rleg_transform_from_global, desired_q_leg);

			for (int i = 1; i < 4; i++) //hip and knee
			{
				desired_q_(i) = desired_q_leg(i); //left swing foot
																					// kp_joint(i) = 900; //swing foot gain
																					// kv_joint(i) = 60;

				// if(walking_phase_ < 0.1)
				// {
				//     cout<<"foot_contact_: \n"<<foot_contact_<<endl;
				//     cout<<"desired_q_leg: \n"<<desired_q_leg<<endl;
				//     cout<<"lleg_transform_from_global.translation(): \n" <<lleg_transform_from_global.translation()<<endl;
				//     cout<<"lleg_transform_from_global.rotation(): \n" <<lleg_transform_from_global.rotation()<<endl;
				//     cout<<"rleg_transform_from_global.translation(): \n" <<rleg_transform_from_global.traznslation()<<endl;
				//     cout<<"rleg_transform_from_global.rotation(): \n" <<rleg_transform_from_global.rotation()<<endl;
				//     cout<<"pelv_transform_from_global.translation(): \n" <<pelv_transform_from_global.translation()<<endl;
				//     cout<<"pelv_transform_from_global.rotation(): \n" <<pelv_transform_from_global.rotation()<<endl;
				// }
			}

			desired_q_(0) = 0.5 * 0 + 0.5 * pre_desired_q_(0);
			desired_q_(3) = DyrosMath::QuinticSpline(walking_phase_, 0.0, switching_phase_duration_, last_desired_q_(3), 0, 0, desired_q_leg(3), 0, 0)(0);

			// desired_q_(0) = motion_q_(0);
			Vector3d phi_swing_ankle;
			// phi_swing_ankle = -DyrosMath::getPhi(rd_.link_[Left_Foot].Rotm, pelv_yaw_rot_current_from_global_);
			phi_swing_ankle = -DyrosMath::getPhi(lfoot_transform_current_from_global_.linear(), Eigen::Matrix3d::Identity());
			// phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

			desired_q_dot_(4) = 200 * bandBlock(phi_swing_ankle(1), 15*DEG2RAD, -15*DEG2RAD); //swing ankle pitch	//(tune)
			desired_q_dot_(5) = 200 * bandBlock(phi_swing_ankle(0), 0*DEG2RAD, 0*DEG2RAD); //swing ankle roll	//(tune)
			desired_q_(4) = current_q_(4) + desired_q_dot_(4) * dt_;
			desired_q_(5) = current_q_(5) + desired_q_dot_(5) * dt_;
			// desired_q_dot_(4) = 0;
			// desired_q_dot_(5) = 0;
			// // desired_q_(4) = current_q_(4);
			// // desired_q_(5) = current_q_(5);
			// desired_q_(4) = 0;
			// desired_q_(5) = 0;

			Vector3d phi_support_ankle;
			// phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Right_Foot].Rotm, pelv_yaw_rot_current_from_global_);
			phi_support_ankle = -DyrosMath::getPhi(rfoot_transform_current_from_global_.linear(), DyrosMath::rotateWithZ(current_q_(6)));


			desired_q_dot_(10) = 200 * bandBlock(phi_support_ankle(1), 15*DEG2RAD, -15*DEG2RAD);		//(tune)
			desired_q_dot_(11) = 200 * bandBlock(phi_support_ankle(0), 10*DEG2RAD, -10*DEG2RAD);		//(tune)
			desired_q_(10) = current_q_(10) + desired_q_dot_(10) * dt_;
			desired_q_(11) = current_q_(11) + desired_q_dot_(11) * dt_;
			// desired_q_dot_(10) = 0;
			// desired_q_dot_(11) = 0;
			// desired_q_(10) = current_q_(10);
			// desired_q_(11) = current_q_(11);

			// low pass filter for suppport foot target position
			// desired_q_(6) = 0.5 * motion_q_(6) + 0.5 * pre_desired_q_(6); //right support foot hip yaw
			desired_q_(6) = DyrosMath::QuinticSpline(turning_phase_, 0, 1, last_desired_q_(6), 0, 0, motion_q_(6), 0, 0)(0); 
			desired_q_(9) = DyrosMath::QuinticSpline(walking_phase_, 0.0, 0.3, last_desired_q_(9), 0, 0, motion_q_(9), 0, 0)(0);
			// desired_q_(9) = 0.5*motion_q_(9) + 0.5*pre_desired_q_(9); //right support foot
			// desired_q_(9) = desired_q_leg(9); //right support foot knee
			// desired_q_(10) = 0.5*desired_q_leg(10) + 0.5*pre_desired_q_(10); //right support foot ankle pitch
			// desired_q_(11) = 0.5*desired_q_leg(11) + 0.5*pre_desired_q_(11); //right support foot anlke roll
			// desired_q_(10) = 0.3*motion_q_(10) + 0.7*pre_desired_q_(10); //right support foot ankle pitc
			// desired_q_(11) = 0.3*motion_q_(11) + 0.7*pre_desired_q_(11); //right support foot anlke roll
			// desired_q_(10) = desired_q_leg(10) ;
			// desired_q_(11) = desired_q_leg(11) ;

			for (int i = 1; i < 6; i++)
			{
				if (kp_joint_(i + 6) == kp_soft_joint_(i + 6))
				{
					kp_joint_(i + 6) = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, kp_soft_joint_(i + 6), kp_stiff_joint_(i + 6), 0, 0); //support foot
				}

				if (kp_joint_(i) == kp_stiff_joint_(i))
				{
					kp_joint_(i) = DyrosMath::cubic(walking_phase_, 0.9, 1, kp_stiff_joint_(i), kp_soft_joint_(i), 0, 0); //swing foot
				}
			}
			// kp_joint(3) = DyrosMath::cubic(walking_phase_, 0.8, 1, 2000, 600, 0, 0); //swing foot knee
			// kv_joint(3) = DyrosMath::cubic(walking_phase_, 0.8, 1, 60, 50, 0, 0);

			// kp_joint(4) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0); //swing foot ankle gain
			// kv_joint(4) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);
			// kp_joint(5) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0);
			// kv_joint(5) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);


			swing_pd_switch = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);

			if (first_step_trigger_ == true)
			{
				pd_control_mask_(0) = 1;
				pd_control_mask_(1) = swing_pd_switch;
				pd_control_mask_(2) = swing_pd_switch;
				// pd_control_mask_(3) = swing_pd_switch;
				// pd_control_mask_(4) = swing_pd_switch; //test
				// pd_control_mask_(5) = swing_pd_switch;

				pd_control_mask_(6) = 1;
				pd_control_mask_(7) = 0;
				pd_control_mask_(8) = 0;
				// pd_control_mask_(9) = 0;
				// pd_control_mask_(10) = 0; //test
				// pd_control_mask_(11) = 0;
			}
			else
			{
				pd_control_mask_(0) = 1;
				pd_control_mask_(1) = swing_pd_switch;
				pd_control_mask_(2) = swing_pd_switch;
				// pd_control_mask_(3) = swing_pd_switch;

				pd_control_mask_(6) = 1;
				pd_control_mask_(7) = 1 - swing_pd_switch;
				pd_control_mask_(8) = 1 - swing_pd_switch;
				// pd_control_mask_(9) = 0;
				// pd_control_mask_(10) = 1 - swing_pd_switch; //test
				// pd_control_mask_(11) = 1 - swing_pd_switch;
			}
		}
		else if (foot_contact_ == 1) //left support
		{

			rleg_transform_from_global.translation() = swing_foot_pos_trajectory_from_global_;
			rleg_transform_from_global.linear() = swing_foot_rot_trajectory_from_global_;

			lleg_transform_from_global.translation()(0) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(0), 0, 0, lleg_transform_target.translation()(0), 0, 0)(0);
			lleg_transform_from_global.translation()(1) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(1), 0, 0, lleg_transform_target.translation()(1), 0, 0)(0);
			lleg_transform_from_global.translation()(2) = DyrosMath::QuinticSpline(walking_phase_, 0, switching_phase_duration_, support_foot_transform_init_.translation()(2), 0, 0, lleg_transform_target.translation()(2), 0, 0)(0);

			lleg_transform_from_global.linear() = lleg_transform_target.linear();
			// lleg_transform_from_global = lleg_transform_target;

			computeIk(pelv_transform_from_global, lleg_transform_from_global, rleg_transform_from_global, desired_q_leg);

			for (int i = 7; i < 10; i++)
			{
				desired_q_(i) = desired_q_leg(i); //right swing foot
																					// kp_joint(i) = 900; //swing foot gain
																					// kv_joint(i) = 60;

				// desired_q_(i-6) = 0.5*motion_q_(i-6) + 0.5*pre_desired_q_(i-6); //left support foot
				// if(walking_phase_ < 0.1)
				// {
				//     cout<<"desired_q_leg: \n"<<desired_q_leg<<endl;
				//     cout<<"foot_contact_: \n"<<foot_contact_<<endl;
				//     cout<<"lleg_transform_from_global.translation(): \n" <<lleg_transform_from_global.translation()<<endl;
				//     cout<<"lleg_transform_from_global.rotation(): \n" <<lleg_transform_from_global.rotation()<<endl;
				//     cout<<"rleg_transform_from_global.translation(): \n" <<rleg_transform_from_global.translation()<<endl;
				//     cout<<"rleg_transform_from_global.rotation(): \n" <<rleg_transform_from_global.rotation()<<endl;
				//     cout<<"pelv_transform_from_global.translation(): \n" <<pelv_transform_from_global.translation()<<endl;
				//     cout<<"pelv_transform_from_global.rotation(): \n" <<pelv_transform_from_global.rotation()<<endl;
				// }
			}

			desired_q_(6) = 0.5 * 0 + 0.5 * pre_desired_q_(6);
			desired_q_(9) = DyrosMath::QuinticSpline(walking_phase_, 0.0, switching_phase_duration_, last_desired_q_(9), 0, 0, desired_q_leg(9), 0, 0)(0);

			Vector3d phi_swing_ankle;
			// phi_swing_ankle = -DyrosMath::getPhi(rd_.link_[Right_Foot].Rotm, pelv_yaw_rot_current_from_global_);
			phi_swing_ankle = -DyrosMath::getPhi(rfoot_transform_current_from_global_.linear(), Eigen::Matrix3d::Identity());
			// phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

			desired_q_dot_(10) = 200 * bandBlock(phi_swing_ankle(1), 15*DEG2RAD, -15*DEG2RAD); //swing ankle pitch
			desired_q_dot_(11) = 200 * bandBlock(phi_swing_ankle(0), 0*DEG2RAD, 0*DEG2RAD); //swing ankle roll
			desired_q_(10) = current_q_(10) + desired_q_dot_(10) * dt_;
			desired_q_(11) = current_q_(11) + desired_q_dot_(11) * dt_;
			// desired_q_dot_(10) = 0;
			// desired_q_dot_(11) = 0;
			// // desired_q_(10) = current_q_(10);
			// // desired_q_(11) = current_q_(11);
			// desired_q_(10) = 0;
			// desired_q_(11) = 0;
			


			Vector3d phi_support_ankle;
			// phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Left_Foot].Rotm, pelv_yaw_rot_current_from_global_);
			phi_support_ankle = -DyrosMath::getPhi(lfoot_transform_current_from_global_.linear(), DyrosMath::rotateWithZ(current_q_(0)));


			desired_q_dot_(4) = 200 * bandBlock(phi_support_ankle(1), 15*DEG2RAD, -15*DEG2RAD);;
			desired_q_dot_(5) = 200 * bandBlock(phi_support_ankle(0), 10*DEG2RAD, -10*DEG2RAD);;
			desired_q_(4) = current_q_(4) + desired_q_dot_(4) * dt_;
			desired_q_(5) = current_q_(5) + desired_q_dot_(5) * dt_;
			// desired_q_dot_(4) = 0;
			// desired_q_dot_(5) = 0;
			// desired_q_(4) = current_q_(4);
			// desired_q_(5) = current_q_(5);

			// desired_q_(0) = 0.5 * motion_q_(0) + 0.5 * pre_desired_q_(0); //left support foot hip yaw
			desired_q_(0) = DyrosMath::QuinticSpline(turning_phase_, 0, 1, last_desired_q_(0), 0, 0, motion_q_(0), 0, 0)(0);  //left support foot hip yaw
			desired_q_(3) = DyrosMath::QuinticSpline(walking_phase_, 0.0, 0.3, last_desired_q_(3), 0, 0, motion_q_(3), 0, 0)(0);
			// desired_q_(3) = 0.5*motion_q_(3) + 0.5*pre_desired_q_(3); //left support foot knee
			// desired_q_(3) = desired_q_leg(3); //left support foot knee
			// desired_q_(4) = 0.5*desired_q_leg(4) + 0.5*pre_desired_q_(4); //left support foot ankle pitch
			// desired_q_(5) = 0.5*desired_q_leg(5) + 0.5*pre_desired_q_(5); //left support foot anlke roll
			// desired_q_(4) = 0.3*motion_q_(4) + 0.7*pre_desired_q_(4); //left support foot ankle pitch
			// desired_q_(5) = 0.3*motion_q_(5) + 0.7*pre_desired_q_(5); //left support foot anlke roll
			// desired_q_(4) = desired_q_leg(4) ;
			// desired_q_(5) = desired_q_leg(5) ;

			for (int i = 1; i < 6; i++)
			{
				if (kp_joint_(i) == kp_soft_joint_(i))
				{
					kp_joint_(i) = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, kp_soft_joint_(i), kp_stiff_joint_(i), 0, 0); //support foot
				}

				if (kp_joint_(i + 6) == kp_stiff_joint_(i + 6))
				{
					kp_joint_(i + 6) = DyrosMath::cubic(walking_phase_, 0.9, 1, kp_stiff_joint_(i + 6), kp_soft_joint_(i + 6), 0, 0); //swing foot
				}
			}
			// kp_joint(9) = DyrosMath::cubic(walking_phase_, 0.8, 1, 2000, 600, 0, 0); //swing foot knee
			// kv_joint(9) = DyrosMath::cubic(walking_phase_, 0.8, 1, 60, 50, 0, 0);

			// kp_joint(10) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0); //swing foot ankle gain
			// kv_joint(10) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);
			// kp_joint(11) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0);
			// kv_joint(11) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);

			/////////////////Swing Assist Torque/////////////////////////////////////////
			// right foot swing height assist feed forward torque on hip roll, hip pitch, knee pitch
			Eigen::VectorXd f_star;
			f_star.setZero(3);
			f_star(2) = swing_foot_acc_trajectory_from_global_(2);

			// torque_swing_assist_.segment(7, 3) = wbc.task_control_torque(rd_, jac_rfoot_.block(0, 0, 3, MODEL_DOF_VIRTUAL), f_star).segment(7, 3);
			// wbc.set_contact(rd_, 1, 0);
			// torque_swing_assist_.segment(7, 3) = wbc.task_control_torque_motor(rd_, jac_rfoot_.block(0, 0, 3, MODEL_DOF_VIRTUAL), f_star).segment(7, 3);

			// torque_swing_assist_.segment(7, 3) = (jac_rfoot_.transpose()).block(13, 0, 3, 6)*rd_.lambda.block(0, 2, 6, 1)*swing_foot_acc_trajectory_from_global_(2);
			/////////////////////////////////////////////////////////////////////////////

			swing_pd_switch = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);

			if (first_step_trigger_ == true)
			{

				pd_control_mask_(0) = 1;
				pd_control_mask_(1) = 0;
				pd_control_mask_(2) = 0;
				// pd_control_mask_(3) = 0;
				// pd_control_mask_(4) = 0; //test
				// pd_control_mask_(5) = 0;

				pd_control_mask_(6) = 1;
				pd_control_mask_(7) = swing_pd_switch;
				pd_control_mask_(8) = swing_pd_switch;
				// pd_control_mask_(9) = swing_pd_switch;
			}
			else
			{
				pd_control_mask_(0) = 1;
				pd_control_mask_(1) = 1 - swing_pd_switch;
				pd_control_mask_(2) = 1 - swing_pd_switch;
				// pd_control_mask_(3) = 0;
				// pd_control_mask_(4) = 1 - swing_pd_switch; //test
				// pd_control_mask_(5) = 1 - swing_pd_switch;

				pd_control_mask_(6) = 1;
				pd_control_mask_(7) = swing_pd_switch;
				pd_control_mask_(8) = swing_pd_switch;
				// pd_control_mask_(9) = swing_pd_switch;		
			}
		}
	}
	else
	{

		lleg_transform_from_global.translation()(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, support_foot_transform_init_.translation()(0), 0, 0, lleg_transform_target.translation()(0), 0, 0)(0);
		lleg_transform_from_global.translation()(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, support_foot_transform_init_.translation()(1), 0, 0, lleg_transform_target.translation()(1), 0, 0)(0);
		lleg_transform_from_global.translation()(2) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, support_foot_transform_init_.translation()(2), 0, 0, lleg_transform_target.translation()(2), 0, 0)(0);
		lleg_transform_from_global.linear() = lleg_transform_target.linear();

		rleg_transform_from_global.translation()(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, support_foot_transform_init_.translation()(0), 0, 0, rleg_transform_target.translation()(0), 0, 0)(0);
		rleg_transform_from_global.translation()(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, support_foot_transform_init_.translation()(1), 0, 0, rleg_transform_target.translation()(1), 0, 0)(0);
		rleg_transform_from_global.translation()(2) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, support_foot_transform_init_.translation()(2), 0, 0, rleg_transform_target.translation()(2), 0, 0)(0);
		rleg_transform_from_global.linear() = rleg_transform_target.linear();
		computeIk(pelv_transform_from_global, lleg_transform_from_global, rleg_transform_from_global, desired_q_leg);

		// for(int i = 4; i<6; i++)
		// {
		//     desired_q_(i) = 0.3*desired_q_leg(i) + 0.7*pre_desired_q_(i); //left ankle
		//     desired_q_(i+6) = 0.3*desired_q_leg(i+6) + 0.7*pre_desired_q_(i+6); //right ankle
		// }
		desired_q_(0) = 0.5 * 0 + 0.5 * pre_desired_q_(0); // hip yaw
		desired_q_(6) = 0.5 * 0 + 0.5 * pre_desired_q_(6);

		desired_q_(3) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 3, last_desired_q_(3), 0, 0, motion_q_(3), 0, 0)(0);
		desired_q_(9) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 3, last_desired_q_(9), 0, 0, motion_q_(9), 0, 0)(0);
		// desired_q_(3) = 0.4 * motion_q_(3) + 0.6 * pre_desired_q_(3); //left knee
		// desired_q_(9) = 0.4 * motion_q_(9) + 0.6 * pre_desired_q_(9); //right knee
		// desired_q_(3) =	current_q_(3);
		// desired_q_(9) =	current_q_(9);

		// desired_q_(4) = 0.3*motion_q_(4) + 0.7*pre_desired_q_(4); //lefft ankle pitch
		// desired_q_(5) = 0.3*motion_q_(5) + 0.7*pre_desired_q_(5); //lefft ankle roll
		// desired_q_(10) = 0.3*motion_q_(10) + 0.7*pre_desired_q_(10); //right ankle pitch
		// desired_q_(11) = 0.3*motion_q_(11) + 0.7*pre_desired_q_(11); //right ankle roll

		// desired_q_(4) = 0.5*desired_q_leg(10) + 0.5*pre_desired_q_(10); //right support foot ankle pitch
		// desired_q_(5) = 0.5*desired_q_leg(11) + 0.5*pre_desired_q_(11); //right support foot anlke roll
		// desired_q_(10) = 0.5*desired_q_leg(10) + 0.5*pre_desired_q_(10); //right support foot ankle pitch
		// desired_q_(11) = 0.5*desired_q_leg(11) + 0.5*pre_desired_q_(11); //right support foot anlke roll

		// desired_q_(4) = desired_q_leg(4);
		// desired_q_(5) = desired_q_leg(5);
		// desired_q_(10) = desired_q_leg(10);
		// desired_q_(11) = desired_q_leg(11);
        
		desired_q_(4) = current_q_(4); // test
		desired_q_(5) = current_q_(5);
		desired_q_(10) = current_q_(10);
		desired_q_(11) = current_q_(11);

		desired_q_dot_(4) = 0; // test
		desired_q_dot_(5) = 0;
		desired_q_dot_(10) = 0;
		desired_q_dot_(11) = 0;
		for (int i = 1; i < 6; i++)
		{
			kp_joint_(i + 6) = kp_stiff_joint_(i + 6); //swing foot
			kp_joint_(i) = kp_stiff_joint_(i);				 //support foot
		}

		swing_pd_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, 0, 1, 0, 0);

		if (program_start_time_ == stance_start_time_)
		{
			// desired_q_(5) = current_q_(5); //aknle roll free for start motion
			// desired_q_(11) = current_q_(11); //aknle roll

			// kp_joint(5) = 600;
			// kv_joint(5) = 40;
			// kp_joint(11)= 600;
			// kv_joint(11)= 40;

			// kp_joint(4) = 400;
			// kv_joint(4) = 40;
			// kp_joint(10)= 400;
			// kv_joint(10)= 40;

			pd_control_mask_(6) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5 * walking_duration_, 0, 1, 0, 0);
			pd_control_mask_(7) = 0;
			pd_control_mask_(8) = 0;
			// pd_control_mask_(9) = 0;
			// pd_control_mask_(10) = 0;	//test
			// pd_control_mask_(11) = 0;
			// pd_control_mask_(9) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5 * walking_duration_, 0, 1, 0, 0);

			pd_control_mask_(0) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5 * walking_duration_, 0, 1, 0, 0);
			pd_control_mask_(1) = 0;
			pd_control_mask_(2) = 0;
			// pd_control_mask_(3) = 0;
			// pd_control_mask_(4) = 0;	//test
			// pd_control_mask_(5) = 0;
			// pd_control_mask_(3) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5 * walking_duration_, 0, 1, 0, 0);


			// cout<<"hip_control_switch: "<< hip_control_switch <<endl;
		}
		else if (stop_walking_trigger_ == true)
		{
			if (foot_contact_ == 1)
			{
				pd_control_mask_(0) = 1;
				pd_control_mask_(1) = 1 - swing_pd_switch;
				pd_control_mask_(2) = 1 - swing_pd_switch;
				// pd_control_mask_(3) = 1 - swing_pd_switch; //test
				// pd_control_mask_(4) = 1 - swing_pd_switch;; //test
				// pd_control_mask_(5) = 1 - swing_pd_switch;; //test

				pd_control_mask_(6) = 1;
				pd_control_mask_(7) = 0;
				pd_control_mask_(8) = 0;
				// pd_control_mask_(9) = 0; 
				// pd_control_mask_(10) = 0; //test
				// pd_control_mask_(11) = 0; //test
			}
			else if (foot_contact_ == -1)
			{
				pd_control_mask_(0) = 1;
				pd_control_mask_(1) = 0;
				pd_control_mask_(2) = 0;
				// pd_control_mask_(3) = 0; //test
				// pd_control_mask_(4) = 0; //test
                // pd_control_mask_(5) = 0; //test

				pd_control_mask_(6) = 1;
				pd_control_mask_(7) = 1 - swing_pd_switch;
				pd_control_mask_(8) = 1 - swing_pd_switch;
				// pd_control_mask_(9) = 1 - swing_pd_switch; //test
				// pd_control_mask_(10) = 1 - swing_pd_switch; //test
				// pd_control_mask_(11) = 1 - swing_pd_switch; //test
			}
		}
	}
	
	//////////////////////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////HIP YAW/////////////////////////////
	// desired_q_(0)=motion_q_(0);
	// desired_q_(6)=motion_q_(6);
	////////////////////////////////////////////////////////////////////

	//////////////////////////////////LEG Q DOT/////////////////////////////////
	desired_q_dot_.segment(0, 4) = (desired_q_.segment(0, 4) - pre_desired_q_.segment(0, 4)) / dt_; //left hip and knee
	desired_q_dot_.segment(6, 4) = (desired_q_.segment(6, 4) - pre_desired_q_.segment(6, 4)) / dt_; //left hip and knee
	if(walking_phase_ == 0)
	{
		desired_q_dot_.segment(0, 4).setZero();
		desired_q_dot_.segment(6, 4).setZero();
	}
	///////////////////////////////////////////////////////////////////////////

	/////////////////////////////////WAIST DESIRED JOINT ANGLES//////////////////////////////
	// Vector3d phi_trunk;
	// Matrix3d upper_body_rotaion_matrix = DyrosMath::rotateWithZ(motion_q_(12));
	// phi_trunk = -DyrosMath::getPhi(pelv_yaw_rot_current_from_global_.transpose()* rd_.link_[Upper_Body].Rotm, upper_body_rotaion_matrix);
	// // phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

	// desired_q_dot_(12) = 50 * phi_trunk(2);	 //waist yaw //(tune)
	// desired_q_dot_(13) = 30 * phi_trunk(1);	 //waist pitch
	// desired_q_dot_(14) = -30 * phi_trunk(0); //waist roll

	// desired_q_.segment(12, 3) = current_q_.segment(12, 3) + desired_q_dot_.segment(12, 3) * dt_;

	// desired_q_(12) = motion_q_(12);
	for (int i = 12; i < 15; i++)
	{
		desired_q_(i) = motion_q_(i);
		desired_q_dot_(i) = motion_q_dot_(i);
	}
	//////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////ARM & HEAD///////////////////////////////////////////////
	for (int i = 15; i < MODEL_DOF; i++)
	{
		desired_q_(i) = motion_q_(i);
		desired_q_dot_(i) = motion_q_dot_(i);
	}
	////////////////////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////MOTION CONTROL/////////////////////////////////////
	// torque += stablePDControl(1000, 1000*dt_*16, current_q_, current_q_dot_, current_q_ddot_, desired_q_, desired_q_dot_);
	// kp_joint_(4) = 0;	//only do damping control for ankle
	// kp_joint_(5) = 0;
	// kp_joint_(10) = 0;
	// kp_joint_(11) = 0;

	pd_control_mask_(1) *= 1 - swingfoot_force_control_converter_;
	pd_control_mask_(2) *= 1 - swingfoot_force_control_converter_;
	// pd_control_mask_(3) *= 1 - swingfoot_force_control_converter_; //knee
	pd_control_mask_(7) *= 1 - swingfoot_force_control_converter_;
	pd_control_mask_(8) *= 1 - swingfoot_force_control_converter_;
	// pd_control_mask_(9) *= 1 - swingfoot_force_control_converter_; //knee

	for (int i = 0; i < MODEL_DOF; i++)
	{
		torque(i) = (kp_joint_(i) * (desired_q_(i) - current_q_(i)) + kv_joint_(i) * (desired_q_dot_(i) - current_q_dot_(i)));
		torque(i) = torque(i) * pd_control_mask_(i); // masking for joint pd control
	}

	return torque;
}

Eigen::VectorQd CustomController::dampingControlCompute(WholebodyController &wbc)
{
	VectorQd torque;
	torque.setZero();
	////////////////////support angle operational space damping////////////////////////////////////////
	f_lfoot_damping_ = -support_foot_damping_gain_*lfoot_vel_current_from_global_.segment(3, 3);
	f_rfoot_damping_ = -support_foot_damping_gain_*rfoot_vel_current_from_global_.segment(3, 3);

	if(foot_swing_trigger_ == true)
	{
		if(foot_contact_ == 1)
		{
			f_lfoot_damping_ = 0.7*f_lfoot_damping_ + 0.3*f_lfoot_damping_pre_;
			f_rfoot_damping_ = 0.3*f_rfoot_damping_pre_;
		}
		else if (foot_contact_ == -1)
		{
			f_rfoot_damping_ = 0.7*f_rfoot_damping_ + 0.3*f_rfoot_damping_pre_;
			f_lfoot_damping_ = 0.3*f_lfoot_damping_pre_;
		}
	}
	else
	{
		f_lfoot_damping_ = 0.7*f_lfoot_damping_ + 0.3*f_lfoot_damping_pre_;
		f_rfoot_damping_ = 0.7*f_rfoot_damping_ + 0.3*f_rfoot_damping_pre_;
	}

	torque(4) += ((jac_lfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_lfoot_damping_ + (jac_rfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_rfoot_damping_)(4);
	torque(5) += ((jac_lfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_lfoot_damping_ + (jac_rfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_rfoot_damping_)(5);
	torque(10) += ((jac_lfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_lfoot_damping_ + (jac_rfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_rfoot_damping_)(10);
	torque(11) += ((jac_lfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_lfoot_damping_ + (jac_rfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_rfoot_damping_)(11);
	/////////////////////////////////////////////////////////////////////////////////

	///////////////////joint angle damping///////////////////////////////////////////
	torque(1) += -0.1*current_q_dot_(1)*swingfoot_force_control_converter_;		// left hip roll
	torque(2) += -0.1*current_q_dot_(2)*swingfoot_force_control_converter_;		// left hip pitch
	torque(7) += -0.1*current_q_dot_(7)*swingfoot_force_control_converter_;		// right hip roll
	torque(8) += -0.1*current_q_dot_(8)*swingfoot_force_control_converter_;		// right hip pitch

	// torque(3) += -20*current_q_dot_(3);
	// torque(9) += -20*current_q_dot_(9);

	// torque(4) += -20*current_q_dot_(4);
	// torque(5) += -50*current_q_dot_(5);
	// torque(10) += -20*current_q_dot_(10);
	// torque(11) += -50*current_q_dot_(11);
	/////////////////////////////////////////////////////////////////////////////////

	////////////////joint limit avoidance control/////////////////////////////////////////

	return torque;
}

// Eigen::VectorQd CustomController::zmpAnkleControl()
// {
// 	VectorQd zmp_ankle_torque;
// 	Vector3d zmp_target;
// 	zmp_ankle_torque.setZero();

// 	Eigen::Vector3d zmp_desired_lfoot_local;
// 	Eigen::Vector3d zmp_desired_rfoot_local;
// 	Eigen::Vector2d zmp_desired_zmp_both;

// 	double kp_zmp = 1;
// 	double kv_zmp = 0;
// 	//////////////////////zmp trajectory///////////////////////

// 	if ((foot_swing_trigger_ == true) || (walking_speed_ != 0))
// 	{
// 		if (foot_contact_ == 1)
// 		{
// 			zmp_target = lfoot_transform_current_from_global_.translation();
// 		}
// 		else if (foot_contact_ == -1)
// 		{
// 			zmp_target = rfoot_transform_current_from_global_.translation();
// 		}
// 	}
// 	else
// 	{
// 		zmp_target = middle_of_both_foot_;
// 	}

// 	// zmp_desired_from_global_ = 0.1*zmp_target + 0.9*zmp_desired_pre_;
// 	// zmp_desired_from_global_ = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, );
// 	zmp_desired_from_global_ = zmp_target;
// 	///////////////////////////////////////////////////////////////////

// 	zmp_desired_lfoot_local = lfoot_transform_current_from_global_.linear().transpose() * (zmp_desired_from_global_ - lfoot_transform_current_from_global_.translation());
// 	zmp_desired_rfoot_local = rfoot_transform_current_from_global_.linear().transpose() * (zmp_desired_from_global_ - rfoot_transform_current_from_global_.translation());

// 	if (foot_swing_trigger_ == true)
// 	{
// 		if (foot_contact_ == 1)
// 		{
// 			// zmp_ankle_torque(4) = -l_ft_(2)*zmp_desired_lfoot_local(0) + kp_zmp*(zmp_desired_lfoot_local(0) - zmp_local_lfoot_(0)) - kv_zmp*zmp_dot_local_lfoot_(0); //ankle pitch
// 			// zmp_ankle_torque(5) = l_ft_(2)*zmp_desired_lfoot_local(1) - kp_zmp*(zmp_desired_lfoot_local(1) - zmp_local_lfoot_(1)) + kv_zmp*zmp_dot_local_lfoot_(1); //ankle roll
// 			zmp_ankle_torque(4) = +kp_zmp * (zmp_desired_lfoot_local(0) - zmp_local_lfoot_(0)) - kv_zmp * zmp_dot_local_lfoot_(0); //ankle pitch
// 			zmp_ankle_torque(5) = -kp_zmp * (zmp_desired_lfoot_local(1) - zmp_local_lfoot_(1)) + kv_zmp * zmp_dot_local_lfoot_(1); //ankle roll
// 		}
// 		else if (foot_contact_ == -1)
// 		{
// 			// zmp_ankle_torque(10) = -r_ft_(2)*zmp_desired_rfoot_local(0) + kp_zmp*(zmp_desired_rfoot_local(0) - zmp_local_rfoot_(0)) - kv_zmp*zmp_dot_local_rfoot_(0); //ankle pitch
// 			// zmp_ankle_torque(11) = r_ft_(2)*zmp_desired_rfoot_local(1) - kp_zmp*(zmp_desired_rfoot_local(1) - zmp_local_rfoot_(1)) + kv_zmp*zmp_dot_local_rfoot_(1); //ankle roll
// 			zmp_ankle_torque(10) = +kp_zmp * (zmp_desired_rfoot_local(0) - zmp_local_rfoot_(0)) - kv_zmp * zmp_dot_local_rfoot_(0); //ankle pitch
// 			zmp_ankle_torque(11) = -kp_zmp * (zmp_desired_rfoot_local(1) - zmp_local_rfoot_(1)) + kv_zmp * zmp_dot_local_rfoot_(1); //ankle roll
// 		}
// 	}
// 	else
// 	{
// 		// Vector3d total_ankle_torque;
// 		// total_ankle_torque(0) = l_ft_(2)*(zmp_desired_from_global_(1) - lfoot_transform_current_from_global_.translation()(1))
// 		// + r_ft_(2)*(zmp_desired_from_global_(1) - rfoot_transform_current_from_global_.translation()(1))
// 		// -kp_zmp*(zmp_desired_from_global_(1) - zmp_measured_(1))
// 		// +kv_zmp*(zmp_dot_measured_(1)); //roll

// 		// total_ankle_torque(1) = -l_ft_(2)*(zmp_desired_from_global_(0) - lfoot_transform_current_from_global_.translation()(0))
// 		// - r_ft_(2)*(zmp_desired_from_global_(0) - rfoot_transform_current_from_global_.translation()(0));
// 		// +kp_zmp*(zmp_desired_from_global_(0) - zmp_measured_(0))
// 		// -kv_zmp*(zmp_dot_measured_(0)); //pitch

// 		// total_ankle_torque(2) = 0;

// 		// double left_zmp_ratio = abs(zmp_desired_from_global_(1) - lfoot_transform_current_from_global_.translation()(1))
// 		// /abs(rfoot_transform_current_from_global_.translation()(1) - lfoot_transform_current_from_global_.translation()(1));

// 		// left_zmp_ratio = DyrosMath::minmax_cut(left_zmp_ratio, 0, 1);

// 		// zmp_ankle_torque(4) = left_zmp_ratio*(lfoot_transform_current_from_global_.linear().transpose()*total_ankle_torque)(1);
// 		// zmp_ankle_torque(5) = left_zmp_ratio*(rfoot_transform_current_from_global_.linear().transpose()*total_ankle_torque)(0);

// 		// zmp_ankle_torque(10) = (1-left_zmp_ratio)*total_ankle_torque(1);
// 		// zmp_ankle_torque(11) = (1-left_zmp_ratio)*total_ankle_torque(0);
// 	}

// 	return zmp_ankle_torque;
// }

// Eigen::VectorQd CustomController::jointComTrackingTuning()
// {
// 	Eigen::VectorQd desired_q_tune;
// 	desired_q_tune.setZero();

// 	//ankle
// 	double kp_ank_sag = 0.1;	//x direction ankle pitch gain for com position error
// 	double kv_ank_sag = 0.10; //x direction ankle pitch gain for com velocity error

// 	double kp_ank_cor = 0.1;	//y direction ankle pitch gain for com position error
// 	double kv_ank_cor = 0.1; //y direction ankle pitch gain for com velocity error

// 	//hip
// 	double kp_hip_sag = 0; //0.01; //x direction ankle pitch gain for com position error
// 	double kv_hip_sag = 0; //0.05; //x direction ankle pitch gain for com velocity error

// 	double kp_hip_cor = 0; //0.2; //y direction ankle pitch gain for com position error
// 	double kv_hip_cor = 0; //0.2; //y direction ankle pitch gain for com velocity error

// 	if (foot_swing_trigger_ == true)
// 	{
// 		double switching = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);
// 		if (foot_contact_ == 1) //left support
// 		{

// 			desired_q_tune(4) -= switching * (kp_ank_sag * (com_pos_desired_(0) - com_pos_current_(0)) + kv_ank_sag * (com_vel_desired_(0) - com_vel_current_(0)));
// 			desired_q_tune(5) += switching * (kp_ank_cor * (com_pos_desired_(1) - com_pos_current_(1)) + kv_ank_cor * (com_vel_desired_(1) - com_vel_current_(1)));

// 			if (desired_q_(8) < 0)
// 			{
// 				desired_q_tune(8) += switching * (kp_hip_sag * (com_pos_desired_(0) - com_pos_current_(0)) + kv_hip_sag * (com_vel_desired_(0) - com_vel_current_(0)));
// 			}
// 			desired_q_tune(7) -= switching * (kp_hip_cor * (com_pos_desired_(1) - com_pos_current_(1)) + kv_hip_cor * (com_vel_desired_(1) - com_vel_current_(1)));
// 		}
// 		else if (foot_contact_ == -1) //right support
// 		{
// 			desired_q_tune(10) -= switching * (kp_ank_sag * (com_pos_desired_(0) - com_pos_current_(0)) + kv_ank_sag * (com_vel_desired_(0) - com_vel_current_(0)));
// 			desired_q_tune(11) += switching * (kp_ank_cor * (com_pos_desired_(1) - com_pos_current_(1)) + kv_ank_cor * (com_vel_desired_(1) - com_vel_current_(1)));

// 			if (desired_q_(2) > 0)
// 			{
// 				desired_q_tune(2) += switching * (kp_hip_sag * (com_pos_desired_(0) - com_pos_current_(0)) + kv_hip_sag * (com_vel_desired_(0) - com_vel_current_(0)));
// 			}

// 			desired_q_tune(1) -= switching * (kp_hip_cor * (com_pos_desired_(1) - com_pos_current_(1)) + kv_hip_cor * (com_vel_desired_(1) - com_vel_current_(1)));
// 		}
// 	}
// 	else
// 	{
// 		desired_q_tune(4) -= kp_ank_sag * (com_pos_desired_(0) - com_pos_current_(0)) + kv_ank_sag * (com_vel_desired_(0) - com_vel_current_(0));
// 		desired_q_tune(5) += kp_ank_cor * (com_pos_desired_(1) - com_pos_current_(1)) + kv_ank_cor * (com_vel_desired_(1) - com_vel_current_(1));

// 		desired_q_tune(10) -= kp_ank_sag * (com_pos_desired_(0) - com_pos_current_(0)) + kv_ank_sag * (com_vel_desired_(0) - com_vel_current_(0));
// 		desired_q_tune(11) += kp_ank_cor * (com_pos_desired_(1) - com_pos_current_(1)) + kv_ank_cor * (com_vel_desired_(1) - com_vel_current_(1));
// 	}

// 	return desired_q_tune;
// }

Eigen::VectorQd CustomController::ikBalanceControlCompute(WholebodyController &wbc)
{
	//real
	// double kp_com = 0.2;
	// double kp_zmp = 0.04;
	//sim
	double kp_com = 0.9;
	double kp_zmp = 0.1;
	Eigen::Isometry3d lfoot_transform_desired;
	Eigen::Isometry3d rfoot_transform_desired;
	Eigen::Isometry3d pelv_transform_desired;
	Vector12d q_leg_desired;
	VectorQd torque_output, torque_g;
	double pd_control_schedule, falling_detection;
	pd_control_schedule = DyrosMath::cubic(current_time_, program_start_time_, program_start_time_ +3, 0, 1, 0, 0);
	q_leg_desired.setZero();
	torque_output.setZero();

	lfoot_transform_desired = lfoot_transform_start_from_global_;
	rfoot_transform_desired = rfoot_transform_start_from_global_;

	pelv_transform_desired.translation().setZero();
	// pelv_transform_desired.translation() = (lfoot_transform_init_from_global_.translation()+ rfoot_transform_init_from_global_.translation())/2;
	// pelv_transform_desired.translation()(2) = 0;
	pelv_transform_desired.linear() = pelv_rot_init_;

	// lfoot_transform_desired.translation()(0) = DyrosMath::cubic(current_time_, program_start_time_, program_start_time_ + 3, lfoot_transform_init_from_global_.translation()(0), 0, 0, 0);
	// rfoot_transform_desired.translation()(0) = DyrosMath::cubic(current_time_, program_start_time_, program_start_time_ + 3, rfoot_transform_init_from_global_.translation()(0), 0, 0, 0);

	// lfoot_transform_desired.linear().setIdentity();
	// rfoot_transform_desired.linear().setIdentity();
	// pelv_transform_desired.linear().setIdentity();
	lfoot_transform_desired.linear() = DyrosMath::rotationCubic(current_time_, program_start_time_, program_start_time_ + 3, lfoot_transform_start_from_global_.linear(), Eigen::Matrix3d::Identity());
	rfoot_transform_desired.linear() = DyrosMath::rotationCubic(current_time_, program_start_time_, program_start_time_ + 3, rfoot_transform_start_from_global_.linear(), Eigen::Matrix3d::Identity());
	pelv_transform_desired.linear() = DyrosMath::rotationCubic(current_time_, program_start_time_, program_start_time_ + 3, pelv_transform_start_from_global_.linear(), Eigen::Matrix3d::Identity());

	// pelv_transform_desired.translation() += kp_com*(com_pos_desired_ - com_pos_current_) - kp_zmp*(middle_of_both_foot_ - zmp_measured_);
	pelv_transform_desired.translation() += kp_compos_*(com_pos_desired_ - com_pos_current_) - kd_compos_*(middle_of_both_foot_ - zmp_measured_);
	computeIk(pelv_transform_desired, lfoot_transform_desired, rfoot_transform_desired, q_leg_desired);

	wbc.set_contact(rd_, 1, 1);
	torque_g = wbc_.gravity_compensation_torque(rd_, false, false);

	for(int i=0; i<12; i++)
	{
		torque_output(i) = pd_control_schedule*( kp_joint_(i)*(q_leg_desired(i) - current_q_(i)) - kv_joint_(i)*current_q_dot_(i) ) + torque_g(i);
	}
	
	return torque_output;
}

void CustomController::computeIk(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d &q_des)
{
	//float = World/ trunk = pelvis
	// 명주 정리 (KAJITA 책 <-> Code 구성)
	// float_trunk_transform.rotation() : float에서 바라본 pelvis rotation -> R1
	// float_trunk_transform.translation() : float에서 바라본 pelvis 좌표 -> P1
	// float_rleg_transform.rotation() : float에서 바라본 발끝 rotation -> R7
	// float_rleg_transform.translation() : float에서 바라본 발끝 좌표 -> P7
	// float_trunk_transform.translation() + float_trunk_transform.rotation()*D  : float에서 바라본 pelvis 좌표 + float 좌표계로 변환 * pelvis 좌표계에서 바라본 pelvis ~ hip까지 거리-> P2

	// R7.transpose * (P2 - P7) , P2 = P1 + R1*D

	Eigen::Vector3d R_r, R_D, L_r, L_D;

	// L_D << 0, 0.1025, -0.1225;
	// R_D << 0, -0.1025, -0.1225;
	L_D << 0.11, 0.1025, -0.1025;
	R_D << 0.11, -0.1025, -0.1025;

	L_r = float_lleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation() * L_D - float_lleg_transform.translation());
	R_r = float_rleg_transform.rotation().transpose() * (float_trunk_transform.translation() + float_trunk_transform.rotation() * R_D - float_rleg_transform.translation());

	double R_C = 0, L_C = 0, L_upper = 0.35, L_lower = 0.35, R_alpha = 0, L_alpha = 0;
	double L_max = L_upper + L_lower;
	L_r(2) = DyrosMath::minmax_cut(L_r(2), 0.2, L_max);
	R_r(2) = DyrosMath::minmax_cut(R_r(2), 0.2, L_max);

	R_C = R_r.norm();
	L_C = L_r.norm();

	//   L_C = sqrt( pow(L_r(0),2) + pow(L_r(1),2) + pow(L_r(2),2) );
	if (foot_swing_trigger_ == true)
	{
		if (foot_contact_ == 1)
		{
			if (R_C > L_max)
			{
				double mapping_xy = sqrt((pow(L_max, 2) - pow(R_r(2), 2)) / (pow(R_r(0), 2) + pow(R_r(1), 2))); //xy mapping
				// double mapping_xy =  (L_max - 1e-2)/R_r.norm();
				R_r(0) *= mapping_xy;
				R_r(1) *= mapping_xy;
				// R_r(2) *= mapping_xy;
				
				// R_r.normalize();	//test
				// R_r *= L_max;

				R_C = L_max;
				if( int(current_time_*1000)%500 == 1)
				{
					// cout<<"Swing leg tajectory is out of the workspace"<<endl;
				}
			}
		}
		else if (foot_contact_ == -1)
		{
			if (L_C > L_max)
			{
				double mapping_xy = sqrt((pow(L_max, 2) - pow(L_r(2), 2)) / (pow(L_r(0), 2) + pow(L_r(1), 2)));
				// double mapping_xy =  (L_max - 1e-2)/L_r.norm();
				L_r(0) *= mapping_xy;
				L_r(1) *= mapping_xy;
				// L_r(2) *= mapping_xy;

				// L_r.normalize();	//test
				// L_r *= L_max;

				L_C = L_max;
				if( int(current_time_*1000)%500 == 1)
				{
					// cout<<"Swing leg tajectory is out of the workspace"<<endl;
				}				
			}
		}
	}

	double temp_q_des;
	temp_q_des = (pow(L_upper, 2) + pow(L_lower, 2) - pow(L_C, 2)) / (2 * L_upper * L_lower);
	temp_q_des = DyrosMath::minmax_cut(temp_q_des, -1, 1);
	q_des(3) = abs(-acos(temp_q_des) + M_PI);
	temp_q_des = (pow(L_upper, 2) + pow(L_lower, 2) - pow(R_C, 2)) / (2 * L_upper * L_lower);
	temp_q_des = DyrosMath::minmax_cut(temp_q_des, -1, 1);
	q_des(9) = abs(-acos(temp_q_des) + M_PI);
	L_alpha = asin(DyrosMath::minmax_cut(L_upper / L_C * sin(M_PI - q_des(3)), -1, 1));
	//   L_alpha = q_des(3)/2;
	R_alpha = asin(DyrosMath::minmax_cut(L_upper / R_C * sin(M_PI - q_des(9)), -1, 1));
	//   R_alpha = q_des(9)/2;

	double temp_q_des_4;
	temp_q_des_4 = -atan2(L_r(0), sqrt(pow(L_r(1), 2) + pow(L_r(2), 2)));
	if (temp_q_des_4 > M_PI / 2)
	{
		temp_q_des_4 -= M_PI;
	}
	else if (temp_q_des_4 < -M_PI / 2)
	{
		temp_q_des_4 += M_PI;
	}
	q_des(4) = temp_q_des_4 - L_alpha;

	double temp_q_des_10;
	temp_q_des_10 = -atan2(R_r(0), sqrt(pow(R_r(1), 2) + pow(R_r(2), 2)));
	if (temp_q_des_10 > M_PI / 2)
	{
		temp_q_des_10 -= M_PI;
	}
	else if (temp_q_des_10 < -M_PI / 2)
	{
		temp_q_des_10 += M_PI;
	}
	q_des(10) = temp_q_des_10 - R_alpha;
	//   q_des(10) = -atan2(R_r(0), sqrt(pow(R_r(1),2) + pow(R_r(2),2))) - R_alpha ;

	//   if( walking_phase_ > 0.1 )
	{
		// cout<<"L_C: "<<L_C<<endl;
		// cout<<"R_C: "<<R_C<<endl;

		// cout<<"L_r: "<<L_r<<endl;
		// cout<<"R_r: "<<R_r<<endl;
		// cout<<"q_des(4): "<<q_des(4)<<endl;
		// cout<<"L_alpha: "<<L_alpha<<endl;
		// cout<<"q_des(10): "<<q_des(10)<<endl;
	}

	// trunk_lleg_rotation -> R1.transpose * R7
	// Ryaw * Rroll * Rpitch = R1.transpose * R7 * ~
	q_des(11) = atan2(R_r(1), R_r(2));
	if (q_des(11) > M_PI / 2)
	{
		q_des(11) -= M_PI;
	}
	else if (q_des(11) < -M_PI / 2)
	{
		q_des(11) += M_PI;
	}

	q_des(5) = atan2(L_r(1), L_r(2)); // Ankle roll
	if (q_des(5) > M_PI / 2)
	{
		q_des(5) -= M_PI;
	}
	else if (q_des(5) < -M_PI / 2)
	{
		q_des(5) += M_PI;
	}

	Eigen::Matrix3d R_Knee_Ankle_Y_rot_mat, L_Knee_Ankle_Y_rot_mat;
	Eigen::Matrix3d R_Ankle_X_rot_mat, L_Ankle_X_rot_mat;
	Eigen::Matrix3d R_Hip_rot_mat, L_Hip_rot_mat;

	L_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(3) - q_des(4));
	L_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(5));
	R_Knee_Ankle_Y_rot_mat = DyrosMath::rotateWithY(-q_des(9) - q_des(10));
	R_Ankle_X_rot_mat = DyrosMath::rotateWithX(-q_des(11));

	L_Hip_rot_mat.setZero();
	R_Hip_rot_mat.setZero();

	L_Hip_rot_mat = float_trunk_transform.linear().transpose() * float_lleg_transform.linear() * L_Ankle_X_rot_mat * L_Knee_Ankle_Y_rot_mat;
	R_Hip_rot_mat = float_trunk_transform.linear().transpose() * float_rleg_transform.linear() * R_Ankle_X_rot_mat * R_Knee_Ankle_Y_rot_mat;

	q_des(0) = -atan2(-L_Hip_rot_mat(0, 1), L_Hip_rot_mat(1, 1)); // Hip yaw
																																//   q_des(1) =  atan2(L_Hip_rot_mat(2,1), -L_Hip_rot_mat(0,1) * sin(q_des(0)) + L_Hip_rot_mat(1,1)*cos(q_des(0))); // Hip roll
	q_des(1) = asin(DyrosMath::minmax_cut(L_Hip_rot_mat(2, 1), -1, 1));
	q_des(2) = atan2(-L_Hip_rot_mat(2, 0), L_Hip_rot_mat(2, 2)); // Hip pitch
	q_des(3) = q_des(3);																				 // Knee pitch
	q_des(4) = q_des(4);																				 // Ankle pitch

	//   cout<<"L_Hip_rot_mat: \n"<<L_Hip_rot_mat<<endl;
	//   cout<<"R_Hip_rot_mat: \n"<<R_Hip_rot_mat<<endl;

	if (q_des(0) > M_PI / 2)
	{
		q_des(0) -= M_PI;
	}
	else if (q_des(0) < -M_PI / 2)
	{
		q_des(0) += M_PI;
	}

	if (q_des(1) > M_PI / 2)
	{
		q_des(1) -= M_PI;
	}
	else if (q_des(1) < -M_PI / 2)
	{
		q_des(1) += M_PI;
	}

	if (q_des(2) > M_PI / 2)
	{
		q_des(2) -= M_PI;
	}
	else if (q_des(2) < -M_PI / 2)
	{
		q_des(2) += M_PI;
	}

	q_des(6) = -atan2(-R_Hip_rot_mat(0, 1), R_Hip_rot_mat(1, 1));
	//   q_des(7) =  atan2(R_Hip_rot_mat(2,1), -R_Hip_rot_mat(0,1) * sin(q_des(6)) + R_Hip_rot_mat(1,1)*cos(q_des(6)));
	q_des(7) = asin(DyrosMath::minmax_cut(R_Hip_rot_mat(2, 1), -1, 1));
	q_des(8) = atan2(-R_Hip_rot_mat(2, 0), R_Hip_rot_mat(2, 2));
	q_des(9) = q_des(9);
	q_des(10) = q_des(10);

	if (q_des(6) > M_PI / 2)
	{
		q_des(6) -= M_PI;
	}
	else if (q_des(6) < -M_PI / 2)
	{
		q_des(6) += M_PI;
	}

	if (q_des(7) > M_PI / 2)
	{
		q_des(7) -= M_PI;
	}
	else if (q_des(7) < -M_PI / 2)
	{
		q_des(7) += M_PI;
	}

	if (q_des(8) > M_PI / 2)
	{
		q_des(8) -= M_PI;
	}
	else if (q_des(8) < -M_PI / 2)
	{
		q_des(8) += M_PI;
	}
}


Eigen::VectorQd CustomController::jointLimit()
{
	Eigen::VectorQd joint_limit_torque;
	joint_limit_torque.setZero();

	
	return joint_limit_torque;
}

void CustomController::savePreData()
{
	pre_time_ = current_time_;
	pre_q_ = rd_.q_;
	pre_desired_q_ = desired_q_;
	motion_q_pre_ = motion_q_;
	motion_q_dot_pre_ = motion_q_dot_;

	zmp_measured_ppre_ = zmp_measured_pre_;
	zmp_measured_pre_ = zmp_measured_;
	com_pos_desired_pre_ = com_pos_desired_;
	com_vel_desired_pre_ = com_vel_desired_;
	com_acc_desired_pre_ = com_acc_desired_;
	
	f_star_xy_pre_ = f_star_xy_;
	f_star_6d_pre_ = f_star_6d_;
	f_star_l_pre_ = f_star_l_;
	f_star_r_pre_ = f_star_r_;

	torque_task_pre_ = torque_task_;
	torque_grav_pre_ = torque_grav_;

	foot_contact_pre_ = foot_contact_;

	zmp_desired_pre_ = zmp_desired_from_global_;
	zmp_local_lfoot_pre_ = zmp_local_lfoot_;
	zmp_local_rfoot_pre_ = zmp_local_rfoot_;

	swing_foot_transform_pre_ = swing_foot_transform_current_;
	support_foot_transform_pre_ = support_foot_transform_current_;

	com_pos_desired_preview_pre_ = com_pos_desired_preview_;
	com_vel_desired_preview_pre_ = com_vel_desired_preview_;
	com_acc_desired_preview_pre_ = com_acc_desired_preview_;

	swingfoot_f_star_l_pre_ = swingfoot_f_star_l_;
	swingfoot_f_star_r_pre_ = swingfoot_f_star_r_;

	f_lfoot_damping_pre_ = f_lfoot_damping_;
	f_rfoot_damping_pre_ = f_rfoot_damping_;

	master_lhand_pose_pre_ = master_lhand_pose_;
	master_rhand_pose_pre_ = master_rhand_pose_;
	master_lelbow_pose_pre_ = master_lelbow_pose_;
	master_relbow_pose_pre_ = master_relbow_pose_;
	master_lshoulder_pose_pre_ = master_lshoulder_pose_;
	master_rshoulder_pose_pre_ = master_rshoulder_pose_;
	master_head_pose_pre_ = master_head_pose_;	
	master_upperbody_pose_pre_ = master_upperbody_pose_;	
	master_relative_lhand_pos_pre_ = master_relative_lhand_pos_;
	master_relative_rhand_pos_pre_ = master_relative_rhand_pos_;

	hmd_tracker_status_pre_ = hmd_tracker_status_;

	hmd_head_pose_pre_ = hmd_head_pose_;
	hmd_lshoulder_pose_pre_ = hmd_lshoulder_pose_;
	hmd_lupperarm_pose_pre_ = hmd_lupperarm_pose_;
	hmd_lhand_pose_pre_ = hmd_lhand_pose_;
	hmd_rshoulder_pose_pre_ = hmd_rshoulder_pose_;
	hmd_rupperarm_pose_pre_ = hmd_rupperarm_pose_;
	hmd_rhand_pose_pre_ = hmd_rhand_pose_;
	hmd_chest_pose_pre_ = hmd_chest_pose_;
	hmd_pelv_pose_pre_ = hmd_chest_pose_;
}

void CustomController::WalkingSliderCommandCallback(const std_msgs::Float32MultiArray &msg)
{
	walking_speed_ = msg.data[0];
	walking_speed_ = DyrosMath::minmax_cut(walking_speed_, -0.5, 1.0);
	
	walking_duration_cmd_ = msg.data[1];
	walking_duration_cmd_ = DyrosMath::minmax_cut(walking_duration_cmd_, 0.4, 1.0);

	yaw_angular_vel_ = msg.data[2];
	yaw_angular_vel_ = DyrosMath::minmax_cut(yaw_angular_vel_, -0.3, 0.3);

	knee_target_angle_ = msg.data[3];
	knee_target_angle_ = DyrosMath::minmax_cut(knee_target_angle_, 0.0, M_PI/2);	

	swing_foot_height_ = msg.data[4];
	swing_foot_height_ = DyrosMath::minmax_cut(swing_foot_height_, 0.005, 0.10);
}

void CustomController::UpperbodyModeCallback(const std_msgs::Float32 &msg)
{
	upper_body_mode_ = msg.data;
	upperbody_mode_recieved_ = true;
}

void CustomController::NextSwinglegCallback(const std_msgs::Float32 &msg)
{
	foot_contact_ = msg.data;
}

void CustomController::ComPosGainCallback(const std_msgs::Float32MultiArray &msg)
{
	kp_compos_(0, 0) = msg.data[0];
	kp_compos_(1, 1) = msg.data[1];
	kp_compos_(2, 2) = msg.data[2];
	kd_compos_(0, 0) = msg.data[3];
	kd_compos_(1, 1) = msg.data[4];
	kd_compos_(2, 2) = msg.data[5];
}

void CustomController::PelvOriGainCallback(const std_msgs::Float32MultiArray &msg)
{
	kp_pelv_ori_(0, 0) = msg.data[0];
	kp_pelv_ori_(1, 1) = msg.data[1];
	kp_pelv_ori_(2, 2) = msg.data[2];
	kd_pelv_ori_(0, 0) = msg.data[3];
	kd_pelv_ori_(1, 1) = msg.data[4];
	kd_pelv_ori_(2, 2) = msg.data[5];
}

void CustomController::SupportFootDampingGainCallback(const std_msgs::Float32MultiArray &msg)
{
	support_foot_damping_gain_(0, 0) = msg.data[0];
	support_foot_damping_gain_(1, 1) = msg.data[1];
	support_foot_damping_gain_(2, 2) = msg.data[2];
}

void CustomController::LegJointGainCallback(const std_msgs::Float32MultiArray &msg)
{
	kp_stiff_joint_(0)	= msg.data[0];
	kp_stiff_joint_(1)	= msg.data[1];
	kp_stiff_joint_(2)	= msg.data[2];
	kp_stiff_joint_(3)	= msg.data[3];
	kp_stiff_joint_(4)	= msg.data[4];
	kp_stiff_joint_(5)	= msg.data[5];

	kp_stiff_joint_(6)	= msg.data[6];
	kp_stiff_joint_(7)	= msg.data[7];
	kp_stiff_joint_(8)	= msg.data[8];
	kp_stiff_joint_(9)	= msg.data[9];
	kp_stiff_joint_(10)	= msg.data[10];
	kp_stiff_joint_(11)	= msg.data[11];

	kv_stiff_joint_(0)	= msg.data[12+0];
	kv_stiff_joint_(1)	= msg.data[12+1];
	kv_stiff_joint_(2)	= msg.data[12+2];
	kv_stiff_joint_(3)	= msg.data[12+3];
	kv_stiff_joint_(4)	= msg.data[12+4];
	kv_stiff_joint_(5)	= msg.data[12+5];

	kv_stiff_joint_(6)	= msg.data[12+6];
	kv_stiff_joint_(7)	= msg.data[12+7];
	kv_stiff_joint_(8)	= msg.data[12+8];
	kv_stiff_joint_(9)	= msg.data[12+9];
	kv_stiff_joint_(10)	= msg.data[12+10];
	kv_stiff_joint_(11)	= msg.data[12+11];
}

void CustomController::AlphaXCallback(const std_msgs::Float32 &msg)
{
	alpha_x_command_ = msg.data;
}

void CustomController::AlphaYCallback(const std_msgs::Float32 &msg)
{
	alpha_y_command_ = msg.data;
}

void CustomController::StepWidthCommandCallback(const std_msgs::Float32 &msg)
{
	step_width_ = msg.data;
}

void CustomController::Test1CommandCallback(const std_msgs::Float32 &msg)
{
	zmp_y_offset_ = msg.data;
}

void CustomController::Test2CommandCallback(const std_msgs::Float32 &msg)
{
	
}

void CustomController::ArmJointGainCallback(const std_msgs::Float32MultiArray &msg)
{
	//left arm kp
	kp_joint_(15)	= msg.data[0];
	kp_joint_(16)	= msg.data[1];
	kp_joint_(17)	= msg.data[2];
	kp_joint_(18)	= msg.data[3];
	kp_joint_(19)	= msg.data[4];
	kp_joint_(20)	= msg.data[5];
	kp_joint_(21)	= msg.data[6];
	kp_joint_(22)	= msg.data[7];
	//right arm kp
	kp_joint_(25)	= msg.data[0];
	kp_joint_(26)	= msg.data[1];
	kp_joint_(27)	= msg.data[2];
	kp_joint_(28)	= msg.data[3];
	kp_joint_(29)	= msg.data[4];
	kp_joint_(30)	= msg.data[5];
	kp_joint_(31)	= msg.data[6];
	kp_joint_(32)	= msg.data[7];

	//left arm kd
	kv_joint_(15)	= msg.data[8];
	kv_joint_(16)	= msg.data[9];
	kv_joint_(17)	= msg.data[10];
	kv_joint_(18)	= msg.data[11];
	kv_joint_(19)	= msg.data[12];
	kv_joint_(20)	= msg.data[13];
	kv_joint_(21)	= msg.data[14];
	kv_joint_(22)	= msg.data[15];
	//right arm kd
	kv_joint_(25)	= msg.data[8];
	kv_joint_(26)	= msg.data[9];
	kv_joint_(27)	= msg.data[10];
	kv_joint_(28)	= msg.data[11];
	kv_joint_(29)	= msg.data[12];
	kv_joint_(30)	= msg.data[13];
	kv_joint_(31)	= msg.data[14];
	kv_joint_(32)	= msg.data[15];
}

void CustomController::WaistJointGainCallback(const std_msgs::Float32MultiArray &msg)
{
	kp_joint_(12)	= msg.data[0];
	kp_joint_(13)	= msg.data[1];
	kp_joint_(14)	= msg.data[2];

	kv_joint_(12)	= msg.data[3];
	kv_joint_(13)	= msg.data[4];
	kv_joint_(14)	= msg.data[5];
}

// void CustomController::LeftControllerCallback(const VR::matrix_3_4 &msg)
// {
// 	master_lhand_pose_raw_.linear()(0, 0) = msg.firstRow[0];
// 	master_lhand_pose_raw_.linear()(0, 1) = msg.firstRow[1];
// 	master_lhand_pose_raw_.linear()(0, 2) = msg.firstRow[2];

// 	master_lhand_pose_raw_.linear()(1, 0) = msg.secondRow[0];
// 	master_lhand_pose_raw_.linear()(1, 1) = msg.secondRow[1];
// 	master_lhand_pose_raw_.linear()(1, 2) = msg.secondRow[2];

// 	master_lhand_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
// 	master_lhand_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
// 	master_lhand_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

// 	master_lhand_pose_raw_.translation()(0) = msg.firstRow[3];
// 	master_lhand_pose_raw_.translation()(1) = msg.secondRow[3];
// 	master_lhand_pose_raw_.translation()(2) = msg.thirdRow[3];
// }

// void CustomController::RightControllerCallback(const VR::matrix_3_4 &msg)
// {
// 	master_rhand_pose_raw_.linear()(0, 0) = msg.firstRow[0];
// 	master_rhand_pose_raw_.linear()(0, 1) = msg.firstRow[1];
// 	master_rhand_pose_raw_.linear()(0, 2) = msg.firstRow[2];

// 	master_rhand_pose_raw_.linear()(1, 0) = msg.secondRow[0];
// 	master_rhand_pose_raw_.linear()(1, 1) = msg.secondRow[1];
// 	master_rhand_pose_raw_.linear()(1, 2) = msg.secondRow[2];

// 	master_rhand_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
// 	master_rhand_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
// 	master_rhand_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

// 	master_rhand_pose_raw_.translation()(0) = msg.firstRow[3];
// 	master_rhand_pose_raw_.translation()(1) = msg.secondRow[3];
// 	master_rhand_pose_raw_.translation()(2) = msg.thirdRow[3];
// }

void CustomController::HmdCallback(const VR::matrix_3_4 &msg)
{
	hmd_head_pose_raw_.linear()(0, 0) = msg.firstRow[0];
	hmd_head_pose_raw_.linear()(0, 1) = msg.firstRow[1];
	hmd_head_pose_raw_.linear()(0, 2) = msg.firstRow[2];

	hmd_head_pose_raw_.linear()(1, 0) = msg.secondRow[0];
	hmd_head_pose_raw_.linear()(1, 1) = msg.secondRow[1];
	hmd_head_pose_raw_.linear()(1, 2) = msg.secondRow[2];

	hmd_head_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
	hmd_head_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
	hmd_head_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

	hmd_head_pose_raw_.translation()(0) = msg.firstRow[3];
	hmd_head_pose_raw_.translation()(1) = msg.secondRow[3];
	hmd_head_pose_raw_.translation()(2) = msg.thirdRow[3];
}

void CustomController::PoseCalibrationCallback(const std_msgs::Int8 &msg)
{
	if(msg.data == 1)	// still pose
	{
		hmd_check_pose_calibration_[0] = true;
		cout<<"Still Pose Calibration is On."<<endl;
	}
	else if(msg.data == 2)	//T pose
	{
		hmd_check_pose_calibration_[1] = true;
		cout<<"T Pose Calibration is On."<<endl;
	}
	else if(msg.data == 3)	//forward stretch
	{
		hmd_check_pose_calibration_[2] = true;
		cout<<"Forward Stretch Pose Calibration is On."<<endl;
	}
	else if(msg.data == 4)	//reset callibration
	{
		for(int i = 0; i<4; i++)
		{
			hmd_check_pose_calibration_[i] = false;
		}
		still_pose_cali_flag_ = false;
		t_pose_cali_flag_ = false;
		forward_pose_cali_flag_ = false;
		hmd_init_pose_calibration_ = true;
		cout<<"Pose Calibration is Reset."<<endl;
	}
	else if(msg.data == 5)
	{
		hmd_check_pose_calibration_[4] = true;
		still_pose_cali_flag_ = true;
		t_pose_cali_flag_ = true;
		forward_pose_cali_flag_ = true;
		cout<<"Reading Calibration Log File..."<<endl;

	}
	cout<<"Calibration Status: ["<<hmd_check_pose_calibration_[0]<<", "<<hmd_check_pose_calibration_[1]<<", "<<hmd_check_pose_calibration_[2]<<"]"<<endl;
}

void CustomController::LeftHandTrackerCallback(const VR::matrix_3_4 &msg)
{
	hmd_lhand_pose_raw_.linear()(0, 0) = msg.firstRow[0];
	hmd_lhand_pose_raw_.linear()(0, 1) = msg.firstRow[1];
	hmd_lhand_pose_raw_.linear()(0, 2) = msg.firstRow[2];

	hmd_lhand_pose_raw_.linear()(1, 0) = msg.secondRow[0];
	hmd_lhand_pose_raw_.linear()(1, 1) = msg.secondRow[1];
	hmd_lhand_pose_raw_.linear()(1, 2) = msg.secondRow[2];

	hmd_lhand_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
	hmd_lhand_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
	hmd_lhand_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

	hmd_lhand_pose_raw_.translation()(0) = msg.firstRow[3];
	hmd_lhand_pose_raw_.translation()(1) = msg.secondRow[3];
	hmd_lhand_pose_raw_.translation()(2) = msg.thirdRow[3];
}

void CustomController::RightHandTrackerCallback(const VR::matrix_3_4 &msg)
{
	hmd_rhand_pose_raw_.linear()(0, 0) = msg.firstRow[0];
	hmd_rhand_pose_raw_.linear()(0, 1) = msg.firstRow[1];
	hmd_rhand_pose_raw_.linear()(0, 2) = msg.firstRow[2];

	hmd_rhand_pose_raw_.linear()(1, 0) = msg.secondRow[0];
	hmd_rhand_pose_raw_.linear()(1, 1) = msg.secondRow[1];
	hmd_rhand_pose_raw_.linear()(1, 2) = msg.secondRow[2];

	hmd_rhand_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
	hmd_rhand_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
	hmd_rhand_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

	hmd_rhand_pose_raw_.translation()(0) = msg.firstRow[3];
	hmd_rhand_pose_raw_.translation()(1) = msg.secondRow[3];
	hmd_rhand_pose_raw_.translation()(2) = msg.thirdRow[3];
}

void CustomController::LeftElbowTrackerCallback(const VR::matrix_3_4 &msg)
{
	hmd_lupperarm_pose_raw_.linear()(0, 0) = msg.firstRow[0];
	hmd_lupperarm_pose_raw_.linear()(0, 1) = msg.firstRow[1];
	hmd_lupperarm_pose_raw_.linear()(0, 2) = msg.firstRow[2];

	hmd_lupperarm_pose_raw_.linear()(1, 0) = msg.secondRow[0];
	hmd_lupperarm_pose_raw_.linear()(1, 1) = msg.secondRow[1];
	hmd_lupperarm_pose_raw_.linear()(1, 2) = msg.secondRow[2];

	hmd_lupperarm_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
	hmd_lupperarm_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
	hmd_lupperarm_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

	hmd_lupperarm_pose_raw_.translation()(0) = msg.firstRow[3];
	hmd_lupperarm_pose_raw_.translation()(1) = msg.secondRow[3];
	hmd_lupperarm_pose_raw_.translation()(2) = msg.thirdRow[3];
}

void CustomController::RightElbowTrackerCallback(const VR::matrix_3_4 &msg)
{
	hmd_rupperarm_pose_raw_.linear()(0, 0) = msg.firstRow[0];
	hmd_rupperarm_pose_raw_.linear()(0, 1) = msg.firstRow[1];
	hmd_rupperarm_pose_raw_.linear()(0, 2) = msg.firstRow[2];

	hmd_rupperarm_pose_raw_.linear()(1, 0) = msg.secondRow[0];
	hmd_rupperarm_pose_raw_.linear()(1, 1) = msg.secondRow[1];
	hmd_rupperarm_pose_raw_.linear()(1, 2) = msg.secondRow[2];

	hmd_rupperarm_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
	hmd_rupperarm_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
	hmd_rupperarm_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

	hmd_rupperarm_pose_raw_.translation()(0) = msg.firstRow[3];
	hmd_rupperarm_pose_raw_.translation()(1) = msg.secondRow[3];
	hmd_rupperarm_pose_raw_.translation()(2) = msg.thirdRow[3];
}

void CustomController::ChestTrackerCallback(const VR::matrix_3_4 &msg)
{
	hmd_chest_pose_raw_.linear()(0, 0) = msg.firstRow[0];
	hmd_chest_pose_raw_.linear()(0, 1) = msg.firstRow[1];
	hmd_chest_pose_raw_.linear()(0, 2) = msg.firstRow[2];

	hmd_chest_pose_raw_.linear()(1, 0) = msg.secondRow[0];
	hmd_chest_pose_raw_.linear()(1, 1) = msg.secondRow[1];
	hmd_chest_pose_raw_.linear()(1, 2) = msg.secondRow[2];

	hmd_chest_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
	hmd_chest_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
	hmd_chest_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

	hmd_chest_pose_raw_.translation()(0) = msg.firstRow[3];
	hmd_chest_pose_raw_.translation()(1) = msg.secondRow[3];
	hmd_chest_pose_raw_.translation()(2) = msg.thirdRow[3];
}

void CustomController::PelvisTrackerCallback(const VR::matrix_3_4 &msg)
{
	hmd_pelv_pose_raw_.linear()(0, 0) = msg.firstRow[0];
	hmd_pelv_pose_raw_.linear()(0, 1) = msg.firstRow[1];
	hmd_pelv_pose_raw_.linear()(0, 2) = msg.firstRow[2];

	hmd_pelv_pose_raw_.linear()(1, 0) = msg.secondRow[0];
	hmd_pelv_pose_raw_.linear()(1, 1) = msg.secondRow[1];
	hmd_pelv_pose_raw_.linear()(1, 2) = msg.secondRow[2];

	hmd_pelv_pose_raw_.linear()(2, 0) = msg.thirdRow[0];
	hmd_pelv_pose_raw_.linear()(2, 1) = msg.thirdRow[1];
	hmd_pelv_pose_raw_.linear()(2, 2) = msg.thirdRow[2];

	hmd_pelv_pose_raw_.translation()(0) = msg.firstRow[3];
	hmd_pelv_pose_raw_.translation()(1) = msg.secondRow[3];
	hmd_pelv_pose_raw_.translation()(2) = msg.thirdRow[3];
}

void CustomController::TrackerStatusCallback(const std_msgs::Bool &msg)
{
	hmd_tracker_status_raw_ = msg.data;
}

void CustomController::ExosuitCallback(const geometry_msgs::PoseArray &msg)
{
	exo_suit_head_pos_raw_(0) = msg.poses[0].position.z;
	exo_suit_head_pos_raw_(1) = -msg.poses[0].position.x;
	exo_suit_head_pos_raw_(2) = msg.poses[0].position.y;
	exo_suit_head_q_raw_.x() = -msg.poses[0].orientation.z;
	exo_suit_head_q_raw_.y() = msg.poses[0].orientation.x;
	exo_suit_head_q_raw_.z() = -msg.poses[0].orientation.y;
	exo_suit_head_q_raw_.w() = msg.poses[0].orientation.w;
	
	exo_suit_lshoulder_pos_raw_(0) = msg.poses[1].position.z;
	exo_suit_lshoulder_pos_raw_(1) = -msg.poses[1].position.x;
	exo_suit_lshoulder_pos_raw_(2) = msg.poses[1].position.y;
	exo_suit_lshoulder_q_raw_.x() = -msg.poses[1].orientation.z;
	exo_suit_lshoulder_q_raw_.y() = msg.poses[1].orientation.x;
	exo_suit_lshoulder_q_raw_.z() = -msg.poses[1].orientation.y;
	exo_suit_lshoulder_q_raw_.w() = msg.poses[1].orientation.w;

	exo_suit_lupperarm_pos_raw_(0) = msg.poses[2].position.z;
	exo_suit_lupperarm_pos_raw_(1) = -msg.poses[2].position.x;
	exo_suit_lupperarm_pos_raw_(2) = msg.poses[2].position.y;
	exo_suit_lupperarm_q_raw_.x() = -msg.poses[2].orientation.z;
	exo_suit_lupperarm_q_raw_.y() = msg.poses[2].orientation.x;
	exo_suit_lupperarm_q_raw_.z() = -msg.poses[2].orientation.y;
	exo_suit_lupperarm_q_raw_.w() = msg.poses[2].orientation.w;
	
	exo_suit_llowerarm_pos_raw_(0) = msg.poses[3].position.z;
	exo_suit_llowerarm_pos_raw_(1) = -msg.poses[3].position.x;
	exo_suit_llowerarm_pos_raw_(2) = msg.poses[3].position.y;
	exo_suit_llowerarm_q_raw_.x() = -msg.poses[3].orientation.z;
	exo_suit_llowerarm_q_raw_.y() = msg.poses[3].orientation.x;
	exo_suit_llowerarm_q_raw_.z() = -msg.poses[3].orientation.y;
	exo_suit_llowerarm_q_raw_.w() = msg.poses[3].orientation.w;

	exo_suit_lhand_pos_raw_(0) = msg.poses[4].position.z;
	exo_suit_lhand_pos_raw_(1) = -msg.poses[4].position.x;
	exo_suit_lhand_pos_raw_(2) = msg.poses[4].position.y;
	exo_suit_lhand_q_raw_.x() = -msg.poses[4].orientation.z;
	exo_suit_lhand_q_raw_.y() = msg.poses[4].orientation.x;
	exo_suit_lhand_q_raw_.z() = -msg.poses[4].orientation.y;
	exo_suit_lhand_q_raw_.w() = msg.poses[4].orientation.w;

	exo_suit_rshoulder_pos_raw_(0) = msg.poses[5].position.z;
	exo_suit_rshoulder_pos_raw_(1) = -msg.poses[5].position.x;
	exo_suit_rshoulder_pos_raw_(2) = msg.poses[5].position.y;
	exo_suit_rshoulder_q_raw_.x() = -msg.poses[5].orientation.z;
	exo_suit_rshoulder_q_raw_.y() = msg.poses[5].orientation.x;
	exo_suit_rshoulder_q_raw_.z() = -msg.poses[5].orientation.y;
	exo_suit_rshoulder_q_raw_.w() = msg.poses[5].orientation.w;
	
	exo_suit_rupperarm_pos_raw_(0) = msg.poses[6].position.z;
	exo_suit_rupperarm_pos_raw_(1) = -msg.poses[6].position.x;
	exo_suit_rupperarm_pos_raw_(2) = msg.poses[6].position.y;
	exo_suit_rupperarm_q_raw_.x() = -msg.poses[6].orientation.z;
	exo_suit_rupperarm_q_raw_.y() = msg.poses[6].orientation.x;
	exo_suit_rupperarm_q_raw_.z() = -msg.poses[6].orientation.y;
	exo_suit_rupperarm_q_raw_.w() = msg.poses[6].orientation.w;

	exo_suit_rlowerarm_pos_raw_(0) = msg.poses[7].position.z;
	exo_suit_rlowerarm_pos_raw_(1) = -msg.poses[7].position.x;
	exo_suit_rlowerarm_pos_raw_(2) = msg.poses[7].position.y;
	exo_suit_rlowerarm_q_raw_.x() = -msg.poses[7].orientation.z;
	exo_suit_rlowerarm_q_raw_.y() = msg.poses[7].orientation.x;
	exo_suit_rlowerarm_q_raw_.z() = -msg.poses[7].orientation.y;
	exo_suit_rlowerarm_q_raw_.w() = msg.poses[7].orientation.w;

	exo_suit_rhand_pos_raw_(0) = msg.poses[8].position.z;
	exo_suit_rhand_pos_raw_(1) = -msg.poses[8].position.x;
	exo_suit_rhand_pos_raw_(2) = msg.poses[8].position.y;
	exo_suit_rhand_q_raw_.x() = -msg.poses[8].orientation.z;
	exo_suit_rhand_q_raw_.y() = msg.poses[8].orientation.x;
	exo_suit_rhand_q_raw_.z() = -msg.poses[8].orientation.y;
	exo_suit_rhand_q_raw_.w() = msg.poses[8].orientation.w;

	exo_suit_pelv_pos_raw_(0) = msg.poses[9].position.z;
	exo_suit_pelv_pos_raw_(1) = -msg.poses[9].position.x;
	exo_suit_pelv_pos_raw_(2) = msg.poses[9].position.y;
	exo_suit_pelv_q_raw_.x() = -msg.poses[9].orientation.z;
	exo_suit_pelv_q_raw_.y() = msg.poses[9].orientation.x;
	exo_suit_pelv_q_raw_.z() = -msg.poses[9].orientation.y;
	exo_suit_pelv_q_raw_.w() = msg.poses[9].orientation.w;

	exo_suit_upperbody_pos_raw_(0) = msg.poses[11].position.z;
	exo_suit_upperbody_pos_raw_(1) = -msg.poses[11].position.x;
	exo_suit_upperbody_pos_raw_(2) = msg.poses[11].position.y;
	exo_suit_upperbody_q_raw_.x() = -msg.poses[11].orientation.z;
	exo_suit_upperbody_q_raw_.y() = msg.poses[11].orientation.x;
	exo_suit_upperbody_q_raw_.z() = -msg.poses[11].orientation.y;
	exo_suit_upperbody_q_raw_.w() = msg.poses[11].orientation.w;
}

void CustomController::AzureKinectCallback(const visualization_msgs::MarkerArray &msg)
{
	azure_kinect_pelv_pos_raw_(0) = -	msg.markers[0].pose.position.z;
	azure_kinect_pelv_pos_raw_(1) = 	msg.markers[0].pose.position.x;
	azure_kinect_pelv_pos_raw_(2) = -	msg.markers[0].pose.position.y;
	azure_kinect_pelv_q_raw_.x() = -	msg.markers[0].pose.orientation.z;
	azure_kinect_pelv_q_raw_.y() = 		msg.markers[0].pose.orientation.x;
	azure_kinect_pelv_q_raw_.z() = -	msg.markers[0].pose.orientation.y;
	azure_kinect_pelv_q_raw_.w() = 		msg.markers[0].pose.orientation.w;

	azure_kinect_upperbody_pos_raw_(0) = -	msg.markers[2].pose.position.z;
	azure_kinect_upperbody_pos_raw_(1) = 	msg.markers[2].pose.position.x;
	azure_kinect_upperbody_pos_raw_(2) = -	msg.markers[2].pose.position.y;
	azure_kinect_upperbody_q_raw_.x() = -	msg.markers[2].pose.orientation.z;
	azure_kinect_upperbody_q_raw_.y() = 	msg.markers[2].pose.orientation.x;
	azure_kinect_upperbody_q_raw_.z() = -	msg.markers[2].pose.orientation.y;
	azure_kinect_upperbody_q_raw_.w() =		msg.markers[2].pose.orientation.w;

	azure_kinect_head_pos_raw_(0) = -	msg.markers[26].pose.position.z;
	azure_kinect_head_pos_raw_(1) = 	msg.markers[26].pose.position.x;
	azure_kinect_head_pos_raw_(2) = -	msg.markers[26].pose.position.y;
	azure_kinect_head_q_raw_.x() = -	msg.markers[26].pose.orientation.z;
	azure_kinect_head_q_raw_.y() = 		msg.markers[26].pose.orientation.x;
	azure_kinect_head_q_raw_.z() = -	msg.markers[26].pose.orientation.y;
	azure_kinect_head_q_raw_.w() = 		msg.markers[26].pose.orientation.w;

	azure_kinect_lshoulder_pos_raw_(0) = -	msg.markers[5].pose.position.z;
	azure_kinect_lshoulder_pos_raw_(1) = 	msg.markers[5].pose.position.x;
	azure_kinect_lshoulder_pos_raw_(2) = -	msg.markers[5].pose.position.y;
	azure_kinect_lshoulder_q_raw_.x() = -	msg.markers[5].pose.orientation.z;
	azure_kinect_lshoulder_q_raw_.y() = 	msg.markers[5].pose.orientation.x;
	azure_kinect_lshoulder_q_raw_.z() = -	msg.markers[5].pose.orientation.y;
	azure_kinect_lshoulder_q_raw_.w() = 	msg.markers[5].pose.orientation.w;

	azure_kinect_lelbow_pos_raw_(0) = -	msg.markers[6].pose.position.z;
	azure_kinect_lelbow_pos_raw_(1) = 	msg.markers[6].pose.position.x;
	azure_kinect_lelbow_pos_raw_(2) = -	msg.markers[6].pose.position.y;
	azure_kinect_lelbow_q_raw_.x() = -	msg.markers[6].pose.orientation.z;
	azure_kinect_lelbow_q_raw_.y() = 	msg.markers[6].pose.orientation.x;
	azure_kinect_lelbow_q_raw_.z() = -	msg.markers[6].pose.orientation.y;
	azure_kinect_lelbow_q_raw_.w() = 	msg.markers[6].pose.orientation.w;

	azure_kinect_lwrist_pos_raw_(0) = -	msg.markers[7].pose.position.z;
	azure_kinect_lwrist_pos_raw_(1) = 	msg.markers[7].pose.position.x;
	azure_kinect_lwrist_pos_raw_(2) = -	msg.markers[7].pose.position.y;
	azure_kinect_lwrist_q_raw_.x() = -	msg.markers[7].pose.orientation.z;
	azure_kinect_lwrist_q_raw_.y() = 	msg.markers[7].pose.orientation.x;
	azure_kinect_lwrist_q_raw_.z() = -	msg.markers[7].pose.orientation.y;
	azure_kinect_lwrist_q_raw_.w() = 	msg.markers[7].pose.orientation.w;

	azure_kinect_lhand_pos_raw_(0) = -	msg.markers[8].pose.position.z;
	azure_kinect_lhand_pos_raw_(1) = 	msg.markers[8].pose.position.x;
	azure_kinect_lhand_pos_raw_(2) = -	msg.markers[8].pose.position.y;
	azure_kinect_lhand_q_raw_.x() = -	msg.markers[8].pose.orientation.z;
	azure_kinect_lhand_q_raw_.y() = 	msg.markers[8].pose.orientation.x;
	azure_kinect_lhand_q_raw_.z() = -	msg.markers[8].pose.orientation.y;
	azure_kinect_lhand_q_raw_.w() = 	msg.markers[8].pose.orientation.w;

	azure_kinect_rshoulder_pos_raw_(0) = -	msg.markers[12].pose.position.z;
	azure_kinect_rshoulder_pos_raw_(1) = 	msg.markers[12].pose.position.x;
	azure_kinect_rshoulder_pos_raw_(2) = -	msg.markers[12].pose.position.y;
	azure_kinect_rshoulder_q_raw_.x() = -	msg.markers[12].pose.orientation.z;
	azure_kinect_rshoulder_q_raw_.y() = 	msg.markers[12].pose.orientation.x;
	azure_kinect_rshoulder_q_raw_.z() = -	msg.markers[12].pose.orientation.y;
	azure_kinect_rshoulder_q_raw_.w() = 	msg.markers[12].pose.orientation.w;

	azure_kinect_relbow_pos_raw_(0) = -	msg.markers[13].pose.position.z;
	azure_kinect_relbow_pos_raw_(1) = 	msg.markers[13].pose.position.x;
	azure_kinect_relbow_pos_raw_(2) = -	msg.markers[13].pose.position.y;
	azure_kinect_relbow_q_raw_.x() = -	msg.markers[13].pose.orientation.z;
	azure_kinect_relbow_q_raw_.y() = 	msg.markers[13].pose.orientation.x;
	azure_kinect_relbow_q_raw_.z() = -	msg.markers[13].pose.orientation.y;
	azure_kinect_relbow_q_raw_.w() = 	msg.markers[13].pose.orientation.w;

	azure_kinect_rwrist_pos_raw_(0) = -	msg.markers[14].pose.position.z;
	azure_kinect_rwrist_pos_raw_(1) = 	msg.markers[14].pose.position.x;
	azure_kinect_rwrist_pos_raw_(2) = -	msg.markers[14].pose.position.y;
	azure_kinect_rwrist_q_raw_.x() = -	msg.markers[14].pose.orientation.z;
	azure_kinect_rwrist_q_raw_.y() = 	msg.markers[14].pose.orientation.x;
	azure_kinect_rwrist_q_raw_.z() = -	msg.markers[14].pose.orientation.y;
	azure_kinect_rwrist_q_raw_.w() = 	msg.markers[14].pose.orientation.w;

	azure_kinect_rhand_pos_raw_(0) = -	msg.markers[15].pose.position.z;
	azure_kinect_rhand_pos_raw_(1) = 	msg.markers[15].pose.position.x;
	azure_kinect_rhand_pos_raw_(2) = -	msg.markers[15].pose.position.y;
	azure_kinect_rhand_q_raw_.x() = -	msg.markers[15].pose.orientation.z;
	azure_kinect_rhand_q_raw_.y() = 	msg.markers[15].pose.orientation.x;
	azure_kinect_rhand_q_raw_.z() = -	msg.markers[15].pose.orientation.y;
	azure_kinect_rhand_q_raw_.w() = 	msg.markers[15].pose.orientation.w;
}


/////////////////////////////////////PREVIEW CONTROL RELATED FUNCTION////////////////////////////////////
void CustomController::getComTrajectory_Preview()
{ 
	// on-line preview 
	xs_(0) = com_pos_current_(0); ys_(0) = com_pos_current_(1);
	// xs_(1) = com_vel_current_(0); ys_(1) = com_vel_current_(1);
	// xs_(2) = com_acc_current_(0); ys_(2) = com_acc_current_(1);

	modifiedPreviewControl_MJ();

	// off-line preview
	// xs_(0) = xd_(0); ys_(0) = yd_(0);
	// // On,off-line preview 
	xs_(1) = xd_(1); ys_(1) = yd_(1); // 여기서부터
	xs_(2) = xd_(2); ys_(2) = yd_(2);
	
	// if( int(current_time_*2000)%200 == 17 )
	// {
		// std::cout<<"yd_(0) :"<<yd_(0) <<std::endl;
		// std::cout<<"yd_(1) :"<<yd_(1) <<std::endl;
		// std::cout<<"yd_(2) :"<<yd_(2) <<std::endl;

		// std::cout<<"yd_b(0) :"<<yd_b(0) <<std::endl;
		// std::cout<<"yd_b(1) :"<<yd_b(1) <<std::endl;
		// std::cout<<"yd_b(2) :"<<yd_b(2) <<std::endl;

		// std::cout<<"current_time_ :"<<current_time_ <<std::endl;
		// std::cout<<"preview_update_time_ :"<<preview_update_time_ <<std::endl;

		// std::cout<<"com_pos_desired_preview_(1) :"<<com_pos_desired_preview_(1) <<std::endl;
		// std::cout<<"com_vel_desired_preview_(1) :"<<com_vel_desired_preview_(1) <<std::endl;
		// std::cout<<"com_acc_desired_preview_(1) :"<<com_acc_desired_preview_(1) <<std::endl;
	// }
}

void CustomController::modifiedPreviewControl_MJ()
{
  /////reference: http://www.tandfonline.com/doi/pdf/10.1080/0020718508961156?needAccess=true/////////////

	// if(false)
	if( (current_time_-last_preview_param_update_time_) >= 0.02 ) //50hz update
	{
		previewParam_MJ(1/preview_hz_, zmp_size_, zc_, K_act_ , Gi_, Gd_, Gx_, A_, B_, C_, D_, A_bar_, B_bar_);
		last_preview_param_update_time_ = current_time_;
		//previewParam_MJ_CPM(1.0/hz_, 16*hz_/10, K_ ,com_support_init_, Gi_, Gd_, Gx_, A_, B_, C_, D_, A_bar_, B_bar_);
	}

	if( (current_time_-preview_update_time_) >= 1/preview_hz_ ) // preview_hz_ update
	{
		xd_b = xd_; //save previous com desired trajectory
		yd_b = yd_;

		preview_MJ(1/preview_hz_, zmp_size_, xi_, yi_, xs_, ys_, UX_, UY_, Gi_, Gd_, Gx_, A_, B_, xd_, yd_);
		preview_update_time_ = current_time_;
	}
}   

void CustomController::previewParam_MJ(double dt, int NL, double zc, Eigen::Matrix4d& K, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, 
  Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D, Eigen::MatrixXd& A_bar, Eigen::VectorXd& B_bar)
{ 
  A.resize(3,3);
  A(0,0) = 1.0;
  A(0,1) = dt;
  A(0,2) = dt*dt*0.5;
  A(1,0) = 0;
  A(1,1) = 1.0;
  A(1,2) = dt;
  A(2,0) = 0;
  A(2,1) = 0;
  A(2,2) = 1;
  
  B.resize(3);
  B(0) = dt*dt*dt/6;
  B(1) = dt*dt/2;
  B(2) = dt;
  
  C.resize(1,3);
  C(0,0) = 1;
  C(0,1) = 0;
  C(0,2) = -zc_/GRAVITY;

  B_bar.resize(4);    
  B_bar.segment(0,1) = C*B; 
  B_bar.segment(1,3) = B;
  
  Eigen::Matrix1x4d B_bar_tran;
  B_bar_tran = B_bar.transpose();
  
  Eigen::MatrixXd I_bar;
  Eigen::MatrixXd F_bar;
  A_bar.resize(4,4);
  I_bar.resize(4,1);
  F_bar.resize(4,3);
  F_bar.setZero();

  F_bar.block<1,3>(0,0) = C*A;
  F_bar.block<3,3>(1,0) = A;
  
  I_bar.setZero();
  I_bar(0,0) = 1.0;

  A_bar.block<4,1>(0,0) = I_bar;
  A_bar.block<4,3>(0,1) = F_bar;
  
  Eigen::MatrixXd Qe;
  Qe.resize(1,1);
  Qe(0,0) = 1.0;

  Eigen::MatrixXd R;
  R.resize(1,1);
  R(0,0) = 0.000001;	//for offline

//   R(0,0) = 0.000000001;		//test

  Eigen::MatrixXd Qx;
  Qx.setZero(3,3);
  
//   Qx(0, 0) = 0.0001;	//test
//   Qx(1, 1) = 0.02;
//   Qx(2, 2) = 0;

  Eigen::MatrixXd Q_bar;
  Q_bar.setZero(4,4);
  Q_bar(0,0) = Qe(0,0);

//   Q_bar.block(1, 1, 3, 3) = Qx; // test

  K=discreteRiccatiEquationPrev(A_bar, B_bar, R, Q_bar);

  Eigen::MatrixXd Temp_mat;
  Eigen::MatrixXd Temp_mat_inv;
  Eigen::MatrixXd Ac_bar;
  Temp_mat.setZero(1,1);
  Temp_mat_inv.setZero(1,1);
  Ac_bar.setZero(4,4);

  Temp_mat = R + B_bar_tran * K * B_bar;
  Temp_mat_inv = Temp_mat.inverse();
  
  Ac_bar = A_bar - B_bar * Temp_mat_inv * B_bar_tran * K * A_bar;
  
  Eigen::MatrixXd Ac_bar_tran(4,4);
  Ac_bar_tran = Ac_bar.transpose();
  
  Gi.resize(1,1); Gx.resize(1,3);
  Gi = Temp_mat_inv * B_bar_tran * K * I_bar ;
  Gx = Temp_mat_inv * B_bar_tran * K * F_bar ;   
  
  Eigen::MatrixXd X_bar;
  Eigen::Vector4d X_bar_col;
  X_bar.setZero(4, NL); 
  X_bar_col.setZero();
  X_bar_col = - Ac_bar_tran * K * I_bar;

  for(int i = 0; i < NL; i++)
  {
    X_bar.block<4,1>(0,i) = X_bar_col;
    X_bar_col = Ac_bar_tran*X_bar_col;
  }           

  Gd.resize(NL);
  Eigen::VectorXd Gd_col(1);
  Gd_col(0) = -Gi(0,0);
  
  for(int i = 0; i < NL; i++)
  {
    Gd.segment(i,1) = Gd_col;
    Gd_col = Temp_mat_inv * B_bar_tran * X_bar.col(i) ;
  }
    
}

void CustomController::preview_MJ(double dt, int NL, double x_i, double y_i, Eigen::Vector3d xs, Eigen::Vector3d ys, double& UX, double& UY, 
       Eigen::MatrixXd Gi, Eigen::VectorXd Gd, Eigen::MatrixXd Gx, Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::Vector3d &XD, Eigen::Vector3d &YD)
{
  
	Eigen::VectorXd px_ref, py_ref;
	px_ref.resize(zmp_size_);
	py_ref.resize(zmp_size_);

	for(int i = 0; i < zmp_size_; i++)
	{
		px_ref(i) = ref_zmp_(i,0);
		py_ref(i) = ref_zmp_(i,1);
	}

	Eigen::Matrix1x3d C;
	C(0,0) = 1; C(0,1) = 0; C(0,2) = -zc_/GRAVITY;

	Eigen::VectorXd px, py;
	px.resize(1); py.resize(1);

	if(current_time_ == program_start_time_)
	{ 	
		preview_x_b.setZero();
		preview_y_b.setZero();
		preview_x.setZero();
		preview_y.setZero();

		preview_x_b(0) = x_i; // 이때 before 값이 없으면 첫 tick에서 deltaC x의 에러가 갑작스럽게 생겨서 발산
		preview_y_b(0) = y_i;
		preview_x(0) = x_i;
		preview_y(0) = y_i;
	}
	else
	{ 
		preview_x_b = preview_x;
		preview_y_b = preview_y;
				
		preview_x = xs; 
		preview_y = ys;
		
		preview_x_b(0) = preview_x(0) - preview_x(1)/preview_hz_;
		preview_y_b(0) = preview_y(0) - preview_y(1)/preview_hz_;
	}      

	px = C*preview_x;
	py = C*preview_y; 
	// px(0) = zmp_measured_(0);
	// py(0) = zmp_measured_(1);
		
	double sum_Gd_px_ref = 0, sum_Gd_py_ref = 0;

	for(int i = 0; i < NL-1; i++) // Preview Step 개수만큼 더함.
	{
		sum_Gd_px_ref = sum_Gd_px_ref + Gd(i)*(px_ref(i+1) - px_ref(i));
		sum_Gd_py_ref = sum_Gd_py_ref + Gd(i)*(py_ref(i+1) - py_ref(i));
	}
	Eigen::MatrixXd del_ux(1,1);
	Eigen::MatrixXd del_uy(1,1);
	del_ux.setZero();
	del_uy.setZero();

	Eigen::VectorXd GX_X(1); 
	GX_X = Gx * (preview_x - preview_x_b);
	Eigen::VectorXd GX_Y(1); 
	GX_Y = Gx * (preview_y - preview_y_b);

	del_ux(0,0) = -(px(0) - px_ref(0))*Gi(0,0) - GX_X(0) - sum_Gd_px_ref;
	del_uy(0,0) = -(py(0) - py_ref(0))*Gi(0,0) - GX_Y(0) - sum_Gd_py_ref;

	UX = UX + del_ux(0,0);
	UY = UY + del_uy(0,0);

	XD = A*preview_x + B*UX;
	YD = A*preview_y + B*UY;  
}

Eigen::MatrixXd CustomController::discreteRiccatiEquationPrev(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd r, Eigen::MatrixXd q)
{
	int n=a.rows(); //number of rows
	int m=b.cols(); //number of columns

	Eigen::MatrixXd z11(n, n), z12(n, n), z21(n, n), z22(n, n);

	z11 = a.inverse();
	z12 = a.inverse()*b*r.inverse()*b.transpose();
	z21 = q*a.inverse();
	z22 = a.transpose() + q*a.inverse()*b*r.inverse()*b.transpose();
		
	Eigen::MatrixXd z; 
	z.setZero(2*n, 2*n);
	z.topLeftCorner(n,n) = z11;
	z.topRightCorner(n,n) = z12;
	z.bottomLeftCorner(n,n) = z21;
	z.bottomRightCorner(n,n) = z22;


	std::vector<Eigen::VectorXd> eigVec_real(2*n);
	std::vector<Eigen::VectorXd> eigVec_img(2*n);

	for(int i=0; i<8; i++)
	{
		eigVec_real[i].setZero(2*n);
		eigVec_img[i].setZero(2*n);
	}

	Eigen::VectorXd deigVal_real(2*n);
	Eigen::VectorXd deigVal_img(2*n);
	deigVal_real.setZero();
	deigVal_img.setZero();
	Eigen::MatrixXd deigVec_real(2*n,2*n);
	Eigen::MatrixXd deigVec_img(2*n,2*n);
	deigVec_real.setZero();
	deigVec_img.setZero();

	deigVal_real = z.eigenvalues().real();
	deigVal_img = z.eigenvalues().imag();
	Eigen::EigenSolver<Eigen::MatrixXd> ev(z);

	for(int i=0;i<2*n; i++)
	{
		for(int j=0; j<2*n; j++)
		{
		deigVec_real(j,i) = ev.eigenvectors().col(i)(j).real();
		deigVec_img(j,i) = ev.eigenvectors().col(i)(j).imag();
		}
	}

	//Order the eigenvectors
	//move e-vectors correspnding to e-value outside the unite circle to the left

	Eigen::MatrixXd tempZ_real(2*n, n), tempZ_img(2*n, n);
	tempZ_real.setZero();
	tempZ_img.setZero();
	int c=0;

	for (int i=0;i<2*n;i++)
	{
		if ((deigVal_real(i)*deigVal_real(i)+deigVal_img(i)*deigVal_img(i))>1.0) //outside the unit cycle
		{
		for(int j=0; j<2*n; j++)
		{
			tempZ_real(j,c) = deigVec_real(j,i);
			tempZ_img(j,c) = deigVec_img(j,i);
		}
		c++;
		}
	}

	Eigen::MatrixXcd tempZ_comp(2*n, n);
	for(int i=0;i<2*n;i++)
	{
		for(int j=0;j<n;j++)
		{
		tempZ_comp.real()(i,j) = tempZ_real(i,j);
		tempZ_comp.imag()(i,j) = tempZ_img(i,j);
		}
	}
	Eigen::MatrixXcd U11(n, n), U21(n, n), X(n, n);
	for(int i=0;i<n;i++)
	{
		for(int j=0;j<n;j++)
		{
		U11(i,j) = tempZ_comp(i,j);
		U21(i,j) = tempZ_comp(i+n,j);
		}
	}
	X = U21*(U11.inverse());

	Eigen::MatrixXd X_sol(n, n);
	for(int i=0;i<n;i++)
	{
		for(int j=0;j<n;j++)
		{
		X_sol(i,j) = X.real()(i,j);
		}
	}

	return X_sol;
}

void CustomController::getZmpTrajectory()
{

	if(stop_walking_trigger_ == true)
	{
		for(int i =0; i<zmp_size_ ; i++)
		{
			ref_zmp_(i, 0) = middle_of_both_foot_(0);
			ref_zmp_(i, 1) = middle_of_both_foot_(1);

		}
	}
	else
	{
		int one_step_ticks = walking_duration_*preview_hz_;

		if(start_walking_trigger_ == true)
		{			
			for(int i = 0; i<zmp_size_; i++)
			{
				int num_of_step = i/one_step_ticks ; //start from 0   
				int step_side = (num_of_step%2); //0: planning support foot side zmp, 1: planning swing foot side zmp

				ref_zmp_(i, 0) = support_foot_transform_current_.translation()(0);
				ref_zmp_(i, 1) = support_foot_transform_current_.translation()(1);
				ref_zmp_(i, 1) = ref_zmp_(i, 1) - step_width_*foot_contact_*step_side - zmp_y_offset_*foot_contact_*(2*step_side - 1); 			// Y direction step_width planning
			}
		}
		else if(walking_speed_ == 0)
		{
			int first_step_residual_tick = int((1-walking_phase_)*walking_duration_*preview_hz_);

			for(int i = 0; i<first_step_residual_tick; i++)
			{

				
				ref_zmp_(i, 0) = support_foot_transform_current_.translation()(0);
				ref_zmp_(i, 1) = support_foot_transform_current_.translation()(1) + zmp_y_offset_*foot_contact_;			
			}

			for(int j = first_step_residual_tick; j<zmp_size_; j++)
			{

				ref_zmp_(j, 0) = middle_of_both_foot_(0);
				ref_zmp_(j, 1) = middle_of_both_foot_(1);
			}
		}
		else
		{
			int first_step_residual_tick = int((1-walking_phase_)*walking_duration_*preview_hz_);

			for(int i = 0; i<first_step_residual_tick; i++)
			{

				
				ref_zmp_(i, 0) = support_foot_transform_current_.translation()(0);
				ref_zmp_(i, 1) = support_foot_transform_current_.translation()(1) + zmp_y_offset_*foot_contact_;			
			}

			for(int j = first_step_residual_tick; j<zmp_size_; j++)
			{
				int num_of_step = (j- first_step_residual_tick)/one_step_ticks +1; //start from 1 

				int step_side = (num_of_step%2); //0: planning support foot side zmp, 1: planning swing foot side zmp



				ref_zmp_(j, 0) = support_foot_transform_current_.translation()(0);
				ref_zmp_(j, 1) = support_foot_transform_current_.translation()(1);
				ref_zmp_(j, 1) = ref_zmp_(j, 1) - step_width_*foot_contact_*step_side - zmp_y_offset_*foot_contact_*(2*step_side - 1); 			// Y direction step_width planning
			}
		}
	}

	// if( int(current_time_*2000)%200 == 0 )
	// {
	// 	// std::cout<<"ref_zmp_(0, 1) :"<<ref_zmp_(0, 1) <<std::endl;
	// }
}

void CustomController::fallDetection()
{
	Vector3d pelv_angle_to_g;
	pelv_angle_to_g = DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());
	pelv_angle_to_g(0) = abs(pelv_angle_to_g(0));
	pelv_angle_to_g(1) = abs(pelv_angle_to_g(1));
	pelv_angle_to_g(2) = abs(pelv_angle_to_g(2));

	if(falling_detection_flag_ == false)
	{
		if((pelv_angle_to_g(0) > 15*DEG2RAD) || (pelv_angle_to_g(1)>15*DEG2RAD) )
		{
			falling_detection_flag_ = true;
			fall_init_q_ = current_q_;
			fall_start_time_ = current_time_;
			// cout<<"###########FALLING IS DETECTED###################"<<endl;
		}

		if(l_ft_.segment(0, 3).norm() + r_ft_.segment(0, 3).norm() < rd_.com_.mass*GRAVITY*0.1)
		{	
			foot_lift_count_ += 1;
		}
		else
		{
			foot_lift_count_ = 0;
		}

		if(foot_lift_count_ > 600)
		{
			falling_detection_flag_ = true;
			fall_init_q_ = current_q_;
			fall_start_time_ = current_time_;
			// cout<<"###########FEET IS DETACHED###################"<<endl;
		}
	}


	
	if(falling_detection_flag_ == true)
	{
		torque_g_ = wbc_.gravity_compensation_torque(rd_, true);

		for (int i = 0; i < MODEL_DOF; i++)
		{
			desired_q_(i) = DyrosMath::QuinticSpline(current_time_, fall_start_time_, fall_start_time_+3.0, fall_init_q_(i), 0, 0, zero_q_(i), 0, 0)(0);
			desired_q_dot_(i) = DyrosMath::QuinticSpline(current_time_, fall_start_time_, fall_start_time_+3.0, fall_init_q_(i), 0, 0, zero_q_(i), 0, 0)(1);
			torque_task_(i) = (kp_joint_(i) * (desired_q_(i) - current_q_(i)) + kv_joint_(i) * (desired_q_dot_(i) - current_q_dot_(i))) + torque_g_(i);
		}	
	}
}

double CustomController::bandBlock(double value, double max, double min)
{
	double y;

	if(value <= min)
	{
		y = - (value - min)*(value - min);
	}
	else if(value >= max)
	{
		y = (value - max)*(value - max);
	}
	else
	{
		y = 0;
	}

	return y;
}

void CustomController::printOutTextFile()
{
	// if (int( (current_time_ - program_start_time_) * 2000) % 1 == 0) // 2000 hz 
	// if ((current_time_ - program_start_time_) >= 0.0)
	// {
		// file[0]<<current_time_ - program_start_time_<<"\t"<<walking_phase_<<"\t"<<foot_contact_<<"\t"
		// <<foot_swing_trigger_<<"\t"<<first_step_trigger_<<"\t"<<start_walking_trigger_<<"\t"
		// <<stop_walking_trigger_<<"\t"<<stance_start_time_<<"\t"<<walking_duration_<<"\t"
		// <<turning_duration_<<"\t"<<turning_phase_<<"\t"<<knee_target_angle_<<endl;

		// file[1]<<current_time_ - program_start_time_<<"\t"<<walking_phase_<<"\t"<<foot_contact_<<"\t"		//1
		// <<com_pos_current_(0)<<"\t"<<com_pos_current_(1)<<"\t"<<com_pos_current_(2)<<"\t"                  	//4
		// <<com_vel_current_(0)<<"\t"<<com_vel_current_(1)<<"\t"<<com_vel_current_(2)<<"\t"					//7
		// <<com_acc_current_(0)<<"\t"<<com_acc_current_(1)<<"\t"<<com_acc_current_(2)<<"\t"					//10
		// <<com_pos_desired_(0)<<"\t"<<com_pos_desired_(1)<<"\t"<<com_pos_desired_(2)<<"\t"					//13
		// <<com_vel_desired_(0)<<"\t"<<com_vel_desired_(1)<<"\t"<<com_vel_desired_(2)<<"\t"					//16
		// <<com_acc_desired_(0)<<"\t"<<com_acc_desired_(1)<<"\t"<<com_acc_desired_(2)<<"\t"					//19
		// <<com_vel_est1_(0)<<"\t"<<com_vel_est1_(1)<<"\t"<<com_vel_est1_(2)<<"\t"
		// <<com_vel_est2_(0)<<"\t"<<com_vel_est2_(1)<<"\t"<<com_vel_est2_(2)<<endl;

		// file[2]<<ref_zmp_(0,1)<<endl;

		// file[3]<<current_time_ - program_start_time_<<"\t"<<walking_phase_<<"\t"<<foot_contact_<<"\t"																								//1
		// <<lfoot_transform_current_from_global_.translation()(0)<<"\t"<<lfoot_transform_current_from_global_.translation()(1)<<"\t"<<lfoot_transform_current_from_global_.translation()(2)<<"\t"		//4
		// <<rfoot_transform_current_from_global_.translation()(0)<<"\t"<<rfoot_transform_current_from_global_.translation()(1)<<"\t"<<rfoot_transform_current_from_global_.translation()(2)<<"\t"		//7
		// <<swing_foot_pos_trajectory_from_global_(0)<<"\t"<<swing_foot_pos_trajectory_from_global_(1)<<"\t"<<swing_foot_pos_trajectory_from_global_(2)<<"\t"											//10
		// <<lfoot_vel_current_from_global_(0)<<"\t"<<lfoot_vel_current_from_global_(1)<<"\t"<<lfoot_vel_current_from_global_(2)<<"\t"																	//13
		// <<lfoot_vel_current_from_global_(3)<<"\t"<<lfoot_vel_current_from_global_(4)<<"\t"<<lfoot_vel_current_from_global_(5)<<"\t"																	//16
		// <<rfoot_vel_current_from_global_(0)<<"\t"<<rfoot_vel_current_from_global_(1)<<"\t"<<rfoot_vel_current_from_global_(2)<<"\t"																	//19
		// <<rfoot_vel_current_from_global_(3)<<"\t"<<rfoot_vel_current_from_global_(4)<<"\t"<<rfoot_vel_current_from_global_(5)<<"\t"																	//22
		// <<swing_foot_vel_trajectory_from_global_(0)<<"\t"<<swing_foot_vel_trajectory_from_global_(1)<<"\t"<<swing_foot_vel_trajectory_from_global_(2)<<"\t"											//25
		// <<support_foot_vel_current_(3)<<"\t"<<support_foot_vel_current_(4)<<"\t"<<support_foot_vel_current_(5)<<endl;																				//28

		// file[4]<<current_time_ - program_start_time_<<"\t"<<walking_phase_<<"\t"<<foot_contact_<<"\t"
		// <<ControlVal_(0)<<"\t"<<ControlVal_(1)<<"\t"<<ControlVal_(2)<<"\t"<<ControlVal_(3)<<"\t"<<ControlVal_(4)<<"\t"<<ControlVal_(5)<<"\t"<<ControlVal_(6)<<"\t"<<ControlVal_(7)<<"\t"<<ControlVal_(8)<<"\t"<<ControlVal_(9)<<"\t"<<ControlVal_(10)<<"\t"<<ControlVal_(11)<<"\t"<<ControlVal_(12)<<"\t"<<ControlVal_(13)<<"\t"<<ControlVal_(14)<<"\t"<<ControlVal_(15)<<"\t"<<ControlVal_(16)<<"\t"<<ControlVal_(17)<<"\t"<<ControlVal_(18)<<"\t"<<ControlVal_(19)<<"\t"<<ControlVal_(20)<<"\t"<<ControlVal_(21)<<"\t"<<ControlVal_(22)<<"\t"<<ControlVal_(23)<<"\t"<<ControlVal_(24)<<"\t"<<ControlVal_(25)<<"\t"<<ControlVal_(26)<<"\t"<<ControlVal_(27)<<"\t"<<ControlVal_(28)<<"\t"<<ControlVal_(29)<<"\t"<<ControlVal_(30)<<"\t"<<ControlVal_(31)<<"\t"<<ControlVal_(32)<<endl; 	

		file[5]<<current_time_ - program_start_time_<<"\t"<<walking_phase_<<"\t"<<foot_contact_<<"\t"
		<<desired_q_(0)<<"\t"<<desired_q_(1)<<"\t"<<desired_q_(2)<<"\t"<<desired_q_(3)<<"\t"<<desired_q_(4)<<"\t"<<desired_q_(5)<<"\t"<<desired_q_(6)<<"\t"<<desired_q_(7)<<"\t"<<desired_q_(8)<<"\t"<<desired_q_(9)<<"\t"<<desired_q_(10)<<"\t"<<desired_q_(11)<<"\t"<<desired_q_(12)<<"\t"<<desired_q_(13)<<"\t"<<desired_q_(14)<<"\t"<<desired_q_(15)<<"\t"<<desired_q_(16)<<"\t"<<desired_q_(17)<<"\t"<<desired_q_(18)<<"\t"<<desired_q_(19)<<"\t"<<desired_q_(20)<<"\t"<<desired_q_(21)<<"\t"<<desired_q_(22)<<"\t"<<desired_q_(23)<<"\t"<<desired_q_(24)<<"\t"<<desired_q_(25)<<"\t"<<desired_q_(26)<<"\t"<<desired_q_(27)<<"\t"<<desired_q_(28)<<"\t"<<desired_q_(29)<<"\t"<<desired_q_(30)<<"\t"<<desired_q_(31)<<"\t"<<desired_q_(32)<<"\t" 
		<<current_q_(0)<<"\t"<<current_q_(1)<<"\t"<<current_q_(2)<<"\t"<<current_q_(3)<<"\t"<<current_q_(4)<<"\t"<<current_q_(5)<<"\t"<<current_q_(6)<<"\t"<<current_q_(7)<<"\t"<<current_q_(8)<<"\t"<<current_q_(9)<<"\t"<<current_q_(10)<<"\t"<<current_q_(11)<<"\t"<<current_q_(12)<<"\t"<<current_q_(13)<<"\t"<<current_q_(14)<<"\t"<<current_q_(15)<<"\t"<<current_q_(16)<<"\t"<<current_q_(17)<<"\t"<<current_q_(18)<<"\t"<<current_q_(19)<<"\t"<<current_q_(20)<<"\t"<<current_q_(21)<<"\t"<<current_q_(22)<<"\t"<<current_q_(23)<<"\t"<<current_q_(24)<<"\t"<<current_q_(25)<<"\t"<<current_q_(26)<<"\t"<<current_q_(27)<<"\t"<<current_q_(28)<<"\t"<<current_q_(29)<<"\t"<<current_q_(30)<<"\t"<<current_q_(31)<<"\t"<<current_q_(32)<<"\t"
		<<desired_q_dot_(0)<<"\t"<<desired_q_dot_(1)<<"\t"<<desired_q_dot_(2)<<"\t"<<desired_q_dot_(3)<<"\t"<<desired_q_dot_(4)<<"\t"<<desired_q_dot_(5)<<"\t"<<desired_q_dot_(6)<<"\t"<<desired_q_dot_(7)<<"\t"<<desired_q_dot_(8)<<"\t"<<desired_q_dot_(9)<<"\t"<<desired_q_dot_(10)<<"\t"<<desired_q_dot_(11)<<"\t"<<desired_q_dot_(12)<<"\t"<<desired_q_dot_(13)<<"\t"<<desired_q_dot_(14)<<"\t"<<desired_q_dot_(15)<<"\t"<<desired_q_dot_(16)<<"\t"<<desired_q_dot_(17)<<"\t"<<desired_q_dot_(18)<<"\t"<<desired_q_dot_(19)<<"\t"<<desired_q_dot_(20)<<"\t"<<desired_q_dot_(21)<<"\t"<<desired_q_dot_(22)<<"\t"<<desired_q_dot_(23)<<"\t"<<desired_q_dot_(24)<<"\t"<<desired_q_dot_(25)<<"\t"<<desired_q_dot_(26)<<"\t"<<desired_q_dot_(27)<<"\t"<<desired_q_dot_(28)<<"\t"<<desired_q_dot_(29)<<"\t"<<desired_q_dot_(30)<<"\t"<<desired_q_dot_(31)<<"\t"<<desired_q_dot_(32)<<"\t" 
		<<current_q_dot_(0)<<"\t"<<current_q_dot_(1)<<"\t"<<current_q_dot_(2)<<"\t"<<current_q_dot_(3)<<"\t"<<current_q_dot_(4)<<"\t"<<current_q_dot_(5)<<"\t"<<current_q_dot_(6)<<"\t"<<current_q_dot_(7)<<"\t"<<current_q_dot_(8)<<"\t"<<current_q_dot_(9)<<"\t"<<current_q_dot_(10)<<"\t"<<current_q_dot_(11)<<"\t"<<current_q_dot_(12)<<"\t"<<current_q_dot_(13)<<"\t"<<current_q_dot_(14)<<"\t"<<current_q_dot_(15)<<"\t"<<current_q_dot_(16)<<"\t"<<current_q_dot_(17)<<"\t"<<current_q_dot_(18)<<"\t"<<current_q_dot_(19)<<"\t"<<current_q_dot_(20)<<"\t"<<current_q_dot_(21)<<"\t"<<current_q_dot_(22)<<"\t"<<current_q_dot_(23)<<"\t"<<current_q_dot_(24)<<"\t"<<current_q_dot_(25)<<"\t"<<current_q_dot_(26)<<"\t"<<current_q_dot_(27)<<"\t"<<current_q_dot_(28)<<"\t"<<current_q_dot_(29)<<"\t"<<current_q_dot_(30)<<"\t"<<current_q_dot_(31)<<"\t"<<current_q_dot_(32)<<endl; 

		file[6]
		<<lhand_transform_current_from_global_.translation()(0)<<"\t"<<lhand_transform_current_from_global_.translation()(1)<<"\t"<<lhand_transform_current_from_global_.translation()(2)<<"\t"<<rhand_transform_current_from_global_.translation()(0)<<"\t"<<rhand_transform_current_from_global_.translation()(1)<<"\t"<<rhand_transform_current_from_global_.translation()(2)<<"\t"
		<<lhand_rpy_current_from_global_(0)<<"\t"<<lhand_rpy_current_from_global_(1)<<"\t"<<lhand_rpy_current_from_global_(2)<<"\t"<<rhand_rpy_current_from_global_(0)<<"\t"<<rhand_rpy_current_from_global_(1)<<"\t"<<rhand_rpy_current_from_global_(2)<<"\t"
		<<lhand_vel_current_from_global_(0)<<"\t"<<lhand_vel_current_from_global_(1)<<"\t"<<lhand_vel_current_from_global_(2)<<"\t"<<rhand_vel_current_from_global_(0)<<"\t"<<rhand_vel_current_from_global_(1)<<"\t"<<rhand_vel_current_from_global_(2)<<endl;

		file[7]
		<<lelbow_transform_current_from_global_.translation()(0)<<"\t"<<lelbow_transform_current_from_global_.translation()(1)<<"\t"<<lelbow_transform_current_from_global_.translation()(2)<<"\t"<<relbow_transform_current_from_global_.translation()(0)<<"\t"<<relbow_transform_current_from_global_.translation()(1)<<"\t"<<relbow_transform_current_from_global_.translation()(2)<<"\t"
		<<lelbow_rpy_current_from_global_(0)<<"\t"<<lelbow_rpy_current_from_global_(1)<<"\t"<<lelbow_rpy_current_from_global_(2)<<"\t"<<relbow_rpy_current_from_global_(0)<<"\t"<<relbow_rpy_current_from_global_(1)<<"\t"<<relbow_rpy_current_from_global_(2)<<"\t"
		<<lelbow_vel_current_from_global_(0)<<"\t"<<lelbow_vel_current_from_global_(1)<<"\t"<<lelbow_vel_current_from_global_(2)<<"\t"<<relbow_vel_current_from_global_(0)<<"\t"<<relbow_vel_current_from_global_(1)<<"\t"<<relbow_vel_current_from_global_(2)<<endl;

		file[8]
		<<lshoulder_transform_current_from_global_.translation()(0)<<"\t"<<lshoulder_transform_current_from_global_.translation()(1)<<"\t"<<lshoulder_transform_current_from_global_.translation()(2)<<"\t"<<rshoulder_transform_current_from_global_.translation()(0)<<"\t"<<rshoulder_transform_current_from_global_.translation()(1)<<"\t"<<rshoulder_transform_current_from_global_.translation()(2)<<"\t"
		<<lshoulder_rpy_current_from_global_(0)<<"\t"<<lshoulder_rpy_current_from_global_(1)<<"\t"<<lshoulder_rpy_current_from_global_(2)<<"\t"<<rshoulder_rpy_current_from_global_(0)<<"\t"<<rshoulder_rpy_current_from_global_(1)<<"\t"<<rshoulder_rpy_current_from_global_(2)<<"\t"
		<<lshoulder_vel_current_from_global_(0)<<"\t"<<lshoulder_vel_current_from_global_(1)<<"\t"<<lshoulder_vel_current_from_global_(2)<<"\t"<<rshoulder_vel_current_from_global_(0)<<"\t"<<rshoulder_vel_current_from_global_(1)<<"\t"<<rshoulder_vel_current_from_global_(2)<<endl;

		file[9]
		<<lacromion_transform_current_from_global_.translation()(0)<<"\t"<<lacromion_transform_current_from_global_.translation()(1)<<"\t"<<lacromion_transform_current_from_global_.translation()(2)<<"\t"<<racromion_transform_current_from_global_.translation()(0)<<"\t"<<racromion_transform_current_from_global_.translation()(1)<<"\t"<<racromion_transform_current_from_global_.translation()(2)<<"\t"
		<<lacromion_rpy_current_from_global_(0)<<"\t"<<lacromion_rpy_current_from_global_(1)<<"\t"<<lacromion_rpy_current_from_global_(2)<<"\t"<<racromion_rpy_current_from_global_(0)<<"\t"<<racromion_rpy_current_from_global_(1)<<"\t"<<racromion_rpy_current_from_global_(2)<<"\t"
		<<lacromion_vel_current_from_global_(0)<<"\t"<<lacromion_vel_current_from_global_(1)<<"\t"<<lacromion_vel_current_from_global_(2)<<"\t"<<racromion_vel_current_from_global_(0)<<"\t"<<racromion_vel_current_from_global_(1)<<"\t"<<racromion_vel_current_from_global_(2)<<endl;

		file[11]
		<<hmd_lhand_pose_.translation()(0)<<"\t"<<hmd_lhand_pose_.translation()(1)<<"\t"<<hmd_lhand_pose_.translation()(2)<<"\t"<<hmd_rhand_pose_.translation()(0)<<"\t"<<hmd_rhand_pose_.translation()(1)<<"\t"<<hmd_rhand_pose_.translation()(2)<<"\t"
		<<master_lhand_pose_raw_.translation()(0)<<"\t"<<master_lhand_pose_raw_.translation()(1)<<"\t"<<master_lhand_pose_raw_.translation()(2)<<"\t"<<master_rhand_pose_raw_.translation()(0)<<"\t"<<master_rhand_pose_raw_.translation()(1)<<"\t"<<master_rhand_pose_raw_.translation()(2)<<"\t"
		<<master_lhand_pose_.translation()(0)<<"\t"<<master_lhand_pose_.translation()(1)<<"\t"<<master_lhand_pose_.translation()(2)<<"\t"<<master_rhand_pose_.translation()(0)<<"\t"<<master_rhand_pose_.translation()(1)<<"\t"<<master_rhand_pose_.translation()(2)<<"\t"
		<<master_lhand_rqy_(0)<<"\t"<<master_lhand_rqy_(1)<<"\t"<<master_lhand_rqy_(2)<<"\t"<<master_rhand_rqy_(0)<<"\t"<<master_rhand_rqy_(1)<<"\t"<<master_rhand_rqy_(2)<<"\t"
 		<<hmd_lupperarm_pose_.translation()(0)<<"\t"<<hmd_lupperarm_pose_.translation()(1)<<"\t"<<hmd_lupperarm_pose_.translation()(2)<<"\t"<<hmd_rupperarm_pose_.translation()(0)<<"\t"<<hmd_rupperarm_pose_.translation()(1)<<"\t"<<hmd_rupperarm_pose_.translation()(2)<<"\t"
		<<master_lelbow_pose_raw_.translation()(0)<<"\t"<<master_lelbow_pose_raw_.translation()(1)<<"\t"<<master_lelbow_pose_raw_.translation()(2)<<"\t"<<master_relbow_pose_raw_.translation()(0)<<"\t"<<master_relbow_pose_raw_.translation()(1)<<"\t"<<master_relbow_pose_raw_.translation()(2)<<"\t"
		<<master_lelbow_pose_.translation()(0)<<"\t"<<master_lelbow_pose_.translation()(1)<<"\t"<<master_lelbow_pose_.translation()(2)<<"\t"<<master_relbow_pose_.translation()(0)<<"\t"<<master_relbow_pose_.translation()(1)<<"\t"<<master_relbow_pose_.translation()(2)<<"\t"
		<<master_lelbow_rqy_(0)<<"\t"<<master_lelbow_rqy_(1)<<"\t"<<master_lelbow_rqy_(2)<<"\t"<<master_relbow_rqy_(0)<<"\t"<<master_relbow_rqy_(1)<<"\t"<<master_relbow_rqy_(2)<<"\t"
		<<hmd_lshoulder_pose_.translation()(0)<<"\t"<<hmd_lshoulder_pose_.translation()(1)<<"\t"<<hmd_lshoulder_pose_.translation()(2)<<"\t"<<hmd_rshoulder_pose_.translation()(0)<<"\t"<<hmd_rshoulder_pose_.translation()(1)<<"\t"<<hmd_rshoulder_pose_.translation()(2)<<"\t"
		<<master_lshoulder_pose_raw_.translation()(0)<<"\t"<<master_lshoulder_pose_raw_.translation()(1)<<"\t"<<master_lshoulder_pose_raw_.translation()(2)<<"\t"<<master_rshoulder_pose_raw_.translation()(0)<<"\t"<<master_rshoulder_pose_raw_.translation()(1)<<"\t"<<master_rshoulder_pose_raw_.translation()(2)<<"\t"
		<<master_lshoulder_pose_.translation()(0)<<"\t"<<master_lshoulder_pose_.translation()(1)<<"\t"<<master_lshoulder_pose_.translation()(2)<<"\t"<<master_rshoulder_pose_.translation()(0)<<"\t"<<master_rshoulder_pose_.translation()(1)<<"\t"<<master_rshoulder_pose_.translation()(2)<<"\t"
		<<master_lshoulder_rqy_(0)<<"\t"<<master_lshoulder_rqy_(1)<<"\t"<<master_lshoulder_rqy_(2)<<"\t"<<master_rshoulder_rqy_(0)<<"\t"<<master_rshoulder_rqy_(1)<<"\t"<<master_rshoulder_rqy_(2)<<"\t"
		<<hmd_head_pose_.translation()(0)<<"\t"<<hmd_head_pose_.translation()(1)<<"\t"<<hmd_head_pose_.translation()(2)<<"\t"
		<<master_head_pose_raw_.translation()(0)<<"\t"<<master_head_pose_raw_.translation()(1)<<"\t"<<master_head_pose_raw_.translation()(2)<<"\t"
		<<master_head_pose_.translation()(0)<<"\t"<<master_head_pose_.translation()(1)<<"\t"<<master_head_pose_.translation()(2)<<"\t"
		<<master_head_rqy_(0)<<"\t"<<master_head_rqy_(1)<<"\t"<<master_head_rqy_(2)<<endl;

		file[12]
		<<lhand_vel_error_(0)<<"\t"<<lhand_vel_error_(1)<<"\t"<<lhand_vel_error_(2)<<"\t"<<lhand_vel_error_(3)<<"\t"<<lhand_vel_error_(4)<<"\t"<<lhand_vel_error_(5)<<"\t"
		<<rhand_vel_error_(0)<<"\t"<<rhand_vel_error_(1)<<"\t"<<rhand_vel_error_(2)<<"\t"<<rhand_vel_error_(3)<<"\t"<<rhand_vel_error_(4)<<"\t"<<rhand_vel_error_(5)<<"\t"
		<<lelbow_vel_error_(0)<<"\t"<<lelbow_vel_error_(1)<<"\t"<<lelbow_vel_error_(2)<<"\t"<<relbow_vel_error_(0)<<"\t"<<relbow_vel_error_(1)<<"\t"<<relbow_vel_error_(2)<<"\t"
		<<lacromion_vel_error_(0)<<"\t"<<lacromion_vel_error_(1)<<"\t"<<lacromion_vel_error_(2)<<"\t"<<racromion_vel_error_(0)<<"\t"<<racromion_vel_error_(1)<<"\t"<<racromion_vel_error_(2)<<endl;

	// }
}
