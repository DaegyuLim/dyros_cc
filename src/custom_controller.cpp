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

	setGains();
    first_loop_ = true;
}
void CustomController::setGains()
{
	//////////Control Gain///////////////////////////
	////sim
	kp_compos_.setZero(); 		
	kd_compos_.setZero();	 	

	kp_compos_(0, 0) = 100;
	kp_compos_(1, 1) = 100;
	kp_compos_(2, 2) = 100;

	kd_compos_(0, 0) = 20;
	kd_compos_(1, 1) = 20;
	kd_compos_(2, 2) = 20;

	kp_pelv_ori_ = 2500*Eigen::Matrix3d::Identity(); 	//angle error gain (sim: 4900)(tune)
	// kp_pelv_ori_(0, 0) = 0;
	kp_pelv_ori_(2, 2) = 0;

	kd_pelv_ori_ = 100*Eigen::Matrix3d::Identity();		//angular velocity gain (sim: 140)(tune)

	support_foot_damping_gain_.setZero();
	support_foot_damping_gain_(0, 0) = 20;
	support_foot_damping_gain_(1, 1) = 100;
	support_foot_damping_gain_(2, 2) = 100;

	////real
	// kp_compos_.setZero(); 		
	// kd_compos_.setZero();	 	

	// kp_compos_(0, 0) = 80;
	// kp_compos_(1, 1) = 100;
	// kp_compos_(2, 2) = 80;

	// kd_compos_(0, 0) = 18;
	// kd_compos_(1, 1) = 20;
	// kd_compos_(2, 2) = 18;

	// kp_pelv_ori_ = 2500*Eigen::Matrix3d::Identity(); 	//angle error gain (sim: 4900)(tune)
	// kd_pelv_ori_ = 100*Eigen::Matrix3d::Identity();		//angular velocity gain (sim: 140)(tune)

	// support_foot_damping_gain_.setZero();
	// support_foot_damping_gain_(0, 0) = 200;
	// support_foot_damping_gain_(1, 1) = 100;
	// support_foot_damping_gain_(2, 2) = 10;

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
	torque_task_min_(21) 	= -300;
	torque_task_min_(22) 	= -300;
	torque_task_min_(23) 	= -300;
	torque_task_min_(24) 	= -300;
	torque_task_min_(25) 	= -300;
	torque_task_min_(26) 	= -300;
	torque_task_min_(27) 	= -300;
	torque_task_min_(28) 	= -300;
	torque_task_min_(29) 	= -300;
	torque_task_min_(30) 	= -300;
	torque_task_min_(31) 	= -300;
	torque_task_min_(32) 	= -300;

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
	torque_task_max_(21) 	= 300;
	torque_task_max_(22) 	= 300;
	torque_task_max_(23) 	= 300;
	torque_task_max_(24) 	= 300;
	torque_task_max_(25) 	= 300;
	torque_task_max_(26) 	= 300;
	torque_task_max_(27) 	= 300;
	torque_task_max_(28) 	= 300;
	torque_task_max_(29) 	= 300;
	torque_task_max_(30) 	= 300;
	torque_task_max_(31) 	= 300;
	torque_task_max_(32) 	= 300;
	////////////////////////////////

	//////////Joint PD Gain/////////
	///For Simulation
	for (int i = 0; i < MODEL_DOF; i++)
	{
		kp_joint_(i) = 225; 		//(tune)
		kv_joint_(i) = 30;		//(tune)
	}

	for (int i = 0; i < 3; i++) //Waist Joint Gains
	{
		kp_joint_(12 + i) = 400;
		kv_joint_(12 + i) = 40;
	}

	//stiff	//(tune)
	kp_stiff_joint_(0) = 4900; //R hip yaw joint gain
	kv_stiff_joint_(0) = 140;
	kp_stiff_joint_(1) = 4900; //L hip roll joint gain
	kv_stiff_joint_(1) = 140;
	kp_stiff_joint_(2) = 4900; //L hip pitch joint gain
	kv_stiff_joint_(2) = 140;

	kp_stiff_joint_(3) = 1600; //L knee joint gain
	kv_stiff_joint_(3) = 80;

	kp_stiff_joint_(4) = 400; //L ankle pitch joint gain
	kv_stiff_joint_(4) = 40;
	kp_stiff_joint_(5) = 400; //L ankle roll joint gain
	kv_stiff_joint_(5) = 40;

	kp_stiff_joint_(6) = 4900; //R hip yaw joint gain
	kv_stiff_joint_(6) = 140;
	kp_stiff_joint_(7) = 4900; //R hip roll joint gain
	kv_stiff_joint_(7) = 140;
	kp_stiff_joint_(8) = 4900; //R hip pitch joint gain
	kv_stiff_joint_(8) = 140;

	kp_stiff_joint_(9) = 1600; //R knee joint gain
	kv_stiff_joint_(9) = 80;

	kp_stiff_joint_(10) = 400; //R ankle pitch joint gain
	kv_stiff_joint_(10) = 40;
	kp_stiff_joint_(11) = 400; //R ankle roll joint gain
	kv_stiff_joint_(11) = 40;

	//soft	//(tune)
	kp_soft_joint_(0) = 2500; //L hip yaw joint gain
	kv_soft_joint_(0) = 100;
	kp_soft_joint_(1) = 400; //L hip roll joint gain
	kv_soft_joint_(1) = 40;
	kp_soft_joint_(2) = 400; //L hip pitch joint gain
	kv_soft_joint_(2) = 40;

	kp_soft_joint_(3) = 100; //L knee joint gain
	kv_soft_joint_(3) = 20;

	kp_soft_joint_(4) = 25; //L ankle pitch joint gain
	kv_soft_joint_(4) = 10;
	kp_soft_joint_(5) = 25; //L ankle roll joint gain
	kv_soft_joint_(5) = 10;

	kp_soft_joint_(6) = 1600; //R hip yaw joint gain
	kv_soft_joint_(6) = 80;
	kp_soft_joint_(7) = 400; //R hip roll joint gain
	kv_soft_joint_(7) = 40;
	kp_soft_joint_(8) = 400; //R hip pitch joint gain
	kv_soft_joint_(8) = 40;

	kp_soft_joint_(9) = 100; //R knee joint gain
	kv_soft_joint_(9) = 20;

	kp_soft_joint_(10) = 25; //R ankle pitch joint gain
	kv_soft_joint_(10) = 10;
	kp_soft_joint_(11) = 25; //R ankle roll joint gain
	kv_soft_joint_(11) = 10;

	for (int i = 0; i < 12; i++) //Leg
	{
		kp_joint_(i) = kp_stiff_joint_(i);
		kv_joint_(i) = kv_stiff_joint_(i);
	}
	/////////////////

	///For Real Robot
	// kp_stiff_joint_(0) 		= -300;
	// kp_stiff_joint_(1) 		= -300;
	// kp_stiff_joint_(2) 		= -300;
	// kp_stiff_joint_(3) 		= -300;
	// kp_stiff_joint_(4) 		= -300;
	// kp_stiff_joint_(5) 		= -300;
	// kp_stiff_joint_(6) 		= -300;
	// kp_stiff_joint_(7) 		= -300;
	// kp_stiff_joint_(8) 		= -300;
	// kp_stiff_joint_(9) 		= -300;
	// kp_stiff_joint_(10) 		= -300;
	// kp_stiff_joint_(11) 		= -300;
	// kp_stiff_joint_(12) 		= -300;
	// kp_stiff_joint_(13) 		= -300;
	// kp_stiff_joint_(14) 		= -300;
	// kp_stiff_joint_(15) 		= -300;
	// kp_stiff_joint_(16) 		= -300;
	// kp_stiff_joint_(17) 		= -300;
	// kp_stiff_joint_(18) 		= -300;
	// kp_stiff_joint_(19) 		= -300;
	// kp_stiff_joint_(20) 		= -300;
	// kp_stiff_joint_(21) 		= -300;
	// kp_stiff_joint_(22) 		= -300;
	// kp_stiff_joint_(23) 		= -300;
	// kp_stiff_joint_(24) 		= -300;
	// kp_stiff_joint_(25) 		= -300;
	// kp_stiff_joint_(26) 		= -300;
	// kp_stiff_joint_(27) 		= -300;
	// kp_stiff_joint_(28) 		= -300;
	// kp_stiff_joint_(29) 		= -300;
	// kp_stiff_joint_(30) 		= -300;
	// kp_stiff_joint_(31) 		= -300;
	// kp_stiff_joint_(32) 		= -300;

	// kv_stiff_joint_(0) 		= 300;
	// kv_stiff_joint_(1) 		= 300;
	// kv_stiff_joint_(2) 		= 300;
	// kv_stiff_joint_(3) 		= 300;
	// kv_stiff_joint_(4) 		= 300;
	// kv_stiff_joint_(5) 		= 300;
	// kv_stiff_joint_(6) 		= 300;
	// kv_stiff_joint_(7) 		= 300;
	// kv_stiff_joint_(8) 		= 300;
	// kv_stiff_joint_(9) 		= 300;
	// kv_stiff_joint_(10) 		= 300;
	// kv_stiff_joint_(11) 		= 300;
	// kv_stiff_joint_(12) 		= 300;
	// kv_stiff_joint_(13) 		= 300;
	// kv_stiff_joint_(14) 		= 300;
	// kv_stiff_joint_(15) 		= 300;
	// kv_stiff_joint_(16) 		= 300;
	// kv_stiff_joint_(17) 		= 300;
	// kv_stiff_joint_(18) 		= 300;
	// kv_stiff_joint_(19) 		= 300;
	// kv_stiff_joint_(20) 		= 300;
	// kv_stiff_joint_(21) 		= 300;
	// kv_stiff_joint_(22) 		= 300;
	// kv_stiff_joint_(23) 		= 300;
	// kv_stiff_joint_(24) 		= 300;
	// kv_stiff_joint_(25) 		= 300;
	// kv_stiff_joint_(26) 		= 300;
	// kv_stiff_joint_(27) 		= 300;
	// kv_stiff_joint_(28) 		= 300;
	// kv_stiff_joint_(29) 		= 300;
	// kv_stiff_joint_(30) 		= 300;
	// kv_stiff_joint_(31) 		= 300;
	// kv_stiff_joint_(32) 		= 300;

	// for (int i = 0; i < MODEL_DOF; i++) 
	// {
	// 	kp_soft_joint_(i) = kp_stiff_joint_(i)/4;
	// 	kp_soft_joint_(i) = kv_stiff_joint_(i)/2;
	// }

	// for (int i = 0; i < MODEL_DOF; i++) 
	// {
	// 	kp_joint_(i) = kp_stiff_joint_(i);
	// 	kv_joint_(i) = kv_stiff_joint_(i);
	// }
	///////////////

	///////////////////////////////
	
	//arm controller
	joint_limit_l_.resize(16);
    joint_limit_h_.resize(16);
    joint_vel_limit_l_.resize(16);
    joint_vel_limit_h_.resize(16);
    joint_limit_l_ << -2.09, -3.14159, -1.9199, -3.14159, -3.14159, -3.14159, -3.14159, -2.094, -1.54, -3.14159, -1.9199,   -2.967, -3.14159, -3.14159, -3.14159, -2.094;
    joint_limit_h_ <<  1.54,  3.14159,  1.9199,  3.14159,      2.8,  3.14159,  3.14159,  2.094,  2.09,  3.14159,  1.9199,  3.14159,      2.8,  3.14159,  3.14159,  2.094;
    joint_vel_limit_l_ << -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159;
    joint_vel_limit_h_ <<  3.14159,  3.14159,  3.14159,  3.14159,  3.14159,  3.14159,  3.14159,  3.14159,  3.14159,  3.14159,  3.14159,  3.14159,  3.14159,  3.14159,  3.14159,  3.14159;


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
        
        if (first_loop_)
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
            first_loop_ = false;
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
        rd_.control_time_pre_ = rd_.control_time_;

		left_x_traj_pre_ =  RigidBodyDynamics::CalcBodyToBaseCoordinates( model_d_, q_virtual_clik_, rd_.link_[Left_Hand].id, Eigen::Vector3d::Zero(), true);
		right_x_traj_pre_ =  RigidBodyDynamics::CalcBodyToBaseCoordinates( model_d_, q_virtual_clik_, rd_.link_[Right_Hand].id, Eigen::Vector3d::Zero(), true);
		left_rotm_pre_ = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_virtual_clik_, rd_.link_[Left_Hand].id, true)).transpose();
        right_rotm_pre_ = (RigidBodyDynamics::CalcBodyWorldOrientation(model_d_, q_virtual_clik_, rd_.link_[Right_Hand].id, true)).transpose();
		// mtx_rbdl.lock();
		// mtx_dc.lock();
        // left_x_traj_pre_ =  RigidBodyDynamics::CalcBodyToBaseCoordinates(*rd_.link_[Left_Hand].model, q_virtual_clik_, rd_.link_[Left_Hand].id, Eigen::Vector3d::Zero(), false);
        // right_x_traj_pre_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(*rd_.link_[Right_Hand].model, q_virtual_clik_, rd_.link_[Right_Hand].id, Eigen::Vector3d::Zero(), false);
        // left_rotm_pre_ = (RigidBodyDynamics::CalcBodyWorldOrientation(*rd_.link_[Left_Hand].model, q_virtual_clik_, rd_.link_[Left_Hand].id, false)).transpose();
        // right_rotm_pre_ = (RigidBodyDynamics::CalcBodyWorldOrientation(*rd_.link_[Right_Hand].model, q_virtual_clik_, rd_.link_[Right_Hand].id, false)).transpose();
        // mtx_rbdl.unlock();
		// mtx_dc.unlock();

	}
	else if (tc.mode == 13)
	{
		////////////////////////////////////////////////////////////////////////////
		/////////////////// Human-like Biped Walking Controller ////////////////////
		////////////////////////////////////////////////////////////////////////////

		torque_task_.setZero();
		if (tc.task_init == true)
		{
			initWalkingParameter();
			tc.task_init = false;
		}

		//data process//
		getRobotData(wbc_);
		walkingStateManager();
		getProcessedRobotData(wbc_);	

		//motion planing and control//
		motionGenerator();
		getZmpTrajectory();
		getComTrajectory_Preview();
		getCOMTrajectory();
		getSwingFootXYTrajectory(walking_phase_, com_pos_current_, com_vel_current_, com_vel_desired_);

		torque_task_ += comVelocityControlCompute(wbc_); 								//support chain control for COM velocity and pelvis orientation
		torque_task_ += swingFootControlCompute(wbc_);									//swing foot control
		torque_task_ += jointTrajectoryPDControlCompute(wbc_) * first_torque_supplier_; //upper body motion + joint damping control
		torque_task_ += dampingControlCompute(wbc_);									

		savePreData();

		////////////////////////////////TORQUE LIMIT//////// //////////////////////
		for (int i = 0; i < MODEL_DOF; i++)
		{
			torque_task_(i) = DyrosMath::minmax_cut(torque_task_(i), -300, 300);
		}
		///////////////////////////////////////////////////////////////////////////
        
		//////////////////////////////FALLING/////////////////////////////////////
		fallDetection();
		//////////////////////////////////////////////////////////////////////////


		ControlVal_ = torque_task_;
 

		printOutTextFile();
	}
	else if (tc.mode == 14) //arm control test in the air 
	{
		getRobotData(wbc_);
		
		motionRetargetting2();
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
void CustomController::setWalkingParameter(double walking_duration, double walking_speed, double step_width, double knee_target_angle)
{
	walking_duration_ = walking_duration;
	walking_speed_ = walking_speed;
	step_width_ = step_width;
	knee_target_angle_ = knee_target_angle;

	walking_duration_ = DyrosMath::minmax_cut(walking_duration_, 0.5, 1.5);
	walking_speed_ = DyrosMath::minmax_cut(walking_speed_, -0.5, 1.0);
	step_width_ = DyrosMath::minmax_cut(step_width_, 0.17, 0.25);
	knee_target_angle_ = DyrosMath::minmax_cut(knee_target_angle_, 0.0, 1.5);
}

void CustomController::initWalkingParameter()
{
	walking_mode_on_ = true;
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
	knee_target_angle_ = 18*DEG2RAD;
	// knee_target_angle_ = M_PI/40;                               //4.5degree
	swingfoot_highest_time_ = 0.35;
	yaw_angular_vel_ = 0; //   rad/s
	swing_foot_height_ = 0.08;
	switching_phase_duration_ = 0.01;
	foot_contact_ = -1;
	foot_contact_pre_ = foot_contact_;
	step_width_ = 0.28;  						//for preview control
	alpha_x_ = 0.02;
	alpha_y_ = 0.18;
	alpha_x_command_ = alpha_x_;
	alpha_y_command_ = alpha_y_;

	start_walking_trigger_ = false;
	first_step_trigger_ = false;
	foot_swing_trigger_ = false;
	stop_walking_trigger_ = true;
	falling_detection_flag_ = false;

	preview_horizon_ = 1.6; //seconds
	preview_hz_ = 200;
	zmp_size_ = preview_horizon_*preview_hz_; 
	ref_zmp_.setZero(zmp_size_, 2);
	zmp_y_offset_ = 0.00;	//outward from com

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

	
}

void CustomController::getRobotData(WholebodyController &wbc)
{
	// Wholebody_controller wc_(dc_, rd_);
	// if(control_time_ == tc.command_time)
	// {
	//     initWalkingParameter();
	// }
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

	lfoot_vel_current_from_global.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot].v;
	lfoot_vel_current_from_global.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot].w;

	rfoot_vel_current_from_global.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot].v;
	rfoot_vel_current_from_global.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot].w;
	///////////////////////////////////////////////////////////////////////////////

	////////////////////////Knee Transformation ////////////////////////////////////
	lknee_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Foot-2].xpos - pelv_pos_current_);
	lknee_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot-2].Rotm;
	rknee_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Foot-2].xpos - pelv_pos_current_);
	rknee_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot-2].Rotm;

	/////////////////////////Hands Transformation and Velocity/////////////////////
	lhand_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Hand].xpos - pelv_pos_current_);
	lhand_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand].Rotm;
	rhand_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Hand].xpos - pelv_pos_current_);
	rhand_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand].Rotm;

	lhand_vel_current_from_global.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand].v;
	lhand_vel_current_from_global.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Hand].w;

	rhand_vel_current_from_global.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand].v;
	rhand_vel_current_from_global.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Hand].w;
	///////////////////////////////////////////////////////////////////////////////

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

	// A_mat_ = rd_.A_;
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

	first_torque_supplier_ = DyrosMath::cubic(current_time_, program_start_time_, program_start_time_ + walking_duration_*0.5, 0, 1, 0, 0);
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
					if (balanceTrigger(com_pos_current_.segment<2>(0), com_vel_current_.segment<2>(0))) //gonna be fall
					// if(false)
					{
						foot_swing_trigger_ = true;
						first_step_trigger_ = true;
						start_walking_trigger_ = false;
						stop_walking_trigger_ = false;
						start_time_ = current_time_;

						if(com_vel_current_(1) >=0)
						{
							foot_contact_ = -1;
						}
						else
						{
							foot_contact_ = 1;
						}
						
						// foot_contact_ = -foot_contact_;  //support foot change
						std::cout << " ################################ Balancing Control ON! ################################" << std::endl;
					}
					else
					{
						foot_swing_trigger_ = false;
						start_walking_trigger_ = false;
						first_step_trigger_ = false;
						start_time_ = current_time_;
					}
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
					std::cout << " ################################ First Step Triggered! ################################" << std::endl;
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
						
						std::cout << " ################ Early Step Change Occured! ("<<walking_phase_<<", "<< -r_ft_(2) <<") #########################" << std::endl;
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
						std::cout << " ################ Early Step Change Occured! ("<<walking_phase_<<", "<< -l_ft_(2) <<") #########################" << std::endl;
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
				std::cout << " ################################ Balancing Control ON! ################################" << std::endl;
			}
			else
			{
				foot_swing_trigger_ = false;
				stop_walking_trigger_ = true;  //robot stop
				first_step_trigger_ = false;
				start_walking_trigger_ = false;

				foot_contact_ = -foot_contact_;
				stance_start_time_ = current_time_;
				std::cout << " ################################ Robot Stops Walking! ################################" << std::endl;
			}
		}
		else
		{
			foot_swing_trigger_ = true;
			stop_walking_trigger_ = false;
			first_step_trigger_ = false;
			start_walking_trigger_ = false;

			foot_contact_ = -foot_contact_;
			std::cout << " ################################ Support Foot Changed! ################################" << std::endl;
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
		std::cout << "Catpure point in X axis is over the safety boundary! balance swing foot control activated" << std::endl;
	}

	if ((capture_point_2d(1) > lfoot_transform_current_from_global_.translation()(1) - 0.02 ) || (capture_point_2d(1) < rfoot_transform_current_from_global_.translation()(1) + 0.02))
	{
		trigger = true;
		std::cout << "Catpure point in Y axis is over the safety boundary! balance swing foot control activated" << std::endl;
	}

	// if( com_vel_2d.norm() > stop_vel_threshold_)
	// {
	//     trigger = true;
	//     cout<<"com vel is over the limit ("<< com_vel_2d.norm()<<")"<<endl;
	// }

	if ( abs(com_vel_2d(0)) > 0.2 || abs(com_vel_2d(1)) > 0.15)
	{
		trigger = true;
		std::cout << "com vel is over the limit (" << com_vel_2d(0) << "," << com_vel_2d(1) << ")" << std::endl;
	}

	if (abs(lfoot_transform_current_from_global_.translation()(0) - rfoot_transform_current_from_global_.translation()(0)) > 0.03 || abs(lfoot_transform_current_from_global_.translation()(1) - rfoot_transform_current_from_global_.translation()(1)) > 0.25)
	{
		trigger = true;
		std::cout << "Foot is not aligned" << std::endl;
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

		com_pos_desired_preview_ = com_pos_current_;
		com_vel_desired_preview_.setZero();
		com_acc_desired_preview_.setZero();

		com_vel_desired_preview_pre_ = com_pos_current_;
		com_vel_desired_preview_pre_.setZero();
		com_vel_desired_preview_pre_.setZero();

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

		lhand_transform_init_from_global_ = lhand_transform_current_from_global_;
		rhand_transform_init_from_global_ = rhand_transform_current_from_global_;
	}

	bool robot_goes_into_stance_phase = (current_time_ == stance_start_time_);
	bool robot_start_walking = ((foot_swing_trigger_ == true) && (current_time_ == start_time_));
	if (robot_goes_into_stance_phase || robot_start_walking)
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
			swing_foot_vel_init_ = rfoot_vel_current_from_global;
		}
		else if (foot_contact_ == -1) //right support foot
		{
			swing_foot_transform_init_ = lfoot_transform_current_from_global_;
			support_foot_transform_init_ = rfoot_transform_current_from_global_;
			swing_foot_vel_init_ = lfoot_vel_current_from_global;
		}
		init_q_ = current_q_;
		last_desired_q_ = desired_q_;
		foot_lift_count_ = 0;

		swingfoot_f_star_l_pre_.setZero();
		swingfoot_f_star_r_pre_.setZero();
	}

	if (foot_contact_ == 1) // left support foot
	{

		swing_foot_transform_current_ = rfoot_transform_current_from_global_;
		support_foot_transform_current_ = lfoot_transform_current_from_global_;
		swing_foot_vel_current_ = rfoot_vel_current_from_global;
		support_foot_vel_current_.setZero();
		support_foot_vel_current_.segment(3, 3) = pelv_angvel_current_ - lfoot_to_com_jac_from_global_.block(3, 6, 3, MODEL_DOF)*current_q_dot_;

		com_vel_est1_ = lfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ ;
		com_vel_est2_ = lfoot_to_com_jac_from_global_.block(0, 6, 3, MODEL_DOF)*current_q_dot_ + DyrosMath::skm(lfoot_transform_current_from_global_.translation())*(support_foot_vel_current_.segment(3, 3));
	}
	else if (foot_contact_ == -1) //right support foot
	{
		swing_foot_transform_current_ = lfoot_transform_current_from_global_;
		support_foot_transform_current_ = rfoot_transform_current_from_global_;
		swing_foot_vel_current_ = lfoot_vel_current_from_global;
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
	}

	swingfoot_force_control_converter_ = DyrosMath::cubic(walking_phase_, 1, 1.1, 0, 1, 0, 0);
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

	if(upper_body_mode_ == 1) // init pose 
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

		///////////////////////ARM/////////////////////////
		//////LEFT ARM///////0.3 0.3 1.5 -1.27 -1 0 -1 0
		motion_q_(15) = 0.3;
		motion_q_(16) = 0.3;
		motion_q_(17) = 1.5;
		motion_q_(18) = -1.27; 
		motion_q_(19) = -1.0; //elbow
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
		motion_q_(29) = 1.0;	//elbow
		motion_q_(30) = 0.0;
		motion_q_(31) = 1.0;
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
	}
	else if(upper_body_mode_ == 2)		// motion capture play
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

		///////////////////////ARM/////////////////////////
		//////LEFT ARM///////0.3 0.3 1.5 -1.27 -1 0 -1 0
		motion_q_(15) = 0.3;
		motion_q_(16) = 0.05 + 5*current_q_(7);
		motion_q_(17) = 1.55;
		motion_q_(18) = -1.2; 
		motion_q_(19) = -0.4 + 3*current_q_(7); //elbow
		motion_q_(20) = 0.0;
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
		motion_q_(26) = -0.05 + 5*current_q_(1);
		motion_q_(27) = -1.55;
		motion_q_(28) = 1.2;
		motion_q_(29) = 0.4 + 3*current_q_(1);	//elbow
		motion_q_(30) = 0.0;
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
	}
	else if(upper_body_mode_ == 3)		// human motion retargetting: joint pd control with CLIK
	{
		motionRetargetting();
	}
	else if(upper_body_mode_ == 4)		// human motion retargetting: task space position pd control
	{
		motionRetargetting2();
	}
	else if(upper_body_mode_ == 4)
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
			swing_foot_pos_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, swingfoot_highest_time_, 1, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2) - 0.001, 0, 0)(0);
			swing_foot_vel_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, swingfoot_highest_time_, 1, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2) - 0.001, 0, 0)(1);
			swing_foot_acc_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, swingfoot_highest_time_, 1, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2) - 0.001, 0, 0)(2);
		}

	}
}

void CustomController::motionRetargetting()
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

	kp_pos_lhand(0, 0) = 1000;
	kp_pos_lhand(1, 1) = 1000;
	kp_pos_lhand(2, 2) = 1000;

	kp_pos_rhand(0, 0) = 1000;
	kp_pos_rhand(1, 1) = 1000;
	kp_pos_rhand(2, 2) = 1000;

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
	
	master_lhand_pose_.translation() 		= lhand_transform_init_from_global_.translation();
	master_lhand_pose_.translation()(0) 	+= 	0.1*sin(current_time_*2*M_PI/1);
	master_lhand_pose_.linear().Identity();
	master_lhand_vel_.setZero();
	master_lhand_vel_(0) 	= 	0.1*cos(current_time_*2*M_PI/1)*2*M_PI/1;

	master_rhand_pose_.translation() 		= rhand_transform_init_from_global_.translation();
	master_rhand_pose_.translation()(0) 	+= 	0.1*sin(current_time_*2*M_PI/1);
	master_rhand_pose_.linear().Identity();
	master_rhand_vel_.setZero();
	master_rhand_vel_(0) 	= 	0.1*cos(current_time_*2*M_PI/1)*2*M_PI/1;
	
	u_dot_lhand.segment(0, 3) = master_lhand_vel_.segment(0, 3) + kp_pos_lhand*(master_lhand_pose_.translation()-lhand_transform_pre_desired_from.translation());
	u_dot_rhand.segment(0, 3) = master_rhand_vel_.segment(0, 3) + kp_pos_rhand*(master_rhand_pose_.translation()-rhand_transform_pre_desired_from.translation());

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
	///////////////////////////////////////////////////////////////////////////////////
}

void CustomController::motionRetargetting2()
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
	// master_lhand_pose_ = lhand_transform_init_from_global_;
	// master_lhand_vel_.setZero();

	// master_lhand_pose_.translation()(0) = 1.5*rknee_transform_current_from_global_.translation()(0)-0.15;
	// master_lhand_pose_.translation()(1) = lhand_transform_init_from_global_.translation()(1);
	// master_lhand_pose_.translation()(2) = lhand_transform_init_from_global_.translation()(2); 

	// master_lhand_pose_.translation()(0) = DyrosMath::minmax_cut(master_lhand_pose_.translation()(0), -0.1, 0.6);

	// master_lhand_pose_.linear() = Eigen::Matrix3d::Identity();

	// master_rhand_pose_ = rhand_transform_init_from_global_;
	// master_rhand_vel_.setZero();

	// master_rhand_pose_.translation()(0) = 1.5*lknee_transform_current_from_global_.translation()(0)-0.15; 
	// master_rhand_pose_.translation()(1) = rhand_transform_init_from_global_.translation()(1);
	// master_rhand_pose_.translation()(2) = rhand_transform_init_from_global_.translation()(2);

	// master_rhand_pose_.translation()(0) = DyrosMath::minmax_cut(master_rhand_pose_.translation()(0), -0.1, 0.6);

	// master_rhand_pose_.linear() = Eigen::Matrix3d::Identity();
	
	/////////////////test////////////////////////
	master_lhand_pose_.translation() 		= 	lhand_transform_init_from_global_.translation();
	master_lhand_pose_.translation()(0) 	= 	0.1*sin(current_time_*2*M_PI/3);
	master_lhand_pose_.linear().Identity();
	master_lhand_vel_.setZero();
	master_lhand_vel_(0) 	= 	0.1*cos(current_time_*2*M_PI/3)*2*M_PI/3;

	master_rhand_pose_.translation() 		= 	rhand_transform_init_from_global_.translation();
	master_rhand_pose_.translation()(0) 	= 	0.1*sin(current_time_*2*M_PI/3);
	master_rhand_pose_.linear().Identity();
	master_rhand_vel_.setZero();
	master_rhand_vel_(0) 	= 	0.1*cos(current_time_*2*M_PI/3)*2*M_PI/3;

	// hand position
	f_d_lhand.segment(0, 3) = kp_pos_lhand*(master_lhand_pose_.translation()-lhand_transform_current_from_global_.translation()) + kd_pos_lhand*(master_lhand_vel_.segment(0,3) - lhand_vel_current_from_global.segment(0,3));
	f_d_rhand.segment(0, 3) = kp_pos_rhand*(master_rhand_pose_.translation()-rhand_transform_current_from_global_.translation()) + kd_pos_rhand*(master_rhand_vel_.segment(0,3) - rhand_vel_current_from_global.segment(0,3));

	f_d_lhand(2) -= 0;
	f_d_rhand(2) -= 0;
	// hand orientation
	Vector3d phi_lhand;
	Vector3d phi_rhand;
	phi_lhand = -DyrosMath::getPhi(lhand_transform_current_from_global_.linear(), master_lhand_pose_.linear());
	phi_rhand = -DyrosMath::getPhi(rhand_transform_current_from_global_.linear(), master_rhand_pose_.linear());

	double kpa_hand = 000; //angle error gain
	double kva_hand = 00;	//angular velocity gain

	f_d_lhand.segment(3, 3) = kpa_hand * phi_lhand + kva_hand * (master_lhand_vel_.segment(3,3) - lhand_vel_current_from_global.segment(3,3));
	f_d_rhand.segment(3, 3) = kpa_hand * phi_rhand + kva_hand * (master_rhand_vel_.segment(3,3) - rhand_vel_current_from_global.segment(3,3));
	
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

void CustomController::getCOMTrajectory()
{
	double desired_step_position_in_y;
	double desired_step_velocity_in_y;
	double d_temp_y;
	com_pos_desired_.setZero();
	com_vel_desired_.setZero();
	com_acc_desired_.setZero();

	com_pos_desired_(2) = com_pos_current_(2);
	// com_vel_desired_(2) = com_vel_current_(2);
	// com_pos_desired_(2) = support_foot_transform_current_.translation()(2) + 0.75;
	com_vel_desired_(2) = 0;
	com_acc_desired_(2) = GRAVITY;

	com_pos_desired_(1) = yd_(0);  // from preview
	com_vel_desired_(1) = yd_(1);
	com_acc_desired_(1) = yd_(2);
	
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

			// com_pos_desired_(0) = (1-w)*middle_of_both_foot_(0) + w*com_pos_current_(0);
			com_pos_desired_(0) = (1-w)*middle_of_both_foot_(0) + w*(com_pos_current_(0) + com_vel_desired_(0)*dt_);
			com_vel_desired_(0) = w*walking_speed_;
			// com_vel_desired_(1) = ( desired_step_position_in_y-com_pos_init_(1)+support_foot_transform_init_.translation()(1) )/(walking_duration_);
			// com_pos_desired_(1) = support_foot_transform_current_.translation()(1) + (1-walking_phase_)*(com_pos_init_(1) - support_foot_transform_init_.translation()(1)) + walking_phase_*desired_step_position_in_y;
			// com_pos_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_ + 0 * walking_duration_, stance_start_time_ + 1 * walking_duration_, support_foot_transform_current_.translation()(0) + (com_pos_init_)(0) - support_foot_transform_init_.translation()(0), 0, 0, middle_of_both_foot_(0), 0, 0)(0);
			// com_vel_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_ + 0 * walking_duration_, stance_start_time_ + 1 * walking_duration_, support_foot_transform_current_.translation()(0) + (com_pos_init_)(0) - support_foot_transform_init_.translation()(0), 0, 0, middle_of_both_foot_(0), 0, 0)(1);
			// com_pos_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 1 * walking_duration_, (com_pos_init_)(1), 0, 0, support_foot_transform_current_.translation()(1) - 0.00 * foot_contact_, 0, 0)(0); //-0.03*foot_contact_
			// com_vel_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 1 * walking_duration_, (com_pos_init_)(1), 0, 0, support_foot_transform_current_.translation()(1) - 0.00 * foot_contact_, 0, 0)(1);
			// com_acc_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+2*walking_duration_, (com_pos_init_)(1), 0, 0, support_foot_transform_current_.translation()(1)  , 0, 0)(2);
		}
		else
		{
			// com_acc_desired_(2) = GRAVITY;
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
			com_pos_desired_(0) = com_pos_current_(0);
			com_vel_desired_(0) = 0;
			// com_pos_desired_(0) = middle_of_both_foot_(0);

			///////////////Y DIRECTIOIN///////////
			// desired_step_position_in_y = - (step_width_)*foot_contact_ ;
			desired_step_position_in_y = -(step_width_)*foot_contact_;
			double target_com_y_speed = (support_foot_transform_init_.translation()(1) + desired_step_position_in_y - com_pos_init_(1)) / (walking_duration_);

			// com_vel_desired_(1) = 0;
			// com_pos_desired_(1) = com_pos_current_(1);
			// com_vel_desired_(1) = DyrosMath::cubic(walking_phase_, 0, 5*switching_phase_duration_, 0, target_com_y_speed, 0, 0);
			// com_pos_desired_(1) = support_foot_transform_current_.translation()(1) + (1 - walking_phase_) * (com_pos_init_(1) - support_foot_transform_init_.translation()(1)) + walking_phase_ * (desired_step_position_in_y);
			// com_vel_desired_(1) = target_com_y_speed;
		}
		else if (program_start_time_ == stance_start_time_)
		{
			// com_pos_desired_(2) = com_pos_init_(2) - support_foot_transform_init_.translation()(2) + support_foot_transform_current_.translation()(2);
			// com_vel_desired_(2) = 0;
			// com_acc_desired_(2) = GRAVITY/2;
			// com_pos_desired_(0) = middle_of_both_foot_(0);
			// com_vel_desired_(0) = 0;

			// com_pos_desired_(1) = middle_of_both_foot_(1);
			// com_vel_desired_(1) = 0;
			
			com_pos_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+1, support_foot_transform_current_.translation()(0) + (com_pos_init_)(0) - support_foot_transform_init_.translation()(0), 0, 0, (middle_of_both_foot_)(0), 0, 0)(0);
			com_vel_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+1, support_foot_transform_current_.translation()(0) + (com_pos_init_)(0) - support_foot_transform_init_.translation()(0), 0, 0, (middle_of_both_foot_)(0), 0, 0)(1);

			// com_pos_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+1, (com_pos_init_)(1), 0, 0, (middle_of_both_foot_)(1), 0, 0)(0);
			// com_vel_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+1, (com_pos_init_)(1), 0, 0, (middle_of_both_foot_)(1), 0, 0)(1);
		}
		else if (stop_walking_trigger_ == true)
		{
			// com_pos_desired_(2) = com_pos_init_(2) - support_foot_transform_init_.translation()(2) + support_foot_transform_current_.translation()(2);
			// com_vel_desired_(2) = 0;
			// com_acc_desired_(2) = GRAVITY/2;		//divide 2 because each legs apply same acc

			double traj_duraiton = walking_duration_*0.5;

			com_pos_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, support_foot_transform_current_.translation()(0) + (com_pos_init_)(0) - support_foot_transform_init_.translation()(0), com_vel_init_(0), 0, (middle_of_both_foot_)(0), 0, 0)(0);
			// com_vel_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(0), com_vel_init_(0), 0, (middle_of_both_foot_)(0), 0, 0)(1);
			// com_pos_desired_(0) = com_pos_init_(0);
			// com_pos_desired_(1) = DyrosMath::QuinticSpline(current_time_, start_time_, start_time_+5, (com_pos_init_)(1), 0, 0, lfoot_transform_current_from_global_.translation()(1), 0, 0)(0);
			// com_vel_desired_(1) = DyrosMath::QuinticSpline(current_time_, start_time_, start_time_+5, (com_pos_init_)(1), 0, 0, lfoot_transform_current_from_global_.translation()(1), 0, 0)(1);
			// com_pos_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(1), com_vel_init_(1), 0, (middle_of_both_foot_)(1), 0, 0)(0);
			// com_vel_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(1), com_vel_init_(1), 0, (middle_of_both_foot_)(1), 0, 0)(1);
			
			// com_pos_desired_(0) = middle_of_both_foot_(0);
			com_vel_desired_(0) = 0;

		}
	}

	// com_acc_desired_(2) = 0.4*com_acc_desired_(2) + 0.6*com_acc_desired_pre_(2);
	// cout<<"com_pos_desired: "<< com_pos_desired_<<endl;
	// cout<<"com_vel_desired: "<< com_vel_desired_<<endl;
	// cout<<"com_pos_current_: "<< com_pos_current_<<endl;
	// cout<<"com_vel_current_: "<< com_vel_current_<<endl;
	// cout<<"com_pos_error: "<< com_pos_desired_ - com_pos_current_<<endl;
	// cout<<"com_vel_error: "<< com_vel_desired_ - com_vel_current_<<endl;
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
		swing_foot_pos_trajectory_from_global_(0) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(0), -0.5, 1.0);
		swing_foot_pos_trajectory_from_global_(1) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(1), -0.5, lfoot_transform_current_from_global_.translation()(1) - 0.20);
	}
	else if (foot_contact_ == -1) // right support
	{
		swing_foot_pos_trajectory_from_global_(0) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(0), - 0.5, 1.0);
		swing_foot_pos_trajectory_from_global_(1) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(1), rfoot_transform_current_from_global_.translation()(1) + 0.20, 0.5);
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
	
	double gamma_l_min;		//tangent value of the angle from CoM to the lfoot;
	double gamma_l_max;		//tangent value of the angle from CoM to the lfoot;
	double gamma_r_min;		//tangent value of the angle from CoM to the rfoot;
	double gamma_r_max;		//tangent value of the angle from CoM to the rfoot;

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
	
	phi_pelv_ = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());
	torque_pelv_ = kp_pelv_ori_ * phi_pelv_ - kd_pelv_ori_ * pelv_angvel_current_;
	// torque_pelv_(2) = 0;
	f_star.segment(3,3) = torque_pelv_*3;
	/////////////////////////////////////////////

	/////////////////////JACOBIAN///////////////////////////////////////
	lfoot_to_com_jac_from_global_.block(0, 6, 6, 1).setZero(); 		// 	left yaw
	lfoot_to_com_jac_from_global_.block(0, 12, 6, 6).setZero(); 	//	right leg
	// lfoot_to_com_jac_from_global_.block(0, 21, 6, 8).setZero();		//	left arm
	lfoot_to_com_jac_from_global_.block(0, 29, 6, 2).setZero();		//	head
	// lfoot_to_com_jac_from_global_.block(0, 31, 6, 8).setZero();		// 	right arm
	
	lfoot_to_com_jac_from_global_.block(3, 9, 3, 3).setZero();		// 	left leg roational component: knee pitch, ankle pitch, ankle roll
	

	rfoot_to_com_jac_from_global_.block(0, 6, 6, 6).setZero();		//	left leg
	rfoot_to_com_jac_from_global_.block(0, 12, 6, 1).setZero(); 	// 	right yaw
	// rfoot_to_com_jac_from_global_.block(0, 21, 6, 8).setZero();		//	left arm
	rfoot_to_com_jac_from_global_.block(0, 29, 6, 2).setZero();		//	head
	// rfoot_to_com_jac_from_global_.block(0, 31, 6, 8).setZero();		//	right arm

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
				torque_l_vel_tun.segment(12, 3).setZero(); // waist torque
				torque_l_vel_tun.segment(23, 2).setZero(); // head torque

				lfoot_task_torque_switch = 1;
				rfoot_task_torque_switch = 1;

				lfoot_torque_g_switch = 0;
				rfoot_torque_g_switch = 0;
			}
			else if (foot_contact_ == 1)
			{
				torque_r_vel_tun.segment(12, 3).setZero(); // waist torque
				torque_r_vel_tun.segment(23, 2).setZero(); // head torque

				lfoot_task_torque_switch = 1;
				rfoot_task_torque_switch = 1;

				lfoot_torque_g_switch = 0;
				rfoot_torque_g_switch = 0;
			}
		}
	}

	torque_g_.segment(0, 6) = torque_g_.segment(0, 6) * lfoot_torque_g_switch;
	torque_g_.segment(6, 6) = torque_g_.segment(6, 6) * rfoot_torque_g_switch;
	
	// if( int(walking_duration_*100)%10 == 0 )
	// cout<<"walking_phase_: \n"<<walking_phase_<<endl;
	// torque += torque_l_vel_tun * lfoot_task_torque_switch + torque_r_vel_tun * rfoot_task_torque_switch;
	torque += torque_l_vel_tun + torque_r_vel_tun;
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

	// if( walking_phase_> 0.1)
	{
		// cout<<"pelv_rot_current_: \n"<<pelv_rot_current_<<endl;
		// cout<<"pelv_rpy_current_: \n"<<pelv_rpy_current_<<endl;
		// cout<<"f_star: \n"<<f_star<<endl;
		// cout<<"torque: \n"<<torque<<endl;
		// cout<<"torque_g: \n"<<torque_g<<endl;
		// cout<<"lfoot_task_torque_switch: \n"<<lfoot_task_torque_switch<<endl;
		// cout<<"rfoot_task_torque_switch: \n"<<rfoot_task_torque_switch<<endl;
		// cout<<"torque_l_vel_tun: \n"<<torque_l_vel_tun<<endl;
		// cout<<"torque_r_vel_tun: \n"<<torque_r_vel_tun<<endl;
		// cout<<"lfoot_to_com_jac_from_global_: \n"<<lfoot_to_com_jac_from_global_<<endl;
		// cout<<"test_com_jac: \n"<<test_com_jac<<endl;
	}

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
			swingfoot_f_star_l_ *= rd_.com_.mass*GRAVITY;
			swingfoot_f_star_l_ = 0.5*swingfoot_f_star_l_ + 0.5*swingfoot_f_star_l_pre_;
			
			swingfoot_f_star_r_ = 0.5*swingfoot_f_star_r_pre_;
		}
		else if(foot_contact_ == 1)
		{
			swingfoot_f_star_r_ = landing_3d_point - rhip_joint_position;
			swingfoot_f_star_r_.normalize();
			swingfoot_f_star_r_ *= rd_.com_.mass*GRAVITY;
			swingfoot_f_star_r_ = 0.5*swingfoot_f_star_r_ + 0.5*swingfoot_f_star_r_pre_;
			
			swingfoot_f_star_l_ = 0.5*swingfoot_f_star_l_pre_;
		}
	}
	else
	{
		swingfoot_f_star_l_ = 0.5*swingfoot_f_star_l_pre_;
		swingfoot_f_star_r_ = 0.5*swingfoot_f_star_r_pre_;
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
			// desired_q_(0) = motion_q_(0);
			Vector3d phi_swing_ankle;
			// phi_swing_ankle = -DyrosMath::getPhi(rd_.link_[Left_Foot].Rotm, pelv_yaw_rot_current_from_global_);
			phi_swing_ankle = -DyrosMath::getPhi(lfoot_transform_current_from_global_.linear(), Eigen::Matrix3d::Identity());
			// phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

			desired_q_dot_(4) = 200 * bandBlock(phi_swing_ankle(1), 20*DEG2RAD, -20*DEG2RAD); //swing ankle pitch	//(tune)
			desired_q_dot_(5) = 200 * bandBlock(phi_swing_ankle(0), 1*DEG2RAD, -1*DEG2RAD); //swing ankle roll	//(tune)
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


			desired_q_dot_(10) = 200 * bandBlock(phi_support_ankle(1), 20*DEG2RAD, -20*DEG2RAD);	//(tune)
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
			Vector3d phi_swing_ankle;
			// phi_swing_ankle = -DyrosMath::getPhi(rd_.link_[Right_Foot].Rotm, pelv_yaw_rot_current_from_global_);
			phi_swing_ankle = -DyrosMath::getPhi(rfoot_transform_current_from_global_.linear(), Eigen::Matrix3d::Identity());
			// phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

			desired_q_dot_(10) = 200 * bandBlock(phi_swing_ankle(1), 20*DEG2RAD, -20*DEG2RAD); //swing ankle pitch
			desired_q_dot_(11) = 200 * bandBlock(phi_swing_ankle(0), 1*DEG2RAD, -1*DEG2RAD); //swing ankle roll
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


			desired_q_dot_(4) = 200 * bandBlock(phi_support_ankle(1), 20*DEG2RAD, -20*DEG2RAD);;
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

		desired_q_(3) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 0.3 * walking_duration_, last_desired_q_(3), 0, 0, motion_q_(3), 0, 0)(0);
		desired_q_(9) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 0.3 * walking_duration_, last_desired_q_(9), 0, 0, motion_q_(9), 0, 0)(0);
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
	Vector3d phi_trunk;
	Matrix3d upper_body_rotaion_matrix = DyrosMath::rotateWithZ(motion_q_(12));
	phi_trunk = -DyrosMath::getPhi(pelv_yaw_rot_current_from_global_.transpose()* rd_.link_[Upper_Body].Rotm, upper_body_rotaion_matrix);
	// phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

	desired_q_dot_(12) = 50 * phi_trunk(2);	 //waist yaw //(tune)
	desired_q_dot_(13) = 30 * phi_trunk(1);	 //waist pitch
	desired_q_dot_(14) = -30 * phi_trunk(0); //waist roll

	desired_q_.segment(12, 3) = current_q_.segment(12, 3) + desired_q_dot_.segment(12, 3) * dt_;

	//////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////ARM & HEAD///////////////////////////////////////////////
	for (int i = 15; i < MODEL_DOF; i++)
	{
		desired_q_(i) = motion_q_(i);
		desired_q_dot_(i) = motion_q_dot_(i);
	}
	////////////////////////////////////////////////////////////////////////////////////
	
	///////////////////////////////ANKLE TUNING FOR VELOCITY TRACKING/////////////////
	// desired_q_ += jointComTrackingTuning(); //reference: Lee, Yoonsang, Sungeun Kim, and Jehee Lee. "Data-driven biped control." ACM SIGGRAPH 2010 papers. 2010. 1-8.
	//////////////////////////////////////////////////////////////////////////////////

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
	f_lfoot_damping_ = -support_foot_damping_gain_*lfoot_vel_current_from_global.segment(3, 3);
	f_rfoot_damping_ = -support_foot_damping_gain_*rfoot_vel_current_from_global.segment(3, 3);

	if(foot_swing_trigger_ == true)
	{
		if(foot_contact_ == 1)
		{
			f_lfoot_damping_ = 0.5*f_lfoot_damping_ + 0.5*f_lfoot_damping_pre_;
			f_rfoot_damping_ = 0.5*f_rfoot_damping_pre_;
		}
		else if (foot_contact_ == -1)
		{
			f_rfoot_damping_ = 0.5*f_rfoot_damping_ + 0.5*f_rfoot_damping_pre_;
			f_lfoot_damping_ = 0.5*f_lfoot_damping_pre_;
		}
	}
	else
	{
		f_lfoot_damping_ = 0.5*f_lfoot_damping_ + 0.5*f_lfoot_damping_pre_;
		f_rfoot_damping_ = 0.5*f_rfoot_damping_ + 0.5*f_rfoot_damping_pre_;
	}

	torque(4) += ((jac_lfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_lfoot_damping_ + (jac_rfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_rfoot_damping_)(4);
	torque(5) += ((jac_lfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_lfoot_damping_ + (jac_rfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_rfoot_damping_)(5);
	torque(10) += ((jac_lfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_lfoot_damping_ + (jac_rfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_rfoot_damping_)(10);
	torque(11) += ((jac_lfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_lfoot_damping_ + (jac_rfoot_.block(3, 6, 3, MODEL_DOF)).transpose()*f_rfoot_damping_)(11);
	/////////////////////////////////////////////////////////////////////////////////

	///////////////////joint angle damping///////////////////////////////////////////
	torque(1) += -1*current_q_dot_(1)*swingfoot_force_control_converter_;		// left hip roll
	torque(2) += -1*current_q_dot_(2)*swingfoot_force_control_converter_;		// left hip pitch
	torque(7) += -1*current_q_dot_(7)*swingfoot_force_control_converter_;		// right hip roll
	torque(8) += -1*current_q_dot_(8)*swingfoot_force_control_converter_;		// right hip pitch

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

Eigen::VectorQd CustomController::zmpAnkleControl()
{
	VectorQd zmp_ankle_torque;
	Vector3d zmp_target;
	zmp_ankle_torque.setZero();

	Eigen::Vector3d zmp_desired_lfoot_local;
	Eigen::Vector3d zmp_desired_rfoot_local;
	Eigen::Vector2d zmp_desired_zmp_both;

	double kp_zmp = 1;
	double kv_zmp = 0;
	//////////////////////zmp trajectory///////////////////////

	if ((foot_swing_trigger_ == true) || (walking_speed_ != 0))
	{
		if (foot_contact_ == 1)
		{
			zmp_target = lfoot_transform_current_from_global_.translation();
		}
		else if (foot_contact_ == -1)
		{
			zmp_target = rfoot_transform_current_from_global_.translation();
		}
	}
	else
	{
		zmp_target = middle_of_both_foot_;
	}

	// zmp_desired_from_global_ = 0.1*zmp_target + 0.9*zmp_desired_pre_;
	// zmp_desired_from_global_ = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, );
	zmp_desired_from_global_ = zmp_target;
	///////////////////////////////////////////////////////////////////

	zmp_desired_lfoot_local = lfoot_transform_current_from_global_.linear().transpose() * (zmp_desired_from_global_ - lfoot_transform_current_from_global_.translation());
	zmp_desired_rfoot_local = rfoot_transform_current_from_global_.linear().transpose() * (zmp_desired_from_global_ - rfoot_transform_current_from_global_.translation());

	if (foot_swing_trigger_ == true)
	{
		if (foot_contact_ == 1)
		{
			// zmp_ankle_torque(4) = -l_ft_(2)*zmp_desired_lfoot_local(0) + kp_zmp*(zmp_desired_lfoot_local(0) - zmp_local_lfoot_(0)) - kv_zmp*zmp_dot_local_lfoot_(0); //ankle pitch
			// zmp_ankle_torque(5) = l_ft_(2)*zmp_desired_lfoot_local(1) - kp_zmp*(zmp_desired_lfoot_local(1) - zmp_local_lfoot_(1)) + kv_zmp*zmp_dot_local_lfoot_(1); //ankle roll
			zmp_ankle_torque(4) = +kp_zmp * (zmp_desired_lfoot_local(0) - zmp_local_lfoot_(0)) - kv_zmp * zmp_dot_local_lfoot_(0); //ankle pitch
			zmp_ankle_torque(5) = -kp_zmp * (zmp_desired_lfoot_local(1) - zmp_local_lfoot_(1)) + kv_zmp * zmp_dot_local_lfoot_(1); //ankle roll
		}
		else if (foot_contact_ == -1)
		{
			// zmp_ankle_torque(10) = -r_ft_(2)*zmp_desired_rfoot_local(0) + kp_zmp*(zmp_desired_rfoot_local(0) - zmp_local_rfoot_(0)) - kv_zmp*zmp_dot_local_rfoot_(0); //ankle pitch
			// zmp_ankle_torque(11) = r_ft_(2)*zmp_desired_rfoot_local(1) - kp_zmp*(zmp_desired_rfoot_local(1) - zmp_local_rfoot_(1)) + kv_zmp*zmp_dot_local_rfoot_(1); //ankle roll
			zmp_ankle_torque(10) = +kp_zmp * (zmp_desired_rfoot_local(0) - zmp_local_rfoot_(0)) - kv_zmp * zmp_dot_local_rfoot_(0); //ankle pitch
			zmp_ankle_torque(11) = -kp_zmp * (zmp_desired_rfoot_local(1) - zmp_local_rfoot_(1)) + kv_zmp * zmp_dot_local_rfoot_(1); //ankle roll
		}
	}
	else
	{
		// Vector3d total_ankle_torque;
		// total_ankle_torque(0) = l_ft_(2)*(zmp_desired_from_global_(1) - lfoot_transform_current_from_global_.translation()(1))
		// + r_ft_(2)*(zmp_desired_from_global_(1) - rfoot_transform_current_from_global_.translation()(1))
		// -kp_zmp*(zmp_desired_from_global_(1) - zmp_measured_(1))
		// +kv_zmp*(zmp_dot_measured_(1)); //roll

		// total_ankle_torque(1) = -l_ft_(2)*(zmp_desired_from_global_(0) - lfoot_transform_current_from_global_.translation()(0))
		// - r_ft_(2)*(zmp_desired_from_global_(0) - rfoot_transform_current_from_global_.translation()(0));
		// +kp_zmp*(zmp_desired_from_global_(0) - zmp_measured_(0))
		// -kv_zmp*(zmp_dot_measured_(0)); //pitch

		// total_ankle_torque(2) = 0;

		// double left_zmp_ratio = abs(zmp_desired_from_global_(1) - lfoot_transform_current_from_global_.translation()(1))
		// /abs(rfoot_transform_current_from_global_.translation()(1) - lfoot_transform_current_from_global_.translation()(1));

		// left_zmp_ratio = DyrosMath::minmax_cut(left_zmp_ratio, 0, 1);

		// zmp_ankle_torque(4) = left_zmp_ratio*(lfoot_transform_current_from_global_.linear().transpose()*total_ankle_torque)(1);
		// zmp_ankle_torque(5) = left_zmp_ratio*(rfoot_transform_current_from_global_.linear().transpose()*total_ankle_torque)(0);

		// zmp_ankle_torque(10) = (1-left_zmp_ratio)*total_ankle_torque(1);
		// zmp_ankle_torque(11) = (1-left_zmp_ratio)*total_ankle_torque(0);
	}

	return zmp_ankle_torque;
}

Eigen::VectorQd CustomController::jointComTrackingTuning()
{
	Eigen::VectorQd desired_q_tune;
	desired_q_tune.setZero();

	//ankle
	double kp_ank_sag = 0.1;	//x direction ankle pitch gain for com position error
	double kv_ank_sag = 0.10; //x direction ankle pitch gain for com velocity error

	double kp_ank_cor = 0.1;	//y direction ankle pitch gain for com position error
	double kv_ank_cor = 0.1; //y direction ankle pitch gain for com velocity error

	//hip
	double kp_hip_sag = 0; //0.01; //x direction ankle pitch gain for com position error
	double kv_hip_sag = 0; //0.05; //x direction ankle pitch gain for com velocity error

	double kp_hip_cor = 0; //0.2; //y direction ankle pitch gain for com position error
	double kv_hip_cor = 0; //0.2; //y direction ankle pitch gain for com velocity error

	if (foot_swing_trigger_ == true)
	{
		double switching = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);
		if (foot_contact_ == 1) //left support
		{

			desired_q_tune(4) -= switching * (kp_ank_sag * (com_pos_desired_(0) - com_pos_current_(0)) + kv_ank_sag * (com_vel_desired_(0) - com_vel_current_(0)));
			desired_q_tune(5) += switching * (kp_ank_cor * (com_pos_desired_(1) - com_pos_current_(1)) + kv_ank_cor * (com_vel_desired_(1) - com_vel_current_(1)));

			if (desired_q_(8) < 0)
			{
				desired_q_tune(8) += switching * (kp_hip_sag * (com_pos_desired_(0) - com_pos_current_(0)) + kv_hip_sag * (com_vel_desired_(0) - com_vel_current_(0)));
			}
			desired_q_tune(7) -= switching * (kp_hip_cor * (com_pos_desired_(1) - com_pos_current_(1)) + kv_hip_cor * (com_vel_desired_(1) - com_vel_current_(1)));
		}
		else if (foot_contact_ == -1) //right support
		{
			desired_q_tune(10) -= switching * (kp_ank_sag * (com_pos_desired_(0) - com_pos_current_(0)) + kv_ank_sag * (com_vel_desired_(0) - com_vel_current_(0)));
			desired_q_tune(11) += switching * (kp_ank_cor * (com_pos_desired_(1) - com_pos_current_(1)) + kv_ank_cor * (com_vel_desired_(1) - com_vel_current_(1)));

			if (desired_q_(2) > 0)
			{
				desired_q_tune(2) += switching * (kp_hip_sag * (com_pos_desired_(0) - com_pos_current_(0)) + kv_hip_sag * (com_vel_desired_(0) - com_vel_current_(0)));
			}

			desired_q_tune(1) -= switching * (kp_hip_cor * (com_pos_desired_(1) - com_pos_current_(1)) + kv_hip_cor * (com_vel_desired_(1) - com_vel_current_(1)));
		}
	}
	else
	{
		desired_q_tune(4) -= kp_ank_sag * (com_pos_desired_(0) - com_pos_current_(0)) + kv_ank_sag * (com_vel_desired_(0) - com_vel_current_(0));
		desired_q_tune(5) += kp_ank_cor * (com_pos_desired_(1) - com_pos_current_(1)) + kv_ank_cor * (com_vel_desired_(1) - com_vel_current_(1));

		desired_q_tune(10) -= kp_ank_sag * (com_pos_desired_(0) - com_pos_current_(0)) + kv_ank_sag * (com_vel_desired_(0) - com_vel_current_(0));
		desired_q_tune(11) += kp_ank_cor * (com_pos_desired_(1) - com_pos_current_(1)) + kv_ank_cor * (com_vel_desired_(1) - com_vel_current_(1));
	}

	return desired_q_tune;
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
					cout<<"Swing leg tajectory is out of the workspace"<<endl;
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
					cout<<"Swing leg tajectory is out of the workspace"<<endl;
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

Eigen::VectorQd CustomController::tuneTorqueForZMPSafety(Eigen::VectorQd task_torque)
{
	Eigen::Vector2d diff_zmp_lfoot;
	Eigen::Vector2d diff_zmp_rfoot;
	Eigen::Vector2d diff_zmp_both;
	Eigen::Vector3d diff_btw_both_foot;
	double foot_size_x_front;
	double foot_size_x_rear;
	double foot_size_y;

	double safe_region_ratio = 0.75;
	double edge_region_ratio = 0.90;
	double left_ankle_pitch_tune = 1;
	double left_ankle_roll_tune = 1;
	double right_ankle_pitch_tune = 1;
	double right_ankle_roll_tune = 1;

	foot_size_x_front = 0.18;
	foot_size_x_rear = 0.12;

	foot_size_y = 0.085;

	Vector3d phi_support_ankle;
	Vector3d angvel_support_ankle;


	if (foot_swing_trigger_ == true)
	{
		if (foot_contact_ == 1)
		{
			// phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());
			// left_ankle_pitch_tune = DyrosMath::cubic(zmp_local_lfoot_(0)/foot_size(0) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);
			// left_ankle_roll_tune = DyrosMath::cubic(zmp_local_lfoot_(1)/foot_size(1) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);

			
			phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Left_Foot].Rotm, pelv_yaw_rot_current_from_global_);
			left_ankle_pitch_tune *= DyrosMath::cubic(abs(phi_support_ankle(1)), 0.00, 0.05, 1, 0, 0, 0);
			left_ankle_roll_tune *= DyrosMath::cubic(abs(phi_support_ankle(0)), 0.00, 0.05, 1, 0, 0, 0);

			angvel_support_ankle = lfoot_transform_current_from_global_.linear().transpose()*lfoot_vel_current_from_global.segment(3,3);
			// left_ankle_pitch_tune *= DyrosMath::cubic(abs(angvel_support_ankle(1)), 0.02, 0.05, 1, 0.0, 0, 0);
			// left_ankle_roll_tune *= DyrosMath::cubic(abs(angvel_support_ankle(0)), 0.02, 0.05, 1, 0.0, 0, 0);
			
			left_ankle_pitch_tune *= DyrosMath::cubic(l_ft_(2), -rd_.com_.mass * GRAVITY / 5, -rd_.com_.mass * GRAVITY / 10, 1, 0, 0, 0);
			left_ankle_roll_tune *= DyrosMath::cubic(l_ft_(2), -rd_.com_.mass * GRAVITY / 5, -rd_.com_.mass * GRAVITY / 10, 1, 0, 0, 0);

			diff_zmp_lfoot(0) = (zmp_local_lfoot_(0) - middle_of_both_foot_(0));
			diff_zmp_lfoot(1) = (zmp_local_lfoot_(1) - middle_of_both_foot_(1));

			// if(diff_zmp_lfoot(0) > 0)
			// {
			// 	left_ankle_pitch_tune *= DyrosMath::cubic(diff_zmp_lfoot(0) , safe_region_ratio*foot_size_x_front, edge_region_ratio*foot_size_x_front, 1, 0, 0, 0);
			// }
			// else
			// {
			// 	left_ankle_pitch_tune *= DyrosMath::cubic( -diff_zmp_lfoot(0) , safe_region_ratio*foot_size_x_rear, edge_region_ratio*foot_size_x_rear, 1, 0, 0, 0);
			// }
			// left_ankle_roll_tune *= DyrosMath::cubic( abs(diff_zmp_lfoot(1)) , safe_region_ratio*foot_size_y, edge_region_ratio*foot_size_y, 1, 0, 0, 0);

			task_torque(4) = task_torque(4) * left_ankle_pitch_tune;
			task_torque(5) = task_torque(5) * left_ankle_roll_tune;
		}
		else if (foot_contact_ == -1)
		{
			// diff_zmp_rfoot(0) = abs(zmp_measured_rfoot_(0) - rfoot_transform_current_from_global_.translation()(0));
			// diff_zmp_rfoot(1) = abs(zmp_measured_rfoot_(1) - rfoot_transform_current_from_global_.translation()(1));

			// right_ankle_pitch_tune = DyrosMath::cubic(zmp_local_rfoot_(0)/foot_size(0) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);
			// right_ankle_roll_tune = DyrosMath::cubic(zmp_local_rfoot_(1)/foot_size(1) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);

			phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Right_Foot].Rotm, pelv_yaw_rot_current_from_global_);
			right_ankle_pitch_tune *= DyrosMath::cubic(abs(phi_support_ankle(1)), 0.00, 0.05, 1, 0, 0, 0);
			right_ankle_roll_tune *= DyrosMath::cubic(abs(phi_support_ankle(0)), 0.00, 0.05, 1, 0, 0, 0);

			angvel_support_ankle = rfoot_transform_current_from_global_.linear().transpose()*rfoot_vel_current_from_global.segment(3,3);
			// right_ankle_pitch_tune *= DyrosMath::cubic(abs(angvel_support_ankle(1)), 0.02, 0.05, 1, 0.0, 0, 0);
			// right_ankle_roll_tune *= DyrosMath::cubic(abs(angvel_support_ankle(0)), 0.02, 0.05, 1, 0.0, 0, 0);
			
			right_ankle_pitch_tune *= DyrosMath::cubic(r_ft_(2), -rd_.com_.mass * GRAVITY / 5, -rd_.com_.mass * GRAVITY / 10, 1, 0, 0, 0);
			right_ankle_roll_tune *= DyrosMath::cubic(r_ft_(2), -rd_.com_.mass * GRAVITY / 5, -rd_.com_.mass * GRAVITY / 10, 1, 0, 0, 0);

			diff_zmp_rfoot(0) = (zmp_local_rfoot_(0) - middle_of_both_foot_(0));
			diff_zmp_rfoot(1) = (zmp_local_rfoot_(1) - middle_of_both_foot_(1));

			// if(diff_zmp_rfoot(0) > 0)
			// {
			// 	right_ankle_pitch_tune *= DyrosMath::cubic(diff_zmp_rfoot(0) , safe_region_ratio*foot_size_x_front, edge_region_ratio*foot_size_x_front, 1, 0, 0, 0);
			// }
			// else
			// {
			// 	right_ankle_pitch_tune *= DyrosMath::cubic( -diff_zmp_rfoot(0) , safe_region_ratio*foot_size_x_rear, edge_region_ratio*foot_size_x_rear, 1, 0, 0, 0);
			// }
			// right_ankle_roll_tune *= DyrosMath::cubic( abs(diff_zmp_rfoot(1)) , safe_region_ratio*foot_size_y, edge_region_ratio*foot_size_y, 1, 0, 0, 0);

			task_torque(10) = task_torque(10) * right_ankle_pitch_tune;
			task_torque(11) = task_torque(11) * right_ankle_roll_tune;
		}
	}
	else
	{
		diff_btw_both_foot = lfoot_transform_current_from_global_.translation() - rfoot_transform_current_from_global_.translation();
		diff_zmp_both(0) = abs(zmp_measured_(0) - middle_of_both_foot_(0));
		diff_zmp_both(1) = abs(zmp_measured_(1) - middle_of_both_foot_(1));

		// left_ankle_pitch_tune = DyrosMath::cubic(diff_zmp_both(0)/foot_size(0) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);
		// right_ankle_pitch_tune = left_ankle_pitch_tune;

		// left_ankle_roll_tune = DyrosMath::cubic(diff_zmp_both(1) , diff_btw_both_foot(1) + safe_region_ratio*foot_size(1), diff_btw_both_foot(1) + edge_region_ratio*foot_size(1), 1, 0, 0, 0);
		// right_ankle_roll_tune = left_ankle_roll_tune;

		if(l_ft_(2) < rd_.com_.mass*GRAVITY/5)
		{
			left_ankle_pitch_tune *= DyrosMath::cubic(l_ft_(2), -rd_.com_.mass * GRAVITY / 3, -rd_.com_.mass * GRAVITY / 100, 1, 0.00, 0, 0);
			left_ankle_roll_tune *= DyrosMath::cubic(l_ft_(2), -rd_.com_.mass * GRAVITY / 3, -rd_.com_.mass * GRAVITY / 100, 1, 0.00, 0, 0);
		}

		if(r_ft_(2) < rd_.com_.mass*GRAVITY/2)
		{
			right_ankle_pitch_tune *= DyrosMath::cubic(r_ft_(2), -rd_.com_.mass * GRAVITY / 3, -rd_.com_.mass * GRAVITY / 100, 1, 0.00, 0, 0);
			right_ankle_roll_tune *= DyrosMath::cubic(r_ft_(2), -rd_.com_.mass * GRAVITY / 3, -rd_.com_.mass * GRAVITY / 100, 1, 0.00, 0, 0);
		}


		phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Left_Foot].Rotm, pelv_yaw_rot_current_from_global_);
		left_ankle_pitch_tune *= DyrosMath::cubic(abs(phi_support_ankle(1)), 0.00, 0.05, 1, 0.0, 0, 0);
		left_ankle_roll_tune *= DyrosMath::cubic(abs(phi_support_ankle(0)), 0.00, 0.05, 1, 0.0, 0, 0);

		// angvel_support_ankle = lfoot_transform_current_from_global_.linear().transpose()*lfoot_vel_current_from_global.segment(3,3);
		// left_ankle_pitch_tune *= DyrosMath::cubic(abs(angvel_support_ankle(1)), 0.005, 0.1, 1, 0.0, 0, 0);
		// left_ankle_roll_tune *= DyrosMath::cubic(abs(angvel_support_ankle(0)), 0.005, 0.1, 1, 0.0, 0, 0);

		phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Right_Foot].Rotm, pelv_yaw_rot_current_from_global_);
		right_ankle_pitch_tune *= DyrosMath::cubic(abs(phi_support_ankle(1)), 0.00, 0.05, 1, 0.0, 0, 0);
		right_ankle_roll_tune *= DyrosMath::cubic(abs(phi_support_ankle(0)), 0.00, 0.05, 1, 0.0, 0, 0);

		// angvel_support_ankle = rfoot_transform_current_from_global_.linear().transpose()*rfoot_vel_current_from_global.segment(3,3);
		// right_ankle_pitch_tune *= DyrosMath::cubic(abs(angvel_support_ankle(1)), 0.005, 0.1, 1, 0.0, 0, 0);
		// right_ankle_roll_tune *= DyrosMath::cubic(abs(angvel_support_ankle(0)), 0.005, 0.1, 1, 0.0, 0, 0);


		task_torque(4) = task_torque(4) * left_ankle_pitch_tune;
		task_torque(5) = task_torque(5) * left_ankle_roll_tune;

		task_torque(10) = task_torque(10) * right_ankle_pitch_tune;
		task_torque(11) = task_torque(11) * right_ankle_roll_tune;
	}

	// task_torque(4) = DyrosMath::minmax_cut(task_torque(4), -(-l_ft_(2))*foot_size_x_rear, (-l_ft_(2))*foot_size_x_front );
	// task_torque(5) = DyrosMath::minmax_cut(task_torque(5), -(-l_ft_(2))*foot_size_y, (-l_ft_(2))*foot_size_y );
	// task_torque(10) = DyrosMath::minmax_cut(task_torque(10), -(-r_ft_(2))*foot_size_x_rear, (-r_ft_(2))*foot_size_x_front );
	// task_torque(11) = DyrosMath::minmax_cut(task_torque(11), -(-r_ft_(2))*foot_size_y, (-r_ft_(2))*foot_size_y );

	if ((left_ankle_pitch_tune * left_ankle_roll_tune * right_ankle_pitch_tune * right_ankle_roll_tune) <= 0.8)
	{
		if ( int(current_time_*2000)%1000 == 0)
		{
			cout<<"############### ankle torque tuning! ###############"<<endl;
			cout<<"########right angvel_support_ankle:"<<angvel_support_ankle<< "############"<<endl;
			cout<<"########left_ankle_pitch_tune:"<<left_ankle_pitch_tune<< "############"<<endl;
			cout<<"########left_ankle_roll_tune:"<<left_ankle_roll_tune<< "############"<<endl;
			cout<<"########right_ankle_pitch_tune:"<<right_ankle_pitch_tune<< "############"<<endl;
			cout<<"########right_ankle_roll_tune:"<<right_ankle_roll_tune<< "############"<<endl;

			cout<<"########diff_zmp_lfoot:"<<diff_zmp_lfoot<< "############"<<endl;
			cout<<"########diff_zmp_rfoot:"<<diff_zmp_rfoot<< "############"<<endl;
			cout<<"############### ankle torque tuning! ###############"<<endl;
		}
	}

	return task_torque;
}

Eigen::VectorQd CustomController::jointLimit()
{

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

/////////////////////////////////////PREVIEW CONTROL RELATED FUNCTION////////////////////////////////////
void CustomController::getComTrajectory_Preview()
{ 
	// on-line preview 
	xs_(0) = com_pos_current_(0); ys_(0) = com_pos_current_(1);
	// xs_(1) = com_vel_current_(0); ys_(1) = com_vel_current_(1);
	//   xs_(2) = com_acc_current_(0); ys_(2) = com_acc_current_(1);

	// if(foot_contact_ == foot_contact_pre_)
	// {
	// 	xs_(0) = support_foot_transform_current_.translation()(0) + (xd_(0) - support_foot_transform_pre_.translation()(0));
	// 	ys_(0) = support_foot_transform_current_.translation()(1) + (yd_(0) - support_foot_transform_pre_.translation()(1));
	// 	// xs_(1) = com_vel_desired_preview_pre_(0);
	// 	// ys_(1) = com_vel_desired_preview_pre_(1);
	// 	// xs_(2) = com_acc_desired_preview_pre_(0);
	// 	// ys_(2) = com_acc_desired_preview_pre_(1);
	// }
	// else
	// {				
	// 	xs_(0) = support_foot_transform_current_.translation()(0) + (xd_(0) - swing_foot_transform_pre_.translation()(0));
	// 	ys_(0) = support_foot_transform_current_.translation()(1) + (yd_(0) - swing_foot_transform_pre_.translation()(1));
	// 	// xs_(1) = com_vel_desired_preview_pre_(0);
	// 	// ys_(1) = com_vel_desired_preview_pre_(1);
	// 	// xs_(2) = com_acc_desired_preview_pre_(0);
	// 	// ys_(2) = com_acc_desired_preview_pre_(1);
	// }


	modifiedPreviewControl_MJ();


	// com_pos_desired_preview_(0) = DyrosMath::QuinticSpline(current_time_, preview_update_time_, preview_update_time_+1/preview_hz_, xd_b(0), xd_b(1), xd_b(2), xd_(0), xd_(1), xd_(2))(0);
	// com_vel_desired_preview_(0) = DyrosMath::QuinticSpline(current_time_, preview_update_time_, preview_update_time_+1/preview_hz_, xd_b(0), xd_b(1), xd_b(2), xd_(0), xd_(1), xd_(2))(1);
	// com_acc_desired_preview_(0) = DyrosMath::QuinticSpline(current_time_, preview_update_time_, preview_update_time_+1/preview_hz_, xd_b(0), xd_b(1), xd_b(2), xd_(0), xd_(1), xd_(2))(2);

	// com_pos_desired_preview_(1) = DyrosMath::QuinticSpline(current_time_, preview_update_time_, preview_update_time_+1/preview_hz_, yd_b(0), yd_b(1), yd_b(2), yd_(0), yd_(1), yd_(2))(0);
	// com_vel_desired_preview_(1) = DyrosMath::QuinticSpline(current_time_, preview_update_time_, preview_update_time_+1/preview_hz_, yd_b(0), yd_b(1), yd_b(2), yd_(0), yd_(1), yd_(2))(1);
	// com_acc_desired_preview_(1) = DyrosMath::QuinticSpline(current_time_, preview_update_time_, preview_update_time_+1/preview_hz_, yd_b(0), yd_b(1), yd_b(2), yd_(0), yd_(1), yd_(2))(2);

	// off-line preview
	// xs_(0) = xd_(0); ys_(0) = yd_(0);
	// // On,off-line preview 
	xs_(1) = xd_(1); ys_(1) = yd_(1); // 여기서부터
	xs_(2) = xd_(2); ys_(2) = yd_(2);



	// com_pos_desired_preview_(0) = xd_(0);
	// com_pos_desired_preview_(1) = yd_(0);
	// com_vel_desired_preview_(0) = xd_(1);
	// com_vel_desired_preview_(1) = yd_(1);
	// com_acc_desired_preview_(0) = xd_(2);
	// com_acc_desired_preview_(1) = yd_(2);

	
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

	if( int((current_time_-program_start_time_)*1e9)%int(2e7) == 0 ) //50hz update
	{
		previewParam_MJ(1/preview_hz_, zmp_size_, zc_, K_act_ , Gi_, Gd_, Gx_, A_, B_, C_, D_, A_bar_, B_bar_);
		//previewParam_MJ_CPM(1.0/hz_, 16*hz_/10, K_ ,com_support_init_, Gi_, Gd_, Gx_, A_, B_, C_, D_, A_bar_, B_bar_);
	}


	if( int((current_time_-program_start_time_)*100000)%int(100000/preview_hz_) == 0 ) // 200hz update
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
  R(0,0) = 0.000001;

  Eigen::MatrixXd Qx;
  Qx.setZero(3,3);


  Eigen::MatrixXd Q_bar;
  Q_bar.setZero(4,4);
  Q_bar(0,0) = Qe(0,0);

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
			cout<<"###########FALLING IS DETECTED###################"<<endl;
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
			cout<<"###########FEET IS DETACHED###################"<<endl;
		}
	}


	
	if(falling_detection_flag_ == true)
	{
		torque_g_ = wbc_.gravity_compensation_torque(rd_, true);

		for (int i = 0; i < MODEL_DOF; i++)
		{
			desired_q_(i) = DyrosMath::QuinticSpline(current_time_, fall_start_time_, fall_start_time_+3.0, fall_init_q_(i), 0, 0, zero_q_(i), 0, 0)(0);
			desired_q_dot_(i) = DyrosMath::QuinticSpline(current_time_, fall_start_time_, fall_start_time_+3.0, fall_init_q_(i), 0, 0, zero_q_(i), 0, 0)(1);
			torque_task_(i) = (100 * (desired_q_(i) - current_q_(i)) + 20 * (desired_q_dot_(i) - current_q_dot_(i))) + torque_g_(i);
		}	
	}
}

double CustomController::bandBlock(double value, double max, double min)
{
	double y;

	if(value <= min)
	{
		y = value - min;
	}
	else if(value >= max)
	{
		y = value - max;
	}
	else
	{
		y = 0;
	}
	
	return y;
}

void CustomController::printOutTextFile()
{
	if (int( (current_time_ - program_start_time_) * 2000) % 1 == 0) // 2000 hz 
	{
		file[0]<<current_time_ - program_start_time_<<"\t"<<walking_phase_<<"\t"<<foot_contact_<<"\t"
		<<foot_swing_trigger_<<"\t"<<first_step_trigger_<<"\t"<<start_walking_trigger_<<"\t"
		<<stop_walking_trigger_<<"\t"<<stance_start_time_<<"\t"<<walking_duration_<<"\t"
		<<turning_duration_<<"\t"<<turning_phase_<<"\t"<<knee_target_angle_<<endl;

		file[1]<<current_time_ - program_start_time_<<"\t"<<walking_phase_<<"\t"<<foot_contact_<<"\t"		//1
		<<com_pos_current_(0)<<"\t"<<com_pos_current_(1)<<"\t"<<com_pos_current_(2)<<"\t"                  	//4
		<<com_vel_current_(0)<<"\t"<<com_vel_current_(1)<<"\t"<<com_vel_current_(2)<<"\t"					//7
		<<com_acc_current_(0)<<"\t"<<com_acc_current_(1)<<"\t"<<com_acc_current_(2)<<"\t"					//10
		<<com_pos_desired_(0)<<"\t"<<com_pos_desired_(1)<<"\t"<<com_pos_desired_(2)<<"\t"					//13
		<<com_vel_desired_(0)<<"\t"<<com_vel_desired_(1)<<"\t"<<com_vel_desired_(2)<<"\t"					//16
		<<com_acc_desired_(0)<<"\t"<<com_acc_desired_(1)<<"\t"<<com_acc_desired_(2)<<"\t"					//19
		<<com_vel_est1_(0)<<"\t"<<com_vel_est1_(1)<<"\t"<<com_vel_est1_(2)<<"\t"
		<<com_vel_est2_(0)<<"\t"<<com_vel_est2_(1)<<"\t"<<com_vel_est2_(2)<<endl;


		file[2]<<current_time_ - program_start_time_<<"\t"<<walking_phase_<<"\t"<<foot_contact_<<"\t"																								//1
		<<lfoot_transform_current_from_global_.translation()(0)<<"\t"<<lfoot_transform_current_from_global_.translation()(1)<<"\t"<<lfoot_transform_current_from_global_.translation()(2)<<"\t"		//4
		<<rfoot_transform_current_from_global_.translation()(0)<<"\t"<<rfoot_transform_current_from_global_.translation()(1)<<"\t"<<rfoot_transform_current_from_global_.translation()(2)<<"\t"		//7
		<<swing_foot_pos_trajectory_from_global_(0)<<"\t"<<swing_foot_pos_trajectory_from_global_(1)<<"\t"<<swing_foot_pos_trajectory_from_global_(2)<<"\t"											//10
		<<lfoot_vel_current_from_global(0)<<"\t"<<lfoot_vel_current_from_global(1)<<"\t"<<lfoot_vel_current_from_global(2)<<"\t"																	//13
		<<lfoot_vel_current_from_global(3)<<"\t"<<lfoot_vel_current_from_global(4)<<"\t"<<lfoot_vel_current_from_global(5)<<"\t"																	//16
		<<rfoot_vel_current_from_global(0)<<"\t"<<rfoot_vel_current_from_global(1)<<"\t"<<rfoot_vel_current_from_global(2)<<"\t"																	//19
		<<rfoot_vel_current_from_global(3)<<"\t"<<rfoot_vel_current_from_global(4)<<"\t"<<rfoot_vel_current_from_global(5)<<"\t"																	//22
		<<swing_foot_vel_trajectory_from_global_(0)<<"\t"<<swing_foot_vel_trajectory_from_global_(1)<<"\t"<<swing_foot_vel_trajectory_from_global_(2)<<"\t"											//25
		<<support_foot_vel_current_(3)<<"\t"<<support_foot_vel_current_(4)<<"\t"<<support_foot_vel_current_(5)<<endl;																				//28

		file[3]<<current_time_ - program_start_time_<<"\t"<<walking_phase_<<"\t"<<foot_contact_<<"\t"
		<<ControlVal_(0)<<"\t"<<ControlVal_(1)<<"\t"<<ControlVal_(2)<<"\t"<<ControlVal_(3)<<"\t"<<ControlVal_(4)<<"\t"<<ControlVal_(5)<<"\t"<<ControlVal_(6)<<"\t"<<ControlVal_(7)<<"\t"<<ControlVal_(8)<<"\t"<<ControlVal_(9)<<"\t"<<ControlVal_(10)<<"\t"<<ControlVal_(11)<<"\t"<<ControlVal_(12)<<"\t"<<ControlVal_(13)<<"\t"<<ControlVal_(14)<<"\t"<<ControlVal_(15)<<"\t"<<ControlVal_(16)<<"\t"<<ControlVal_(17)<<"\t"<<ControlVal_(18)<<"\t"<<ControlVal_(19)<<"\t"<<ControlVal_(20)<<"\t"<<ControlVal_(21)<<"\t"<<ControlVal_(22)<<"\t"<<ControlVal_(23)<<"\t"<<ControlVal_(24)<<"\t"<<ControlVal_(25)<<"\t"<<ControlVal_(26)<<"\t"<<ControlVal_(27)<<"\t"<<ControlVal_(28)<<"\t"<<ControlVal_(29)<<"\t"<<ControlVal_(30)<<"\t"<<ControlVal_(31)<<"\t"<<ControlVal_(32)<<endl; 	

		file[4]<<current_time_ - program_start_time_<<"\t"<<walking_phase_<<"\t"<<foot_contact_<<"\t"
		<<desired_q_(0)<<"\t"<<desired_q_(1)<<"\t"<<desired_q_(2)<<"\t"<<desired_q_(3)<<"\t"<<desired_q_(4)<<"\t"<<desired_q_(5)<<"\t"<<desired_q_(6)<<"\t"<<desired_q_(7)<<"\t"<<desired_q_(8)<<"\t"<<desired_q_(9)<<"\t"<<desired_q_(10)<<"\t"<<desired_q_(11)<<"\t"<<desired_q_(12)<<"\t"<<desired_q_(13)<<"\t"<<desired_q_(14)<<"\t"<<desired_q_(15)<<"\t"<<desired_q_(16)<<"\t"<<desired_q_(17)<<"\t"<<desired_q_(18)<<"\t"<<desired_q_(19)<<"\t"<<desired_q_(20)<<"\t"<<desired_q_(21)<<"\t"<<desired_q_(22)<<"\t"<<desired_q_(23)<<"\t"<<desired_q_(24)<<"\t"<<desired_q_(25)<<"\t"<<desired_q_(26)<<"\t"<<desired_q_(27)<<"\t"<<desired_q_(28)<<"\t"<<desired_q_(29)<<"\t"<<desired_q_(30)<<"\t"<<desired_q_(31)<<"\t"<<desired_q_(32)<<"\t" 
		<<current_q_(0)<<"\t"<<current_q_(1)<<"\t"<<current_q_(2)<<"\t"<<current_q_(3)<<"\t"<<current_q_(4)<<"\t"<<current_q_(5)<<"\t"<<current_q_(6)<<"\t"<<current_q_(7)<<"\t"<<current_q_(8)<<"\t"<<current_q_(9)<<"\t"<<current_q_(10)<<"\t"<<current_q_(11)<<"\t"<<current_q_(12)<<"\t"<<current_q_(13)<<"\t"<<current_q_(14)<<"\t"<<current_q_(15)<<"\t"<<current_q_(16)<<"\t"<<current_q_(17)<<"\t"<<current_q_(18)<<"\t"<<current_q_(19)<<"\t"<<current_q_(20)<<"\t"<<current_q_(21)<<"\t"<<current_q_(22)<<"\t"<<current_q_(23)<<"\t"<<current_q_(24)<<"\t"<<current_q_(25)<<"\t"<<current_q_(26)<<"\t"<<current_q_(27)<<"\t"<<current_q_(28)<<"\t"<<current_q_(29)<<"\t"<<current_q_(30)<<"\t"<<current_q_(31)<<"\t"<<current_q_(32)<<"\t"
		<<desired_q_dot_(0)<<"\t"<<desired_q_dot_(1)<<"\t"<<desired_q_dot_(2)<<"\t"<<desired_q_dot_(3)<<"\t"<<desired_q_dot_(4)<<"\t"<<desired_q_dot_(5)<<"\t"<<desired_q_dot_(6)<<"\t"<<desired_q_dot_(7)<<"\t"<<desired_q_dot_(8)<<"\t"<<desired_q_dot_(9)<<"\t"<<desired_q_dot_(10)<<"\t"<<desired_q_dot_(11)<<"\t"<<desired_q_dot_(12)<<"\t"<<desired_q_dot_(13)<<"\t"<<desired_q_dot_(14)<<"\t"<<desired_q_dot_(15)<<"\t"<<desired_q_dot_(16)<<"\t"<<desired_q_dot_(17)<<"\t"<<desired_q_dot_(18)<<"\t"<<desired_q_dot_(19)<<"\t"<<desired_q_dot_(20)<<"\t"<<desired_q_dot_(21)<<"\t"<<desired_q_dot_(22)<<"\t"<<desired_q_dot_(23)<<"\t"<<desired_q_dot_(24)<<"\t"<<desired_q_dot_(25)<<"\t"<<desired_q_dot_(26)<<"\t"<<desired_q_dot_(27)<<"\t"<<desired_q_dot_(28)<<"\t"<<desired_q_dot_(29)<<"\t"<<desired_q_dot_(30)<<"\t"<<desired_q_dot_(31)<<"\t"<<desired_q_dot_(32)<<"\t" 
		<<current_q_dot_(0)<<"\t"<<current_q_dot_(1)<<"\t"<<current_q_dot_(2)<<"\t"<<current_q_dot_(3)<<"\t"<<current_q_dot_(4)<<"\t"<<current_q_dot_(5)<<"\t"<<current_q_dot_(6)<<"\t"<<current_q_dot_(7)<<"\t"<<current_q_dot_(8)<<"\t"<<current_q_dot_(9)<<"\t"<<current_q_dot_(10)<<"\t"<<current_q_dot_(11)<<"\t"<<current_q_dot_(12)<<"\t"<<current_q_dot_(13)<<"\t"<<current_q_dot_(14)<<"\t"<<current_q_dot_(15)<<"\t"<<current_q_dot_(16)<<"\t"<<current_q_dot_(17)<<"\t"<<current_q_dot_(18)<<"\t"<<current_q_dot_(19)<<"\t"<<current_q_dot_(20)<<"\t"<<current_q_dot_(21)<<"\t"<<current_q_dot_(22)<<"\t"<<current_q_dot_(23)<<"\t"<<current_q_dot_(24)<<"\t"<<current_q_dot_(25)<<"\t"<<current_q_dot_(26)<<"\t"<<current_q_dot_(27)<<"\t"<<current_q_dot_(28)<<"\t"<<current_q_dot_(29)<<"\t"<<current_q_dot_(30)<<"\t"<<current_q_dot_(31)<<"\t"<<current_q_dot_(32)<<endl; 

		file[5]<<current_time_ - program_start_time_<<"\t"<<walking_phase_<<"\t"<<foot_contact_<<"\t"
		<<master_lhand_pose_.translation()(0)<<"\t"<<master_lhand_pose_.translation()(1)<<"\t"<<master_lhand_pose_.translation()(2)<<"\t"<<master_rhand_pose_.translation()(0)<<"\t"<<master_rhand_pose_.translation()(1)<<"\t"<<master_rhand_pose_.translation()(2)<<"\t"
		<<lhand_transform_current_from_global_.translation()(0)<<"\t"<<lhand_transform_current_from_global_.translation()(1)<<"\t"<<lhand_transform_current_from_global_.translation()(2)<<"\t"<<rhand_transform_current_from_global_.translation()(0)<<"\t"<<rhand_transform_current_from_global_.translation()(1)<<"\t"<<rhand_transform_current_from_global_.translation()(2)<<"\t"
		<<master_lhand_vel_(0)<<"\t"<<master_lhand_vel_(1)<<"\t"<<master_lhand_vel_(2)<<"\t"<<master_rhand_vel_(0)<<"\t"<<master_rhand_vel_(1)<<"\t"<<master_rhand_vel_(2)<<"\t"
		<<lhand_vel_current_from_global(0)<<"\t"<<lhand_vel_current_from_global(1)<<"\t"<<lhand_vel_current_from_global(2)<<"\t"<<rhand_vel_current_from_global(0)<<"\t"<<rhand_vel_current_from_global(1)<<"\t"<<rhand_vel_current_from_global(2)<<endl;
	}
}