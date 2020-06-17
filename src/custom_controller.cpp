#include "custom_controller.h"

CustomController::CustomController(DataContainer &dc, RobotData &rd) : dc_(dc), rd_(rd), wbc_(dc.wbc_)
{
	ControlVal_.setZero();
	walking_speed_command = dc.nh.subscribe("/tocabi/walkingspeedcommand", 100, &CustomController::WalkingSpeedCommandCallback, this);
	walking_duration_command = dc.nh.subscribe("/tocabi/walkingdurationcommand", 100, &CustomController::WalkingDurationCommandCallback, this);
	walking_angvel_command = dc.nh.subscribe("/tocabi/walkingangvelcommand", 100, &CustomController::WalkingAngVelCommandCallback, this);
	knee_target_angle_command = dc.nh.subscribe("/tocabi/kneetargetanglecommand", 100, &CustomController::KneeTargetAngleCommandCallback, this);
	foot_height_command = dc.nh.subscribe("/tocabi/footheightcommand", 100, &CustomController::FootHeightCommandCallback, this);
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
		wbc_.set_contact(rd_, 1, 1);

		int task_number = 6;
		rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
		rd_.f_star.setZero(task_number);

		rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac;

		if (tc.custom_taskgain)
		{
			rd_.link_[COM_id].pos_p_gain = Vector3d::Ones() * tc.pos_p;
			rd_.link_[COM_id].pos_d_gain = Vector3d::Ones() * tc.pos_d;
			rd_.link_[COM_id].rot_p_gain = Vector3d::Ones() * tc.ang_p;
			rd_.link_[COM_id].rot_d_gain = Vector3d::Ones() * tc.ang_d;
		}

		rd_.link_[COM_id].x_desired = tc.ratio * rd_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos;
		rd_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * rd_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos(2);
		rd_.link_[COM_id].rot_desired = Matrix3d::Identity();
		rd_.link_[COM_id].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
		rd_.link_[COM_id].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, true);
		//rd_.link_[COM_id].Set_T
		rd_.f_star = wbc_.getfstar6d(rd_, COM_id);

		ControlVal_ = wbc_.task_control_torque_QP(rd_, rd_.J_task, rd_.f_star);
		//task controller for mode 11 ....
	}
	else if (tc.mode == 12)
	{
		wbc_.set_contact(rd_, 1, 1);

		int task_number = 9;
		rd_.J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
		rd_.f_star.setZero(task_number);

		rd_.J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac;
		rd_.J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Upper_Body].Jac_COM_r;

		if (tc.custom_taskgain)
		{
			rd_.link_[COM_id].pos_p_gain = Vector3d::Ones() * tc.pos_p;
			rd_.link_[COM_id].pos_d_gain = Vector3d::Ones() * tc.pos_d;
			rd_.link_[COM_id].rot_p_gain = Vector3d::Ones() * tc.ang_p;
			rd_.link_[COM_id].rot_d_gain = Vector3d::Ones() * tc.ang_d;
			rd_.link_[Upper_Body].rot_p_gain = Vector3d::Ones() * tc.ang_p;
			rd_.link_[Upper_Body].rot_d_gain = Vector3d::Ones() * tc.ang_d;
		}

		rd_.link_[COM_id].x_desired = tc.ratio * rd_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos;
		rd_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * rd_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * rd_.link_[Right_Foot].xpos(2);

		rd_.link_[COM_id].Set_Trajectory_from_quintic(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time);
		rd_.link_[COM_id].rot_desired = Matrix3d::Identity();
		rd_.link_[COM_id].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
		rd_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
		rd_.link_[Upper_Body].Set_Trajectory_rotation(rd_.control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
		//rd_.link_[COM_id].Set_T

		rd_.f_star.segment(0, 6) = wbc_.getfstar6d(rd_, COM_id);
		rd_.f_star.segment(6, 3) = wbc_.getfstar_rot(rd_, Upper_Body);

		ControlVal_ = wbc_.task_control_torque_QP(rd_, rd_.J_task, rd_.f_star);
		//task controller for mode 11 ....
	}
	else if (tc.mode == 13)
	{
		//////////Generalized Biped Walking Controller/////////////////////

		torque_task_.setZero();
		if (tc.task_init == true)
		{
			initWalkingParameter();
			tc.task_init = false;
		}

		getRobotData(wbc_);
		walkingStateManager();
		getProcessedRobotData(wbc_);
			
		motionGenerator();
		getCOMTrajectory();
		getSwingFootXYTrajectory(walking_phase_, com_pos_current_, com_vel_current_, com_vel_desired_);

		// torque_task_ += comVelocityControlCompute(wbc_);             //support control for COM velocity control
		torque_task_ += comVelocityControlCompute2(wbc_); //support control for COM velocity control
		if (int(current_time_ * 2000) % 400 == 0)
			cout<<"torque_task_com_vel: \n" << torque_task_<<endl;
		// cout<<"walking_speed: \n" << walking_speed_<<endl;
		// first_torque_supplier_ = 1;
		torque_task_ += jointTrajectoryPDControlCompute(wbc_) * first_torque_supplier_; //upper body motion + swing foot control + knee angle control
		// if(walking_phase_ > 0.1)
		// {
		// cout<<"torque_task_joint_pd: \n" << jointTrajectoryPDControlCompute(wbc_)*first_torque_supplier_<<endl;
		//     cout<<"first_torque_supplier_: \n" << first_torque_supplier_<<endl;
		// }
			
		savePreData();

		////////////////////////////////TORQUE LIMIT//////////////////////////////
		for (int i = 0; i < MODEL_DOF; i++)
		{
			torque_task_(i) = DyrosMath::minmax_cut(torque_task_(i), -300, 300);
		}
		///////////////////////////////////////////////////////////////////////////
		ControlVal_ = torque_task_;
			
		if (int(current_time_ * 2000) % 400 == 0)
		{
			cout << "walking_phase_: " << walking_phase_ << endl;
			cout << "foot_contact_: " << foot_contact_ << endl;
			
			cout << "foot_swing_trigger_: " << foot_swing_trigger_ << endl;
			cout << "stop_walking_trigger_: " << stop_walking_trigger_ << endl;
			cout << "first_step_trigger_: " << first_step_trigger_ << endl;

			cout << "ControlVal_: " << ControlVal_ << endl;
		}

	}
}

void CustomController::computeFast()
{
	if (tc.mode == 10)
	{
	}
	else if (tc.mode == 11)
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
	stop_vel_threshold_ = 0.20;
	walking_duration_ = 0.6;
	walking_speed_ = 0.3;
	step_width_ = 0.091;
	knee_target_angle_ = 0.1; //
	// knee_target_angle_ = M_PI/40;                               //4.5degree
	yaw_angular_vel_ = 0; //   rad/s
	swing_foot_height_ = 0.05;
	switching_phase_duration_ = 0.01;
	foot_contact_ = 1;

	start_walking_trigger_ = false;
	first_step_trigger_ = false;
	foot_swing_trigger_ = false;
	stop_walking_trigger_ = true;

	jac_rhand_.setZero(6, MODEL_DOF_VIRTUAL);
	jac_lhand_.setZero(6, MODEL_DOF_VIRTUAL);
	jac_rfoot_.setZero(6, MODEL_DOF_VIRTUAL);
	jac_lfoot_.setZero(6, MODEL_DOF_VIRTUAL);
	//set init pre data
	com_pos_desired_pre_ = dc_.link_[COM_id].xpos;
	com_vel_desired_pre_.setZero();
	pre_time_ = rd_.control_time_;
	pre_desired_q_ = rd_.q_;

	init_q_ = rd_.q_;
	desired_q_ = init_q_;
	desired_q_dot_.setZero();
	desired_q_ddot_.setZero();
	torque_task_.setZero();
	torque_task_pre_.setZero();
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

	com_pos_current_ = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[COM_id].xpos - pelv_pos_current_);
	com_vel_current_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[COM_id].v;
	// com_acc_current_ = ;

	lfoot_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Left_Foot].xpos - pelv_pos_current_);
	lfoot_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot].Rotm;
	rfoot_transform_current_from_global_.translation() = pelv_yaw_rot_current_from_global_.transpose() * (rd_.link_[Right_Foot].xpos - pelv_pos_current_);
	rfoot_transform_current_from_global_.linear() = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot].Rotm;

	lfoot_vel_current_from_global.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot].v;
	lfoot_vel_current_from_global.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Left_Foot].w;

	rfoot_vel_current_from_global.segment(0, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot].v;
	rfoot_vel_current_from_global.segment(3, 3) = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Right_Foot].w;

	Matrix6d R_R;
	R_R.setZero();
	R_R.block(0, 0, 3, 3) = pelv_yaw_rot_current_from_global_.transpose();
	R_R.block(3, 3, 3, 3) = pelv_yaw_rot_current_from_global_.transpose();

	jac_com_ = R_R * rd_.link_[COM_id].Jac;
	jac_com_pos_ = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[COM_id].Jac_COM_p;
	jac_rhand_ = R_R * rd_.link_[Right_Hand].Jac;
	jac_lhand_ = R_R * rd_.link_[Left_Hand].Jac;
	jac_rfoot_ = R_R * rd_.link_[Right_Foot].Jac;
	jac_lfoot_ = R_R * rd_.link_[Left_Foot].Jac;

	rd_.link_[Left_Foot].v;

	Eigen::Vector3d zmp_local_both_foot;
	// rd_.ZMP_ft = wc_.GetZMPpos(rd_);

	// zmp_local_both_foot = wc.GetZMPpos_fromFT(rd_).segment(0, 2);  //get zmp using f/t sensors on both foot

	zmp_local_lfoot_(0) = -rd_.ContactForce_FT(4) / rd_.ContactForce_FT(2);
	zmp_local_lfoot_(1) = -rd_.ContactForce_FT(3) / rd_.ContactForce_FT(2);
	zmp_local_rfoot_(0) = -rd_.ContactForce_FT(10) / rd_.ContactForce_FT(8);
	zmp_local_rfoot_(1) = -rd_.ContactForce_FT(9) / rd_.ContactForce_FT(8);

	zmp_dot_local_lfoot_ = (zmp_local_lfoot_ - zmp_local_lfoot_pre_) / dt_;
	zmp_dot_local_rfoot_ = (zmp_local_rfoot_ - zmp_local_rfoot_pre_) / dt_;

	zmp_measured_lfoot_ = lfoot_transform_current_from_global_.linear() * zmp_local_lfoot_ + lfoot_transform_current_from_global_.translation(); //from global

	zmp_measured_rfoot_ = rfoot_transform_current_from_global_.linear() * zmp_local_lfoot_ + rfoot_transform_current_from_global_.translation();

	zmp_measured_ = (zmp_measured_lfoot_ * rd_.ContactForce_FT(2) + zmp_measured_rfoot_ * rd_.ContactForce_FT(8)) / (rd_.ContactForce_FT(2) + rd_.ContactForce_FT(8)); //from global
	zmp_dot_measured_ = (zmp_measured_ - zmp_measured_pre_) / dt_;

	l_ft_ = rd_.ContactForce_FT.segment(0, 6);
	r_ft_ = rd_.ContactForce_FT.segment(6, 6);

	first_torque_supplier_ = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.1, 0, 1, 0, 0);
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
						foot_contact_ = -foot_contact_;  //support foot change
						cout << " ################################ Balancing Control ON! ################################" << endl;
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
				// if( checkZMPinWhichFoot(zmp_measured_) == true )
				if ((com_pos_current_(0) - support_foot_transform_current_.translation()(0)) / (walking_speed_ * walking_duration_) > 0.2 || (com_vel_current_(0) / (walking_speed_ + 1e-3) > 0.4))
				{
					first_step_trigger_ = true;
					foot_swing_trigger_ = true;
					start_walking_trigger_ = false;
					cout << " ################################ First Step Triggered! ################################" << endl;
				}
				else
				{
					foot_swing_trigger_ = false;
					first_step_trigger_ = false;
					start_walking_trigger_ = true;
					start_time_ = current_time_;
					;
				}
			}
			else
			{

				if (foot_contact_ == 1)
				{
					Vector3d phi_swing_ankle;
					phi_swing_ankle = -DyrosMath::getPhi(rfoot_transform_current_from_global_.linear(), swing_foot_rot_trajectory_from_global_);

					if ((walking_phase_ > 0.5) && (-r_ft_(2) > rd_.com_.mass * GRAVITY / 2) && (abs(phi_swing_ankle.norm()) < 0.03))
					{
						start_time_ = current_time_ - walking_duration_; //make walking_phase_ 1
						walking_phase_ = 1;
						cout << " ################ Early Step Change Occured! #########################" << endl;
					}
				}
				else if (foot_contact_ == -1)
				{
					Vector3d phi_swing_ankle;
					phi_swing_ankle = -DyrosMath::getPhi(lfoot_transform_current_from_global_.linear(), swing_foot_rot_trajectory_from_global_);

					if ((walking_phase_ > 0.5) && (-l_ft_(2) > rd_.com_.mass * GRAVITY / 2) && (abs(phi_swing_ankle.norm()) < 0.03))
					{
						start_time_ = current_time_ - walking_duration_; //make walking_phase_ 1
						walking_phase_ = 1;
						cout << " ################ Early Step Change Occured! #########################" << endl;
					}
				}
			}
		}
	}
	else if (walking_phase_ == 1)
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
				cout << " ################################ Balancing Control ON! ################################" << endl;
			}
			else
			{
				foot_swing_trigger_ = false;
				stop_walking_trigger_ = true;  //robot stop
				first_step_trigger_ = false;
				start_walking_trigger_ = false;

				foot_contact_ = -foot_contact_;
				stance_start_time_ = current_time_;
				cout << " ################################ Robot Stops Walking! ################################" << endl;
			}
		}
		else
		{
			foot_swing_trigger_ = true;
			stop_walking_trigger_ = false;
			first_step_trigger_ = false;
			start_walking_trigger_ = false;

			foot_contact_ = -foot_contact_;
			cout << " ################################ Support Foot Changed! ################################" << endl;
		}

		start_time_ = current_time_;
	}

	// walking_duration_ = 1;
	walking_phase_ = (current_time_ - start_time_) / walking_duration_;
	walking_phase_ = DyrosMath::minmax_cut(walking_phase_, 0, 1);
	// cout <<"walking_phase_: "<<walking_phase_<<endl;
	// cout <<"foot_contact_: "<<foot_contact_<<endl;
	// cout <<"foot_contact_: "<<foot_contact_<<endl;

	// cout <<"foot_swing_trigger_: "<<foot_swing_trigger_<<endl;
	// cout <<"stop_walking_trigger_: "<<stop_walking_trigger_<<endl;
	// cout <<"first_step_trigger_: "<<first_step_trigger_<<endl;
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
	if ((capture_point_2d(0) > middle_point_of_foot_2d(0) + 0.12) || (capture_point_2d(0) < middle_point_of_foot_2d(0) - 0.12))
	{
		trigger = true;
		cout << "Catpure point in X axis is over the safety boundary! balance swing foot control activated" << endl;
	}

	if ((capture_point_2d(1) > lfoot_transform_current_from_global_.translation()(1) + 0.02) || (capture_point_2d(1) < rfoot_transform_current_from_global_.translation()(1) - 0.02))
	{
		trigger = true;
		cout << "Catpure point in Y axis is over the safety boundary! balance swing foot control activated" << endl;
	}

	// if( com_vel_2d.norm() > stop_vel_threshold_)
	// {
	//     trigger = true;
	//     cout<<"com vel is over the limit ("<< com_vel_2d.norm()<<")"<<endl;
	// }

	if (com_vel_2d(0) > 0.2 || abs(com_vel_2d(1)) > 0.4)
	{
		trigger = true;
		cout << "com vel is over the limit (" << com_vel_2d(0) << "," << com_vel_2d(1) << ")" << endl;
	}

	if (abs(lfoot_transform_current_from_global_.translation()(0) - rfoot_transform_current_from_global_.translation()(0)) > 0.03 || abs(lfoot_transform_current_from_global_.translation()(1) - rfoot_transform_current_from_global_.translation()(1)) > 0.22)
	{
		trigger = true;
		cout << "Foot is not aligned" << endl;
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

		walking_mode_on_ = false;
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
			swing_foot_vel_init_ = lfoot_vel_current_from_global;
		}
		else if (foot_contact_ == -1) //right support foot
		{
			swing_foot_transform_init_ = lfoot_transform_current_from_global_;
			support_foot_transform_init_ = rfoot_transform_current_from_global_;
			swing_foot_vel_init_ = rfoot_vel_current_from_global;
		}

		init_q_ = current_q_;
	}

	if (foot_contact_ == 1) // left support foot
	{

		swing_foot_transform_current_ = rfoot_transform_current_from_global_;
		support_foot_transform_current_ = lfoot_transform_current_from_global_;
		swing_foot_vel_current_ = rfoot_vel_current_from_global;
	}
	else if (foot_contact_ == -1) //right support foot
	{
		swing_foot_transform_current_ = lfoot_transform_current_from_global_;
		support_foot_transform_current_ = rfoot_transform_current_from_global_;
		swing_foot_vel_current_ = lfoot_vel_current_from_global;
	}
	else if (foot_swing_trigger_ == false)
	{
	}

	zmp_measured_local_ = wbc.GetZMPpos_fromFT(rd_, true);

	middle_of_both_foot_ = (lfoot_transform_current_from_global_.translation() + rfoot_transform_current_from_global_.translation()) / 2;
}

void CustomController::motionGenerator()
{

	motion_q_dot_.setZero();
	motion_q_.setZero();
	pd_control_mask_.setZero();

	///////////////////////LEG/////////////////////////
	//////LEFT LEG///////0 0 0.02 0.15 -0.17 0
	motion_q_(0) = 0;
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
	motion_q_(6) = 0;
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
	///////////////////////WAIST/////////////////////////
	motion_q_(12) = 0; //yaw
	motion_q_(13) = 0; //pitch
	motion_q_(14) = 0; //roll
	pd_control_mask_(12) = 1;
	pd_control_mask_(13) = 1;
	pd_control_mask_(14) = 1;
	/////////////////////////////////////////////////////

	///////////////////////HEAD/////////////////////////
	motion_q_(23) = 0; //yaw
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
	motion_q_(29) = 1.0;
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

	/////////////////FOOT HEIGHT/////////////////////////
	double default_stance_foot_z_from_pelv = -0.349 * (cos(0.02) + cos(0.12)) - 0.1225;

	if (foot_swing_trigger_ == true)
	{
		if (walking_phase_ < 0.5)
		{
			// swing_foot_pos_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.00, 0.6, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0)(0);
			// swing_foot_vel_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.00, 0.6, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2)+swing_foot_height_, 0, 0)(1);
			swing_foot_pos_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 1 * switching_phase_duration_, 0.5, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0)(0);
			swing_foot_vel_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 1 * switching_phase_duration_, 0.5, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0)(1);
			swing_foot_acc_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 1 * switching_phase_duration_, 0.5, swing_foot_transform_init_.translation()(2), 0, 0, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0)(2);
		}
		else
		{
			swing_foot_pos_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.5, 1, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2) + 0.000, 0, 0)(0);
			swing_foot_vel_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.5, 1, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2) + 0.000, 0, 0)(1);
			swing_foot_acc_trajectory_from_global_(2) = DyrosMath::QuinticSpline(walking_phase_, 0.5, 1, support_foot_transform_current_.translation()(2) + swing_foot_height_, 0, 0, support_foot_transform_current_.translation()(2) + 0.000, 0, 0)(2);
		}

		double current_swing_foot_height_from_ground = swing_foot_transform_current_.translation()(2) - support_foot_transform_current_.translation()(2);
		double swing_foot_z_pos_error = swing_foot_pos_trajectory_from_global_(2) - swing_foot_transform_current_.translation()(2);
		double swing_foot_z_vel_error = swing_foot_vel_trajectory_from_global_(2) - swing_foot_vel_current_(2);

		double kp = 0.0;
		double kv = 0.00;

		double swithching = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);
		swing_foot_pos_trajectory_from_global_(2) += swithching * (kp * swing_foot_z_pos_error + kv * swing_foot_z_vel_error);
	}
}

void CustomController::getCOMTrajectory()
{
	double desired_step_position_in_y;
	double desired_step_velocity_in_y;
	double d_temp_y;

	com_pos_desired_(2) = com_pos_current_(2);
	com_vel_desired_(2) = com_vel_current_(2);
	com_acc_desired_.setZero();

	if ((walking_speed_ != 0)) // when the robot want to move
	{

		// com_vel_desired_(0) = walking_speed_;
		// com_pos_desired_(0) = com_pos_current_(0) + com_vel_desired_(0)*dt_;

		if (start_walking_trigger_ == true)
		{
			// com_vel_desired_(0) = walking_speed_;
			// com_pos_desired_(0) = com_pos_current_(0) + com_vel_desired_(0)*dt_;

			desired_step_position_in_y = -(step_width_)*foot_contact_;

			// com_vel_desired_(1) = 0;
			// com_pos_desired_(1) = com_pos_current_(1);
			// com_vel_desired_(1) = ( desired_step_position_in_y-com_pos_init_(1)+support_foot_transform_init_.translation()(1) )/(walking_duration_);
			// com_pos_desired_(1) = support_foot_transform_current_.translation()(1) + (1-walking_phase_)*(com_pos_init_(1) - support_foot_transform_init_.translation()(1)) + walking_phase_*desired_step_position_in_y;
			com_pos_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_ + 0 * walking_duration_, stance_start_time_ + 3 * walking_duration_, (com_pos_init_)(0), 0, 0, middle_of_both_foot_(0) + walking_speed_ * walking_duration_, 0, 0)(0);
			com_vel_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_ + 0 * walking_duration_, stance_start_time_ + 3 * walking_duration_, (com_pos_init_)(0), 0, 0, middle_of_both_foot_(0) + walking_speed_ * walking_duration_, 0, 0)(1);
			com_pos_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 4 * walking_duration_, (com_pos_init_)(1), 0, 0, support_foot_transform_current_.translation()(1) - 0.03 * foot_contact_, 0, 0)(0); //-0.03*foot_contact_
			com_vel_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_ + 4 * walking_duration_, (com_pos_init_)(1), 0, 0, support_foot_transform_current_.translation()(1) - 0.03 * foot_contact_, 0, 0)(1);
			// com_acc_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+2*walking_duration_, (com_pos_init_)(1), 0, 0, support_foot_transform_current_.translation()(1)  , 0, 0)(2);
		}
		else
		{
			///////////////X DIRECTIOIN///////////
			com_pos_desired_(0) = com_pos_current_(0);
			com_vel_desired_(0) = walking_speed_;
			// com_pos_desired_(0) = com_pos_current_(0) + com_vel_desired_(0)*dt_;

			///////////////Y DIRECTIOIN///////////
			desired_step_position_in_y = -(step_width_)*foot_contact_;
			double target_com_y_speed = (support_foot_transform_init_.translation()(1) + desired_step_position_in_y - com_pos_init_(1)) / (walking_duration_);

			com_pos_desired_(1) = support_foot_transform_current_.translation()(1) + (1 - walking_phase_) * (com_pos_init_(1) - support_foot_transform_init_.translation()(1)) + walking_phase_ * (desired_step_position_in_y);
			com_vel_desired_(1) = target_com_y_speed;

			// cout<<"l: "<<l<<endl;
			// cout<<"desired_step_velocity_in_y: "<<desired_step_velocity_in_y<<endl;
			// cout<<"desired_step_position_in_y: "<<desired_step_position_in_y<<endl;
		}
	}
	else
	{
		if ((foot_swing_trigger_ == true))
		{
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
			com_pos_desired_(1) = support_foot_transform_current_.translation()(1) + (1 - walking_phase_) * (com_pos_init_(1) - support_foot_transform_init_.translation()(1)) + walking_phase_ * (desired_step_position_in_y);
			com_vel_desired_(1) = target_com_y_speed;
		}
		else if (program_start_time_ == stance_start_time_)
		{
			com_pos_desired_(0) = middle_of_both_foot_(0);
			com_vel_desired_(0) = 0;

			com_pos_desired_(1) = middle_of_both_foot_(1);
			com_vel_desired_(1) = 0;
		}
		else if (stop_walking_trigger_ == true)
		{
			double traj_duraiton = walking_duration_;

			// com_pos_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(0), com_vel_init_(0), 0, (middle_of_both_foot_)(0), 0, 0)(0);
			// com_vel_desired_(0) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(0), com_vel_init_(0), 0, (middle_of_both_foot_)(0), 0, 0)(1);
			// com_pos_desired_(0) = com_pos_init_(0);
			// com_pos_desired_(1) = DyrosMath::QuinticSpline(current_time_, start_time_, start_time_+5, (com_pos_init_)(1), 0, 0, lfoot_transform_current_from_global_.translation()(1), 0, 0)(0);
			// com_vel_desired_(1) = DyrosMath::QuinticSpline(current_time_, start_time_, start_time_+5, (com_pos_init_)(1), 0, 0, lfoot_transform_current_from_global_.translation()(1), 0, 0)(1);
			// com_pos_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(1), com_vel_init_(1), 0, (middle_of_both_foot_)(1), 0, 0)(0);
			// com_vel_desired_(1) = DyrosMath::QuinticSpline(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, (com_pos_init_)(1), com_vel_init_(1), 0, (middle_of_both_foot_)(1), 0, 0)(1);
			
			com_pos_desired_(0) = middle_of_both_foot_(0);
			com_vel_desired_(0) = 0;

			// com_pos_desired_(1) = middle_of_both_foot_(1);
			// com_vel_desired_(1) = 0;
			
			// com_pos_desired_(1) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, com_pos_init_(1), (support_foot_transform_current_)(1), com_vel_init_(1), 0);
			// com_vel_desired_(1) = DyrosMath::cubicDot(current_time_, stance_start_time_, stance_start_time_+traj_duraiton, com_pos_init_(1), (support_foot_transform_current_)(1), com_vel_init_(1), 0, 0);
			
			if(current_time_ < stance_start_time_ + traj_duraiton)
			{
				desired_step_position_in_y = -(step_width_*3)*foot_contact_; // positive sign because foot_contact_ does not changed at the walking stop.
				double target_com_y_speed = (support_foot_transform_init_.translation()(1) + desired_step_position_in_y - com_pos_init_(1)) / (walking_duration_);
				com_pos_desired_(1) = support_foot_transform_current_.translation()(1) + (stance_start_time_ + traj_duraiton - current_time_)/traj_duraiton * (com_pos_init_(1) - support_foot_transform_init_.translation()(1)) + (current_time_- stance_start_time_)/traj_duraiton* (desired_step_position_in_y);
				com_vel_desired_(1) = target_com_y_speed;
			}
			else
			{
				com_pos_desired_(0) = middle_of_both_foot_(0);
				com_vel_desired_(0) = 0;
				com_pos_desired_(1) = middle_of_both_foot_(1);
				com_vel_desired_(1) = 0;
				// com_pos_desired_(1) = DyrosMath::cubic(current_time_, stance_start_time_+traj_duraiton, stance_start_time_+2*traj_duraiton, com_pos_init_(1), (support_foot_transform_current_)(1), com_vel_init_(1), 0);
				// com_vel_desired_(1) = DyrosMath::cubicDot(current_time_, stance_start_time_+traj_duraiton, stance_start_time_+2*traj_duraiton, com_pos_init_(1), (support_foot_transform_current_)(1), com_vel_init_(1), 0, 0);
			}

			if (int(current_time_ * 2000) % 400 == 0)
			{
				cout << " WALKING STOPPED " << endl;
				cout << " com_pos_desired_: " << com_pos_desired_<<endl;
				cout << " com_vel_desired_: " << com_vel_desired_<<endl;
				cout << " support_foot_transform_init_.translation()(1): " << support_foot_transform_init_.translation()(1)<<endl;
				cout << " desired_step_position_in_y: " << desired_step_position_in_y <<endl;
			}
				
		}
	}

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
	double alpha_x = 0.05;
	double alpha_y = 0.0;

	if (foot_swing_trigger_ == true)
	{
		// x axis
		d_temp_x = (com_pos_current(2) - support_foot_transform_current_.translation()(2)) / GRAVITY + (com_vel_current(0) / (2 * GRAVITY)) * (com_vel_current(0) / (2 * GRAVITY));

		// if (d_temp<0) d_temp = 0;
		d_temp_x = sqrt(d_temp_x);

		d(0) = com_vel_current(0) * d_temp_x;

		// y axis
		d_temp_y = (com_pos_current(2) - support_foot_transform_current_.translation()(2)) / GRAVITY + (com_vel_current(1) / (2 * GRAVITY)) * (com_vel_current(1) / (2 * GRAVITY));

		// if (d_temp<0) d_temp = 0;
		d_temp_y = sqrt(d_temp_y);

		d(1) = com_vel_current(1) * d_temp_y;

		if (com_vel_current(0) * walking_speed_ < 0)
		{
			alpha_x = 0.15;
		}

		d_prime(0) = d(0) - alpha_x * com_vel_desired(0);
		// d_prime(1) = d(1) - 0.06*step_width_/walking_duration_*foot_contact_;
		d_prime(1) = d(1) - alpha_y * com_vel_desired(1) * foot_contact_;


		if (walking_phase_ < 0.95) // during the last 0.05phase, the swing foot only lands on the ground in z direction.
		{
			target_foot_landing_from_pelv_ = com_pos_current.segment<2>(0) + d_prime; 
		}

		double dsp_coeff = 1;

		if (walking_phase_ < dsp_coeff * switching_phase_duration_)
		{
			swing_foot_pos_trajectory_from_global_ = swing_foot_transform_init_.translation();
			swing_foot_vel_trajectory_from_global_.setZero();
			swing_foot_rot_trajectory_from_global_ = swing_foot_transform_init_.linear();
		}
		else if (walking_phase_ < 0.95)
		{
			double ssp = (phase - dsp_coeff * switching_phase_duration_) / (0.95 - dsp_coeff * switching_phase_duration_);

			swing_foot_pos_trajectory_from_global_.segment(0, 2) = support_foot_transform_current_.translation().segment<2>(0) + 
			(1 - ssp) * (swing_foot_transform_init_.translation().segment<2>(0) - support_foot_transform_init_.translation().segment<2>(0)) +
			(ssp) * (target_foot_landing_from_pelv_ - support_foot_transform_current_.translation().segment<2>(0));

			swing_foot_vel_trajectory_from_global_.segment(0, 2) = (target_foot_landing_from_pelv_ - swing_foot_transform_init_.translation().segment<2>(0)) / (walking_duration_*0.95 - dsp_coeff * switching_phase_duration_);
			swing_foot_rot_trajectory_from_global_.setIdentity(); 
		}
		else
		{
			swing_foot_pos_trajectory_from_global_.segment(0, 2) = target_foot_landing_from_pelv_ - support_foot_transform_current_.translation().segment<2>(0);
			swing_foot_vel_trajectory_from_global_.segment(0, 2).setZero();
			swing_foot_rot_trajectory_from_global_.setIdentity(); 
		}
		


		if (foot_contact_ == 1) //left support
		{
			swing_foot_pos_trajectory_from_global_(0) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(0), lfoot_transform_current_from_global_.translation()(0) - 0.5, lfoot_transform_current_from_global_.translation()(0) + 1.0);
			swing_foot_pos_trajectory_from_global_(1) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(1), lfoot_transform_current_from_global_.translation()(1) - 0.8, lfoot_transform_current_from_global_.translation()(1) - 0.2);
		}
		else if (foot_contact_ == -1) // right support
		{
			swing_foot_pos_trajectory_from_global_(0) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(0), rfoot_transform_current_from_global_.translation()(0) - 0.5, rfoot_transform_current_from_global_.translation()(0) + 1.0);
			swing_foot_pos_trajectory_from_global_(1) = DyrosMath::minmax_cut(swing_foot_pos_trajectory_from_global_(1), rfoot_transform_current_from_global_.translation()(1) + 0.2, rfoot_transform_current_from_global_.translation()(1) + 0.8);
		}

	}
	else
	{
		swing_foot_pos_trajectory_from_global_ = swing_foot_transform_init_.translation();
		swing_foot_vel_trajectory_from_global_.setZero();
		swing_foot_rot_trajectory_from_global_ = swing_foot_transform_init_.linear();
	}


	// if( int(current_time_*10)%5 == 0 )
	if (true)
	{
		// cout<<"current_time_: "<<current_time_<<endl;
		// cout<<"stance_start_time_: "<<stance_start_time_<<endl;
		// cout<<"start_time_: "<<start_time_<<endl;
		// cout<<"tc.command_time: "<<tc.command_time<<endl;
		// cout<<"foot_contact_: "<<foot_contact_<<endl;
		// cout<<"walking_phase_: "<<walking_phase_<<endl;
		// cout<<"start_walking_trigger_: "<<start_walking_trigger_<<endl;
		// cout<<"first_step_trigger_: "<<first_step_trigger_<<endl;
		// cout<<"foot_swing_trigger_: "<<foot_swing_trigger_<<endl;
		// cout<<"stop_walking_trigger_: "<<stop_walking_trigger_<<endl;
		// cout<<"support_foot_transform_current_.translation()(2):"<< support_foot_transform_current_.translation()(2)<<endl;
		// cout<<"d_temp_y:"<< d_temp_y<<endl;
		// cout<<"d_prime:"<< d_prime<<endl;
		// cout<<"com_vel_desired_:"<< com_vel_desired<<endl;
		// cout<<"com_pos_desired_:"<< com_pos_desired_<<endl;
		// cout<<"com_vel_current:"<< com_vel_current<<endl;
		// cout<<"com_pos_current_:"<< com_pos_current<<endl;
		// cout<<"com_pos_init_:"<< com_pos_init_<<endl;
		// cout<<"target_foot_landing_from_pelv_:"<< target_foot_landing_from_pelv_<<endl;
		// cout<<"swing_foot_pos_trajectory_from_global_:"<< swing_foot_pos_trajectory_from_global_<<endl;
		// cout<<"swing_foot_transform_current_.translation():"<< swing_foot_transform_current_.translation()<<endl;
	}
}

Eigen::VectorQd CustomController::comVelocityControlCompute(WholebodyController &wbc)
{

	/////////////////////////////////////////////////////////////////////////////
	//     const int arm_task_number = 6;
	// const int arm_dof = 8;
	// ////////// CoM Control //////////////////////
	// wc_.set_contact(rd_, 1, 1);
	// task_number = 6;
	// J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
	// f_star.setZero(task_number);

	// J_task = rd_.link_[COM_id].Jac;
	// J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac_COM_p;
	// J_task.block(0, 21, 6, arm_dof).setZero(); // Exclude Left Arm Jacobian
	// J_task.block(0, 31, 6, arm_dof).setZero(); // Exclude Right Arm Jacobian

	// rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;
	// rd_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, atc.command_time, atc.command_time + atc.traj_time);

	// // f_star = wc_.getfstar6d(rd_, COM_id);
	// torque_grav.setZero();
	// torque_task = wc_.task_control_torque_QP2(rd_, J_task, f_star);
	//////////////////////////////////////////////////////////////////

	Eigen::VectorQd torque;

	Eigen::VectorXd f_star;
	Eigen::MatrixXd J_task;
	const int task_dof = 6;

	J_task.setZero(task_dof, MODEL_DOF_VIRTUAL);
	f_star.setZero(task_dof);

	wbc.set_contact(rd_, 1, 1);
	if (foot_swing_trigger_ == true)
	{
		if ((foot_contact_ == 1))
		{
			wbc.set_contact(rd_, 1, 0);
		}
		else if ((foot_contact_ == -1))
		{
			wbc.set_contact(rd_, 0, 1);
		}
	}

	////////////// Set J_task  ////////////////
	J_task = rd_.link_[COM_id].Jac;
	// J_task = rd_.link_[COM_id].Jac.block(0, 0, 2, MODEL_DOF_VIRTUAL);
	J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[COM_id].Jac_COM_p;
	// J_task.block(3, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[Head].Jac.block(3, 0, 3, MODEL_DOF_VIRTUAL);
	J_task.block(0, 21, 6, 8).setZero(); // Exclude Left hand Jacobian
	J_task.block(0, 31, 6, 8).setZero(); // Exclude Right hand Jacobian
	// J_task.block(0, 29, 6, 2).setZero(); // Exclude Head Jacobian

	if (foot_swing_trigger_ == true)
	{
		if (foot_contact_ == 1) //left support
		{
			J_task.block(0, 12, 6, 8).setZero(); //Exclude Rifht Leg Jacobian
		}
		else if (foot_contact_ == -1) //right support
		{
			J_task.block(0, 6, 6, 8).setZero(); //Exclude Left Leg Jacobian
		}
	}
	////////////////////////////////////////////

	////////////// Set f_start  ////////////////
	double kp = 2500; // 2500(reference paper)
	double kv = 100;	// 100(reference paper)

	// f_star(0) = kv*(com_vel_desired_(0) - com_vel_current_(0)) + kp*(com_pos_desired_(1)- com_pos_current_(1));                                                          //X axis D control
	// f_star(1) = kv*(com_vel_desired_(1) - com_vel_current_(1)) + kp*(com_pos_desired_(1)- com_pos_current_(1));          //Y axis PD control
	// f_star(2) = kv*(com_vel_desired_(2) - com_vel_current_(2)) + kp*(com_pos_desired_(2)- com_pos_current_(2));
	rd_.link_[COM_id].x_traj = com_pos_desired_;
	rd_.link_[COM_id].v_traj = com_vel_desired_;
	rd_.link_[COM_id].a_traj.setZero();
	rd_.link_[COM_id].r_traj = Eigen::Matrix3d::Identity();
	rd_.link_[COM_id].w_traj.setZero();

	// rd_.link_[Head].r_traj = Eigen::Matrix3d::Identity();
	// rd_.link_[Head].w_traj = Eigen::Vector3d::Zero();
	// f_star.segment(3, 3) = wc.getfstar_rot(rd_, COM_id);
	// f_star.segment(3, 3) = wc.getfstar_rot(rd_, Head);
	f_star = wbc.getfstar6d(rd_, COM_id);
	f_star(2) = 0;
	f_star(5) = 0;
	/////////////////////////////////////////////

	double contact_dist_ratio = 0;
	// if( (foot_swing_trigger_ == false) && (walking_speed_ != 0))
	// {
	//     if(foot_contact_ == 1)
	//     {
	//         contact_dist_ratio = 1;
	//     }
	//     else if(foot_contact_ == -1)
	//     {
	//         contact_dist_ratio = -1;
	//     }
	// }

	torque = wbc.task_control_torque_QP_dg(rd_, J_task, f_star, 1); //jacobian control + gravity torque

	// if(int(control_time_) %2 ==0)
	// {
	//     cout<<"Com Vel torque: \n"<< torque <<endl;
	//     cout<<"f_com: \n"<< f_com <<endl;
	// }
	// torque = tuneTorqueForZMPSafety( torque );        //turn off velocity tuning if the zmp is outside of the foot

	return torque;
}

Eigen::VectorQd CustomController::comVelocityControlCompute2(WholebodyController &wbc)
{
	Eigen::VectorQd torque;
	Eigen::VectorQd torque_g;

	Eigen::VectorXd f_star;
	Eigen::MatrixXd J_task;
	VectorQd torque_r_vel_tun;
	VectorQd torque_l_vel_tun;
	const int task_dof = 3;
	torque.setZero();
	torque_g.setZero();
	J_task.setZero(task_dof, MODEL_DOF_VIRTUAL);
	f_star.setZero(task_dof);

	wbc.set_contact(rd_, 0, 0); // for graviti torque calc
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
	torque_g = wbc.gravity_compensation_torque(rd_, true);

	////////////// Set f_start  ////////////////
	double kp = 14400; // 2500(reference paper)
	double kv = 240;	 // 100(reference paper)

	// com_pos_desired_(1) = com_pos_init_(0) + sin(2*M_PI/8*current_time_-tc.command_time);
	// com_vel_desired_(1) = M_PI/2*cos(2*M_PI/8*current_time_-tc.command_time);
	f_star(0) = kv * (com_vel_desired_(0) - (com_vel_current_)(0)) + kp * (com_pos_desired_(0) - com_pos_current_(0)); // + rd_.com_.mass*com_acc_desired_(0);                                                          //X axis D control
	f_star(1) = kv * (com_vel_desired_(1) - (com_vel_current_)(1)) + kp * (com_pos_desired_(1) - com_pos_current_(1)); // + rd_.com_.mass*com_acc_desired_(1);          //Y axis PD control
	f_star(2) = 0;

	// cout<<"com_vel_desired_: "<<com_vel_desired_<<endl;
	// cout<<"com_vel_current_: "<<com_vel_current_<<endl;
	// cout<<"com_pos_desired_: "<<com_pos_desired_<<endl;

	// cout<<"f_star: "<<f_star<<endl;

	// f_star(2) = kv*(com_vel_desired_(2) - com_vel_current_(2)) + kp*(com_pos_desired_(2)- com_pos_current_(2));
	// rd_.link_[COM_id].x_traj = com_pos_desired_;
	// rd_.link_[COM_id].v_traj = com_vel_desired_;
	// rd_.link_[COM_id].a_traj.setZero();
	// rd_.link_[COM_id].r_traj = Eigen::Matrix3d::Identity();
	// rd_.link_[COM_id].w_traj.setZero();

	// rd_.link_[Head].r_traj = Eigen::Matrix3d::Identity();
	// rd_.link_[Head].w_traj = Eigen::Vector3d::Zero();
	// f_star.segment(2, 3) = wc.getfstar_rot(rd_, COM_id);
	// f_star.segment(3, 3) = wc.getfstar_rot(rd_, Head);
	// f_star = wc.getfstar6d(rd_, COM_id);

	// f_star_xy_ = 0.3*f_star + 0.7*f_star_xy_pre_;
	/////////////////////////////////////////////

	/////////////////////JACOBIAN///////////////////////////////////////

	MatrixXd lfoot_to_com_jac_from_global;
	MatrixXd rfoot_to_com_jac_from_global;
	lfoot_to_com_jac_from_global.setZero(3, MODEL_DOF_VIRTUAL);
	rfoot_to_com_jac_from_global.setZero(3, MODEL_DOF_VIRTUAL);
	Matrix6d adjoint_pelv_to_ankle;
	adjoint_pelv_to_ankle.block(0, 0, 3, 3) = -Eigen::Matrix3d::Identity();
	adjoint_pelv_to_ankle.block(0, 3, 3, 3) = DyrosMath::skm(com_pos_current_ - lfoot_transform_current_from_global_.translation());
	adjoint_pelv_to_ankle.block(3, 3, 3, 3) = -Eigen::Matrix3d::Identity();

	lfoot_to_com_jac_from_global = (adjoint_pelv_to_ankle * rd_.link_[Left_Foot].Jac).block(0, 0, 3, MODEL_DOF_VIRTUAL) + rd_.link_[COM_id].Jac_COM_p;
	lfoot_to_com_jac_from_global.block(0, 12, 3, 6).setZero();
	lfoot_to_com_jac_from_global.block(0, 21, 3, 8).setZero();
	lfoot_to_com_jac_from_global.block(0, 31, 3, 8).setZero();

	adjoint_pelv_to_ankle.block(0, 0, 3, 3) = -Eigen::Matrix3d::Identity();
	adjoint_pelv_to_ankle.block(0, 3, 3, 3) = DyrosMath::skm(com_pos_current_ - rfoot_transform_current_from_global_.translation());
	adjoint_pelv_to_ankle.block(3, 3, 3, 3) = -Eigen::Matrix3d::Identity();

	rfoot_to_com_jac_from_global = (adjoint_pelv_to_ankle * rd_.link_[Right_Foot].Jac).block(0, 0, 3, MODEL_DOF_VIRTUAL) + rd_.link_[COM_id].Jac_COM_p;
	rfoot_to_com_jac_from_global.block(0, 6, 3, 6).setZero();
	rfoot_to_com_jac_from_global.block(0, 21, 3, 8).setZero();
	rfoot_to_com_jac_from_global.block(0, 31, 3, 8).setZero();

	torque_l_vel_tun = (lfoot_to_com_jac_from_global.transpose() * f_star).segment(6, MODEL_DOF);
	torque_r_vel_tun = (rfoot_to_com_jac_from_global.transpose() * f_star).segment(6, MODEL_DOF);

	// cout<<"adjoint_pelv_to_ankle: "<< adjoint_pelv_to_ankle<<endl;
	// cout<<"rd_.link_[Right_Foot].Jac: "<< rd_.link_[Right_Foot].Jac<<endl;
	// cout<<"rd_.link_[COM_id].Jac_COM_p: "<< rd_.link_[COM_id].Jac_COM_p<<endl;

	// cout<<"lfoot_to_com_jac_from_global: "<< lfoot_to_com_jac_from_global<<endl;
	// cout<<"rfoot_to_com_jac_from_global: "<< rfoot_to_com_jac_from_global<<endl;
	// J_task = jac_com_pos_;
	/////////////////////////////////////////////////////////////////////////////

	// torque = wc.task_control_torque_QP_dg(rd_, J_task, f_star, 0);
	// torque = (J_task.transpose()*f_star).segment(6, MODEL_DOF);
	// if(foot_swing_trigger_ == true)
	// {
	//     wc.set_contact(rd_, 1, 0);

	//     torque_l_vel_tun = wc.task_control_torque(rd_, J_task, f_star);
	//     torque_l_vel_tun.segment(6, 6).setZero();
	//     torque_l_vel_tun.segment(15, 8).setZero();
	//     torque_l_vel_tun.segment(25, 8).setZero();

	//     wc.set_contact(rd_, 0, 1);

	//     torque_r_vel_tun = wc.task_control_torque(rd_, J_task, f_star);
	//     torque_r_vel_tun.segment(0, 6).setZero();
	//     torque_r_vel_tun.segment(15, 8).setZero();
	//     torque_r_vel_tun.segment(25, 8).setZero();
	// }
	// else
	// {
	//     wc.set_contact(rd_, 1, 1);

	//     torque_l_vel_tun = wc.task_control_torque(rd_, J_task, f_star);
	//     torque_l_vel_tun.segment(6, 6).setZero();
	//     torque_l_vel_tun.segment(15, 8).setZero();
	//     torque_l_vel_tun.segment(25, 8).setZero();

	//     torque_r_vel_tun = wc.task_control_torque(rd_, J_task, f_star);
	//     torque_r_vel_tun.segment(0, 6).setZero();
	//     torque_r_vel_tun.segment(15, 8).setZero();
	//     torque_r_vel_tun.segment(25, 8).setZero();
	// }

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
		if (stop_walking_trigger_ == true)
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
		else
		{
			lfoot_task_torque_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.1 * walking_duration_, 0, 1, 0, 0);
			rfoot_task_torque_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.1 * walking_duration_, 0, 1, 0, 0);

			lfoot_torque_g_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.1 * walking_duration_, 1, 0, 0, 0);
			rfoot_torque_g_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.1 * walking_duration_, 1, 0, 0, 0);
		}
	}

	torque_g.segment(0, 6) = torque_g.segment(0, 6) * lfoot_torque_g_switch;
	torque_g.segment(6, 6) = torque_g.segment(6, 6) * rfoot_torque_g_switch;

	// if( int(walking_duration_*100)%10 == 0 )
	// cout<<"walking_phase_: \n"<<walking_phase_<<endl;
	torque += torque_l_vel_tun * lfoot_task_torque_switch + torque_r_vel_tun * rfoot_task_torque_switch;
	// torque(0) =0;
	// torque(6) =0;
	/////////////////////////////////////////////////////////////////

	// torque = wc.task_control_torque_QP2(rd_, jac_com_xy, f_com);  //jacobian control + gravity torque

	// if(int(control_time_) %2 ==0)
	// {
	//     cout<<"Com Vel torque: \n"<< torque <<endl;
	//     cout<<"f_com: \n"<< f_com <<endl;
	// }

	torque += torque_g;

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
		// cout<<"lfoot_to_com_jac_from_global: \n"<<lfoot_to_com_jac_from_global<<endl;
		// cout<<"test_com_jac: \n"<<test_com_jac<<endl;
	}

	// if( (walking_phase_ > 0.2) && (walking_phase_ < 0.7))
	torque = tuneTorqueForZMPSafety(torque); //turn off velocity tuning if the zmp is outside of the foot

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

	double default_stance_foot_z_from_pelv = -0.349 * (cos(0.02) + cos(0.12)) - 0.1225;
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
	Vector3d phi_pelv;
	Vector3d ang_vel_pelv;
	Vector3d torque_pelv;
	VectorQd torque_stance_hip;
	VectorQd torque_swing_assist;

	torque_pelv.setZero();
	torque_stance_hip.setZero();
	torque_swing_assist.setZero();

	phi_pelv = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());
	ang_vel_pelv = pelv_yaw_rot_current_from_global_.transpose() * rd_.link_[Pelvis].w;
	double kpa_pelv = 6400; //angle error gain
	double kva_pelv = 160;	//angular velocity gain
	torque_pelv = kpa_pelv * phi_pelv - kva_pelv * ang_vel_pelv;
	Vector3d axis;
	double angle;
	// angle = (phi_pelv.norm())*dt_ ;
	// axis = phi_pelv.normalized();

	Eigen::AngleAxisd aa(pelv_rot_current_yaw_aline_.transpose());
	angle = aa.angle();
	axis = aa.axis();

	if(foot_swing_trigger_ == false)
	{
		torque_pelv(0) = 2500 * phi_pelv(0) - 100 * ang_vel_pelv(0);
	}

	// if( walking_phase_>0.1 )
	// {
	// cout <<"phi_pelv: \n"<<phi_pelv<<endl;
	// cout <<"torque_pelv: \n"<<torque_pelv<<endl;

	//     // cout <<"axis: \n"<<axis<<endl;
	// }

	// Matrix3d pelv_rot;
	// pelv_rot = pelv_yaw_rot_current_from_global_.transpose()*pelv_rot_current_*Eigen::AngleAxisd(angle*100*dt_, axis);
	// pelv_transform_from_global.linear() = pelv_rot;

	// desired_q_.segment(12, 3) = current_q_.segment(12, 3) + desired_q_dot_.segment(12, 3)*dt_;

	//////////////////////////////////////////////////////////////////////////////////////////

	Eigen::MatrixXd kp_joint(MODEL_DOF, 1);
	Eigen::MatrixXd kv_joint(MODEL_DOF, 1);

	Eigen::MatrixXd kp_stiff_joint(MODEL_DOF, 1);
	Eigen::MatrixXd kv_stiff_joint(MODEL_DOF, 1);
	Eigen::MatrixXd kp_soft_joint(MODEL_DOF, 1);
	Eigen::MatrixXd kv_soft_joint(MODEL_DOF, 1);
	double swing_pd_switch;
	for (int i = 0; i < MODEL_DOF; i++)
	{
		kp_joint(i) = 64;
		kv_joint(i) = 16;
	}

	// for(int i = 0; i<12; i++)      //Leg
	// {
	//     kp_joint(i) = 600;
	//     kv_joint(i) = 50;
	// }
	// kp_joint(1) = 8100;
	// kv_joint(1) = 180;
	// kp_joint(2) = 6400;
	// kv_joint(2) = 160;

	// kp_joint(7) = 8100;
	// kv_joint(7) = 180;
	// kp_joint(8) = 6400;
	// kv_joint(8) = 160;

	for (int i = 0; i < 3; i++) //Waist Joint Gains
	{
		kp_joint(12 + i) = 169;
		kv_joint(12 + i) = 26;
	}
	// kp_joint(12) = 50;
	// kv_joint(12) = 14;
	// kp_joint(13) = 50;
	// kv_joint(13) = 14;
	// kp_joint(14) = 50;
	// kv_joint(14) = 14;

	// for (int i = 0; i < 2; i++) //Head Joint Gains
	// {
	// 	kp_joint(23 + i) = 64;
	// 	kv_joint(23 + i) = 16;
	// }

	//stiff
	kp_stiff_joint(0) = 2500; //R hip yaw joint gain
	kv_stiff_joint(0) = 100;
	kp_stiff_joint(1) = 2500; //L hip roll joint gain
	kv_stiff_joint(1) = 100;
	kp_stiff_joint(2) = 2500; //L hip pitch joint gain
	kv_stiff_joint(2) = 100;

	kp_stiff_joint(3) = 2500; //L knee joint gain
	kv_stiff_joint(3) = 100;

	kp_stiff_joint(4) = 100; //L ankle pitch joint gain
	kv_stiff_joint(4) = 20;
	kp_stiff_joint(5) = 100; //L ankle roll joint gain
	kv_stiff_joint(5) = 20;

	kp_stiff_joint(6) = 2500; //R hip yaw joint gain
	kv_stiff_joint(6) = 100;
	kp_stiff_joint(7) = 2500; //R hip roll joint gain
	kv_stiff_joint(7) = 100;
	kp_stiff_joint(8) = 2500; //R hip pitch joint gain
	kv_stiff_joint(8) = 100;

	kp_stiff_joint(9) = 2500; //R knee joint gain
	kv_stiff_joint(9) = 100;

	kp_stiff_joint(10) = 100; //R ankle pitch joint gain
	kv_stiff_joint(10) = 20;
	kp_stiff_joint(11) = 100; //R ankle roll joint gain
	kv_stiff_joint(11) = 20;

	//soft
	kp_soft_joint(0) = 2500; //L hip yaw joint gain
	kv_soft_joint(0) = 100;
	kp_soft_joint(1) = 400; //L hip roll joint gain
	kv_soft_joint(1) = 40;
	kp_soft_joint(2) = 400; //L hip pitch joint gain
	kv_soft_joint(2) = 40;

	kp_soft_joint(3) = 100; //L knee joint gain
	kv_soft_joint(3) = 20;

	kp_soft_joint(4) = 64; //L ankle pitch joint gain
	kv_soft_joint(4) = 16;
	kp_soft_joint(5) = 64; //L ankle roll joint gain
	kv_soft_joint(5) = 16;

	kp_soft_joint(6) = 2500; //R hip yaw joint gain
	kv_soft_joint(6) = 100;
	kp_soft_joint(7) = 400; //R hip roll joint gain
	kv_soft_joint(7) = 40;
	kp_soft_joint(8) = 400; //R hip pitch joint gain
	kv_soft_joint(8) = 40;

	kp_soft_joint(9) = 100; //R knee joint gain
	kv_soft_joint(9) = 20;

	kp_soft_joint(10) = 64; //R ankle pitch joint gain
	kv_soft_joint(10) = 16;
	kp_soft_joint(11) = 64; //R ankle roll joint gain
	kv_soft_joint(11) = 16;

	for (int i = 0; i < 12; i++) //Leg
	{
		kp_joint(i) = kp_stiff_joint(i);
		kv_joint(i) = kv_stiff_joint(i);
	}
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

			desired_q_(0) = 0.5 * motion_q_(0) + 0.5 * pre_desired_q_(0);
			Vector3d phi_swing_ankle;
			phi_swing_ankle = -DyrosMath::getPhi(rd_.link_[Left_Foot].Rotm, pelv_yaw_rot_current_from_global_);
			// phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

			desired_q_dot_(4) = 200 * phi_swing_ankle(1); //swing ankle pitch
			desired_q_dot_(5) = 200 * phi_swing_ankle(0); //swing ankle roll
			desired_q_(4) = current_q_(4) + desired_q_dot_(4) * dt_;
			desired_q_(5) = current_q_(5) + desired_q_dot_(5) * dt_;

			Vector3d phi_support_ankle;
			phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Right_Foot].Rotm, pelv_yaw_rot_current_from_global_);

			desired_q_dot_(10) = 200 * phi_support_ankle(1);
			desired_q_dot_(11) = 200 * phi_support_ankle(0);
			desired_q_(10) = current_q_(10) + desired_q_dot_(10) * dt_;
			desired_q_(11) = current_q_(11) + desired_q_dot_(11) * dt_;

			// low pass filter for suppport foot target position
			desired_q_(6) = 0.5 * motion_q_(6) + 0.5 * pre_desired_q_(6); //right support foot hip yaw
			desired_q_(9) = DyrosMath::QuinticSpline(walking_phase_, 0.0, switching_phase_duration_, init_q_(9), 0, 0, motion_q_(9), 0, 0)(0);
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
				if (kp_joint(i + 6) == kp_soft_joint(i + 6))
				{
					kp_joint(i + 6) = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, kp_soft_joint(i + 6), kp_stiff_joint(i + 6), 0, 0); //support foot
				}

				if (kp_joint(i) == kp_stiff_joint(i))
				{
					kp_joint(i) = DyrosMath::cubic(walking_phase_, 0.9, 1, kp_stiff_joint(i), kp_soft_joint(i), 0, 0); //swing foot
				}
			}
			// kp_joint(3) = DyrosMath::cubic(walking_phase_, 0.8, 1, 2000, 600, 0, 0); //swing foot knee
			// kv_joint(3) = DyrosMath::cubic(walking_phase_, 0.8, 1, 60, 50, 0, 0);

			// kp_joint(4) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0); //swing foot ankle gain
			// kv_joint(4) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);
			// kp_joint(5) = DyrosMath::cubic(walking_phase_, 0.8, 1, 600, 50, 0, 0);
			// kv_joint(5) = DyrosMath::cubic(walking_phase_, 0.8, 1, 50, 10, 0, 0);

			/////////////////Swing Assist Torque/////////////////////////////////////////
			// left foot swing height assist feed forward torque on hip roll, hip pitch, knee pitch
			Eigen::VectorXd f_star;
			f_star.setZero(3);
			f_star(2) = swing_foot_acc_trajectory_from_global_(2);

			// torque_swing_assist.segment(1, 3) = jac_lfoot_.block(2, 7, 1, 3).transpose()*swing_foot_acc_trajectory_from_global_(2);
			// torque_swing_assist.segment(1, 3) = (jac_lfoot_.transpose()).block(7, 0, 3, 6)*rd_.lambda.block(0, 2, 6, 1)*swing_foot_acc_trajectory_from_global_(2);

			torque_swing_assist.segment(1, 3) = wbc.task_control_torque(rd_, jac_lfoot_.block(0, 0, 3, MODEL_DOF_VIRTUAL), f_star).segment(1, 3);

			/////////////////////////////////////////////////////////////////////////////

			swing_pd_switch = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);

			if (first_step_trigger_ == true)
			{
				pd_control_mask_(0) = 1;
				pd_control_mask_(1) = swing_pd_switch;
				pd_control_mask_(2) = swing_pd_switch;

				pd_control_mask_(6) = 1;
				pd_control_mask_(7) = 0;
				pd_control_mask_(8) = 0;

				torque_stance_hip(2) = -torque_pelv(1) * (1 - swing_pd_switch); // left hip pitch
				torque_stance_hip(1) = -torque_pelv(0) * (1 - swing_pd_switch); // left hip roll

				torque_stance_hip(8) = -torque_pelv(1); // right hip pitch
				torque_stance_hip(7) = -torque_pelv(0); // right hip roll
			}
			else
			{
				pd_control_mask_(0) = 1;
				pd_control_mask_(1) = swing_pd_switch;
				pd_control_mask_(2) = swing_pd_switch;

				pd_control_mask_(6) = 1;
				pd_control_mask_(7) = 1 - swing_pd_switch;
				pd_control_mask_(8) = 1 - swing_pd_switch;

				torque_stance_hip(2) = -torque_pelv(1) * (1 - swing_pd_switch); // left hip pitch
				torque_stance_hip(1) = -torque_pelv(0) * (1 - swing_pd_switch); // left hip roll

				torque_stance_hip(8) = -torque_pelv(1) * swing_pd_switch; // right hip pitch
				torque_stance_hip(7) = -torque_pelv(0) * swing_pd_switch; // right hip roll
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

			desired_q_(6) = 0.5 * motion_q_(6) + 0.5 * pre_desired_q_(6);
			Vector3d phi_swing_ankle;
			phi_swing_ankle = -DyrosMath::getPhi(rd_.link_[Right_Foot].Rotm, pelv_yaw_rot_current_from_global_);
			// phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

			desired_q_dot_(10) = 200 * phi_swing_ankle(1); //swing ankle pitch
			desired_q_dot_(11) = 200 * phi_swing_ankle(0); //swing ankle roll
			desired_q_(10) = current_q_(10) + desired_q_dot_(10) * dt_;
			desired_q_(11) = current_q_(11) + desired_q_dot_(11) * dt_;

			Vector3d phi_support_ankle;
			phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Left_Foot].Rotm, pelv_yaw_rot_current_from_global_);

			desired_q_dot_(4) = 200 * phi_support_ankle(1);
			desired_q_dot_(5) = 200 * phi_support_ankle(0);
			desired_q_(4) = current_q_(4) + desired_q_dot_(4) * dt_;
			desired_q_(5) = current_q_(5) + desired_q_dot_(5) * dt_;

			desired_q_(0) = 0.5 * motion_q_(0) + 0.5 * pre_desired_q_(0); //left support foot hip yaw
			desired_q_(3) = DyrosMath::QuinticSpline(walking_phase_, 0.0, switching_phase_duration_, init_q_(3), 0, 0, motion_q_(3), 0, 0)(0);
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
				if (kp_joint(i) == kp_soft_joint(i))
				{
					kp_joint(i) = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, kp_soft_joint(i), kp_stiff_joint(i), 0, 0); //support foot
				}

				if (kp_joint(i + 6) == kp_stiff_joint(i + 6))
				{
					kp_joint(i + 6) = DyrosMath::cubic(walking_phase_, 0.9, 1, kp_stiff_joint(i + 6), kp_soft_joint(i + 6), 0, 0); //swing foot
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

			torque_swing_assist.segment(7, 3) = wbc.task_control_torque(rd_, jac_rfoot_.block(0, 0, 3, MODEL_DOF_VIRTUAL), f_star).segment(7, 3);

			// torque_swing_assist.segment(7, 3) = (jac_lfoot_.transpose()).block(13, 0, 3, 6)*rd_.lambda.block(0, 2, 6, 1)*swing_foot_acc_trajectory_from_global_(2);
			/////////////////////////////////////////////////////////////////////////////

			swing_pd_switch = DyrosMath::cubic(walking_phase_, 0, switching_phase_duration_, 0, 1, 0, 0);

			if (first_step_trigger_ == true)
			{

				pd_control_mask_(0) = 1;
				pd_control_mask_(1) = 0;
				pd_control_mask_(2) = 0;

				pd_control_mask_(6) = 1;
				pd_control_mask_(7) = swing_pd_switch;
				pd_control_mask_(8) = swing_pd_switch;

				torque_stance_hip(2) = -torque_pelv(1); // left hip pitch
				torque_stance_hip(1) = -torque_pelv(0); // left hip roll

				torque_stance_hip(8) = -torque_pelv(1) * (1 - swing_pd_switch); // right hip pitch
				torque_stance_hip(7) = -torque_pelv(0) * (1 - swing_pd_switch); // right hip roll
			}
			else
			{
				pd_control_mask_(0) = 1;
				pd_control_mask_(1) = 1 - swing_pd_switch;
				pd_control_mask_(2) = 1 - swing_pd_switch;

				pd_control_mask_(6) = 1;
				pd_control_mask_(7) = swing_pd_switch;
				pd_control_mask_(8) = swing_pd_switch;

				torque_stance_hip(2) = -torque_pelv(1) * swing_pd_switch; // left hip pitch
				torque_stance_hip(1) = -torque_pelv(0) * swing_pd_switch; // left hip roll

				torque_stance_hip(8) = -torque_pelv(1) * (1 - swing_pd_switch); // right hip pitch
				torque_stance_hip(7) = -torque_pelv(0) * (1 - swing_pd_switch); // right hip roll
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
		desired_q_(0) = 0.5 * motion_q_(0) + 0.5 * pre_desired_q_(0);
		desired_q_(6) = 0.5 * motion_q_(6) + 0.5 * pre_desired_q_(6);

		desired_q_(3) = 0.4 * motion_q_(3) + 0.6 * pre_desired_q_(3); //left knee
		desired_q_(9) = 0.4 * motion_q_(9) + 0.6 * pre_desired_q_(9); //right knee

		// desired_q_(4) = 0.3*motion_q_(4) + 0.7*pre_desired_q_(4); //lefft ankle pitch
		// desired_q_(5) = 0.3*motion_q_(5) + 0.7*pre_desired_q_(5); //lefft ankle roll
		// desired_q_(10) = 0.3*motion_q_(10) + 0.7*pre_desired_q_(10); //right ankle pitch
		// desired_q_(11) = 0.3*motion_q_(11) + 0.7*pre_desired_q_(11); //right ankle roll

		// desired_q_(4) = 0.5*desired_q_leg(10) + 0.5*pre_desired_q_(10); //right support foot ankle pitch
		// desired_q_(5) = 0.5*desired_q_leg(11) + 0.5*pre_desired_q_(11); //right support foot anlke roll
		// desired_q_(10) = 0.5*desired_q_leg(10) + 0.5*pre_desired_q_(10); //right support foot ankle pitch
		// desired_q_(11) = 0.5*desired_q_leg(11) + 0.5*pre_desired_q_(11); //right support foot anlke roll

		desired_q_(4) = desired_q_leg(4);
		desired_q_(5) = desired_q_leg(5);
		desired_q_(10) = desired_q_leg(10);
		desired_q_(11) = desired_q_leg(11);

		for (int i = 1; i < 6; i++)
		{
			kp_joint(i + 6) = kp_stiff_joint(i + 6); //swing foot
			kp_joint(i) = kp_stiff_joint(i);				 //support foot
		}

		swing_pd_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + switching_phase_duration_ * walking_duration_, 0, 1, 0, 0);
		if (stop_walking_trigger_ == true)
		{
			if (foot_contact_ == 1)
			{
				pd_control_mask_(0) = 1 - swing_pd_switch;
				pd_control_mask_(1) = 1 - swing_pd_switch;
				pd_control_mask_(2) = 1 - swing_pd_switch;

				pd_control_mask_(6) = 1;
				pd_control_mask_(7) = 0;
				pd_control_mask_(8) = 0;

				torque_stance_hip(2) = -torque_pelv(1) * swing_pd_switch; // left hip pitch
				torque_stance_hip(1) = -torque_pelv(0) * swing_pd_switch; // left hip roll
				torque_stance_hip(8) = -torque_pelv(1);										// right hip pitch
				torque_stance_hip(7) = -torque_pelv(0);										// right hip roll
			}
			else if (foot_contact_ == -1)
			{
				pd_control_mask_(0) = 1;
				pd_control_mask_(1) = 0;
				pd_control_mask_(2) = 0;

				pd_control_mask_(6) = 1 - swing_pd_switch;
				pd_control_mask_(7) = 1 - swing_pd_switch;
				pd_control_mask_(8) = 1 - swing_pd_switch;

				torque_stance_hip(2) = -torque_pelv(1);										// left hip pitch
				torque_stance_hip(1) = -torque_pelv(0);										// left hip roll
				torque_stance_hip(8) = -torque_pelv(1) * swing_pd_switch; // right hip pitch
				torque_stance_hip(7) = -torque_pelv(0) * swing_pd_switch; // right hip roll
			}
		}
		else if (program_start_time_ == stance_start_time_)
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

			pd_control_mask_(0) = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5 * walking_duration_, 0, 1, 0, 0);
			pd_control_mask_(1) = 0;
			pd_control_mask_(2) = 0;

			double hip_control_switch;
			hip_control_switch = DyrosMath::cubic(current_time_, stance_start_time_, stance_start_time_ + 0.5 * walking_duration_, 0, 1, 0, 0);
			torque_stance_hip(2) = -torque_pelv(1) * hip_control_switch; // left hip pitch
			torque_stance_hip(1) = -torque_pelv(0) * hip_control_switch; // left hip roll
			torque_stance_hip(8) = -torque_pelv(1) * hip_control_switch; // right hip pitch
			torque_stance_hip(7) = -torque_pelv(0) * hip_control_switch; // right hip roll

			// cout<<"hip_control_switch: "<< hip_control_switch <<endl;
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
	///////////////////////////////////////////////////////////////////////////

	/////////////////////////////////WAIST DESIRED JOINT ANGLES//////////////////////////////
	Vector3d phi_trunk;
	phi_trunk = -DyrosMath::getPhi(rd_.link_[Upper_Body].Rotm, pelv_yaw_rot_current_from_global_);
	// phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());

	desired_q_dot_(12) = 30 * phi_trunk(2);	 //waist yaw
	desired_q_dot_(13) = 30 * phi_trunk(1);	 //waist pitch
	desired_q_dot_(14) = -30 * phi_trunk(0); //waist roll

	desired_q_.segment(12, 3) = current_q_.segment(12, 3) + desired_q_dot_.segment(12, 3) * dt_;

	//////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////HEAD///////////////////////////////////////////////
	for (int i = 15; i < MODEL_DOF; i++)
	{
		desired_q_(i) = motion_q_(i);
	}
	////////////////////////////////////////////////////////////////////////////////////
	
	///////////////////////////////ANKLE TUNING FOR VELOCITY TRACKING/////////////////
	desired_q_ += jointComTrackingTuning(); //reference: Lee, Yoonsang, Sungeun Kim, and Jehee Lee. "Data-driven biped control." ACM SIGGRAPH 2010 papers. 2010. 1-8.
	//////////////////////////////////////////////////////////////////////////////////

	//////////////////////////////////MOTION CONTROL/////////////////////////////////////
	// torque += stablePDControl(1000, 1000*dt_*16, current_q_, current_q_dot_, current_q_ddot_, desired_q_, desired_q_dot_);
	for (int i = 0; i < MODEL_DOF; i++)
	{
		torque(i) = kp_joint(i) * (desired_q_(i) - current_q_(i)) + kv_joint(i) * (desired_q_dot_(i) - current_q_dot_(i));
		torque(i) = torque(i) * pd_control_mask_(i); // masking for joint pd control
	}
	////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////SWING FOOT ASSIST TORQUE///////////////////////////
	for (int i = 0; i < 12; i++)
	{
		torque(i) += torque_swing_assist(i) * pd_control_mask_(i);
	}
	//////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////PELVIS ORIENTATION CONTROL TORQUE/////////////////
	torque += torque_stance_hip;
	// cout<<"torque_stance_hip: \n"<<torque_stance_hip<<endl;
	//////////////////////////////////////////////////////////////////////////////////
	
	////////////////////////////PRINT DATA//////////////////////////////////////////////////////////////////
	// if( walking_phase_ > 0.1)
	{
		// cout<<"pelv_rot_current_yaw_aline_: \n"<<pelv_rot_current_yaw_aline_<<endl;
		// // cout<<"pelv_rot_current_yaw_aline_.eulerAngles(2,1,0): \n"<<pelv_rot_current_yaw_aline_.eulerAngles(2,1,0)<<endl;
		// cout<<"desired_q_: \n"<<desired_q_<<endl;
		// cout<<"current_q_: \n"<<current_q_<<endl;
		// cout<<"error_q_: \n"<< desired_q_ - current_q_<<endl;
		// cout<<"joint_pd_torque: \n"<<torque<<endl;
		// cout<<"torque_stance_hip: \n"<<torque_stance_hip<<endl;

		// cout<<"swing_foot_acc_trajectory_from_global_(2): \n"<<swing_foot_acc_trajectory_from_global_(2) <<endl;
		// cout<<"torque_swing_assist: \n"<<torque_swing_assist.segment(0, 12) <<endl;
	}

	// torque(4) = DyrosMath::minmax_cut(torque(4), -0.15*rd_.com_.mass*GRAVITY, +0.15*rd_.com_.mass*GRAVITY);
	// torque(5) = DyrosMath::minmax_cut(torque(5), -0.085*rd_.com_.mass*GRAVITY, +0.085*rd_.com_.mass*GRAVITY);
	// torque(10) = DyrosMath::minmax_cut(torque(10), -0.15*rd_.com_.mass*GRAVITY, +0.15*rd_.com_.mass*GRAVITY);
	// torque(11) = DyrosMath::minmax_cut(torque(11), -0.085*rd_.com_.mass*GRAVITY, +0.085*rd_.com_.mass*GRAVITY);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////

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
	double kp_ank_sag = 0.2;	//x direction ankle pitch gain for com position error
	double kv_ank_sag = 0.25; //x direction ankle pitch gain for com velocity error

	double kp_ank_cor = 0.3;	//y direction ankle pitch gain for com position error
	double kv_ank_cor = 0.35; //y direction ankle pitch gain for com velocity error

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

	L_D << 0, 0.1025, -0.1225;
	R_D << 0, -0.1025, -0.1225;

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

				R_C = L_max;
				// cout<<"swing xy projection gain: "<<mapping_xy<<endl;
				// cout<<"swing xy projection point: "<<R_r<<endl;
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

				L_C = L_max;
				// cout<<"swing xy projection gain: "<<mapping_xy<<endl;
				// cout<<"swing xy projection point: "<<L_r<<endl;
			}
		}
	}

	double temp_q_des;
	temp_q_des = (pow(L_upper, 2) + pow(L_lower, 2) - pow(L_C, 2)) / (2 * L_upper * L_lower);
	temp_q_des = DyrosMath::minmax_cut(temp_q_des, -1, 1);
	q_des(3) = (-acos(temp_q_des) + M_PI);
	temp_q_des = (pow(L_upper, 2) + pow(L_lower, 2) - pow(R_C, 2)) / (2 * L_upper * L_lower);
	temp_q_des = DyrosMath::minmax_cut(temp_q_des, -1, 1);
	q_des(9) = (-acos(temp_q_des) + M_PI);
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
	Eigen::Vector2d foot_size;
	double safe_region_ratio = 0.8;
	double edge_region_ratio = 0.95;
	double left_ankle_pitch_tune = 1;
	double left_ankle_roll_tune = 1;
	double right_ankle_pitch_tune = 1;
	double right_ankle_roll_tune = 1;

	foot_size(0) = 0.15;
	foot_size(1) = 0.085;

	if (foot_swing_trigger_ == true)
	{
		if (foot_contact_ == 1)
		{
			// phi_trunk = -DyrosMath::getPhi(pelv_rot_current_yaw_aline_, Eigen::Matrix3d::Identity());
			// left_ankle_pitch_tune = DyrosMath::cubic(zmp_local_lfoot_(0)/foot_size(0) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);
			// left_ankle_roll_tune = DyrosMath::cubic(zmp_local_lfoot_(1)/foot_size(1) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);

			Vector3d phi_support_ankle;
			phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Left_Foot].Rotm, pelv_yaw_rot_current_from_global_);
			left_ankle_pitch_tune *= DyrosMath::cubic(abs(phi_support_ankle(1)), 0.00, 0.05, 1, 0, 0, 0);
			left_ankle_roll_tune *= DyrosMath::cubic(abs(phi_support_ankle(0)), 0.00, 0.05, 1, 0, 0, 0);

			left_ankle_pitch_tune *= DyrosMath::cubic(l_ft_(2), -rd_.com_.mass * GRAVITY / 5, -rd_.com_.mass * GRAVITY / 10, 1, 0, 0, 0);
			left_ankle_roll_tune *= DyrosMath::cubic(l_ft_(2), -rd_.com_.mass * GRAVITY / 5, -rd_.com_.mass * GRAVITY / 10, 1, 0, 0, 0);

			task_torque(4) = task_torque(4) * left_ankle_pitch_tune;
			task_torque(5) = task_torque(5) * left_ankle_roll_tune;
		}
		else if (foot_contact_ == -1)
		{
			// diff_zmp_rfoot(0) = abs(zmp_measured_rfoot_(0) - rfoot_transform_current_from_global_.translation()(0));
			// diff_zmp_rfoot(1) = abs(zmp_measured_rfoot_(1) - rfoot_transform_current_from_global_.translation()(1));

			// right_ankle_pitch_tune = DyrosMath::cubic(zmp_local_rfoot_(0)/foot_size(0) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);
			// right_ankle_roll_tune = DyrosMath::cubic(zmp_local_rfoot_(1)/foot_size(1) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);

			Vector3d phi_support_ankle;
			phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Right_Foot].Rotm, pelv_yaw_rot_current_from_global_);
			right_ankle_pitch_tune *= DyrosMath::cubic(abs(phi_support_ankle(1)), 0.00, 0.05, 1, 0, 0, 0);
			right_ankle_roll_tune *= DyrosMath::cubic(abs(phi_support_ankle(0)), 0.00, 0.05, 1, 0, 0, 0);

			right_ankle_pitch_tune *= DyrosMath::cubic(r_ft_(2), -rd_.com_.mass * GRAVITY / 5, -rd_.com_.mass * GRAVITY / 10, 1, 0, 0, 0);
			right_ankle_roll_tune *= DyrosMath::cubic(r_ft_(2), -rd_.com_.mass * GRAVITY / 5, -rd_.com_.mass * GRAVITY / 10, 1, 0, 0, 0);

			task_torque(10) = task_torque(10) * right_ankle_pitch_tune;
			task_torque(11) = task_torque(11) * right_ankle_roll_tune;
		}
	}
	else
	{
		middle_of_both_foot_ = (lfoot_transform_current_from_global_.translation() + lfoot_transform_current_from_global_.translation()) / 2;
		diff_btw_both_foot = lfoot_transform_current_from_global_.translation() - rfoot_transform_current_from_global_.translation();
		diff_zmp_both(0) = abs(zmp_measured_(0) - middle_of_both_foot_(0));
		diff_zmp_both(1) = abs(zmp_measured_(1) - middle_of_both_foot_(1));

		// left_ankle_pitch_tune = DyrosMath::cubic(diff_zmp_both(0)/foot_size(0) , safe_region_ratio, edge_region_ratio, 1, 0, 0, 0);
		// right_ankle_pitch_tune = left_ankle_pitch_tune;

		// left_ankle_roll_tune = DyrosMath::cubic(diff_zmp_both(1) , diff_btw_both_foot(1) + safe_region_ratio*foot_size(1), diff_btw_both_foot(1) + edge_region_ratio*foot_size(1), 1, 0, 0, 0);
		// right_ankle_roll_tune = left_ankle_roll_tune;

		// if(l_ft_(2) < rd_.com_.mass*GRAVITY/5)
		{
			left_ankle_pitch_tune *= DyrosMath::cubic(l_ft_(2), -rd_.com_.mass * GRAVITY / 10, -rd_.com_.mass * GRAVITY / 20, 1, 0.01, 0, 0);
			left_ankle_roll_tune *= DyrosMath::cubic(l_ft_(2), -rd_.com_.mass * GRAVITY / 10, -rd_.com_.mass * GRAVITY / 20, 1, 0.01, 0, 0);
		}

		// if(r_ft_(2) < rd_.com_.mass*GRAVITY/2)
		{
			right_ankle_pitch_tune *= DyrosMath::cubic(r_ft_(2), -rd_.com_.mass * GRAVITY / 10, -rd_.com_.mass * GRAVITY / 20, 1, 0.01, 0, 0);
			right_ankle_roll_tune *= DyrosMath::cubic(r_ft_(2), -rd_.com_.mass * GRAVITY / 10, -rd_.com_.mass * GRAVITY / 20, 1, 0.01, 0, 0);
		}

		Vector3d phi_support_ankle;
		phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Left_Foot].Rotm, pelv_yaw_rot_current_from_global_);
		left_ankle_pitch_tune *= DyrosMath::cubic(abs(phi_support_ankle(1)), 0.01, 0.05, 1, 0.0, 0, 0);
		left_ankle_roll_tune *= DyrosMath::cubic(abs(phi_support_ankle(0)), 0.01, 0.05, 1, 0.0, 0, 0);

		phi_support_ankle = -DyrosMath::getPhi(rd_.link_[Right_Foot].Rotm, pelv_yaw_rot_current_from_global_);
		right_ankle_pitch_tune *= DyrosMath::cubic(abs(phi_support_ankle(1)), 0.01, 0.05, 1, 0.0, 0, 0);
		right_ankle_roll_tune *= DyrosMath::cubic(abs(phi_support_ankle(0)), 0.01, 0.05, 1, 0.0, 0, 0);

		task_torque(4) = task_torque(4) * left_ankle_pitch_tune;
		task_torque(5) = task_torque(5) * left_ankle_roll_tune;

		task_torque(10) = task_torque(10) * right_ankle_pitch_tune;
		task_torque(11) = task_torque(11) * right_ankle_roll_tune;
	}

	if ((left_ankle_pitch_tune * left_ankle_roll_tune * right_ankle_pitch_tune * right_ankle_roll_tune) <= 0.8)
	{
		if (true)
		{
			// cout<<"############### ankle torque tuning! ###############"<<endl;
			// cout<<"########left_ankle_pitch_tune:"<<left_ankle_pitch_tune<< "############"<<endl;
			// cout<<"########left_ankle_roll_tune:"<<left_ankle_roll_tune<< "############"<<endl;
			// cout<<"########right_ankle_pitch_tune:"<<right_ankle_pitch_tune<< "############"<<endl;
			// cout<<"########right_ankle_roll_tune:"<<right_ankle_roll_tune<< "############"<<endl;
			// cout<<"############### ankle torque tuning! ###############"<<endl;
		}
	}

	return task_torque;
}

void CustomController::savePreData()
{
	pre_time_ = current_time_;
	pre_q_ = rd_.q_;
	pre_desired_q_ = desired_q_;
	zmp_measured_ppre_ = zmp_measured_pre_;
	zmp_measured_pre_ = zmp_measured_;
	com_pos_desired_pre_ = com_pos_desired_;
	com_vel_desired_pre_ = com_vel_desired_;
	f_star_xy_pre_ = f_star_xy_;
	f_star_6d_pre_ = f_star_6d_;
	torque_task_pre_ = torque_task_;
	torque_grav_pre_ = torque_grav_;

	zmp_desired_pre_ = zmp_desired_from_global_;
	zmp_local_lfoot_pre_ = zmp_local_lfoot_;
	zmp_local_rfoot_pre_ = zmp_local_rfoot_;
}

void CustomController::WalkingSpeedCommandCallback(const std_msgs::Float32 &msg)
{
	walking_speed_ = msg.data;
}

void CustomController::WalkingDurationCommandCallback(const std_msgs::Float32 &msg)
{
	walking_duration_ = msg.data;
}

void CustomController::WalkingAngVelCommandCallback(const std_msgs::Float32 &msg)
{
	yaw_angular_vel_ = msg.data;
}

void CustomController::KneeTargetAngleCommandCallback(const std_msgs::Float32 &msg)
{
	knee_target_angle_ = msg.data;
}

void CustomController::FootHeightCommandCallback(const std_msgs::Float32 &msg)
{
	swing_foot_height_ = msg.data;
}