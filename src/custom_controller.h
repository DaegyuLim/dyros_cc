#include <tocabi_controller/data_container.h>
#include <tocabi_controller/link.h>
#include "math_type_define.h"

class CustomController
{
public:
    CustomController(DataContainer &dc,RobotData &rd);
    Eigen::VectorQd getControl();

    void taskCommandToCC(TaskCommand tc_);
    
    void computeSlow();
    void computeFast();
    void computePlanner();
    
    DataContainer &dc_;
    RobotData &rd_;
    WholebodyController &wbc_;
    TaskCommand tc;

    //////////dg custom controller variables////////
    void setWalkingParameter(double walking_duration, double walking_speed, double step_width, double knee_target_angle);

    void getRobotData(WholebodyController &wbc);
    void getProcessedRobotData(WholebodyController &wbc);
    void walkingStateManager();
    void motionGenerator();
    void getCOMTrajectory();
    void getSwingFootXYTrajectory(double phase, Eigen::Vector3d com_pos_current, Eigen::Vector3d com_vel_current, Eigen::Vector3d com_vel_desired);
    void computeIk(Eigen::Isometry3d float_trunk_transform, Eigen::Isometry3d float_lleg_transform, Eigen::Isometry3d float_rleg_transform, Eigen::Vector12d& q_des);
    Eigen::VectorQd comVelocityControlCompute(WholebodyController &wbc);
    Eigen::VectorQd comVelocityControlCompute2(WholebodyController &wbc);
    Eigen::VectorQd jointTrajectoryPDControlCompute(WholebodyController &wbc);
    bool balanceTrigger(Eigen::Vector2d com_pos_2d, Eigen::Vector2d com_vel_2d);
    int checkZMPinWhichFoot(Eigen::Vector2d zmp_measured); // check where the zmp is
    Eigen::VectorQd tuneTorqueForZMPSafety(Eigen::VectorQd task_torque); // check where the zmp is
    Eigen::VectorQd zmpAnkleControl();
    Eigen::VectorQd jointComTrackingTuning();
  
    void computeZmp();
    void savePreData();
    
    ros::Subscriber walking_speed_command;
    ros::Subscriber walking_duration_command;
    ros::Subscriber walking_angvel_command;
    ros::Subscriber knee_target_angle_command;
    ros::Subscriber foot_height_command;

    void WalkingSpeedCommandCallback(const std_msgs::Float32 &msg);
    void WalkingDurationCommandCallback(const std_msgs::Float32 &msg);
    void WalkingAngVelCommandCallback(const std_msgs::Float32 &msg);
    void KneeTargetAngleCommandCallback(const std_msgs::Float32 &msg);
    void FootHeightCommandCallback(const std_msgs::Float32 &msg);



    bool walking_mode_on_;                                  // turns on when the walking control command is received and truns off after saving start time
    double stop_vel_threshold_;                             // acceptable capture point deviation from support foot

    int foot_contact_;                                      // 1:left,   -1:right,   0:double
    bool foot_swing_trigger_;                               // trigger swing if the robot needs to balance.
    bool first_step_trigger_;                               // ture if this is first swing foot. turn on at the start of the swing.
    bool start_walking_trigger_;                            // true when the walking_speed_ is not zero and swint do not start.
    bool stop_walking_trigger_;                             // turns on when the robot's speed become zero and lands last foot step
    double stance_start_time_;
    double program_start_time_;

    double walking_duration_;
    double walking_phase_;
    double turning_phase_;
    double switching_phase_duration_;
    
    double current_time_;
    double pre_time_;
    double start_time_;
    double dt_;

    double walking_speed_;
    double yaw_angular_vel_;
    double knee_target_angle_;

    double step_width_;
    double step_length_;

    double swing_foot_height_;
    
    double first_torque_supplier_;

    // CoM variables
    Eigen::Vector3d com_pos_desired_; 
    Eigen::Vector3d com_vel_desired_;
    Eigen::Vector3d com_acc_desired_;
    Eigen::Vector3d com_pos_current_;
    Eigen::Vector3d com_vel_current_;
    Eigen::Vector3d com_acc_current_; 
    Eigen::Vector3d com_pos_init_;
    Eigen::Vector3d com_vel_init_;
    Eigen::Vector3d com_acc_init_;
    Eigen::Vector3d com_pos_desired_pre_; 
    Eigen::Vector3d com_vel_desired_pre_;
    Eigen::Vector3d com_acc_desired_pre_;

    Eigen::Vector3d com_pos_current_pelvis_;
    Eigen::Vector3d com_vel_current_pelvis_; 
    Eigen::Vector3d com_pos_init_pelvis_;
    Eigen::Vector3d com_vel_init_pelvis_;
    Eigen::Vector3d com_pos_desired_pre_pelvis_; 
    Eigen::Vector3d com_vel_desired_pre_pelvis_;
    // Pevlis related variables
    Eigen::Vector3d pelv_pos_current_;
    Eigen::Vector3d pelv_vel_current_;
    Eigen::Matrix3d pelv_rot_current_;
    Eigen::Vector3d pelv_rpy_current_;
    Eigen::Matrix3d pelv_rot_current_yaw_aline_;

    Eigen::Matrix3d pelv_yaw_rot_current_from_global_;

    Eigen::Vector3d pelv_pos_init_;
    Eigen::Vector3d pelv_vel_init_;
    Eigen::Matrix3d pelv_rot_init_;
    Eigen::Vector3d pelv_rpy_init_;
    Eigen::Matrix3d pelv_rot_init_yaw_aline_;

    // Joint related variables
    Eigen::VectorQd current_q_;
    Eigen::VectorQd current_q_dot_;
    Eigen::VectorQd current_q_ddot_;
    Eigen::VectorQd desired_q_;
    Eigen::VectorQd desired_q_dot_;
    Eigen::VectorQd desired_q_ddot_;
    Eigen::VectorQd pre_q_;
    Eigen::VectorQd pre_desired_q_;

    Eigen::VectorQd motion_q_;
    Eigen::VectorQd motion_q_dot_;
    Eigen::VectorQd init_q_;

    Eigen::VectorQd pd_control_mask_; //1 for joint ik pd control

    Eigen::Vector2d target_foot_landing_from_pelv_;
    Eigen::Vector2d target_foot_landing_from_sup_;
    Eigen::Vector3d swing_foot_pos_trajectory_from_global_;
    Eigen::Vector6d swing_foot_vel_trajectory_from_global_;
    Eigen::Vector6d swing_foot_acc_trajectory_from_global_;
    Eigen::Matrix3d swing_foot_rot_trajectory_from_global_;

    Eigen::Isometry3d swing_foot_transform_init_;
    Eigen::Isometry3d support_foot_transform_init_;

    Eigen::Isometry3d swing_foot_transform_current_;
    Eigen::Isometry3d support_foot_transform_current_;

    Eigen::Vector6d swing_foot_vel_current_;
    Eigen::Vector6d swing_foot_vel_init_;


    Eigen::MatrixXd jac_com_;
    Eigen::MatrixXd jac_com_pos_;
    Eigen::MatrixXd jac_rhand_;
    Eigen::MatrixXd jac_lhand_;
    Eigen::MatrixXd jac_rfoot_;
    Eigen::MatrixXd jac_lfoot_;


    Eigen::Isometry3d lfoot_transform_init_from_global_;
    Eigen::Isometry3d rfoot_transform_init_from_global_;
    Eigen::Isometry3d lfoot_transform_init_from_pelvis_;
    Eigen::Isometry3d rfoot_transform_init_from_pelvis_;
    Eigen::Isometry3d lfoot_transform_init_from_sup_;
    Eigen::Isometry3d rfoot_transform_init_from_sup_;

    Eigen::Isometry3d lfoot_transform_current_from_global_;
    Eigen::Isometry3d rfoot_transform_current_from_global_;
    Eigen::Isometry3d lfoot_transform_current_from_pelvis_;
    Eigen::Isometry3d rfoot_transform_current_from_pelvis_;
    Eigen::Isometry3d lfoot_transform_current_from_sup_;
    Eigen::Isometry3d rfoot_transform_current_from_sup_;

    Eigen::Vector6d lfoot_vel_current_from_global;
    Eigen::Vector6d rfoot_vel_current_from_global;

    Eigen::Vector3d middle_of_both_foot_;

    Eigen::Vector3d zmp_measured_;
    Eigen::Vector3d zmp_measured_pre_;
    Eigen::Vector3d zmp_measured_ppre_;
    Eigen::Vector3d zmp_dot_measured_;

    Eigen::Vector3d zmp_measured_local_; //calc zmp with F/T sensors according to the Robot.ee_[0].contact
    Eigen::Vector3d zmp_dot_measured_local_;

    Eigen::Vector3d zmp_local_lfoot_;
    Eigen::Vector3d zmp_local_rfoot_;
    Eigen::Vector3d zmp_local_lfoot_pre_;
    Eigen::Vector3d zmp_local_rfoot_pre_;
    Eigen::Vector3d zmp_dot_local_rfoot_;
    Eigen::Vector3d zmp_dot_local_lfoot_;

    Eigen::Vector3d zmp_measured_lfoot_; //calc only left foot zmp with a F/T sensor
    Eigen::Vector3d zmp_measured_rfoot_;

    Eigen::Vector3d zmp_desired_from_global_;
    Eigen::Vector3d zmp_desired_pre_;

    Eigen::Vector6d l_ft_;
    Eigen::Vector6d r_ft_;

    Eigen::Vector2d f_star_xy_;
    Eigen::Vector2d f_star_xy_pre_;
    Eigen::Vector6d f_star_6d_;
    Eigen::Vector6d f_star_6d_pre_;

    Eigen::VectorQd torque_task_;
    Eigen::VectorQd torque_grav_;
    Eigen::VectorQd torque_task_pre_;
    Eigen::VectorQd torque_grav_pre_;
private:
    Eigen::VectorQd ControlVal_;
    void initWalkingParameter();
};
