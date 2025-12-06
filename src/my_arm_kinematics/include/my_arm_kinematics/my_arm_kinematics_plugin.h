#pragma once

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace my_arm_kinematics
{

class MyArmKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  MyArmKinematicsPlugin() = default;

  bool initialize(
      const rclcpp::Node::SharedPtr& node,
      const moveit::core::RobotModel& robot_model,
      const std::string& group_name,
      const std::string& base_frame,
      const std::vector<std::string>& tip_frames,
      double search_discretization) override;

  bool supportsGroup(const moveit::core::JointModelGroup* jmg,
                     std::string* error_text_out = nullptr) const override
  {
    (void)jmg;
    (void)error_text_out;
    return true;
  }

  bool getPositionIK(
      const geometry_msgs::msg::Pose& ik_pose,
      const std::vector<double>& ik_seed_state,
      std::vector<double>& solution,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options) const override;

  // ----- All required searchPositionIK overloads -----
  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose,
      const std::vector<double>& ik_seed_state,
      double timeout,
      std::vector<double>& solution,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options) const override;

  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose,
      const std::vector<double>& ik_seed_state,
      double timeout,
      const std::vector<double>& consistency_limits,
      std::vector<double>& solution,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options) const override;

  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose,
      const std::vector<double>& ik_seed_state,
      double timeout,
      std::vector<double>& solution,
      const IKCallbackFn& solution_callback,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options) const override;

  bool searchPositionIK(
      const geometry_msgs::msg::Pose& ik_pose,
      const std::vector<double>& ik_seed_state,
      double timeout,
      const std::vector<double>& consistency_limits,
      std::vector<double>& solution,
      const IKCallbackFn& solution_callback,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options) const override;

  bool getPositionFK(
      const std::vector<std::string>& link_names,
      const std::vector<double>& joint_angles,
      std::vector<geometry_msgs::msg::Pose>& poses) const override;

  // ----- Required by KinematicsBase (pure virtual) -----
  const std::vector<std::string>& getJointNames() const override
  {
    return joint_names_;
  }

  const std::vector<std::string>& getLinkNames() const override
  {
    return link_names_;
  }

private:
  bool computeIkForPose(
      const Eigen::Isometry3d& T_base_ee,
      std::vector<std::vector<double>>& all_solutions) const;

  bool withinJointLimits(const std::vector<double>& q) const;

  std::string group_name_;
  std::string base_frame_;
  std::string tip_frame_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  std::vector<double> joint_min_;
  std::vector<double> joint_max_;

  // Real geometry (meters)
  double L1_{0.135};
  double L2_{0.147};
  double hs_{0.096122711};
};

}  // namespace my_arm_kinematics

PLUGINLIB_EXPORT_CLASS(my_arm_kinematics::MyArmKinematicsPlugin,
                       kinematics::KinematicsBase)
