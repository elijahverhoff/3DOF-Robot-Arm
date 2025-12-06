#include <my_arm_kinematics/my_arm_kinematics_plugin.h>

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/logger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace my_arm_kinematics
{

bool MyArmKinematicsPlugin::initialize(
    const rclcpp::Node::SharedPtr& node,
    const moveit::core::RobotModel& robot_model,
    const std::string& group_name,
    const std::string& base_frame,
    const std::vector<std::string>& tip_frames,
    double search_discretization)
{
  (void)node;
  (void)search_discretization;

  group_name_ = group_name;
  base_frame_ = base_frame;

  if (tip_frames.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("MyArmKinematicsPlugin"),
                 "No tip frame specified for group '%s'", group_name_.c_str());
    return false;
  }
  tip_frame_ = tip_frames[0];

  const moveit::core::JointModelGroup* jmg =
      robot_model.getJointModelGroup(group_name_);
  if (!jmg)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MyArmKinematicsPlugin"),
                 "JointModelGroup '%s' not found", group_name_.c_str());
    return false;
  }

  joint_names_ = jmg->getVariableNames();
  std::size_t nj = joint_names_.size();

  joint_min_.resize(nj);
  joint_max_.resize(nj);

  for (std::size_t i = 0; i < nj; ++i)
  {
    const moveit::core::VariableBounds& bounds =
        robot_model.getVariableBounds(joint_names_[i]);
    joint_min_[i] = bounds.min_position_;
    joint_max_[i] = bounds.max_position_;
  }

  // Link names
  link_names_.clear();
  link_names_.push_back(tip_frame_);

  RCLCPP_INFO(rclcpp::get_logger("MyArmKinematicsPlugin"),
              "IK plugin initialized for group '%s', base '%s', tip '%s'",
              group_name_.c_str(), base_frame_.c_str(), tip_frame_.c_str());

  return true;
}

bool MyArmKinematicsPlugin::withinJointLimits(const std::vector<double>& q) const
{
  if (q.size() != joint_min_.size())
    return false;

  for (std::size_t i = 0; i < q.size(); ++i)
  {
    if (q[i] < joint_min_[i] - 1e-6 || q[i] > joint_max_[i] + 1e-6)
      return false;
  }
  return true;
}

bool MyArmKinematicsPlugin::computeIkForPose(
    const Eigen::Isometry3d& T_base_ee,
    std::vector<std::vector<double>>& all_solutions) const
{
  all_solutions.clear();

  Eigen::Vector3d p = T_base_ee.translation();
  double X = p.x();
  double Y = p.y();
  double Z = p.z();

  // 1) Base rotation q1 about world Z
  double q1 = std::atan2(Y, X);

  // 2) Project into vertical plane
  double xw = std::hypot(X, Y);  // radial distance from base axis
  double zw = Z;                 // vertical

  // Shoulder-frame coordinates
  double xp = xw;
  double zp = zw - hs_;          // subtract shoulder height

  double L1 = L1_;
  double L2 = L2_;

  double r2 = xp * xp + zp * zp;

  double c3 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

  if (c3 < -1.0 || c3 > 1.0)
    return false;  // unreachable

  double s3_pos = std::sqrt(std::max(0.0, 1.0 - c3 * c3));
  double s3_neg = -s3_pos;

  auto make_solution = [&](double s3_val) -> std::vector<double>
  {
    double q3 = std::atan2(s3_val, c3);
    double k1 = L1 + L2 * c3;
    double k2 = L2 * s3_val;
    double q2 = std::atan2(zp, xp) - std::atan2(k2, k1);

    // Virtual wrist DOF: choose q4 so wrist/trialink are rotated
    // away from the forearm towards "horizontal".
    // Simple model: q_tool ~ q2 + q3 + q4, so we want q_tool ≈ pi/2
    const double q4_offset = 1.57079632679;  // ≈ pi/2
    double q4 = q4_offset - (q2 + q3);

    // IMPORTANT: order must match arm group joint order in SRDF
    return { q1, q2, q3, q4 };
  };

  std::vector<std::vector<double>> candidates;
  candidates.push_back(make_solution(s3_pos));
  candidates.push_back(make_solution(s3_neg));

  for (const auto& sol : candidates)
  {
    if (withinJointLimits(sol))
      all_solutions.push_back(sol);
  }

  return !all_solutions.empty();
}

bool MyArmKinematicsPlugin::getPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
  (void)ik_seed_state;
  (void)options;

  Eigen::Isometry3d T;
  tf2::fromMsg(ik_pose, T);

  std::vector<std::vector<double>> sols;
  if (!computeIkForPose(T, sols))
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  solution = sols.front();
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

// ===== searchPositionIK overloads (all forward to getPositionIK) =====

bool MyArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double timeout,
    std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
  (void)timeout;
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool MyArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double timeout,
    const std::vector<double>& consistency_limits,
    std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
  (void)consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, error_code, options);
}

bool MyArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double timeout,
    std::vector<double>& solution,
    const IKCallbackFn& solution_callback,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
  (void)solution_callback;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, error_code, options);
}

bool MyArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::msg::Pose& ik_pose,
    const std::vector<double>& ik_seed_state,
    double timeout,
    const std::vector<double>& consistency_limits,
    std::vector<double>& solution,
    const IKCallbackFn& solution_callback,
    moveit_msgs::msg::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const
{
  (void)consistency_limits;
  (void)solution_callback;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, error_code, options);
}

// ===== FK using RobotState =====

bool MyArmKinematicsPlugin::getPositionFK(
    const std::vector<std::string>& link_names,
    const std::vector<double>& joint_angles,
    std::vector<geometry_msgs::msg::Pose>& poses) const
{
  poses.clear();
  poses.resize(link_names.size());

  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(group_name_, joint_angles);
  state.update();

  for (std::size_t i = 0; i < link_names.size(); ++i)
  {
    const Eigen::Isometry3d& T = state.getGlobalLinkTransform(link_names[i]);
    poses[i] = tf2::toMsg(T);
  }

  return true;
}

}  // namespace my_arm_kinematics
