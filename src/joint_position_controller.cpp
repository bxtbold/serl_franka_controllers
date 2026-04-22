/*
Refered to Source file:
  https://github.com/frankaemika/franka_ros/blob/develop/franka_example_controllers/src/joint_position_example_controller.cpp
*/

#include <serl_franka_controllers/joint_position_controller.h>

#include <cmath>
#include <sstream>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace serl_franka_controllers {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::vector<double> target_positions;
  if (!node_handle.getParam("/target_joint_positions", target_positions) || target_positions.size() != 7) {
    ROS_ERROR("JointPositionController: Could not read target joint positions from parameter server or incorrect size");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    reset_pose_[i] = target_positions[i];
  }

  node_handle.param("startup_hold_time", startup_hold_time_, 0.2);
  if (!std::isfinite(startup_hold_time_) || startup_hold_time_ < 0.0) {
    ROS_WARN_STREAM("JointPositionController: Invalid startup_hold_time="
                    << startup_hold_time_ << ", using 0.2s");
    startup_hold_time_ = 0.2;
  }

  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    const double q_curr = position_joint_handles_[i].getPosition();
    initial_pose_[i] = q_curr;
    position_joint_handles_[i].setCommand(q_curr);
  }
  elapsed_time_ = ros::Duration(0.0);
  startup_elapsed_ = ros::Duration(0.0);
  motion_initialized_ = false;
}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  // Timing sanity
  double dt = period.toSec();
  const double min_dt = 1e-6;
  if (!std::isfinite(dt) || dt < 0.0 || dt > 0.1) {
    ROS_WARN_THROTTLE(1.0, "JointPositionController: Bad control period: %.9f", dt);
    return;
  }

  // A zero/near-zero period can happen transiently; skip this cycle quietly.
  if (dt < min_dt) {
    ROS_DEBUG_THROTTLE(1.0,
                       "JointPositionController: skipping near-zero control period: %.9f",
                       dt);
    return;
  }

  startup_elapsed_ += period;
  if (startup_elapsed_.toSec() < startup_hold_time_) {
    for (size_t i = 0; i < 7; ++i) {
      const double q_curr = position_joint_handles_[i].getPosition();
      position_joint_handles_[i].setCommand(q_curr);
    }
    ROS_DEBUG_THROTTLE(1.0,
                       "JointPositionController: startup hold active for %.3fs / %.3fs",
                       startup_elapsed_.toSec(), startup_hold_time_);
    return;
  }

  if (!motion_initialized_) {
    for (size_t i = 0; i < 7; ++i) {
      initial_pose_[i] = position_joint_handles_[i].getPosition();
    }
    elapsed_time_ = ros::Duration(0.0);
    motion_initialized_ = true;
    ROS_DEBUG("JointPositionController: startup hold complete, motion initialized.");
    return;
  }

  elapsed_time_ += period;

  const double T = 5.0;
  double t = elapsed_time_.toSec();

  // Clamp time (prevents overshoot)
  if (t > T) t = T;

  double s = t / T;

  // Check s validity
  if (!std::isfinite(s) || s < 0.0 || s > 1.0) {
    ROS_ERROR_THROTTLE(1.0, "JointPositionController: Invalid time scaling s=%f", s);
    return;
  }

  // Quintic time scaling
  double s2 = s * s;
  double s3 = s2 * s;
  double s4 = s3 * s;
  double s5 = s4 * s;
  double s_curve = 10*s3 - 15*s4 + 6*s5;

  // Derivative (for velocity limiting)
  double ds_dt = (30*s2 - 60*s3 + 30*s4) / T;

  ROS_DEBUG_THROTTLE(
      1.0,
      "JointPositionController: update dt=%.6f t=%.3f/%.3f s=%.4f s_curve=%.4f ds_dt=%.4f",
      dt, t, T, s, s_curve, ds_dt);

  const double max_vel = 1.0;    // rad/s
  const double max_delta = 2.5;  // rad (sanity limit)
  const double max_step = 0.02;  // rad per cycle (extra safety)
  std::ostringstream joint_debug_stream;

  for (size_t i = 0; i < 7; ++i) {
    double q_curr = position_joint_handles_[i].getPosition();

    // Validate state
    if (!std::isfinite(q_curr)) {
      ROS_ERROR_THROTTLE(1.0,
                         "JointPositionController: NaN/Inf joint state at joint %zu (q_curr=%f)",
                         i, q_curr);
      continue;  // skip this joint
    }

    double q_target = reset_pose_[i];
    double q_init = initial_pose_[i];

    // Validate target
    if (!std::isfinite(q_target) || !std::isfinite(q_init)) {
      ROS_ERROR_THROTTLE(
          1.0,
          "JointPositionController: NaN/Inf target/init at joint %zu (q_target=%f q_init=%f)",
          i, q_target, q_init);
      position_joint_handles_[i].setCommand(q_curr);
      continue;
    }

    double delta = q_target - q_init;

    // Prevent insane jumps
    if (std::abs(delta) > max_delta) {
      ROS_ERROR_THROTTLE(1.0,
                         "JointPositionController: Target too far (joint %zu, q_init=%f q_target=%f "
                         "delta=%f, max_delta=%f). Holding.",
                         i, q_init, q_target, delta, max_delta);
      position_joint_handles_[i].setCommand(q_curr);
      continue;
    }

    // Nominal trajectory
    double q_des = q_init + delta * s_curve;
    double qd_des = delta * ds_dt;

    // Velocity limiting
    if (std::abs(qd_des) > max_vel) {
      double scale = max_vel / std::abs(qd_des);
      q_des = q_curr + qd_des * scale * dt;
      ROS_WARN_THROTTLE(
          1.0,
          "JointPositionController: velocity limit hit at joint %zu (qd_des=%f, max_vel=%f, "
          "scale=%f, dt=%f, q_curr=%f, q_des_limited=%f)",
          i, qd_des, max_vel, scale, dt, q_curr, q_des);
    }

    // Step-size limiter
    double step = q_des - q_curr;
    if (std::abs(step) > max_step) {
      q_des = q_curr + std::copysign(max_step, step);
      ROS_WARN_THROTTLE(
          1.0,
          "JointPositionController: step limit hit at joint %zu (step=%f, max_step=%f, "
          "q_curr=%f, q_des_limited=%f)",
          i, step, max_step, q_curr, q_des);
    }

    // Sanity check
    if (!std::isfinite(q_des)) {
      ROS_ERROR_THROTTLE(1.0, "JointPositionController: q_des NaN!");
      q_des = q_curr;
    }

    if (i > 0) {
      joint_debug_stream << " | ";
    }
    joint_debug_stream << "j" << i + 1 << ":"
                       << " qi=" << q_init
                       << " qc=" << q_curr
                       << " qt=" << q_target
                       << " d=" << delta
                       << " qd=" << q_des;

    position_joint_handles_[i].setCommand(q_des);
  }

  ROS_DEBUG_THROTTLE(1.0,
                    "JointPositionController: joints %s",
                    joint_debug_stream.str().c_str());
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::JointPositionController,
                       controller_interface::ControllerBase)