//
// Hero chassis controller for mecanum drive.
// RoboMaster DynamicX 27-season reserve member final assignment.
//

#include "hero_chassis_controller/hero_chassis_controller.h"

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <string>

namespace hero_chassis_controller {

// Joint names MUST match the URDF exactly.
// Verified against rm_description_for_task/urdf/hero/chassis.urdf.xacro.
static const std::string kJointLf = "left_front_wheel_joint";
static const std::string kJointRf = "right_front_wheel_joint";
static const std::string kJointLb = "left_back_wheel_joint";
static const std::string kJointRb = "right_back_wheel_joint";

bool HeroChassisController::init(hardware_interface::EffortJointInterface *hw,
                                 ros::NodeHandle &root_nh,
                                 ros::NodeHandle &controller_nh) {
  // ---- 1. Get joint handles. Throws if the name does not exist. ----
  try {
    joint_lf_ = hw->getHandle(kJointLf);
    joint_rf_ = hw->getHandle(kJointRf);
    joint_lb_ = hw->getHandle(kJointLb);
    joint_rb_ = hw->getHandle(kJointRb);
  } catch (const hardware_interface::HardwareInterfaceException &e) {
    ROS_ERROR("Failed to get joint handle: %s", e.what());
    return false;
  }

  // ---- 2. Read kinematic parameters from YAML (with safe defaults). ----
  controller_nh.param<double>("wheel_radius", wheel_radius_, 0.07625);
  controller_nh.param<double>("wheel_track", wheel_track_, 0.4);
  controller_nh.param<double>("wheel_base", wheel_base_, 0.4);
  controller_nh.param<double>("max_acceleration", max_acceleration_, 0.0);
  controller_nh.param<std::string>("velocity_mode", velocity_mode_, "base_link");
  // Assignment: the right-side transmissions have a NEGATIVE mechanical
  // reduction (-19.2032). Make the compensation switch configurable from YAML
  // so it can be verified experimentally without recompiling.
  controller_nh.param<bool>("flip_right_side", flip_right_side_, false);

  // ---- 3. Initialise four PIDs from the controller's private namespace. ----
  // Each PID reads p / i / d / i_clamp_max / i_clamp_min under its own sub-ns.
  // IMPORTANT: verify the exact parameter names in
  //   /opt/ros/noetic/include/control_toolbox/pid.h
  ros::NodeHandle pid_lf_nh(controller_nh, "left_front_pid");
  ros::NodeHandle pid_rf_nh(controller_nh, "right_front_pid");
  ros::NodeHandle pid_lb_nh(controller_nh, "left_back_pid");
  ros::NodeHandle pid_rb_nh(controller_nh, "right_back_pid");
  if (!pid_lf_.init(pid_lf_nh, false) || !pid_rf_.init(pid_rf_nh, false) ||
      !pid_lb_.init(pid_lb_nh, false) || !pid_rb_.init(pid_rb_nh, false)) {
    ROS_ERROR("Failed to initialise PID gains. Check controllers.yaml.");
    return false;
  }

  // ---- 4. tf buffer for the world-frame mode (assignment 7). ----
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ---- 5. Subscribers and publishers. ----
  cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);
  odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("/odom", 10);
  debug_pub_ = root_nh.advertise<std_msgs::Float64MultiArray>(
      "/hero_chassis/debug/velocity", 10);

  ROS_INFO("HeroChassisController initialised: R=%.5f track=%.3f base=%.3f mode=%s",
           wheel_radius_, wheel_track_, wheel_base_, velocity_mode_.c_str());
  return true;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  // Store only. All computation happens in update() on the realtime thread.
  cmd_vel_ = *msg;
}

void HeroChassisController::computeInverseKinematics(double vx, double vy, double wz,
                                                     double wheel_vel[4]) {
  // TODO(you): re-derive this yourself before the defence.
  // See section 1.5 of the companion document "核心原理与代码骨架".
  // lx = wheel_track/2, ly = wheel_base/2, k = lx + ly
  const double k = (wheel_track_ + wheel_base_) / 2.0;
  const double inv_r = 1.0 / wheel_radius_;

  wheel_vel[0] = inv_r * (vx - vy - k * wz);  // left front
  wheel_vel[1] = inv_r * (vx + vy + k * wz);  // right front
  wheel_vel[2] = inv_r * (vx + vy - k * wz);  // left back
  wheel_vel[3] = inv_r * (vx - vy + k * wz);  // right back
}

void HeroChassisController::computeForwardKinematics(const double wheel_vel[4],
                                                     double &vx, double &vy, double &wz) {
  // TODO(you): re-derive this yourself before the defence (section 1.6).
  const double k = (wheel_track_ + wheel_base_) / 2.0;
  const double r4 = wheel_radius_ / 4.0;

  vx = r4 * (wheel_vel[0] + wheel_vel[1] + wheel_vel[2] + wheel_vel[3]);
  vy = r4 * (-wheel_vel[0] + wheel_vel[1] + wheel_vel[2] - wheel_vel[3]);
  wz = r4 * (-wheel_vel[0] + wheel_vel[1] - wheel_vel[2] + wheel_vel[3]) / k;
}

void HeroChassisController::publishOdometry(double vx, double vy, double wz,
                                            const ros::Time &time,
                                            const ros::Duration &period) {
  const double dt = period.toSec();
  if (dt <= 0.0) {
    return;
  }

  // Integrate in the WORLD frame. The rotation matrix is mandatory:
  // vx/vy are expressed in base_link, which rotates with the robot.
  // Using the mid-point yaw reduces discretisation error (section 1.7).
  const double yaw_mid = odom_yaw_ + 0.5 * wz * dt;
  odom_x_ += (vx * std::cos(yaw_mid) - vy * std::sin(yaw_mid)) * dt;
  odom_y_ += (vx * std::sin(yaw_mid) + vy * std::cos(yaw_mid)) * dt;
  odom_yaw_ += wz * dt;

  // Keep yaw in [-pi, pi] to avoid precision loss over long runs.
  odom_yaw_ = std::atan2(std::sin(odom_yaw_), std::cos(odom_yaw_));

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, odom_yaw_);

  // ---- Publish nav_msgs/Odometry ----
  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = odom_x_;
  odom.pose.pose.position.y = odom_y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  // By convention twist is expressed in child_frame_id (base_link).
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = wz;
  odom_pub_.publish(odom);

  // ---- Broadcast tf: "odom" -> "base_link" ----
  // Parent frame goes in header.frame_id, child in child_frame_id.
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = time;
  tf_msg.header.frame_id = "odom";
  tf_msg.child_frame_id = "base_link";
  tf_msg.transform.translation.x = odom_x_;
  tf_msg.transform.translation.y = odom_y_;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = odom.pose.pose.orientation;
  tf_broadcaster_.sendTransform(tf_msg);
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
  // ==================== Step 1: resolve the velocity command ====================
  double vx_cmd = cmd_vel_.linear.x;
  double vy_cmd = cmd_vel_.linear.y;
  double wz_cmd = cmd_vel_.angular.z;

  // Assignment 7: if the command is expressed in the world frame, rotate it
  // into base_link before feeding it to the inverse kinematics.
  if (velocity_mode_ == "odom" || velocity_mode_ == "map") {
    try {
      // "base_link" <- source frame: express an odom-frame vector in base_link.
      const geometry_msgs::TransformStamped tf_odom_base =
          tf_buffer_->lookupTransform("base_link", "odom", ros::Time(0));

      geometry_msgs::Vector3Stamped vec_world, vec_base;
      vec_world.header.frame_id = "odom";
      vec_world.vector = cmd_vel_.linear;
      // doTransform on a Vector3 ROTATES ONLY (no translation), which is exactly
      // what a free vector such as velocity needs. Never use transformPoint here.
      tf2::doTransform(vec_world, vec_base, tf_odom_base);

      vx_cmd = vec_base.vector.x;
      vy_cmd = vec_base.vector.y;
      // angular.z is a scalar about the shared z axis in planar motion: unchanged.
    } catch (const tf2::TransformException &ex) {
      // tf may not be available yet. Warn and skip, never crash the controller.
      ROS_WARN_THROTTLE(1.0, "tf lookup failed: %s", ex.what());
      return;
    }
  }

  // Assignment 8: optional acceleration limiting (ramp limiter).
  if (max_acceleration_ > 0.0) {
    const double max_delta = max_acceleration_ * period.toSec();
    auto limit = [max_delta](double target, double current) {
      return std::max(current - max_delta, std::min(current + max_delta, target));
    };
    cmd_vel_limited_.linear.x = limit(vx_cmd, cmd_vel_limited_.linear.x);
    cmd_vel_limited_.linear.y = limit(vy_cmd, cmd_vel_limited_.linear.y);
    cmd_vel_limited_.angular.z = limit(wz_cmd, cmd_vel_limited_.angular.z);
    vx_cmd = cmd_vel_limited_.linear.x;
    vy_cmd = cmd_vel_limited_.linear.y;
    wz_cmd = cmd_vel_limited_.angular.z;
  }

  // ==================== Step 2: inverse kinematics ====================
  double target[4];
  computeInverseKinematics(vx_cmd, vy_cmd, wz_cmd, target);

  // The right-side transmissions have a NEGATIVE mechanical reduction
  // (-19.2032) in the URDF. Whether that requires a sign flip here MUST be
  // verified experimentally -- see section 4.5 of the assignment guide.
  if (flip_right_side_) {
    target[1] = -target[1];  // right front
    target[3] = -target[3];  // right back
  }

  // ==================== Step 3: four independent PID loops ====================
  // EffortJointInterface -> setCommand() expects TORQUE in N*m.
  const double err_lf = target[0] - joint_lf_.getVelocity();
  const double err_rf = target[1] - joint_rf_.getVelocity();
  const double err_lb = target[2] - joint_lb_.getVelocity();
  const double err_rb = target[3] - joint_rb_.getVelocity();

  joint_lf_.setCommand(pid_lf_.computeCommand(err_lf, period));
  joint_rf_.setCommand(pid_rf_.computeCommand(err_rf, period));
  joint_lb_.setCommand(pid_lb_.computeCommand(err_lb, period));
  joint_rb_.setCommand(pid_rb_.computeCommand(err_rb, period));

  // ==================== Step 4: forward kinematics -> odometry ====================
  double actual[4] = {joint_lf_.getVelocity(), joint_rf_.getVelocity(),
                      joint_lb_.getVelocity(), joint_rb_.getVelocity()};
  double vx_act = 0.0, vy_act = 0.0, wz_act = 0.0;
  computeForwardKinematics(actual, vx_act, vy_act, wz_act);
  publishOdometry(vx_act, vy_act, wz_act, time, period);

  // ==================== Step 5: debug topics for rqt_plot ====================
  // Layout: [target_lf, target_rf, target_lb, target_rb,
  //          actual_lf, actual_rf, actual_lb, actual_rb]
  if (debug_pub_.getNumSubscribers() > 0) {
    std_msgs::Float64MultiArray dbg;
    dbg.data.assign(target, target + 4);
    dbg.data.insert(dbg.data.end(), actual, actual + 4);
    debug_pub_.publish(dbg);
  }
}

// Register the class with pluginlib so controller_manager can load it by name.
// The second argument must be controller_interface::ControllerBase.
PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)

}  // namespace hero_chassis_controller
