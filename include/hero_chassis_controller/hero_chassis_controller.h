//
// Hero chassis controller for mecanum drive.
// RoboMaster DynamicX 27-season reserve member final assignment.
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <control_toolbox/pid.h>

#include <memory>
#include <string>

namespace hero_chassis_controller {

/**
 * @brief Mecanum chassis controller based on ros_control.
 *
 * Subscribes to /cmd_vel, computes per-wheel target velocity with inverse
 * kinematics, tracks it with four independent PIDs and outputs joint effort.
 * Publishes odometry from forward kinematics together with the tf transform
 * from "odom" to "base_link".
 */
class HeroChassisController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  /**
   * @brief Called once when the controller is loaded.
   * @param hw   Pointer to the effort joint interface provided by RobotHW.
   * @param root_nh   Node handle in the root namespace ("/").
   * @param controller_nh  Node handle in the controller's private namespace.
   * @return True on success.
   */
  bool init(hardware_interface::EffortJointInterface *hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  /**
   * @brief Called every control period by the controller manager.
   * @param time   Current simulation time.
   * @param period Time elapsed since the previous call.
   */
  void update(const ros::Time &time, const ros::Duration &period) override;

 private:
  /// Inverse kinematics: chassis velocity -> four wheel angular velocities.
  void computeInverseKinematics(double vx, double vy, double wz, double wheel_vel[4]);

  /// Forward kinematics: four wheel angular velocities -> chassis velocity.
  void computeForwardKinematics(const double wheel_vel[4], double &vx, double &vy, double &wz);

  /// Publish nav_msgs/Odometry and broadcast the odom -> base_link tf.
  void publishOdometry(double vx, double vy, double wz,
                       const ros::Time &time, const ros::Duration &period);

  /// /cmd_vel callback. Only stores the command; never does heavy work.
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

  // ---- Joint handles: the only channel to the hardware ----
  // NOTE: EffortJointInterface -> setCommand() takes TORQUE in N*m.
  hardware_interface::JointHandle joint_lf_, joint_rf_, joint_lb_, joint_rb_;

  // ---- Four independent PIDs, one per wheel (assignment 4) ----
  control_toolbox::Pid pid_lf_, pid_rf_, pid_lb_, pid_rb_;

  // ---- ROS interfaces ----
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher debug_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- Kinematic parameters, read from YAML ----
  double wheel_radius_{0.07625};
  double wheel_track_{0.4};
  double wheel_base_{0.4};
  double max_acceleration_{0.0};   // 0 means disabled (assignment 8)

  // ---- Mode switch (assignment 7): "base_link" or "odom" ----
  std::string velocity_mode_{"base_link"};

  // ---- Command and state ----
  geometry_msgs::Twist cmd_vel_{};          // latest /cmd_vel
  geometry_msgs::Twist cmd_vel_limited_{};  // after acceleration limiting
  double odom_x_{0.0}, odom_y_{0.0}, odom_yaw_{0.0};

  // ---- Whether a sign flip is needed on the right side. ----
  // The right-side transmissions have a NEGATIVE mechanical reduction
  // (-19.2032) in the URDF. Determine this EXPERIMENTALLY, see the guide.
  bool flip_right_side_{false};
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
