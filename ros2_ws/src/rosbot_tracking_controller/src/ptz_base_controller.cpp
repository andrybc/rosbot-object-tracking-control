#include <chrono>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

static double clamp_double(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

class PtzBaseController : public rclcpp::Node
{
public:
  PtzBaseController() : Node("ptz_base_controller")
  {
    // --- Parameters (tuning knobs)
    // Tracker errors are in PIXELS from center. Your mono_Tracker uses 640x480.
    this->declare_parameter<int>("img_width", 640);
    this->declare_parameter<int>("img_height", 480);

    // Zones (as fraction of half-width/half-height)
    // inner: deadzone where we do nothing (prevents jitter)
    // outer: near-edge zone where we turn the base
    this->declare_parameter<double>("inner_zone", 0.15);  // 15% of half-width
    this->declare_parameter<double>("outer_zone", 0.60);  // 60% of half-width

    // Servo behavior
    this->declare_parameter<double>("servo_kp", 0.005);     // deg per pixel (scaled)
    this->declare_parameter<double>("servo_max_step_deg", 0.47); // max change per control tick
    this->declare_parameter<int>("servo1_min", -90);
    this->declare_parameter<int>("servo1_max", 90);
    this->declare_parameter<int>("servo2_min", -90);
    this->declare_parameter<int>("servo2_max", 20);
    this->declare_parameter<int>("servo1_center", 0);
    this->declare_parameter<int>("servo2_center", 10);

    // Base turn behavior
    this->declare_parameter<double>("base_turn_kp", 0.8);    // rad/s per normalized error
    this->declare_parameter<double>("base_turn_max", 0.25);  // clamp
    this->declare_parameter<double>("base_turn_min", 0.06);  // minimum when turning
    this->declare_parameter<double>("lost_timeout_sec", 0.35); // stop if no target recently

    // Smoothing (EMA). Higher = smoother but laggier.
    this->declare_parameter<double>("ema_alpha", 0.35);

    // Control loop rate
    this->declare_parameter<double>("control_hz", 20.0);

    // --- Publishers
    pub_servo1_ = this->create_publisher<std_msgs::msg::Int32>("/servo_s1", 10);
    pub_servo2_ = this->create_publisher<std_msgs::msg::Int32>("/servo_s2", 10);
    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // --- Subscribers (tracker outputs)
    sub_found_ = this->create_subscription<std_msgs::msg::Bool>(
      "/tracker/target_found", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        target_found_ = msg->data;
        if (target_found_) {
          last_seen_time_ = this->now();
        }
      });

    sub_ex_ = this->create_subscription<std_msgs::msg::Float32>(
      "/tracker/error_x", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg)
      {
        raw_ex_ = msg->data;
        have_ex_ = true;
      });

    sub_ey_ = this->create_subscription<std_msgs::msg::Float32>(
      "/tracker/error_y", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg)
      {
        raw_ey_ = msg->data;
        have_ey_ = true;
      });

    // Initial servo positions
    servo1_ = this->get_parameter("servo1_center").as_int();
    servo2_ = this->get_parameter("servo2_center").as_int();

    publish_servos(servo1_, servo2_);
    publish_stop();

    // Timer loop
    const double hz = this->get_parameter("control_hz").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      std::bind(&PtzBaseController::control_step, this));

    RCLCPP_INFO(this->get_logger(), "ptz_base_controller started. Waiting for /tracker/* topics...");
  }

private:
  void publish_servos(int s1, int s2)
  {
    std_msgs::msg::Int32 m1;
    std_msgs::msg::Int32 m2;
    m1.data = s1;
    m2.data = s2;
    pub_servo1_->publish(m1);
   // pub_servo2_->publish(m2);
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = 0.0;
    t.angular.z = 0.0;
    //pub_cmd_->publish(t);
  }

  void publish_turn(double wz)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = 0.0;
    t.angular.z = wz;
    //pub_cmd_->publish(t);
  }

  void control_step()
  {
    // If we don't even have error values yet, do nothing
    if (!have_ex_ || !have_ey_) {
      return;
    }

    const int img_w = this->get_parameter("img_width").as_int();
    const int img_h = this->get_parameter("img_height").as_int();
    const double half_w = img_w / 2.0;
    const double half_h = img_h / 2.0;

    // Normalize errors to [-1, 1] for zone logic
    // raw_ex_ is pixels from center (point_x - 320)
    // so raw_ex_ / half_w is normalized horizontal error
    const double ex_norm_raw = clamp_double(raw_ex_ / half_w, -1.0, 1.0);
    const double ey_norm_raw = clamp_double(raw_ey_ / half_h, -1.0, 1.0);

    // Smooth the errors (EMA)
    const double alpha = this->get_parameter("ema_alpha").as_double();
    ex_norm_ = alpha * ex_norm_raw + (1.0 - alpha) * ex_norm_;
    ey_norm_ = alpha * ey_norm_raw + (1.0 - alpha) * ey_norm_;

    // If target isn't currently found OR it hasn't been seen recently, stop base
    const double lost_timeout = this->get_parameter("lost_timeout_sec").as_double();
    const bool recently_seen = (this->now() - last_seen_time_).seconds() <= lost_timeout;

    if (!target_found_ || !recently_seen) {
      // No target: stop base motion. Keep servos where they are (stable).
      publish_stop();
      return;
    }

    const double inner_zone = this->get_parameter("inner_zone").as_double();
    const double outer_zone = this->get_parameter("outer_zone").as_double();

    const double ax = std::abs(ex_norm_);
    const double ay = std::abs(ey_norm_);

    // Decide: base turn or servo only
    const bool in_edge_zone = (ax >= outer_zone);

    if (in_edge_zone) {
      // --- Base turning mode (slow)
      // Turn direction: if target is to the right (ex_norm > 0),
      // we want the robot to rotate right so target moves toward center.
      // Depending on your robot, you may need to flip this sign.
      double wz = -this->get_parameter("base_turn_kp").as_double() * ex_norm_;
      const double wz_max = this->get_parameter("base_turn_max").as_double();
      wz = clamp_double(wz, -wz_max, wz_max);

      // Ensure it actually turns (avoid tiny values)
      const double wz_min = this->get_parameter("base_turn_min").as_double();
      if (std::abs(wz) < wz_min) {
        wz = (wz >= 0.0) ? wz_min : -wz_min;
      }
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "BASE MODE: ex_norm=%.2f wz=%.3f servo1=%d",
        ex_norm_, wz, servo1_);
      publish_turn(wz);

      // While base turns, gently pull servo1 back toward center (prevents hitting servo limits)
      const int s1_center = this->get_parameter("servo1_center").as_int();
      const double pull = 1.0;  // 1 degree per tick toward center
      if (servo1_ > s1_center) servo1_ -= static_cast<int>(pull);
      if (servo1_ < s1_center) servo1_ += static_cast<int>(pull);

      // (Optional) you can also keep tilt stable here
      publish_servos(servo1_, servo2_);
      return;
    }

    // --- Servo mode (no base motion)
    publish_stop();

    // Inner deadzone: don't jitter
    if (ax < inner_zone && ay < inner_zone) {
      return;
    }

    // Convert normalized error into servo updates
    // If target is to the right (ex_norm > 0), pan right (servo1 increases or decreases?)
    // Your system: servo1 pans left/right. If it moves the wrong way, flip the sign here.
    const double servo_kp = this->get_parameter("servo_kp").as_double();
    double s1_step = servo_kp * (raw_ex_);  // proportional to pixel error
    double s2_step = servo_kp * (raw_ey_);  // proportional to pixel error (we'll flip below)

    // Clamp step size for smoothness
    const double max_step = this->get_parameter("servo_max_step_deg").as_double();
    const double ax_norm = std::abs(ex_norm_); // 0..1
const double gain = clamp_double(ax_norm, 0.1, 1.0); // never zero, but smaller near center
s1_step = clamp_double(s1_step * gain, -max_step, max_step);
    s1_step = clamp_double(s1_step, -max_step, max_step);

    // Tilt direction is usually inverted relative to image y (down is positive).
    // If your tilt moves wrong way, flip sign.
    s2_step = clamp_double(-s2_step, -max_step, max_step);
    RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "SERVO MODE: raw_ex=%.1f s1_step=%.2f servo1(before)=%d",
    raw_ex_, s1_step, servo1_);

    servo1_ += static_cast<int>(std::round(s1_step));
    servo2_ += static_cast<int>(std::round(s2_step));

    const int s1_min = this->get_parameter("servo1_min").as_int();
    const int s1_max = this->get_parameter("servo1_max").as_int();
    const int s2_min = this->get_parameter("servo2_min").as_int();
    const int s2_max = this->get_parameter("servo2_max").as_int();

    servo1_ = static_cast<int>(clamp_double(servo1_, s1_min, s1_max));
    servo2_ = static_cast<int>(clamp_double(servo2_, s2_min, s2_max));

    publish_servos(servo1_, servo2_);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_servo1_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_servo2_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_found_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_ex_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_ey_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Tracker state
  bool target_found_{false};
  bool have_ex_{false};
  bool have_ey_{false};
  float raw_ex_{0.0f};
  float raw_ey_{0.0f};
  rclcpp::Time last_seen_time_{0, 0, RCL_ROS_TIME};

  // Smoothed normalized errors
  double ex_norm_{0.0};
  double ey_norm_{0.0};

  // Servo state
  int servo1_{0};
  int servo2_{10};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PtzBaseController>());
  rclcpp::shutdown();
  return 0;
}