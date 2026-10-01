#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "l1w_driver/command_map.hpp"
#include "l1w_driver/srv/set_wifi.hpp"
#include "zsibot_sdk/zsibot_api.h"  // never `using namespace zsibot`: float32_t clashes on aarch64

namespace l1w_driver
{
using Trigger = std_srvs::srv::Trigger;
using SetWifi = l1w_driver::srv::SetWifi;
using Clock = std::chrono::steady_clock;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
const std::array<float, 14> kNoButtons{};

/// Scale a velocity to a joystick value in [-1, 1].
float normalize(double v, double full_scale)
{
  if (full_scale < 1e-6) {
    return 0.0F;
  }
  return static_cast<float>(std::clamp(v / full_scale, -1.0, 1.0));
}

/// Default joint names in SDK order: legs [LF, RF, LR, RR] x joints [abad, hip, knee, foot].
std::vector<std::string> default_joint_names()
{
  std::vector<std::string> names;
  for (const char * leg : {"fbl", "far", "rbl", "rar"}) {
    for (const char * joint : {"abad", "hip", "knee", "foot"}) {
      names.push_back(std::string(leg) + "_" + joint + "_joint");
    }
  }
  return names;
}
}  // namespace

class L1wDriverNode : public rclcpp::Node
{
public:
  /// Declare parameters, connect to the SDK and set up all ROS interfaces.
  L1wDriverNode()
  : Node("l1w_driver_node")
  {
    const auto ip = declare_parameter<std::string>("robot_ip", "192.168.234.1");
    const auto send_port = declare_parameter<int>("send_port", 8081);
    const auto recv_port = declare_parameter<int>("recv_port", 8080);

    stick_vx_ = declare_parameter<double>("stick_max_vx", 1.0);
    stick_vy_ = declare_parameter<double>("stick_max_vy", 0.5);
    stick_wz_ = declare_parameter<double>("stick_max_wz", 1.5);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);

    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    const auto control_rate = declare_parameter<double>("control_frequency", 50.0);
    const auto fast_rate = declare_parameter<double>("fast_state_rate", 50.0);
    const auto slow_rate = declare_parameter<double>("slow_state_rate", 1.0);

    joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", default_joint_names());
    if (joint_names_.size() != 16) {
      RCLCPP_WARN(get_logger(), "joint_names must have 16 entries, using defaults");
      joint_names_ = default_joint_names();
    }

    exec_ = std::make_unique<zsibot::ZsibotExecutor>(
      zsibot::Role::ROLE_SDK, ip, static_cast<uint32_t>(send_port),
      static_cast<uint32_t>(recv_port));

    main_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    estop_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    setup_publishers();
    setup_cmd_vel();
    setup_command_services();
    setup_wifi_service();

    control_timer_ = make_timer(control_rate, [this] {control_loop();});
    fast_timer_ = make_timer(fast_rate, [this] {publish_fast();});
    slow_timer_ = make_timer(slow_rate, [this] {publish_slow();});

    RCLCPP_INFO(get_logger(), "l1w_driver -> %s:%d (recv :%d)", ip.c_str(), send_port, recv_port);
  }

  /// Zero the joystick before shutdown so the robot stops.
  ~L1wDriverNode() override
  {
    if (exec_ && exec_->IsConnected()) {
      exec_->SetRemote({0.F, 0.F, 0.F, 0.F}, kNoButtons);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));  // let the send thread flush
    }
  }

private:
  /// Create a wall timer at `rate_hz` in the main callback group.
  rclcpp::TimerBase::SharedPtr make_timer(double rate_hz, std::function<void()> cb)
  {
    return create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), cb, main_group_);
  }

  /// Create all state publishers.
  void setup_publishers()
  {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    rpy_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    world_vel_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("odom/world_velocity", 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    motor_temp_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("motor_temperature", 10);
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery", 10);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  }

  /// Subscribe to cmd_vel and cache the latest command.
  void setup_cmd_vel()
  {
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = main_group_;
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(cmd_mtx_);
        last_cmd_ = *msg;
        last_cmd_time_ = Clock::now();
        have_cmd_ = true;
      },
      opts);
  }

  /// Create one Trigger service per SDK command; emergency_stop gets its own callback group.
  void setup_command_services()
  {
    for (const auto & entry : command_map()) {
      const std::string name = entry.first;
      const bool is_estop = (name == "emergency_stop");
      command_srvs_.push_back(create_service<Trigger>(
        name,
        [this, name](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
          res->success = send_command(name);
          res->message = (res->success ? "sent: " : "rejected: ") + name;
        },
        rmw_qos_profile_services_default, is_estop ? estop_group_ : main_group_));
    }
  }

  /// Create the set_wifi service.
  void setup_wifi_service()
  {
    set_wifi_srv_ = create_service<SetWifi>(
      "set_wifi",
      [this](const std::shared_ptr<SetWifi::Request> req, std::shared_ptr<SetWifi::Response> res) {
        if (!exec_->IsConnected()) {
          res->success = false;
          res->message = "not connected";
          return;
        }
        zsibot::WifiInfo info;
        info.ssid = req->ssid;
        info.password = req->password;
        exec_->SetWifi(info);
        res->success = true;
        res->message = "sent, check diagnostics wifi_result_ok for the outcome";
      },
      rmw_qos_profile_services_default, main_group_);
  }

  /// Send a named command to the robot; returns false if unknown or disconnected.
  bool send_command(const std::string & name)
  {
    const auto & map = command_map();
    const auto it = map.find(name);
    if (it == map.end()) {
      RCLCPP_WARN(get_logger(), "unknown command '%s'", name.c_str());
      return false;
    }
    if (!exec_->IsConnected()) {
      RCLCPP_WARN(get_logger(), "robot not connected, dropping '%s'", name.c_str());
      return false;
    }
    if (name == "emergency_stop") {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      have_cmd_ = false;
    }
    exec_->SetCmd(it->second);
    RCLCPP_INFO(get_logger(), "cmd -> %s", name.c_str());
    return true;
  }

  /// Forward the latest cmd_vel as joystick input; send zero when it is stale.
  void control_loop()
  {
    if (!exec_->IsConnected()) {
      return;
    }
    std::array<float, 4> joy{0.F, 0.F, 0.F, 0.F};  // [forward, yaw, lateral, head]
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      const double age = std::chrono::duration<double>(Clock::now() - last_cmd_time_).count();
      if (have_cmd_ && age < cmd_timeout_) {
        joy[0] = normalize(last_cmd_.linear.x, stick_vx_);
        joy[1] = normalize(last_cmd_.angular.z, stick_wz_);
        joy[2] = normalize(last_cmd_.linear.y, stick_vy_);
      }
    }
    exec_->SetRemote(joy, kNoButtons);
  }

  /// Publish high-rate state: IMU, odometry, joints and motor temperatures.
  void publish_fast()
  {
    if (!exec_->IsConnected()) {
      return;
    }
    const auto stamp = get_clock()->now();
    const auto imu = publish_imu(stamp);
    publish_odom(stamp, imu.orientation);
    publish_joints(stamp);

    std_msgs::msg::Float32MultiArray temp;
    const auto motor_temp = exec_->GetMotorTemp();
    temp.data.assign(motor_temp.begin(), motor_temp.end());
    motor_temp_pub_->publish(temp);
  }

  /// Publish IMU and RPY; returns the IMU message for reuse.
  sensor_msgs::msg::Imu publish_imu(const rclcpp::Time & stamp)
  {
    const auto q = exec_->GetQuaternion();  // [w, x, y, z]
    const auto rpy = exec_->GetRPY();
    const auto gyro = exec_->GetBodyGyro();
    const auto acc = exec_->GetBodyAcc();

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = base_frame_;
    imu.orientation.w = q[0];
    imu.orientation.x = q[1];
    imu.orientation.y = q[2];
    imu.orientation.z = q[3];
    imu.angular_velocity.x = gyro[0];
    imu.angular_velocity.y = gyro[1];
    imu.angular_velocity.z = gyro[2];
    imu.linear_acceleration.x = acc[0];
    imu.linear_acceleration.y = acc[1];
    imu.linear_acceleration.z = acc[2];
    imu_pub_->publish(imu);

    geometry_msgs::msg::Vector3Stamped rpy_msg;
    rpy_msg.header = imu.header;
    rpy_msg.vector.x = rpy[0];
    rpy_msg.vector.y = rpy[1];
    rpy_msg.vector.z = rpy[2];
    rpy_pub_->publish(rpy_msg);
    return imu;
  }

  /// Publish SDK odometry (no TF) and world-frame velocity.
  void publish_odom(const rclcpp::Time & stamp, const geometry_msgs::msg::Quaternion & orientation)
  {
    const auto pos = exec_->GetPosition();
    const auto body_vel = exec_->GetBodyVelocity();
    const auto world_vel = exec_->GetWorldVelocity();
    const auto gyro = exec_->GetBodyGyro();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pos[0];
    odom.pose.pose.position.y = pos[1];
    odom.pose.pose.position.z = pos[2];
    odom.pose.pose.orientation = orientation;
    odom.twist.twist.linear.x = body_vel[0];
    odom.twist.twist.linear.y = body_vel[1];
    odom.twist.twist.linear.z = body_vel[2];
    odom.twist.twist.angular.z = gyro[2];
    odom_pub_->publish(odom);

    geometry_msgs::msg::Vector3Stamped wv;
    wv.header.stamp = stamp;
    wv.header.frame_id = odom_frame_;
    wv.vector.x = world_vel[0];
    wv.vector.y = world_vel[1];
    wv.vector.z = world_vel[2];
    world_vel_pub_->publish(wv);
  }

  /// Publish joint states; SDK arrays are per joint type, each [LF, RF, LR, RR].
  void publish_joints(const rclcpp::Time & stamp)
  {
    const std::array<std::array<float, 4>, 4> pos{
      exec_->GetLegAbadJoint(), exec_->GetLegHipJoint(),
      exec_->GetLegKneeJoint(), exec_->GetLegFootJoint()};
    const std::array<std::array<float, 4>, 4> vel{
      exec_->GetLegAbadJointVel(), exec_->GetLegHipJointVel(),
      exec_->GetLegKneeJointVel(), exec_->GetLegFootJointVel()};
    const std::array<std::array<float, 4>, 4> eff{
      exec_->GetLegAbadJointTorque(), exec_->GetLegHipJointTorque(),
      exec_->GetLegKneeJointTorque(), exec_->GetLegFootJointTorque()};

    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = joint_names_;
    for (size_t leg = 0; leg < 4; ++leg) {
      for (size_t j = 0; j < 4; ++j) {
        js.position.push_back(pos[j][leg]);
        js.velocity.push_back(vel[j][leg]);
        js.effort.push_back(eff[j][leg]);
      }
    }
    joint_pub_->publish(js);
  }

  /// Publish low-rate state: diagnostics and battery.
  void publish_slow()
  {
    DiagStatus st;
    st.name = "l1w_driver";
    const bool connected = exec_->IsConnected();
    add_kv(st, "connected", connected ? "true" : "false");

    if (!connected) {
      st.level = DiagStatus::ERROR;
      st.message = "disconnected";
    } else {
      fill_diagnostics(st);
      publish_battery(st);
    }

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = get_clock()->now();
    arr.status.push_back(st);
    diag_pub_->publish(arr);
  }

  /// Fill robot mode, version, speed, wifi and fault info into a diagnostic status.
  void fill_diagnostics(DiagStatus & st)
  {
    const auto ver = exec_->GetVersion();
    const auto speed = exec_->GetSpeed();
    const auto faults = exec_->GetFaultInfo();
    const auto wifi_ok = exec_->GetWifiResutlf();  // sic, SDK typo

    st.hardware_id = exec_->GetSn();
    add_kv(st, "dev_name", exec_->GetDevName());
    add_kv(st, "wifi_ssid", exec_->GetWifiSsid());
    add_kv(st, "function_mode", function_mode_name(exec_->GetFunctionMode()));
    add_kv(st, "control_mode", control_mode_name(exec_->GetControlMode()));
    add_kv(st, "motion_mode", motion_mode_name(exec_->GetMotionMode()));
    add_kv(st, "motion_type", std::to_string(static_cast<int>(exec_->GetMotionType())));
    add_kv(st, "speed_level", speed_level_name(exec_->GetSpeedLevel()));
    add_kv(st, "model", model_name(exec_->GetModel()));
    add_kv(st, "mc_version", ver.mc_version);
    add_kv(st, "dog_task_version", ver.dog_task_version);
    add_kv(st, "temperature_c", std::to_string(exec_->GetTemperature()));
    add_kv(st, "speed_forward", std::to_string(speed.speed));
    add_kv(st, "speed_angle", std::to_string(speed.angle_speed));
    add_kv(st, "speed_shift", std::to_string(speed.shift_speed));
    add_kv(st, "head_angle", std::to_string(speed.angle));
    add_kv(st, "wifi_result_received", exec_->IsRecvWifiResult() ? "true" : "false");
    add_kv(st, "wifi_result_ok", wifi_ok.has_value() ? (*wifi_ok ? "true" : "false") : "n/a");

    for (size_t i = 0; i < faults.size(); ++i) {
      const auto & f = faults[i];
      add_kv(
        st, "fault_" + std::to_string(i),
        f.module + "/" + f.submodule + " code=" + std::to_string(f.error_code) +
        " [" + f.level + "] " + f.info);
    }
    st.level = faults.empty() ? DiagStatus::OK : DiagStatus::WARN;
    st.message = faults.empty() ? "ok" : std::to_string(faults.size()) + " fault(s)";
  }

  /// Publish battery state and append its error code to diagnostics.
  void publish_battery(DiagStatus & st)
  {
    const auto b = exec_->GetBatteryInfo();
    sensor_msgs::msg::BatteryState bs;
    bs.header.stamp = get_clock()->now();
    bs.voltage = b.volt;
    bs.current = b.current;
    bs.temperature = b.temp;
    bs.percentage = static_cast<float>(b.power) / 100.0F;
    bs.present = true;
    battery_pub_->publish(bs);
    add_kv(st, "battery_error", std::to_string(b.error));
  }

  /// Append a key/value pair to a diagnostic status.
  static void add_kv(DiagStatus & st, const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    st.values.push_back(kv);
  }

  std::unique_ptr<zsibot::ZsibotExecutor> exec_;

  std::string odom_frame_;
  std::string base_frame_;
  double stick_vx_{1.0};
  double stick_vy_{0.5};
  double stick_wz_{1.5};
  double cmd_timeout_{0.5};
  std::vector<std::string> joint_names_;

  std::mutex cmd_mtx_;
  geometry_msgs::msg::Twist last_cmd_;
  Clock::time_point last_cmd_time_{};
  bool have_cmd_{false};

  rclcpp::CallbackGroup::SharedPtr main_group_;
  rclcpp::CallbackGroup::SharedPtr estop_group_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr world_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> command_srvs_;
  rclcpp::Service<SetWifi>::SharedPtr set_wifi_srv_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr fast_timer_;
  rclcpp::TimerBase::SharedPtr slow_timer_;
};
}  // namespace l1w_driver

/// Spin the driver on a 2-thread executor so emergency_stop is never blocked.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<l1w_driver::L1wDriverNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
