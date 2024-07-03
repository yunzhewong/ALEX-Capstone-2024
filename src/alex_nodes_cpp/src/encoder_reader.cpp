#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using sensor_msgs::msg::JointState;
using std::placeholders::_1;
using std_msgs::msg::Float64MultiArray;

bool doubleEqual(double d1, double d2) { return std::abs(d1 - d2) < 1e-9; }

class MinimalSubscriber : public rclcpp::Node {
 private:
  rclcpp::Subscription<JointState>::SharedPtr joint_sub;
  rclcpp::Subscription<Float64MultiArray>::SharedPtr torque_sub;
  std::ofstream file;
  double starting_time;
  double torque;
  bool torque_read;

 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    joint_sub = this->create_subscription<JointState>(
        "/joint_states", 10,
        std::bind(&MinimalSubscriber::on_joint_msg, this, _1));

    torque_sub = this->create_subscription<Float64MultiArray>(
        "/bottom_motor_controller/commands", 10,
        std::bind(&MinimalSubscriber::on_torque, this, _1));

    file.open("data.csv");
    file << "Times (s), Currents (A), Velocities (rads^-1), Positions(rads)"
         << '\n';

    starting_time = -1;
    torque = 0;
    torque_read = false;
  }
  ~MinimalSubscriber() { file.close(); }

 private:
  void on_joint_msg(const JointState &msg) {
    if (!torque_read) {
      return;
    }
    auto timestamp = msg.header.stamp;
    double time = timestamp.sec + (timestamp.nanosec / 1e9);
    if (doubleEqual(starting_time, -1.0)) {
      starting_time = time;
    }
    double relative_time = time - starting_time;
    file << relative_time << ',' << torque << ',' << msg.velocity[0] << ','
         << msg.position[0] << '\n';
  }

  void on_torque(const Float64MultiArray &msg) {
    torque_read = true;
    torque = msg.data[0];
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}