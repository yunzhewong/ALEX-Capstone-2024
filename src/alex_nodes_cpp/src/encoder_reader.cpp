#include <memory>
#include <iostream>
#include <fstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
using sensor_msgs::msg::JointState;

bool doublesEqual(double d1, double d2) {
    return std::abs(d1 - d2) < 1e-9;
}

class MinimalSubscriber : public rclcpp::Node
{
  private:
    rclcpp::Subscription<JointState>::SharedPtr joint_sub;
    std::ofstream file;
    double starting_time;

  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      joint_sub = this->create_subscription<JointState>(
      "/joint_states", 10, std::bind(&MinimalSubscriber::on_joint_msg, this, _1));

      file.open("data.csv");
      file << "Times (s), Currents (A), Velocities (rads^-1), Positions(rads)" << '\n';
      
      starting_time = -1.0;
    }
    ~MinimalSubscriber() {
        file.close();
    }

  private:
    void on_joint_msg(const JointState & msg) {
        auto timestamp = msg.header.stamp;
        double time = timestamp.sec + (timestamp.nanosec / 1e9);
        if (doublesEqual(starting_time, -1.0)) {
            starting_time = time;
        }
        double relative_time = time - starting_time;
        file << relative_time << ',' << 0  << ',' << msg.velocity[0] << ',' << msg.position[0] << '\n';
    }
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}