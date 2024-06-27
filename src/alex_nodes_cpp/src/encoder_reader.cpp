#include <memory>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::ofstream file;

  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      file.open("data.csv");
      file << "Times (s), Currents (A), Velocities (rads^-1), Positions(rads)" << '\n';
    }
    ~MinimalSubscriber() {
        file.close();
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) {
        file << msg.data << '\n';
    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}