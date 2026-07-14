#include "TeleopWbcNode.hpp"

#include <memory>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopWbcNode>());
  rclcpp::shutdown();
  return 0;
}
