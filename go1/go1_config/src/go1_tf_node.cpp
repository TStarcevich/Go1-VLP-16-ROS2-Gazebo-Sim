#include "go1_tf.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GO1TFbroadcaster>());
  rclcpp::shutdown();
  return 0;
}
