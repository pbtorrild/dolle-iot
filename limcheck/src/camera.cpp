#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <recording.h>

int main(int argc, char * argv[])
{
  int cam_id=0;

  rclcpp::init(argc, argv);  
  rclcpp::spin(std::make_shared<dolle_iot::vision::record>(cam_id));
  rclcpp::shutdown();
}