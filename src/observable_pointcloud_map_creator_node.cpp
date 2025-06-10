#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "observable_pointcloud_map_creator/observable_pointcloud_map_creator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ObservablePointcloudMapCreator>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
