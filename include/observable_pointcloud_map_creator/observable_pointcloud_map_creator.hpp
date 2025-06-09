#ifndef OBSERVABLE_POINTCLOUD_MAP_CREATER__OBSERVABLE_POINTCLOUD_MAP_CREATER_HPP_
#define OBSERVABLE_POINTCLOUD_MAP_CREATER__OBSERVABLE_POINTCLOUD_MAP_CREATER_HPP_

#include <rclcpp/rclcpp.hpp>

class ObservablePointcloudMapCreator : public rclcpp::Node
{
public:
  explicit ObservablePointcloudMapCreator(const rclcpp::NodeOptions & options);

private:
  std::string input_pcd_path_;

};

#endif  // OBSERVABLE_POINTCLOUD_MAP_CREATER__OBSERVABLE_POINTCLOUD_MAP_CREATER_HPP_