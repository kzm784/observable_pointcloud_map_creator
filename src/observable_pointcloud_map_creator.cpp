#include "observable_pointcloud_map_creator/observable_pointcloud_map_creator.hpp"

ObservablePointcloudMapCreator::ObservablePointcloudMapCreator(const rclcpp::NodeOptions & options) : Node("observable_pointcloud_map_creater", options)
{
    // Parameters
    declare_parameter<std::string>("input_pcd_path", "");

    get_parameter("input_pcd_path", input_pcd_path_);

    RCLCPP_INFO(get_logger(), "input_pcd_path_: %s", input_pcd_path_.c_str());

    
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ObservablePointcloudMapCreator)