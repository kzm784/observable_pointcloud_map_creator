#ifndef OBSERVABLE_POINTCLOUD_MAP_CREATER__OBSERVABLE_POINTCLOUD_MAP_CREATER_HPP_
#define OBSERVABLE_POINTCLOUD_MAP_CREATER__OBSERVABLE_POINTCLOUD_MAP_CREATER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include <open3d/Open3D.h>

#include <Eigen/Core>

#include <filesystem>
#include <sstream>
#include <unordered_set>

class ObservablePointcloudMapCreator : public rclcpp::Node
{
public:
  explicit ObservablePointcloudMapCreator(const rclcpp::NodeOptions & options);

private:
  void splitAndProcessGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr processLocalGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr & local);

  pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_global_;

  std::string input_pcd_path_;
  std::string output_dir_;
  double grid_size_;
  double voxel_leaf_size_;
  double hpr_radius_;
};

#endif  // OBSERVABLE_POINTCLOUD_MAP_CREATER__OBSERVABLE_POINTCLOUD_MAP_CREATER_HPP_