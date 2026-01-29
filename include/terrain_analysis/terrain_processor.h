#pragma once

#include <mutex>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>

// Define the point type
using PointType = pcl::PointXYZI;

namespace terrain_analysis
{
class TerrainProcessor : public rclcpp::Node
{
public:
  TerrainProcessor();
  ~TerrainProcessor();

  void start();

private:
  void loadAndFilterMap();
  void scanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // Parameters
  std::string map_path_;
  std::string map_frame_id_;
  // std::string scan_topic_;
  // std::string terrain_topic_;
  double map_leaf_size_;

  // PMF Parameters
  bool use_pmf_; 
  double pmf_max_window_size_;
  double pmf_slope_;
  double pmf_initial_distance_;
  double pmf_max_distance_;

  // Ceiling Filter Parameter
  double ceiling_height_threshold_;

  // Operator FOV Filter Parameters
  bool filter_operator_fov_;
  double operator_fov_deg_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_map_; // For visual/debug/NDT
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_terrain_cloud_; // For far_planner

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;

  // Data
  pcl::PointCloud<PointType>::Ptr global_map_cloud_;
};
} // namespace terrain_analysis
