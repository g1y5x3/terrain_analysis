#include "terrain_analysis/terrain_processor.h"

namespace terrain_analysis
{

TerrainProcessor::TerrainProcessor() : Node("terrain_processor_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing Terrain Processor Node");

  // Parameters
  this->declare_parameter<std::string>("map_path", "");
  this->declare_parameter<std::string>("global_cloud.map_frame_id", "map");
  this->declare_parameter<double>("global_cloud.map_leaf_size", 0.1);
  this->declare_parameter<bool>("terrain_cloud.use_pmf", true);
  this->declare_parameter<double>("terrain_cloud.pmf_max_window_size", 6.0);
  this->declare_parameter<double>("terrain_cloud.pmf_slope", 2.5);
  this->declare_parameter<double>("terrain_cloud.pmf_initial_distance", 0.5);
  this->declare_parameter<double>("terrain_cloud.pmf_max_distance", 1.0);
  this->declare_parameter<double>("terrain_cloud.ceiling_height_threshold", 1.5);
  this->declare_parameter<bool>("terrain_cloud.filter_operator_fov", false);
  this->declare_parameter<double>("terrain_cloud.operator_fov_deg", 60.0);

  this->get_parameter("map_path", this->map_path_);
  this->get_parameter("global_cloud.map_frame_id", this->map_frame_id_);
  this->get_parameter("global_cloud.map_leaf_size", this->map_leaf_size_);
  this->get_parameter("terrain_cloud.use_pmf", this->use_pmf_);
  this->get_parameter("terrain_cloud.pmf_max_window_size", this->pmf_max_window_size_);
  this->get_parameter("terrain_cloud.pmf_slope", this->pmf_slope_);
  this->get_parameter("terrain_cloud.pmf_initial_distance", this->pmf_initial_distance_);
  this->get_parameter("terrain_cloud.pmf_max_distance", this->pmf_max_distance_);
  this->get_parameter("terrain_cloud.ceiling_height_threshold", this->ceiling_height_threshold_);
  this->get_parameter("terrain_cloud.filter_operator_fov", this->filter_operator_fov_);
  this->get_parameter("terrain_cloud.operator_fov_deg", this->operator_fov_deg_);

  // Publishers
  rclcpp::QoS qos_profile(1);
  qos_profile.transient_local();
  this->pub_global_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", qos_profile);
  this->pub_terrain_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_cloud", 10);

  // Subscriber
  this->sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/scan_cloud", 10, std::bind(&TerrainProcessor::scanCallback, this, std::placeholders::_1));

  // Initialization
  this->global_map_cloud_.reset(new pcl::PointCloud<PointType>());
  this->loadAndFilterMap();
}

TerrainProcessor::~TerrainProcessor() {}

void TerrainProcessor::loadAndFilterMap()
{
  if (this->map_path_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No map path specified. Skipping static map load.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", this->map_path_.c_str());

  pcl::PointCloud<PointType>::Ptr raw_map(new pcl::PointCloud<PointType>());

  if (pcl::io::loadPCDFile<PointType>(this->map_path_, *raw_map) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded %zu points.", raw_map->size());

  // voxel grid filter
  pcl::PointCloud<PointType>::Ptr downsampled_map(new pcl::PointCloud<PointType>());
  if (this->map_leaf_size_ > 0.0) {
    RCLCPP_INFO(this->get_logger(), "Downsampling map (leaf size: %.2f)...", this->map_leaf_size_);
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setLeafSize(this->map_leaf_size_, this->map_leaf_size_, this->map_leaf_size_);
    voxel_grid.setInputCloud(raw_map);
    voxel_grid.filter(*downsampled_map);
  } else {
    downsampled_map = raw_map;
  }
  RCLCPP_INFO(this->get_logger(), "Map size after downsampling: %zu", downsampled_map->size());

  sensor_msgs::msg::PointCloud2 full_msg;
  pcl::toROSMsg(*downsampled_map, full_msg);
  full_msg.header.frame_id = this->map_frame_id_;
  full_msg.header.stamp = this->now();
  this->pub_global_map_->publish(full_msg); // For NDT / Visualization

  RCLCPP_INFO(this->get_logger(), "Published static map.");
}

void TerrainProcessor::scanCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  pcl::PointCloud<PointType>::Ptr scan(new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*msg, *scan);

  if (scan->empty()) return;

  double fov_rad_threshold = (180.0 - this->operator_fov_deg_ / 2.0) * M_PI / 180.0;

  // 1. Filter Ceiling (Simple Z-Threshold in Body Frame) and Operator FOV
  pcl::PointCloud<PointType>::Ptr filtered_scan(new pcl::PointCloud<PointType>());
  for (const auto& pt : scan->points) {
    // Operator FOV Filter
    if (this->filter_operator_fov_) {
       double angle = std::atan2(pt.y, pt.x);
       if (std::abs(angle) > fov_rad_threshold) continue;
    }

    if (pt.z < this->ceiling_height_threshold_) { 
      PointType p = pt;
      p.intensity = 1.0f; // Default to obstacle
      filtered_scan->points.push_back(p);
    }
  }
  filtered_scan->width = filtered_scan->points.size();
  filtered_scan->height = 1;
  filtered_scan->is_dense = true;

  // 2. Ground Segmentation (PMF on local scan)
  if (this->use_pmf_ && !filtered_scan->empty()) {
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
    pcl::ApproximateProgressiveMorphologicalFilter<PointType> pmf;
    pmf.setInputCloud(filtered_scan);
    pmf.setMaxWindowSize(this->pmf_max_window_size_); 
    pmf.setSlope(this->pmf_slope_);
    pmf.setInitialDistance(this->pmf_initial_distance_);
    pmf.setMaxDistance(this->pmf_max_distance_);
    pmf.extract(ground_indices->indices);

    for (int index : ground_indices->indices) {
      filtered_scan->points[index].intensity = 0.0f; // Ground
    }
  }

  // 3. Publish to Terrain Cloud
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*filtered_scan, output_msg);
  output_msg.header = msg->header;
  this->pub_terrain_cloud_->publish(output_msg);
}

} // namespace terrain_analysis

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<terrain_analysis::TerrainProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
