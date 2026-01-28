#include "terrain_analysis/terrain_processor.h"

namespace terrain_analysis
{

TerrainProcessor::TerrainProcessor() : Node("terrain_processor_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing Terrain Processor Node");

  // Parameters
  this->declare_parameter<std::string>("scan_topic", "/cloud_registered_body");
  this->declare_parameter<std::string>("terrain_topic", "/terrain_cloud");
  this->declare_parameter<double>("vehicle_height", 0.75);
  this->declare_parameter<double>("ceiling_height_threshold", 1.5);
  this->declare_parameter<std::string>("map_path", "");
  this->declare_parameter<std::string>("global_map.map_frame_id", "map");
  this->declare_parameter<double>("global_map.map_leaf_size", 0.1);

  // PMF Parameters
  this->declare_parameter<bool>("global_map.use_pmf", true);
  this->declare_parameter<double>("global_map.pmf_max_window_size", 6.0);
  this->declare_parameter<double>("global_map.pmf_slope", 2.5);
  this->declare_parameter<double>("global_map.pmf_initial_distance", 0.8);
  this->declare_parameter<double>("global_map.pmf_max_distance", 1.0);

  this->get_parameter("scan_topic", this->scan_topic_);
  this->get_parameter("terrain_topic", this->terrain_topic_);
  this->get_parameter("vehicle_height", this->vehicle_height_);
  this->get_parameter("ceiling_height_threshold", this->ceiling_height_threshold_);
  this->get_parameter("map_path", this->map_path_);
  this->get_parameter("global_map.map_frame_id", this->map_frame_id_);
  this->get_parameter("global_map.map_leaf_size", this->map_leaf_size_);
  this->get_parameter("global_map.use_pmf", this->use_pmf_);

  this->get_parameter("global_map.pmf_max_window_size", this->pmf_max_window_size_);
  this->get_parameter("global_map.pmf_slope", this->pmf_slope_);
  this->get_parameter("global_map.pmf_initial_distance", this->pmf_initial_distance_);
  this->get_parameter("global_map.pmf_max_distance", this->pmf_max_distance_);

  // Publishers
  rclcpp::QoS qos_profile(1);
  qos_profile.transient_local();

  this->pub_global_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", qos_profile);
  this->pub_terrain_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->terrain_topic_, 10);

  // Subscriber
  this->sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      this->scan_topic_, 10, std::bind(&TerrainProcessor::scanCallback, this, std::placeholders::_1));

  this->global_map_cloud_.reset(new pcl::PointCloud<PointType>());
}

TerrainProcessor::~TerrainProcessor() {}

void TerrainProcessor::start()
{
  this->loadAndFilterMap();
}

void TerrainProcessor::loadAndFilterMap()
{
  if (this->map_path_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No map path specified. Skipping static map load.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", this->map_path_.c_str());

  // Load as PointXYZ first
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_xyz(new pcl::PointCloud<pcl::PointXYZ>());

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(this->map_path_, *raw_map_xyz) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded %zu points (XYZ only).", raw_map_xyz->size());

  // Convert to PointXYZI
  pcl::PointCloud<PointType>::Ptr raw_map(new pcl::PointCloud<PointType>());
  raw_map->points.resize(raw_map_xyz->size());
  for (size_t i = 0; i < raw_map_xyz->size(); i++) {
    raw_map->points[i].x = raw_map_xyz->points[i].x;
    raw_map->points[i].y = raw_map_xyz->points[i].y;
    raw_map->points[i].z = raw_map_xyz->points[i].z;
    raw_map->points[i].intensity = 1.0f; // Default: Obstacle
  }
  raw_map->width = raw_map_xyz->width;
  raw_map->height = raw_map_xyz->height;
  raw_map->is_dense = raw_map_xyz->is_dense;

  // 1. Voxel Grid Filter
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

  // 2. Process Terrain (PMF)
  if (this->use_pmf_) {
    RCLCPP_INFO(this->get_logger(), "Applying PMF Ground Segmentation on Static Map...");
    pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
    pcl::ApproximateProgressiveMorphologicalFilter<PointType> pmf;
    pmf.setInputCloud(downsampled_map);
    pmf.setMaxWindowSize(this->pmf_max_window_size_);
    pmf.setSlope(this->pmf_slope_);
    pmf.setInitialDistance(this->pmf_initial_distance_);
    pmf.setMaxDistance(this->pmf_max_distance_);
    pmf.extract(ground_indices->indices);

    RCLCPP_INFO(this->get_logger(), "PMF complete. Found %zu ground points.", ground_indices->indices.size());

    // Mark Ground points as Safe (0.0)
    for (int index : ground_indices->indices) {
      downsampled_map->points[index].intensity = 0.0f;
    }
  }

  // 3. Publish Static Map
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

  // 1. Filter Ceiling (Simple Z-Threshold in Body Frame)
  // Assumes body frame z=0 is robot center. Ceiling is anything above vehicle_height.
  pcl::PointCloud<PointType>::Ptr filtered_scan(new pcl::PointCloud<PointType>());
  for (const auto& pt : scan->points) {
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
  output_msg.header = msg->header; // Keep original frame (base_link/body) and timestamp
  this->pub_terrain_cloud_->publish(output_msg);
}

} // namespace terrain_analysis

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<terrain_analysis::TerrainProcessor>();
  node->start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
