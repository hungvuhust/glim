#include <glim/localization/map_manager.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <filesystem>
#include <fstream>
#include <spdlog/spdlog.h>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/ann/incremental_voxelmap.hpp>
#include <gtsam_points/ann/flat_container.hpp>

namespace glim {

MapManager::MapManager() = default;

MapManager::~MapManager() = default;

bool MapManager::load_map_from_pcd(const std::string& pcd_path,
                                   double             voxel_resolution) {
  auto logger = spdlog::get("glim_logger");
  if (!logger)
    logger = spdlog::default_logger();

  logger->info("Loading map from PCD file: {}", pcd_path);

  if (!std::filesystem::exists(pcd_path)) {
    logger->error("PCD file not found: {}", pcd_path);
    return false;
  }

  // Load PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
    logger->error("Failed to load PCD file: {}", pcd_path);
    return false;
  }

  logger->info("Loaded {} points from PCD file", cloud->size());

  // Convert to GLIM point cloud format
  std::vector<Eigen::Vector4d> points_vec;
  points_vec.reserve(cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    points_vec.emplace_back(cloud->points[i].x,
                            cloud->points[i].y,
                            cloud->points[i].z,
                            1.0);
  }

  auto glim_cloud = std::make_shared<gtsam_points::PointCloudCPU>(points_vec);

  // Create global map
  global_map_           = std::make_shared<GlobalMap>();
  global_map_->map_name = std::filesystem::path(pcd_path).stem().string();
  global_map_->voxel_resolution = voxel_resolution;
  global_map_->total_points     = cloud->size();

  // Create iVox map
  global_map_->global_ivox =
    std::make_shared<gtsam_points::iVox>(voxel_resolution);
  global_map_->global_ivox->insert(*glim_cloud);

  // Store original point cloud
  global_map_->global_point_cloud = glim_cloud;

  // Compute map bounds
  compute_map_bounds();

  logger->info("Map loaded successfully: {} points, {} voxels",
               cloud->size(),
               global_map_->global_ivox->num_voxels());

  return validate_map_data();
}

bool MapManager::load_map_from_submaps(const std::string& submaps_dir,
                                       double             voxel_resolution) {
  auto logger = spdlog::get("glim_logger");
  if (!logger)
    logger = spdlog::default_logger();

  logger->info("Loading map from submaps directory: {}", submaps_dir);

  if (!std::filesystem::exists(submaps_dir)) {
    logger->error("Submaps directory not found: {}", submaps_dir);
    return false;
  }

  // Find all PCD files in directory
  std::vector<std::string> pcd_files;
  for (const auto& entry : std::filesystem::directory_iterator(submaps_dir)) {
    if (entry.path().extension() == ".pcd") {
      pcd_files.push_back(entry.path().string());
    }
  }

  if (pcd_files.empty()) {
    logger->error("No PCD files found in directory: {}", submaps_dir);
    return false;
  }

  std::sort(pcd_files.begin(), pcd_files.end());
  logger->info("Found {} submap files", pcd_files.size());

  // Create global map
  global_map_ = std::make_shared<GlobalMap>();
  global_map_->map_name =
    std::filesystem::path(submaps_dir).filename().string();
  global_map_->voxel_resolution = voxel_resolution;
  global_map_->global_ivox =
    std::make_shared<gtsam_points::iVox>(voxel_resolution);

  // Combined point cloud vector
  std::vector<Eigen::Vector4d> combined_points;

  // Load and merge all submaps
  size_t total_points = 0;
  for (const auto& pcd_file : pcd_files) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) {
      logger->warn("Failed to load submap: {}", pcd_file);
      continue;
    }

    // Convert to GLIM format
    std::vector<Eigen::Vector4d> points_vec;
    points_vec.reserve(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
      Eigen::Vector4d pt(cloud->points[i].x,
                         cloud->points[i].y,
                         cloud->points[i].z,
                         1.0);
      points_vec.push_back(pt);
      combined_points.push_back(pt);
    }

    auto glim_cloud = std::make_shared<gtsam_points::PointCloudCPU>(points_vec);

    // Insert into global iVox
    global_map_->global_ivox->insert(*glim_cloud);
    total_points += cloud->size();

    // Store keyframe
    global_map_->keyframes.push_back(glim_cloud);
    global_map_->keyframe_poses.push_back(Eigen::Isometry3d::Identity());

    logger->debug("Loaded submap {}: {} points",
                  std::filesystem::path(pcd_file).filename().string(),
                  cloud->size());
  }

  global_map_->global_point_cloud =
    std::make_shared<gtsam_points::PointCloudCPU>(combined_points);
  global_map_->total_points = total_points;

  // Compute map bounds
  compute_map_bounds();

  logger->info("Map loaded from {} submaps: {} total points, {} voxels",
               pcd_files.size(),
               total_points,
               global_map_->global_ivox->num_voxels());

  return validate_map_data();
}

bool MapManager::create_map_from_clouds(
  const std::vector<gtsam_points::PointCloud::ConstPtr>& point_clouds,
  const std::vector<Eigen::Isometry3d>&                  poses,
  double                                                 voxel_resolution) {
  auto logger = spdlog::get("glim_logger");
  if (!logger)
    logger = spdlog::default_logger();

  if (point_clouds.size() != poses.size()) {
    logger->error("Point clouds and poses size mismatch: {} vs {}",
                  point_clouds.size(),
                  poses.size());
    return false;
  }

  logger->info("Creating map from {} point clouds", point_clouds.size());

  // Create global map
  global_map_                   = std::make_shared<GlobalMap>();
  global_map_->map_name         = "created_map";
  global_map_->voxel_resolution = voxel_resolution;
  global_map_->global_ivox =
    std::make_shared<gtsam_points::iVox>(voxel_resolution);

  std::vector<Eigen::Vector4d> combined_points;
  size_t                       total_points = 0;

  for (size_t i = 0; i < point_clouds.size(); ++i) {
    const auto& cloud = point_clouds[i];
    const auto& pose  = poses[i];

    // Transform point cloud to global frame
    std::vector<Eigen::Vector4d> transformed_points;
    transformed_points.reserve(cloud->size());

    for (size_t j = 0; j < cloud->size(); ++j) {
      Eigen::Vector4d global_point = pose * cloud->points[j];
      transformed_points.push_back(global_point);
      combined_points.push_back(global_point);
    }

    auto transformed_cloud =
      std::make_shared<gtsam_points::PointCloudCPU>(transformed_points);

    // Insert into iVox
    global_map_->global_ivox->insert(*transformed_cloud);

    // Store keyframe
    global_map_->keyframes.push_back(cloud);
    global_map_->keyframe_poses.push_back(pose);

    total_points += cloud->size();
  }

  global_map_->global_point_cloud =
    std::make_shared<gtsam_points::PointCloudCPU>(combined_points);
  global_map_->total_points = total_points;

  // Compute map bounds
  compute_map_bounds();

  logger->info("Map created: {} total points, {} voxels",
               total_points,
               global_map_->global_ivox->num_voxels());

  return validate_map_data();
}

gtsam_points::PointCloud::Ptr MapManager::get_local_map_region(
  const Eigen::Isometry3d& center_pose,
  double                   radius) const {
  if (!global_map_ || !global_map_->global_point_cloud) {
    return nullptr;
  }

  std::vector<Eigen::Vector4d> local_points;
  Eigen::Vector3d              center = center_pose.translation();

  for (size_t i = 0; i < global_map_->global_point_cloud->size(); ++i) {
    const Eigen::Vector4d& point = global_map_->global_point_cloud->points[i];
    Eigen::Vector3d        pt    = point.head<3>();
    if ((pt - center).norm() <= radius) {
      local_points.push_back(point);
    }
  }

  return std::make_shared<gtsam_points::PointCloudCPU>(local_points);
}

void MapManager::print_map_statistics() const {
  auto logger = spdlog::get("glim_logger");
  if (!logger)
    logger = spdlog::default_logger();

  if (!global_map_) {
    logger->info("No map loaded");
    return;
  }

  logger->info("=== Map Statistics ===");
  logger->info("Name: {}", global_map_->map_name);
  logger->info("Total points: {}", global_map_->total_points);
  logger->info("Voxel resolution: {:.3f}m", global_map_->voxel_resolution);
  logger->info("iVox voxels: {}",
               global_map_->global_ivox ? global_map_->global_ivox->num_voxels()
                                        : 0);
  logger->info("Keyframes: {}", global_map_->keyframes.size());
  logger->info(
    "Map bounds: [{:.2f}, {:.2f}, {:.2f}] to [{:.2f}, {:.2f}, {:.2f}]",
    global_map_->map_bounds_min.x(),
    global_map_->map_bounds_min.y(),
    global_map_->map_bounds_min.z(),
    global_map_->map_bounds_max.x(),
    global_map_->map_bounds_max.y(),
    global_map_->map_bounds_max.z());
}

bool MapManager::save_map(const std::string& output_path) const {
  auto logger = spdlog::get("glim_logger");
  if (!logger)
    logger = spdlog::default_logger();

  if (!global_map_ || !global_map_->global_point_cloud) {
    logger->error("No map to save");
    return false;
  }

  // Convert to PCL format
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl_cloud->points.reserve(global_map_->global_point_cloud->size());

  for (size_t i = 0; i < global_map_->global_point_cloud->size(); ++i) {
    const auto&   pt = global_map_->global_point_cloud->points[i];
    pcl::PointXYZ point;
    point.x = pt.x();
    point.y = pt.y();
    point.z = pt.z();
    pcl_cloud->points.push_back(point);
  }
  pcl_cloud->width  = pcl_cloud->points.size();
  pcl_cloud->height = 1;

  // Save as PCD
  if (pcl::io::savePCDFileBinary(output_path, *pcl_cloud) == -1) {
    logger->error("Failed to save map to: {}", output_path);
    return false;
  }

  logger->info("Map saved to: {}", output_path);
  return true;
}

bool MapManager::build_spatial_index(double voxel_resolution) {
  if (!global_map_ || !global_map_->global_point_cloud) {
    return false;
  }

  // iVox already provides spatial indexing
  global_map_->voxel_resolution = voxel_resolution;
  return true;
}

void MapManager::compute_map_bounds() {
  if (!global_map_ || !global_map_->global_point_cloud ||
      global_map_->global_point_cloud->size() == 0) {
    return;
  }

  Eigen::Vector3d min_bound =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector3d max_bound =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

  for (size_t i = 0; i < global_map_->global_point_cloud->size(); ++i) {
    Eigen::Vector3d pt = global_map_->global_point_cloud->points[i].head<3>();
    min_bound          = min_bound.cwiseMin(pt);
    max_bound          = max_bound.cwiseMax(pt);
  }

  global_map_->map_bounds_min = min_bound;
  global_map_->map_bounds_max = max_bound;
}

bool MapManager::validate_map_data() const {
  auto logger = spdlog::get("glim_logger");
  if (!logger)
    logger = spdlog::default_logger();

  if (!global_map_) {
    logger->error("No global map loaded");
    return false;
  }

  if (!global_map_->global_ivox) {
    logger->error("Global iVox map is null");
    return false;
  }

  if (global_map_->global_ivox->num_voxels() == 0) {
    logger->error("Global iVox map is empty");
    return false;
  }

  if (!global_map_->global_point_cloud ||
      global_map_->global_point_cloud->size() == 0) {
    logger->error("Global point cloud is empty");
    return false;
  }

  logger->info("Map validation passed: {} voxels, {} points",
               global_map_->global_ivox->num_voxels(),
               global_map_->global_point_cloud->size());
  return true;
}

}  // namespace glim