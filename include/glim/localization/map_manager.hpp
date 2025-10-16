#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/ann/incremental_voxelmap.hpp>

namespace gtsam_points {
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;

class IncrementalCovarianceVoxelMap;
using iVoxCovarianceEstimation = IncrementalCovarianceVoxelMap;
class IncrementalFixedLagSmootherExt;
class IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_points

namespace glim {

class SubMap;

/**
 * @brief Global map representation for localization
 */
struct GlobalMap {
public:
  using Ptr      = std::shared_ptr<GlobalMap>;
  using ConstPtr = std::shared_ptr<const GlobalMap>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Map data
  gtsam_points::PointCloud::Ptr       global_point_cloud;
  std::shared_ptr<gtsam_points::iVox> global_ivox;

  // Map metadata
  std::string     map_name;
  std::string     map_version;
  double          creation_time;
  Eigen::Vector3d map_bounds_min;
  Eigen::Vector3d map_bounds_max;

  // Keyframes used to create the map
  std::vector<gtsam_points::PointCloud::ConstPtr> keyframes;
  std::vector<Eigen::Isometry3d>                  keyframe_poses;

  // Spatial indexing
  double voxel_resolution = 0.5;
  int    total_points     = 0;
};

/**
 * @brief Manager for global map operations
 */
class MapManager {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MapManager();
  ~MapManager();

  /**
   * @brief Load map from PCD file
   * @param pcd_file_path Path to PCD file
   * @param voxel_resolution Voxel resolution for map representation
   * @return True if loaded successfully
   */
  bool load_map_from_pcd(const std::string& pcd_file_path,
                         double             voxel_resolution = 0.5);

  /**
   * @brief Load map from PLY file
   * @param ply_file_path Path to PLY file
   * @param voxel_resolution Voxel resolution for map representation
   * @return True if loaded successfully
   */
  bool load_map_from_ply(const std::string& ply_file_path,
                         double             voxel_resolution = 0.5);

  /**
   * @brief Load map from GLIM submaps
   * @param submap_directory Directory containing submap files
   * @param voxel_resolution Voxel resolution for map representation
   * @return True if loaded successfully
   */
  bool load_map_from_submaps(const std::string& submap_directory,
                             double             voxel_resolution = 0.5);

  /**
   * @brief Create map from point clouds and poses
   * @param point_clouds Vector of point clouds
   * @param poses Corresponding poses for each point cloud
   * @param voxel_resolution Voxel resolution for map representation
   * @return True if created successfully
   */
  bool create_map_from_clouds(
    const std::vector<gtsam_points::PointCloud::ConstPtr>& point_clouds,
    const std::vector<Eigen::Isometry3d>&                  poses,
    double voxel_resolution = 0.5);

  /**
   * @brief Get global map
   * @return Pointer to global map
   */
  GlobalMap::ConstPtr get_global_map() const {
    return global_map_;
  }

  /**
   * @brief Get local map region around a pose
   * @param center_pose Center pose for local region
   * @param radius Radius of local region [m]
   * @return Local point cloud
   */
  gtsam_points::PointCloud::Ptr get_local_map_region(
    const Eigen::Isometry3d& center_pose,
    double                   radius = 50.0) const;

  /**
   * @brief Check if map is loaded
   * @return True if map is available
   */
  bool is_map_loaded() const {
    return global_map_ != nullptr;
  }

  /**
   * @brief Get map statistics
   */
  void print_map_statistics() const;

  /**
   * @brief Save current map to file
   * @param output_path Output file path
   * @return True if saved successfully
   */
  bool save_map(const std::string& output_path) const;

private:
  GlobalMap::Ptr global_map_;

  // Helper methods
  bool build_spatial_index(double voxel_resolution);
  void compute_map_bounds();
  bool validate_map_data() const;
};

}  // namespace glim
