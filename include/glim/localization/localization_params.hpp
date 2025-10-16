#pragma once

#include <string>
#include <Eigen/Core>

namespace glim {

/**
 * @brief Base parameters for localization modules
 */
struct LocalizationParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocalizationParams();
  virtual ~LocalizationParams();

  // Matching strategy parameters
  double scan_to_map_threshold =
    0.7;  ///< Minimum score for scan-to-map matching
  double scan_to_scan_threshold =
    0.5;  ///< Minimum score for scan-to-scan matching

  // Correspondence parameters
  double max_correspondence_distance =
    1.0;  ///< Maximum distance for point correspondences
  int min_correspondences =
    100;  ///< Minimum number of correspondences required

  // Quality evaluation weights
  double w_overlap     = 0.4;  ///< Weight for overlap ratio
  double w_residual    = 0.3;  ///< Weight for residual score
  double w_consistency = 0.2;  ///< Weight for temporal consistency
  double w_geometric   = 0.1;  ///< Weight for geometric consistency

  // Graph optimization parameters
  double regularization_weight = 1.0;  ///< Regularization weight for pose jumps
  int    sliding_window_size = 50;  ///< Size of sliding window for optimization

  // Adaptive noise modeling
  double min_translation_noise = 0.01;   ///< Minimum translation noise [m]
  double max_translation_noise = 0.5;    ///< Maximum translation noise [m]
  double min_rotation_noise    = 0.001;  ///< Minimum rotation noise [rad]
  double max_rotation_noise    = 0.1;    ///< Maximum rotation noise [rad]

  // Threading
  int num_threads = 4;  ///< Number of threads for parallel processing

  // Map parameters
  std::string map_file_path;  ///< Path to global map file
  double      map_voxel_resolution =
    0.5;  ///< Voxel resolution for map representation

  // Initialization parameters
  double initial_pose_uncertainty = 1.0;  ///< Initial pose uncertainty [m]
  int    initialization_frames    = 5;  ///< Number of frames for initialization
};

}  // namespace glim
