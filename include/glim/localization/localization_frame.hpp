#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glim/odometry/estimation_frame.hpp>

namespace glim {

/**
 * @brief Matching type enumeration
 */
enum class MatchingType {
  SCAN_TO_SCAN,  ///< Scan-to-scan matching only
  SCAN_TO_MAP,   ///< Scan-to-map matching only
  HYBRID,        ///< Hybrid matching (combination)
  FAILED         ///< Matching failed
};

/**
 * @brief Result of matching operation
 */
struct MatchingResult {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Isometry3d           estimated_pose   = Eigen::Isometry3d::Identity();
  double                      confidence_score = 0.0;
  MatchingType                matching_type    = MatchingType::FAILED;
  Eigen::Matrix<double, 6, 6> information_matrix =
    Eigen::Matrix<double, 6, 6>::Identity();

  // Detailed matching information
  double overlap_ratio       = 0.0;
  double residual_error      = 0.0;
  int    num_correspondences = 0;
  double computation_time    = 0.0;

  // Quality metrics
  double scan_to_scan_score = 0.0;
  double scan_to_map_score  = 0.0;
};

/**
 * @brief Extended estimation frame for localization
 */
struct LocalizationFrame : public EstimationFrame {
public:
  using Ptr      = std::shared_ptr<LocalizationFrame>;
  using ConstPtr = std::shared_ptr<const LocalizationFrame>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Create a localization frame from estimation frame
   */
  static LocalizationFrame::Ptr from_estimation_frame(
    const EstimationFrame::ConstPtr& est_frame);

  // Localization-specific data
  MatchingResult matching_result;
  double         localization_confidence = 0.0;
  bool           is_keyframe             = false;

  // Timing information
  double processing_time = 0.0;
  double total_time      = 0.0;
};

}  // namespace glim
