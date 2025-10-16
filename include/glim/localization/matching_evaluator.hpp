#pragma once

#include <memory>
#include <vector>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/NoiseModel.h>
#include <glim/localization/localization_frame.hpp>
#include <glim/localization/localization_params.hpp>
#include <glim/localization/map_manager.hpp>
#include <glim/preprocess/preprocessed_frame.hpp>
#include <spdlog/spdlog.h>

namespace glim {

/**
 * @brief Evaluator for matching quality assessment
 */
class MatchingEvaluator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit MatchingEvaluator(const LocalizationParams& params);
  ~MatchingEvaluator();

  /**
   * @brief Evaluate overall matching quality using multi-criteria assessment
   * @param result Matching result to evaluate
   * @param current_frame Current preprocessed frame
   * @param global_map Global map (for scan-to-map evaluation)
   * @param recent_frames Recent frames for temporal consistency
   * @return Overall quality score [0.0, 1.0]
   */
  double evaluate_matching_quality(
    const MatchingResult&                          result,
    const PreprocessedFrame::Ptr&                  current_frame,
    const GlobalMap::ConstPtr&                     global_map,
    const std::deque<LocalizationFrame::ConstPtr>& recent_frames);

  /**
   * @brief Create adaptive noise model based on matching quality
   * @param matching_result Matching result with confidence score
   * @param params Localization parameters
   * @return Noise model for graph optimization
   */
  gtsam::noiseModel::Base::shared_ptr create_adaptive_noise_model(
    const MatchingResult&     matching_result,
    const LocalizationParams& params);

  /**
   * @brief Evaluate scan-to-scan matching quality
   * @param current_frame Current frame
   * @param previous_frame Previous frame
   * @param result Matching result
   * @return Quality score [0.0, 1.0]
   */
  double evaluate_scan_to_scan_quality(
    const PreprocessedFrame::Ptr&      current_frame,
    const LocalizationFrame::ConstPtr& previous_frame,
    const MatchingResult&              result);

  /**
   * @brief Evaluate scan-to-map matching quality
   * @param current_frame Current frame
   * @param global_map Global map
   * @param result Matching result
   * @return Quality score [0.0, 1.0]
   */
  double evaluate_scan_to_map_quality(
    const PreprocessedFrame::Ptr& current_frame,
    const GlobalMap::ConstPtr&    global_map,
    const MatchingResult&         result);

private:
  /**
   * @brief Calculate overlap ratio between current frame and map
   * @param result Matching result
   * @param current_frame Current frame
   * @param global_map Global map
   * @return Overlap ratio [0.0, 1.0]
   */
  double calculate_overlap_ratio(const MatchingResult&         result,
                                 const PreprocessedFrame::Ptr& current_frame,
                                 const GlobalMap::ConstPtr&    global_map);

  /**
   * @brief Calculate residual score from matching result
   * @param result Matching result
   * @return Residual score [0.0, 1.0]
   */
  double calculate_residual_score(const MatchingResult& result);

  /**
   * @brief Calculate temporal consistency with recent frames
   * @param result Current matching result
   * @param recent_frames Recent localization frames
   * @return Consistency score [0.0, 1.0]
   */
  double calculate_temporal_consistency(
    const MatchingResult&                          result,
    const std::deque<LocalizationFrame::ConstPtr>& recent_frames);

  /**
   * @brief Calculate geometric consistency of the result
   * @param result Matching result
   * @return Geometric consistency score [0.0, 1.0]
   */
  double calculate_geometric_consistency(const MatchingResult& result);

private:
  LocalizationParams              params_;
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim