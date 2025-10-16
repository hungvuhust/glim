#include <glim/localization/matching_evaluator.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/ann/flat_container.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <spdlog/spdlog.h>

namespace glim {

MatchingEvaluator::MatchingEvaluator(const LocalizationParams& params)
  : params_(params) {
  logger = spdlog::get("glim_logger");
  if (!logger) {
    logger = spdlog::default_logger();
  }
}

MatchingEvaluator::~MatchingEvaluator() = default;

double MatchingEvaluator::evaluate_matching_quality(
  const MatchingResult&                          result,
  const PreprocessedFrame::Ptr&                  current_frame,
  const GlobalMap::ConstPtr&                     global_map,
  const std::deque<LocalizationFrame::ConstPtr>& recent_frames) {
  // Multi-criteria evaluation
  double overlap_score =
    calculate_overlap_ratio(result, current_frame, global_map);
  double residual_score = calculate_residual_score(result);
  double consistency_score =
    calculate_temporal_consistency(result, recent_frames);
  double geometric_score = calculate_geometric_consistency(result);

  // Weighted combination
  double total_score = params_.w_overlap * overlap_score +
                       params_.w_residual * residual_score +
                       params_.w_consistency * consistency_score +
                       params_.w_geometric * geometric_score;

  logger->debug(
    "Quality scores - Overlap: {:.3f}, Residual: {:.3f}, Consistency: {:.3f}, "
    "Geometric: {:.3f}, Total: {:.3f}",
    overlap_score,
    residual_score,
    consistency_score,
    geometric_score,
    total_score);

  return std::clamp(total_score, 0.0, 1.0);
}

gtsam::noiseModel::Base::shared_ptr MatchingEvaluator::
  create_adaptive_noise_model(const MatchingResult&     matching_result,
                              const LocalizationParams& params) {
  double confidence = matching_result.confidence_score;

  // Adaptive noise based on confidence and matching type
  double base_trans_noise = 0.1;   // 10cm
  double base_rot_noise   = 0.05;  // ~3 degrees

  double confidence_factor = 2.0 - confidence;  // 1.0 to 2.0

  // Different noise levels for different matching types
  double type_factor = 1.0;
  switch (matching_result.matching_type) {
    case MatchingType::SCAN_TO_SCAN:
      type_factor = 1.2;  // Slightly higher uncertainty
      break;
    case MatchingType::SCAN_TO_MAP:
      type_factor = 0.8;  // Lower uncertainty (more reliable)
      break;
    case MatchingType::HYBRID:
      type_factor = 1.0;  // Balanced
      break;
    default:
      type_factor = 5.0;  // Very high uncertainty
      break;
  }

  double trans_noise = base_trans_noise * confidence_factor * type_factor;
  double rot_noise   = base_rot_noise * confidence_factor * type_factor;

  gtsam::Vector6 sigmas;
  sigmas << rot_noise, rot_noise, rot_noise, trans_noise, trans_noise,
    trans_noise;

  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

double MatchingEvaluator::calculate_overlap_ratio(
  const MatchingResult&         result,
  const PreprocessedFrame::Ptr& current_frame,
  const GlobalMap::ConstPtr&    global_map) {
  if (!global_map || !global_map->global_ivox ||
      current_frame->points.empty()) {
    return 0.0;
  }

  // Transform current frame to estimated pose
  std::vector<Eigen::Vector4d> transformed_vec;
  transformed_vec.reserve(current_frame->points.size());
  for (const auto& point : current_frame->points) {
    transformed_vec.push_back(result.estimated_pose * point);
  }
  auto transformed_points =
    std::make_shared<gtsam_points::PointCloudCPU>(transformed_vec);

  // Count points that have correspondences in the map
  int    correspondences = 0;
  double max_correspondence_dist_sq =
    params_.max_correspondence_distance * params_.max_correspondence_distance;

  for (size_t i = 0; i < transformed_points->size(); ++i) {
    Eigen::Vector3d query_point = transformed_points->points[i].head<3>();

    // Query iVox for nearest neighbors
    size_t k_index;
    double k_sq_dist;

    if (global_map->global_ivox->knn_search(
          query_point.data(), 1, &k_index, &k_sq_dist) > 0) {
      if (k_sq_dist < max_correspondence_dist_sq) {
        correspondences++;
      }
    }
  }

  double overlap_ratio =
    static_cast<double>(correspondences) / transformed_points->size();

  // Apply minimum correspondence threshold
  if (correspondences < params_.min_correspondences) {
    overlap_ratio *= 0.5;  // Penalize low correspondence count
  }

  return std::clamp(overlap_ratio, 0.0, 1.0);
}

double MatchingEvaluator::calculate_residual_score(
  const MatchingResult& result) {
  // Use information matrix trace as residual indicator
  if (result.information_matrix.trace() == 0.0) {
    return 0.5;  // Default score when no information available
  }

  // Higher information matrix trace indicates better constraint
  double trace = result.information_matrix.trace();
  double normalized_trace =
    std::min(trace / 1000.0, 1.0);  // Normalize to [0,1]

  return normalized_trace;
}

double MatchingEvaluator::calculate_temporal_consistency(
  const MatchingResult&                          result,
  const std::deque<LocalizationFrame::ConstPtr>& recent_frames) {
  if (recent_frames.size() < 2) {
    return 0.8;  // Default score for insufficient history
  }

  // Check consistency with recent motion
  auto last_frame        = recent_frames.back();
  auto second_last_frame = recent_frames[recent_frames.size() - 2];

  // Calculate expected motion based on recent velocity
  Eigen::Isometry3d recent_motion =
    second_last_frame->T_world_sensor().inverse() *
    last_frame->T_world_sensor();

  Eigen::Isometry3d predicted_pose =
    last_frame->T_world_sensor() * recent_motion;

  // Compare with estimated pose
  Eigen::Isometry3d pose_diff =
    predicted_pose.inverse() * result.estimated_pose;

  double            translation_error = pose_diff.translation().norm();
  Eigen::AngleAxisd rotation_error(pose_diff.rotation());
  double            rotation_error_rad = std::abs(rotation_error.angle());

  // Normalize errors
  double trans_consistency =
    std::exp(-translation_error / 0.5);  // 50cm characteristic length
  double rot_consistency =
    std::exp(-rotation_error_rad / 0.2);  // ~11 degrees characteristic angle

  return 0.7 * trans_consistency + 0.3 * rot_consistency;
}

double MatchingEvaluator::calculate_geometric_consistency(
  const MatchingResult& result) {
  // Check if the pose is geometrically reasonable

  // 1. Check for reasonable translation (not too large jumps)
  double translation_norm = result.estimated_pose.translation().norm();
  if (translation_norm > 1000.0) {  // 1km limit
    return 0.0;
  }

  // 2. Check rotation matrix validity
  Eigen::Matrix3d R   = result.estimated_pose.rotation();
  double          det = R.determinant();
  if (std::abs(det - 1.0) > 0.1) {  // Should be close to 1
    return 0.0;
  }

  // 3. Check orthogonality
  Eigen::Matrix3d should_be_identity = R.transpose() * R;
  double          orthogonality_error =
    (should_be_identity - Eigen::Matrix3d::Identity()).norm();
  if (orthogonality_error > 0.1) {
    return 0.0;
  }

  // 4. Confidence-based score
  double confidence_score = result.confidence_score;

  return confidence_score;
}

double MatchingEvaluator::evaluate_scan_to_scan_quality(
  const PreprocessedFrame::Ptr&      current_frame,
  const LocalizationFrame::ConstPtr& previous_frame,
  const MatchingResult&              result) {
  if (current_frame->points.empty() || !previous_frame->frame) {
    return 0.0;
  }

  // Convert current frame points to PointCloud
  auto current_cloud = std::make_shared<gtsam_points::PointCloudCPU>(current_frame->points);
  gtsam_points::PointCloud::ConstPtr current_cloud_base = current_cloud;

  // Create GICP factor for evaluation (binary factor between two frames)
  auto gicp_factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(
    gtsam::Symbol('x', 0),
    gtsam::Symbol('x', 1),
    previous_frame->frame,
    current_cloud_base);

  // Set up optimization problem
  gtsam::Values values;
  values.insert(gtsam::Symbol('x', 0), gtsam::Pose3());

  Eigen::Isometry3d relative_pose =
    previous_frame->T_world_sensor().inverse() * result.estimated_pose;
  values.insert(gtsam::Symbol('x', 1), gtsam::Pose3(relative_pose.matrix()));

  gtsam::NonlinearFactorGraph graph;
  graph.add(gicp_factor);

  try {
    // Evaluate error at current estimate
    double error            = graph.error(values);
    double normalized_error = std::exp(-error / 1000.0);  // Normalize error

    return std::clamp(normalized_error, 0.0, 1.0);
  } catch (const std::exception& e) {
    logger->warn("Failed to evaluate scan-to-scan quality: {}", e.what());
    return 0.1;
  }
}

double MatchingEvaluator::evaluate_scan_to_map_quality(
  const PreprocessedFrame::Ptr& current_frame,
  const GlobalMap::ConstPtr&    global_map,
  const MatchingResult&         result) {
  if (current_frame->points.empty() || !global_map ||
      !global_map->global_ivox) {
    return 0.0;
  }

  // Convert current frame points to PointCloud
  auto current_cloud = std::make_shared<gtsam_points::PointCloudCPU>(current_frame->points);
  gtsam_points::PointCloud::ConstPtr current_cloud_base = current_cloud;

  // Create map matching factor for evaluation
  auto map_factor = gtsam::make_shared<
    gtsam_points::IntegratedGICPFactor_<gtsam_points::iVox,
                                        gtsam_points::PointCloud>>(
    gtsam::Pose3(),
    gtsam::Symbol('x', 0),
    global_map->global_ivox,
    current_cloud_base,
    global_map->global_ivox);

  gtsam::Values values;
  values.insert(gtsam::Symbol('x', 0),
                gtsam::Pose3(result.estimated_pose.matrix()));

  gtsam::NonlinearFactorGraph graph;
  graph.add(map_factor);

  try {
    double error = graph.error(values);
    double normalized_error =
      std::exp(-error / 2000.0);  // Map matching typically has higher residuals

    return std::clamp(normalized_error, 0.0, 1.0);
  } catch (const std::exception& e) {
    logger->warn("Failed to evaluate scan-to-map quality: {}", e.what());
    return 0.1;
  }
}

}  // namespace glim
