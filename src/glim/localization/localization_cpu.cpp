#include <glim/localization/localization_cpu.hpp>

#include <chrono>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp>
#include <gtsam_points/ann/incremental_voxelmap.hpp>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
namespace glim {

LocalizationCPUParams::LocalizationCPUParams() {
  // Default values are set in header file
}

LocalizationCPUParams::~LocalizationCPUParams() {
}

LocalizationCPU::LocalizationCPU(const LocalizationCPUParams& params)
  : params_(params), mt_(std::random_device{}()) {
  logger_ = spdlog::get("glim_logger");
  if (!logger_) {
    logger_ = spdlog::default_logger();
  }

  // Initialize components
  map_manager_ = std::make_unique<MapManager>();
  evaluator_   = std::make_unique<MatchingEvaluator>(params_);

  // Initialize FixedLagSmoother
  gtsam::ISAM2Params isam2_params;
  if (params_.use_isam2_dogleg) {
    isam2_params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  }
  isam2_params.relinearizeSkip = params_.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params_.isam2_relinearize_threshold);

  smoother_.reset(
    new gtsam_points::FixedLagSmootherExt(params_.smoother_lag, isam2_params));

  // Initialize scan-to-scan target
  target_ivox_ = std::make_shared<gtsam_points::iVox>(params_.ivox_resolution);
  target_ivox_->voxel_insertion_setting().set_min_dist_in_cell(
    params_.ivox_min_dist);
  target_ivox_->set_lru_horizon(params_.ivox_lru_thresh);
  target_ivox_->set_neighbor_voxel_mode(1);

  logger_->info("LocalizationCPU initialized with smoother_lag={:.1f}s",
                params_.smoother_lag);
}

LocalizationCPU::~LocalizationCPU() {
  logger_->info("LocalizationCPU destroyed. Processed {} frames", frame_count_);
}

EstimationFrame::ConstPtr LocalizationCPU::localize_frame(
  const PreprocessedFrame::Ptr&           frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  auto start_time = std::chrono::high_resolution_clock::now();

  if (!is_initialized()) {
    logger_->error(
      "Localization not initialized. Call load_global_map() and "
      "set_initial_pose() first");
    return nullptr;
  }

  const int current_id = frame_count_;
  frame_count_++;

  // Perform hybrid matching
  MatchingResult s2s_result = perform_scan_to_scan_matching(frame);
  MatchingResult s2m_result = perform_scan_to_map_matching(frame);

  // Evaluate matching quality
  auto   global_map = map_manager_->get_global_map();
  double s2s_score =
    evaluator_->evaluate_scan_to_scan_quality(frame,
                                              recent_frames_.empty()
                                                ? nullptr
                                                : recent_frames_.back(),
                                              s2s_result);
  double s2m_score =
    evaluator_->evaluate_scan_to_map_quality(frame, global_map, s2m_result);

  // Decide matching strategy
  MatchingResult final_result =
    decide_matching_strategy(s2s_result, s2m_result, s2s_score, s2m_score);

  // Update pose graph and handle marginalization
  update_pose_graph(final_result, marginalized_frames);

  // Create localization frame
  auto loc_frame = create_localization_frame(frame, final_result);
  loc_frame->id  = current_id;

  // Update current state
  current_pose_       = final_result.estimated_pose;
  current_confidence_ = final_result.confidence_score;
  confidence_history_.push_back(current_confidence_);

  // Update velocity estimate
  update_velocity_estimate(current_pose_);

  // Add to recent frames
  recent_frames_.push_back(loc_frame);
  if (recent_frames_.size() > params_.sliding_window_size) {
    recent_frames_.pop_front();
  }

  // Update scan-to-scan target
  if (final_result.confidence_score > 0.5) {
    // Transform points to world frame
    std::vector<Eigen::Vector4d> transformed_vec;
    transformed_vec.reserve(frame->points.size());
    for (size_t i = 0; i < frame->points.size(); ++i) {
      transformed_vec.push_back(current_pose_ * frame->points[i]);
    }
    auto transformed_points =
      std::make_shared<gtsam_points::PointCloudCPU>(transformed_vec);
    target_ivox_->insert(*transformed_points);
  }

  // Check if should add keyframe
  if (should_add_keyframe(loc_frame)) {
    add_keyframe(loc_frame);
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
    end_time - start_time);
  loc_frame->processing_time = duration.count();
  total_processing_time_ += duration.count();

  logger_->debug(
    "Frame {} localized: confidence={:.3f}, type={}, time={:.1f}ms",
    current_id,
    final_result.confidence_score,
    static_cast<int>(final_result.matching_type),
    duration.count());

  return loc_frame;
}

bool LocalizationCPU::load_global_map(const std::string& map_path) {
  logger_->info("Loading global map from: {}", map_path);

  bool success = false;
  if (map_path.size() >= 4 && map_path.substr(map_path.size() - 4) == ".pcd") {
    success =
      map_manager_->load_map_from_pcd(map_path, params_.ivox_resolution);
  } else {
    success =
      map_manager_->load_map_from_submaps(map_path, params_.ivox_resolution);
  }

  if (success) {
    map_manager_->print_map_statistics();
    logger_->info("Global map loaded successfully");
  } else {
    logger_->error("Failed to load global map");
  }

  return success;
}

bool LocalizationCPU::set_initial_pose(const Eigen::Isometry3d& initial_pose) {
  if (!map_manager_->is_map_loaded()) {
    logger_->error("Cannot set initial pose: global map not loaded");
    return false;
  }

  current_pose_ = initial_pose;
  initialized_  = true;

  logger_->info("Initial pose set: x={:.2f}, y={:.2f}, z={:.2f}",
                initial_pose.translation().x(),
                initial_pose.translation().y(),
                initial_pose.translation().z());

  return true;
}

double LocalizationCPU::get_localization_confidence() const {
  return current_confidence_;
}

void LocalizationCPU::reset() {
  recent_frames_.clear();
  keyframes_.clear();
  current_estimates_.clear();
  confidence_history_.clear();

  current_pose_        = Eigen::Isometry3d::Identity();
  current_confidence_  = 0.0;
  initialized_         = false;
  frame_count_         = 0;
  marginalized_cursor_ = 0;

  // Reset smoother
  gtsam::ISAM2Params isam2_params;
  if (params_.use_isam2_dogleg) {
    isam2_params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  }
  isam2_params.relinearizeSkip = params_.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params_.isam2_relinearize_threshold);

  smoother_ =
    std::make_unique<gtsam_points::FixedLagSmootherExt>(params_.smoother_lag,
                                                        isam2_params);

  // Reset target
  target_ivox_ = std::make_shared<gtsam_points::iVox>(params_.ivox_resolution);
  target_ivox_->voxel_insertion_setting().set_min_dist_in_cell(
    params_.ivox_min_dist);
  target_ivox_->set_lru_horizon(params_.ivox_lru_thresh);
  target_ivox_->set_neighbor_voxel_mode(1);

  logger_->info("Localization reset");
}

Eigen::Isometry3d LocalizationCPU::get_current_pose() const {
  return current_pose_;
}

bool LocalizationCPU::is_initialized() const {
  return initialized_ && map_manager_->is_map_loaded();
}

std::vector<EstimationFrame::ConstPtr> LocalizationCPU::get_remaining_frames() {
  std::vector<EstimationFrame::ConstPtr> remaining_frames;

  // Return all non-marginalized frames
  for (int i = marginalized_cursor_; i < recent_frames_.size(); ++i) {
    if (i < recent_frames_.size()) {
      remaining_frames.push_back(recent_frames_[i]);
    }
  }

  logger_->info("Returning {} remaining frames", remaining_frames.size());
  return remaining_frames;
}

MatchingResult LocalizationCPU::perform_scan_to_scan_matching(
  const PreprocessedFrame::Ptr& current_frame) {
  MatchingResult result;
  result.matching_type = MatchingType::SCAN_TO_SCAN;

  if (recent_frames_.empty()) {
    result.estimated_pose   = current_pose_;
    result.confidence_score = 0.1;
    return result;
  }

  // Use predicted pose as initial guess
  Eigen::Isometry3d predicted_pose = predict_pose_from_velocity();
  auto              last_frame     = recent_frames_.back();

  // Convert current frame points to PointCloud
  auto current_cloud =
    std::make_shared<gtsam_points::PointCloudCPU>(current_frame->points);
  gtsam_points::PointCloud::ConstPtr current_cloud_base = current_cloud;

  // Create GICP factor with target iVox (unary factor with fixed target at identity)
  auto gicp_factor = gtsam::make_shared<
    gtsam_points::IntegratedGICPFactor_<gtsam_points::iVox,
                                        gtsam_points::PointCloud>>(
    gtsam::Pose3(),           // Fixed target pose at identity
    gtsam::Symbol('x', 0),    // Source key (to be optimized)
    target_ivox_,             // Target iVox
    current_cloud_base,       // Source point cloud
    target_ivox_);            // Target tree for nearest neighbor search

  gtsam::Values     values;
  Eigen::Isometry3d relative_pose =
    last_frame->T_world_sensor().inverse() * predicted_pose;

  // Add source pose (to be optimized)
  values.insert(gtsam::Symbol('x', 0), gtsam::Pose3(relative_pose.matrix()));

  gtsam::NonlinearFactorGraph graph;
  graph.add(gicp_factor);

  // Add regularization to prevent large jumps
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    gtsam::Symbol('x', 0),
    gtsam::Pose3(relative_pose.matrix()),
    gtsam::noiseModel::Isotropic::Sigma(6, 0.1));

  try {
    gtsam::LevenbergMarquardtParams lm_params;
    lm_params.setMaxIterations(20);
    lm_params.setRelativeErrorTol(1e-6);

    auto optimized =
      gtsam::LevenbergMarquardtOptimizer(graph, values, lm_params).optimize();
    auto optimized_relative_pose =
      optimized.at<gtsam::Pose3>(gtsam::Symbol('x', 0));

    result.estimated_pose = last_frame->T_world_sensor() *
                            Eigen::Isometry3d(optimized_relative_pose.matrix());

    // Calculate confidence based on optimization residual
    double final_error = graph.error(optimized);
    result.confidence_score =
      std::exp(-final_error / 100.0);  // Normalize error
    result.confidence_score = std::clamp(result.confidence_score, 0.1, 0.95);

    // Store information matrix
    try {
      gtsam::Marginals marginals(graph, optimized);
      result.information_matrix =
        marginals.marginalInformation(gtsam::Symbol('x', 0));
    } catch (const std::exception& e) {
      logger_->debug("Failed to compute marginals for scan-to-scan: {}",
                     e.what());
      result.information_matrix =
        gtsam::Matrix6::Identity() * result.confidence_score;
    }

  } catch (const std::exception& e) {
    logger_->warn("Scan-to-scan matching failed: {}", e.what());
    result.estimated_pose     = predicted_pose;
    result.confidence_score   = 0.1;
    result.information_matrix = gtsam::Matrix6::Identity() * 0.01;
  }

  return result;
}

MatchingResult LocalizationCPU::perform_scan_to_map_matching(
  const PreprocessedFrame::Ptr& current_frame) {
  MatchingResult result;
  result.matching_type = MatchingType::SCAN_TO_MAP;

  auto global_map = map_manager_->get_global_map();
  if (!global_map || !global_map->global_ivox) {
    result.estimated_pose   = current_pose_;
    result.confidence_score = 0.0;
    return result;
  }

  // Use current pose as initial guess (or predicted pose if available)
  Eigen::Isometry3d initial_guess = current_pose_;
  if (!recent_frames_.empty()) {
    initial_guess = predict_pose_from_velocity();
  }

  // Convert current frame points to PointCloud
  auto current_cloud =
    std::make_shared<gtsam_points::PointCloudCPU>(current_frame->points);
  gtsam_points::PointCloud::ConstPtr current_cloud_base = current_cloud;

  // Create map matching factor (unary factor with fixed target at identity)
  auto map_factor = gtsam::make_shared<
    gtsam_points::IntegratedGICPFactor_<gtsam_points::iVox,
                                        gtsam_points::PointCloud>>(
    gtsam::Pose3(),                // Fixed target pose at identity
    gtsam::Symbol('x', 0),         // Source key (to be optimized)
    global_map->global_ivox,       // Target iVox (global map)
    current_cloud_base,            // Source point cloud (current scan)
    global_map->global_ivox);      // Target tree for nearest neighbor search

  gtsam::Values values;
  // Add source pose (to be optimized)
  values.insert(gtsam::Symbol('x', 0), gtsam::Pose3(initial_guess.matrix()));

  gtsam::NonlinearFactorGraph graph;
  graph.add(map_factor);

  // Add regularization to prevent large jumps from current pose
  double reg_weight = params_.regularization_weight;
  if (current_confidence_ < 0.5) {
    reg_weight *= 0.5;  // Allow more freedom when confidence is low
  }

  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    gtsam::Symbol('x', 0),
    gtsam::Pose3(current_pose_.matrix()),
    gtsam::noiseModel::Isotropic::Sigma(6, reg_weight));

  try {
    gtsam::LevenbergMarquardtParams lm_params;
    lm_params.setMaxIterations(30);  // More iterations for map matching
    lm_params.setRelativeErrorTol(1e-6);
    lm_params.setAbsoluteErrorTol(1e-6);

    auto optimized =
      gtsam::LevenbergMarquardtOptimizer(graph, values, lm_params).optimize();
    result.estimated_pose = Eigen::Isometry3d(
      optimized.at<gtsam::Pose3>(gtsam::Symbol('x', 0)).matrix());

    // Calculate confidence based on optimization residual and convergence
    double final_error = graph.error(optimized);
    result.confidence_score =
      std::exp(-final_error / 500.0);  // Map matching has higher residuals
    result.confidence_score = std::clamp(result.confidence_score, 0.1, 0.98);

    // Boost confidence if pose change is reasonable
    double pose_change =
      (result.estimated_pose.inverse() * current_pose_).translation().norm();
    if (pose_change < 2.0) {  // Less than 2m change
      result.confidence_score *= 1.1;
      result.confidence_score = std::min(result.confidence_score, 0.98);
    }

    // Store information matrix
    try {
      gtsam::Marginals marginals(graph, optimized);
      result.information_matrix =
        marginals.marginalInformation(gtsam::Symbol('x', 0));
    } catch (const std::exception& e) {
      logger_->debug("Failed to compute marginals for scan-to-map: {}",
                     e.what());
      result.information_matrix =
        gtsam::Matrix6::Identity() * result.confidence_score;
    }

  } catch (const std::exception& e) {
    logger_->warn("Scan-to-map matching failed: {}", e.what());
    result.estimated_pose     = initial_guess;
    result.confidence_score   = 0.1;
    result.information_matrix = gtsam::Matrix6::Identity() * 0.01;
  }

  return result;
}

MatchingResult LocalizationCPU::decide_matching_strategy(
  const MatchingResult& s2s_result,
  const MatchingResult& s2m_result,
  double                s2s_score,
  double                s2m_score) {
  logger_->debug(
    "Matching scores - S2S: {:.3f}, S2M: {:.3f}, Thresholds - S2S: {:.3f}, "
    "S2M: {:.3f}",
    s2s_score,
    s2m_score,
    params_.scan_to_scan_threshold,
    params_.scan_to_map_threshold);

  // Adaptive thresholds based on recent performance
  double adaptive_s2m_threshold = params_.scan_to_map_threshold;
  double adaptive_s2s_threshold = params_.scan_to_scan_threshold;

  if (confidence_history_.size() > 5) {
    double avg_confidence = 0.0;
    for (size_t i = confidence_history_.size() - 5;
         i < confidence_history_.size();
         ++i) {
      avg_confidence += confidence_history_[i];
    }
    avg_confidence /= 5.0;

    // Lower thresholds if recent performance is poor
    if (avg_confidence < 0.5) {
      adaptive_s2m_threshold *= 0.8;
      adaptive_s2s_threshold *= 0.8;
    }
  }

  // Decision logic with confidence consideration
  if (s2m_score > adaptive_s2m_threshold) {
    if (s2s_score > adaptive_s2s_threshold) {
      // Both are good - create hybrid result
      logger_->debug("Using hybrid matching (both good)");
      return create_hybrid_result(s2s_result, s2m_result, s2s_score, s2m_score);
    } else {
      // Only map matching is good
      logger_->debug("Using scan-to-map matching");
      return s2m_result;
    }
  } else {
    if (s2s_score > adaptive_s2s_threshold) {
      // Only scan-to-scan is good
      logger_->debug("Using scan-to-scan matching");
      return s2s_result;
    } else {
      // Both are poor - choose the better one but with reduced confidence
      logger_->debug("Both matching poor, choosing better one");
      if (s2m_score > s2s_score) {
        auto result             = s2m_result;
        result.confidence_score = std::min(result.confidence_score * 0.5, 0.3);
        result.matching_type    = MatchingType::SCAN_TO_MAP;
        return result;
      } else {
        auto result             = s2s_result;
        result.confidence_score = std::min(result.confidence_score * 0.5, 0.3);
        result.matching_type    = MatchingType::SCAN_TO_SCAN;
        return result;
      }
    }
  }
}

MatchingResult LocalizationCPU::create_hybrid_result(
  const MatchingResult& s2s_result,
  const MatchingResult& s2m_result,
  double                s2s_score,
  double                s2m_score) {
  MatchingResult result;
  result.matching_type = MatchingType::HYBRID;

  double total_weight = s2s_score + s2m_score;
  if (total_weight < 1e-6) {
    // Fallback if both scores are very low
    result                  = s2m_result;
    result.confidence_score = 0.1;
    result.matching_type    = MatchingType::HYBRID;
    return result;
  }

  double w_s2s = s2s_score / total_weight;
  double w_s2m = s2m_score / total_weight;

  // Proper SE(3) interpolation using SLERP for rotation
  Eigen::Quaterniond q_s2s(s2s_result.estimated_pose.rotation());
  Eigen::Quaterniond q_s2m(s2m_result.estimated_pose.rotation());

  // Ensure quaternions are in the same hemisphere
  if (q_s2s.dot(q_s2m) < 0) {
    q_s2m.coeffs() *= -1;
  }

  // SLERP interpolation for rotation
  Eigen::Quaterniond q_interp = q_s2s.slerp(w_s2m, q_s2m);

  // Weighted interpolation for translation
  Eigen::Vector3d t_interp = w_s2s * s2s_result.estimated_pose.translation() +
                             w_s2m * s2m_result.estimated_pose.translation();

  result.estimated_pose               = Eigen::Isometry3d::Identity();
  result.estimated_pose.translation() = t_interp;
  result.estimated_pose.linear()      = q_interp.toRotationMatrix();

  // Hybrid confidence calculation
  double base_confidence = std::max(s2s_score, s2m_score);
  double agreement_bonus =
    1.0 - std::abs(s2s_score - s2m_score) / std::max(s2s_score, s2m_score);
  result.confidence_score = base_confidence * (1.0 + 0.2 * agreement_bonus);
  result.confidence_score = std::clamp(result.confidence_score, 0.0, 0.99);

  // Store individual scores
  result.scan_to_scan_score = s2s_score;
  result.scan_to_map_score  = s2m_score;

  // Weighted information matrix
  if (s2s_result.information_matrix.trace() > 0 &&
      s2m_result.information_matrix.trace() > 0) {
    result.information_matrix = w_s2s * s2s_result.information_matrix +
                                w_s2m * s2m_result.information_matrix;
  } else if (s2s_result.information_matrix.trace() > 0) {
    result.information_matrix = s2s_result.information_matrix;
  } else if (s2m_result.information_matrix.trace() > 0) {
    result.information_matrix = s2m_result.information_matrix;
  } else {
    result.information_matrix =
      gtsam::Matrix6::Identity() * result.confidence_score;
  }

  logger_->debug("Hybrid result: confidence={:.3f}, weights=[{:.2f}, {:.2f}]",
                 result.confidence_score,
                 w_s2s,
                 w_s2m);

  return result;
}

void LocalizationCPU::update_pose_graph(
  const MatchingResult&                   matching_result,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  const int current_key = frame_count_ - 1;

  // Add new pose variable
  gtsam::FixedLagSmootherKeyTimestampMap new_stamps;
  gtsam::Values                          new_values;
  gtsam::NonlinearFactorGraph            new_factors;

  new_stamps[gtsam::Symbol('x', current_key)] = last_timestamp_;
  new_values.insert(gtsam::Symbol('x', current_key),
                    gtsam::Pose3(matching_result.estimated_pose.matrix()));

  // Create factors based on matching type and quality
  add_localization_factors(matching_result, current_key, new_factors);

  // Add temporal consistency factors if we have history
  if (current_key > 0) {
    add_temporal_factors(matching_result, current_key, new_factors);
  }

  // Add map constraint factors for high-confidence scan-to-map results
  if (matching_result.matching_type == MatchingType::SCAN_TO_MAP ||
      matching_result.matching_type == MatchingType::HYBRID) {
    if (matching_result.confidence_score > params_.scan_to_map_threshold) {
      add_map_constraint_factors(matching_result, current_key, new_factors);
    }
  }

  try {
    // Update smoother with new factors
    smoother_->update(new_factors, new_values, new_stamps);

    logger_->debug("Updated pose graph: key={}, factors={}, confidence={:.3f}",
                   current_key,
                   new_factors.size(),
                   matching_result.confidence_score);
  } catch (const std::exception& e) {
    logger_->error("Failed to update pose graph: {}", e.what());
    return;
  }

  // Find marginalized frames
  find_marginalized_frames(current_key, marginalized_frames);

  // Update frame estimates with latest optimization results
  update_frame_estimates();
}

void LocalizationCPU::find_marginalized_frames(
  int                                     current_frame_id,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  while (marginalized_cursor_ < current_frame_id) {
    if (marginalized_cursor_ >= recent_frames_.size())
      break;

    double span = last_timestamp_ - recent_frames_[marginalized_cursor_]->stamp;
    if (span < params_.smoother_lag - 0.1) {
      break;
    }

    marginalized_frames.push_back(recent_frames_[marginalized_cursor_]);
    marginalized_cursor_++;
  }

  if (!marginalized_frames.empty()) {
    logger_->debug("Marginalized {} frames", marginalized_frames.size());
  }
}

void LocalizationCPU::update_frame_estimates() {
  // Update poses of non-marginalized frames with smoother results
  for (int i = marginalized_cursor_;
       i < static_cast<int>(recent_frames_.size());
       ++i) {
    try {
      auto optimized_pose =
        smoother_->calculateEstimate<gtsam::Pose3>(gtsam::Symbol('x', i));

      // Update current estimates for tracking
      current_estimates_.update(gtsam::Symbol('x', i), optimized_pose);

      // Update current pose if this is the latest frame
      if (i == static_cast<int>(recent_frames_.size()) - 1) {
        current_pose_ = Eigen::Isometry3d(optimized_pose.matrix());

        // Update velocity estimate
        update_velocity_estimate(current_pose_);

        logger_->debug("Updated current pose from smoother optimization");
      }

    } catch (const std::exception& e) {
      logger_->warn("Failed to update frame {} estimate: {}", i, e.what());
    }
  }

  // Update covariance estimates if available
  try {
    if (!recent_frames_.empty()) {
      int              latest_key = static_cast<int>(recent_frames_.size()) - 1;
      gtsam::Marginals marginals(smoother_->getFactors(), current_estimates_);
      auto             covariance =
        marginals.marginalCovariance(gtsam::Symbol('x', latest_key));

      // Store covariance for uncertainty tracking
      logger_->debug("Current pose uncertainty trace: {:.6f}",
                     covariance.trace());
    }
  } catch (const std::exception& e) {
    logger_->debug("Failed to compute marginal covariances: {}", e.what());
  }
}

gtsam::noiseModel::Base::shared_ptr LocalizationCPU::
  create_adaptive_noise_model(const MatchingResult& matching_result) {
  double confidence = matching_result.confidence_score;

  // Adaptive noise based on confidence
  double trans_noise = params_.min_translation_noise +
                       (1.0 - confidence) * (params_.max_translation_noise -
                                             params_.min_translation_noise);
  double rot_noise = params_.min_rotation_noise +
                     (1.0 - confidence) * (params_.max_rotation_noise -
                                           params_.min_rotation_noise);

  gtsam::Vector6 sigmas;
  sigmas << rot_noise, rot_noise, rot_noise, trans_noise, trans_noise,
    trans_noise;

  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

bool LocalizationCPU::should_add_keyframe(
  const EstimationFrame::ConstPtr& frame) {
  if (keyframes_.empty()) {
    return true;
  }

  auto   last_keyframe = keyframes_.back();
  double distance      = (frame->T_world_sensor().translation() -
                     last_keyframe->T_world_sensor().translation())
                      .norm();

  Eigen::AngleAxisd angle_diff(frame->T_world_sensor().linear().transpose() *
                               last_keyframe->T_world_sensor().linear());

  return distance > params_.keyframe_distance_threshold ||
         std::abs(angle_diff.angle()) > params_.keyframe_angle_threshold;
}

void LocalizationCPU::add_keyframe(const EstimationFrame::ConstPtr& frame) {
  auto loc_frame = std::dynamic_pointer_cast<const LocalizationFrame>(frame);
  if (loc_frame) {
    keyframes_.push_back(loc_frame);
    manage_keyframes();
    logger_->debug("Added keyframe {}, total keyframes: {}",
                   frame->id,
                   keyframes_.size());
  }
}

void LocalizationCPU::manage_keyframes() {
  if (keyframes_.size() > params_.max_keyframes) {
    keyframes_.erase(keyframes_.begin());
  }
}

Eigen::Isometry3d LocalizationCPU::predict_pose_from_velocity() {
  if (recent_frames_.empty()) {
    return current_pose_;
  }

  double dt = 0.1;  // Assume 10Hz, should use actual timestamp

  Eigen::Isometry3d predicted_pose = current_pose_;
  predicted_pose.translation() += linear_velocity_ * dt;

  // Simple angular velocity integration (should use proper SO(3) integration)
  Eigen::AngleAxisd angular_motion(angular_velocity_.norm() * dt,
                                   angular_velocity_.normalized());
  predicted_pose.linear() =
    predicted_pose.linear() * angular_motion.toRotationMatrix();

  return predicted_pose;
}

void LocalizationCPU::update_velocity_estimate(
  const Eigen::Isometry3d& new_pose) {
  if (recent_frames_.empty()) {
    linear_velocity_.setZero();
    angular_velocity_.setZero();
    return;
  }

  double dt = 0.1;  // Should use actual timestamp difference

  // Update linear velocity
  linear_velocity_ =
    (new_pose.translation() - current_pose_.translation()) / dt;

  // Update angular velocity (simplified)
  Eigen::AngleAxisd angle_diff(new_pose.linear().transpose() *
                               current_pose_.linear());
  angular_velocity_ = angle_diff.axis() * angle_diff.angle() / dt;

  last_timestamp_ =
    recent_frames_.back()->stamp + dt;  // Should use actual timestamp
}

LocalizationFrame::Ptr LocalizationCPU::create_localization_frame(
  const PreprocessedFrame::Ptr& preprocessed_frame,
  const MatchingResult&         matching_result) {
  auto loc_frame = std::make_shared<LocalizationFrame>();

  // Set basic frame data
  loc_frame->stamp     = preprocessed_frame->stamp;
  loc_frame->raw_frame = preprocessed_frame;
  loc_frame->frame =
    std::make_shared<gtsam_points::PointCloudCPU>(preprocessed_frame->points);
  loc_frame->frame_id = FrameID::LIDAR;

  // Set pose
  loc_frame->set_T_world_sensor(FrameID::LIDAR, matching_result.estimated_pose);

  // Set localization-specific data
  loc_frame->matching_result         = matching_result;
  loc_frame->localization_confidence = matching_result.confidence_score;

  return loc_frame;
}

void LocalizationCPU::add_localization_factors(
  const MatchingResult&        matching_result,
  int                          current_key,
  gtsam::NonlinearFactorGraph& factors) {
  if (current_key == 0) {
    // First frame - add strong prior
    auto prior_noise = gtsam::noiseModel::Isotropic::Precision(6, 1e6);
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      gtsam::Symbol('x', current_key),
      gtsam::Pose3(matching_result.estimated_pose.matrix()),
      prior_noise);

    logger_->debug("Added prior factor for first frame");
    return;
  }

  // Create adaptive noise model based on matching result
  auto noise_model =
    evaluator_->create_adaptive_noise_model(matching_result, params_);

  // Add odometry factor (between consecutive poses)
  if (!recent_frames_.empty()) {
    Eigen::Isometry3d relative_pose =
      recent_frames_.back()->T_world_sensor().inverse() *
      matching_result.estimated_pose;

    factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      gtsam::Symbol('x', current_key - 1),
      gtsam::Symbol('x', current_key),
      gtsam::Pose3(relative_pose.matrix()),
      noise_model);

    logger_->debug("Added odometry factor between frames {} and {}",
                   current_key - 1,
                   current_key);
  }
}

void LocalizationCPU::add_temporal_factors(
  const MatchingResult&        matching_result,
  int                          current_key,
  gtsam::NonlinearFactorGraph& factors) {
  // Add velocity consistency factor if we have sufficient history
  if (recent_frames_.size() >= 2 && current_key >= 2) {
    // Get recent poses for velocity estimation
    auto frame_t2 = recent_frames_[recent_frames_.size() - 2];  // t-2
    auto frame_t1 = recent_frames_[recent_frames_.size() - 1];  // t-1

    // Estimate velocity from t-2 to t-1
    Eigen::Isometry3d velocity_motion =
      frame_t2->T_world_sensor().inverse() * frame_t1->T_world_sensor();

    // Predict motion from t-1 to t (current)
    Eigen::Isometry3d predicted_relative = velocity_motion;

    // Create velocity consistency factor
    double velocity_confidence =
      std::min(matching_result.confidence_score * 1.2, 0.9);
    auto velocity_noise =
      gtsam::noiseModel::Isotropic::Sigma(6, 0.2 / velocity_confidence);

    factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      gtsam::Symbol('x', current_key - 1),
      gtsam::Symbol('x', current_key),
      gtsam::Pose3(predicted_relative.matrix()),
      velocity_noise);

    logger_->debug("Added velocity consistency factor for frame {}",
                   current_key);
  }

  // Add loop closure factors if we detect revisited areas
  if (matching_result.confidence_score > 0.8 && keyframes_.size() > 5) {
    add_loop_closure_factors(matching_result, current_key, factors);
  }
}

void LocalizationCPU::add_map_constraint_factors(
  const MatchingResult&        matching_result,
  int                          current_key,
  gtsam::NonlinearFactorGraph& factors) {
  auto global_map = map_manager_->get_global_map();
  if (!global_map || !global_map->global_ivox) {
    return;
  }

  // Add map constraint factor for high-confidence scan-to-map results
  if (matching_result.confidence_score > params_.scan_to_map_threshold) {
    // Create map prior factor (soft constraint to prevent drift)
    double map_confidence = matching_result.confidence_score;
    double constraint_strength =
      map_confidence * 100.0;  // Scale confidence to precision

    auto map_noise =
      gtsam::noiseModel::Isotropic::Precision(6, constraint_strength);
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      gtsam::Symbol('x', current_key),
      gtsam::Pose3(matching_result.estimated_pose.matrix()),
      map_noise);

    logger_->debug(
      "Added map constraint factor for frame {} with strength {:.1f}",
      current_key,
      constraint_strength);
  }

  // Add GPS-like absolute position constraint if confidence is very high
  if (matching_result.confidence_score > 0.95) {
    // Very strong position constraint (like GPS)
    Eigen::Vector3d position = matching_result.estimated_pose.translation();

    // Create position-only factor (allow rotation freedom)
    auto position_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, 0.1);  // 10cm accuracy

    // Note: This would require a custom factor for position-only constraints
    // For now, we use a strong prior on the full pose
    auto strong_prior = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05).finished());

    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      gtsam::Symbol('x', current_key),
      gtsam::Pose3(matching_result.estimated_pose.matrix()),
      strong_prior);

    logger_->debug("Added strong map constraint for high-confidence result");
  }
}

void LocalizationCPU::add_loop_closure_factors(
  const MatchingResult&        matching_result,
  int                          current_key,
  gtsam::NonlinearFactorGraph& factors) {
  // Simple loop closure detection based on distance to keyframes
  Eigen::Vector3d current_position =
    matching_result.estimated_pose.translation();

  for (size_t i = 0; i < keyframes_.size(); ++i) {
    if (static_cast<int>(i) >= current_key - 10)
      continue;  // Skip recent keyframes

    auto            keyframe = keyframes_[i];
    Eigen::Vector3d keyframe_position =
      keyframe->T_world_sensor().translation();

    double distance = (current_position - keyframe_position).norm();

    // Potential loop closure if within 2 meters
    if (distance < 2.0) {
      // Compute relative pose
      Eigen::Isometry3d relative_pose =
        keyframe->T_world_sensor().inverse() * matching_result.estimated_pose;

      // Add loop closure factor with moderate confidence
      double loop_confidence =
        std::exp(-distance / 1.0) * matching_result.confidence_score;
      auto loop_noise =
        gtsam::noiseModel::Isotropic::Sigma(6, 0.5 / loop_confidence);

      factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        gtsam::Symbol('x', static_cast<int>(i)),
        gtsam::Symbol('x', current_key),
        gtsam::Pose3(relative_pose.matrix()),
        loop_noise);

      logger_->info(
        "Added loop closure factor: frame {} <-> frame {}, distance {:.2f}m",
        i,
        current_key,
        distance);

      break;  // Only add one loop closure per frame
    }
  }
}

}  // namespace glim
