#pragma once

#include <memory>
#include <deque>
#include <random>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <spdlog/spdlog.h>

#include <glim/localization/localization_base.hpp>
#include <glim/localization/localization_params.hpp>
#include <glim/localization/localization_frame.hpp>
#include <glim/localization/map_manager.hpp>
#include <glim/localization/matching_evaluator.hpp>

namespace gtsam_points {
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;

class IncrementalCovarianceVoxelMap;
using iVoxCovarianceEstimation = IncrementalCovarianceVoxelMap;
class IncrementalFixedLagSmootherExt;
class IncrementalFixedLagSmootherExtWithFallback;
using FixedLagSmootherExt = IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_points

namespace glim {

/**
 * @brief Parameters for CPU-based localization
 */
struct LocalizationCPUParams : public LocalizationParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocalizationCPUParams();
  virtual ~LocalizationCPUParams();

  // Registration parameters
  std::string registration_type =
    "GICP";                           ///< Registration type: "GICP" or "VGICP"
  int    max_iterations        = 10;  ///< Maximum LM iterations
  double convergence_threshold = 1e-3;  ///< Convergence threshold

  // iVox parameters for scan-to-scan
  double ivox_resolution = 0.5;  ///< iVox voxel resolution
  double ivox_min_dist   = 0.1;  ///< Minimum distance in voxel
  int    ivox_lru_thresh = 200;  ///< LRU cache threshold

  // Fixed-lag smoother parameters
  double smoother_lag = 5.0;  ///< Fixed-lag smoother window [sec]
  double isam2_relinearize_threshold = 0.1;    ///< Relinearization threshold
  int    isam2_relinearize_skip      = 1;      ///< Relinearization skip
  bool   use_isam2_dogleg            = false;  ///< Use dogleg optimizer

  // Keyframe management
  double keyframe_distance_threshold =
    2.0;                                  ///< Distance threshold for keyframes
  double keyframe_angle_threshold = 0.5;  ///< Angle threshold for keyframes
  int    max_keyframes            = 100;  ///< Maximum number of keyframes
};

/**
 * @brief CPU-based localization implementation
 */
class LocalizationCPU : public LocalizationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit LocalizationCPU(
    const LocalizationCPUParams& params = LocalizationCPUParams());
  virtual ~LocalizationCPU() override;

  // Implement base class interface
  virtual EstimationFrame::ConstPtr localize_frame(
    const PreprocessedFrame::Ptr&           frame,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;

  virtual bool load_global_map(const std::string& map_path) override;

  virtual bool set_initial_pose(const Eigen::Isometry3d& initial_pose) override;

  virtual double get_localization_confidence() const override;

  virtual void reset() override;

  virtual Eigen::Isometry3d get_current_pose() const override;

  virtual bool is_initialized() const override;

  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames()
    override;

private:
  // Core matching methods
  MatchingResult perform_scan_to_scan_matching(
    const PreprocessedFrame::Ptr& current_frame);

  MatchingResult perform_scan_to_map_matching(
    const PreprocessedFrame::Ptr& current_frame);

  MatchingResult decide_matching_strategy(const MatchingResult& s2s_result,
                                          const MatchingResult& s2m_result,
                                          double                s2s_score,
                                          double                s2m_score);

  MatchingResult create_hybrid_result(const MatchingResult& s2s_result,
                                      const MatchingResult& s2m_result,
                                      double                s2s_score,
                                      double                s2m_score);

  // Graph optimization
  void update_pose_graph(
    const MatchingResult&                   matching_result,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames);
  gtsam::noiseModel::Base::shared_ptr create_adaptive_noise_model(
    const MatchingResult& matching_result);

  // Marginalization
  void find_marginalized_frames(
    int                                     current_frame_id,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames);
  void update_frame_estimates();

  // Keyframe management
  bool should_add_keyframe(const EstimationFrame::ConstPtr& frame);
  void add_keyframe(const EstimationFrame::ConstPtr& frame);
  void manage_keyframes();

  // Factor graph construction
  void add_localization_factors(const MatchingResult&        matching_result,
                                int                          current_key,
                                gtsam::NonlinearFactorGraph& factors);
  void add_temporal_factors(const MatchingResult&        matching_result,
                            int                          current_key,
                            gtsam::NonlinearFactorGraph& factors);
  void add_map_constraint_factors(const MatchingResult&        matching_result,
                                  int                          current_key,
                                  gtsam::NonlinearFactorGraph& factors);
  void add_loop_closure_factors(const MatchingResult&        matching_result,
                                int                          current_key,
                                gtsam::NonlinearFactorGraph& factors);

  // Utility methods
  Eigen::Isometry3d predict_pose_from_velocity();
  void              update_velocity_estimate(const Eigen::Isometry3d& new_pose);
  LocalizationFrame::Ptr create_localization_frame(
    const PreprocessedFrame::Ptr& preprocessed_frame,
    const MatchingResult&         matching_result);

private:
  // Parameters
  LocalizationCPUParams params_;

  // Core components
  std::unique_ptr<MapManager>        map_manager_;
  std::unique_ptr<MatchingEvaluator> evaluator_;

  std::unique_ptr<gtsam_points::FixedLagSmootherExt> smoother_;

  // State management
  std::deque<LocalizationFrame::ConstPtr>  recent_frames_;
  std::vector<LocalizationFrame::ConstPtr> keyframes_;
  gtsam::Values                            current_estimates_;
  int                                      marginalized_cursor_ = 0;

  // Scan-to-scan target
  std::shared_ptr<gtsam_points::iVox> target_ivox_;

  // Velocity estimation
  Eigen::Vector3d linear_velocity_  = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
  double          last_timestamp_   = 0.0;

  // Statistics
  int                 frame_count_           = 0;
  double              total_processing_time_ = 0.0;
  std::vector<double> confidence_history_;

  // Random number generator
  std::mt19937 mt_;

  // Logging
  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace glim
