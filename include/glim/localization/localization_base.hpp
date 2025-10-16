#pragma once

#include <memory>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <glim/preprocess/preprocessed_frame.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/util/load_module.hpp>

namespace glim {

/**
 * @brief Base class for localization modules
 *
 * This class provides the interface for localization on a given map.
 * It supports hybrid approach combining scan-to-scan and scan-to-map matching.
 */
class LocalizationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocalizationBase()          = default;
  virtual ~LocalizationBase() = default;

  /**
   * @brief Localize a new frame against the global map
   * @param frame Input preprocessed frame
   * @param marginalized_frames [out] Marginalized localization frames
   * @return Localized estimation frame with pose and confidence
   */
  virtual EstimationFrame::ConstPtr localize_frame(
    const PreprocessedFrame::Ptr&           frame,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames) = 0;

  /**
   * @brief Load global map from file or data
   * @param map_path Path to map file or map data
   * @return True if map loaded successfully
   */
  virtual bool load_global_map(const std::string& map_path) = 0;

  /**
   * @brief Set initial pose for localization
   * @param initial_pose Initial pose in world frame
   * @return True if initial pose set successfully
   */
  virtual bool set_initial_pose(const Eigen::Isometry3d& initial_pose) = 0;

  /**
   * @brief Get current localization status
   * @return Current localization confidence [0.0, 1.0]
   */
  virtual double get_localization_confidence() const = 0;

  /**
   * @brief Reset localization state
   */
  virtual void reset() = 0;

  /**
   * @brief Get current estimated pose
   * @return Current pose in world frame
   */
  virtual Eigen::Isometry3d get_current_pose() const = 0;

  /**
   * @brief Check if localization is initialized
   * @return True if localization is ready
   */
  virtual bool is_initialized() const = 0;

  /**
   * @brief Get remaining non-marginalized frames (called at sequence end)
   * @return Vector of remaining localization frames
   */
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() {
    return std::vector<EstimationFrame::ConstPtr>();
  }

  static std::shared_ptr<LocalizationBase> load_module(
    const std::string& module_name) {
    return load_module_from_so<LocalizationBase>(module_name,
                                                 "create_localization_module");
  }

protected:
  // Current localization state
  Eigen::Isometry3d current_pose_       = Eigen::Isometry3d::Identity();
  double            current_confidence_ = 0.0;
  bool              initialized_        = false;
};

}  // namespace glim
