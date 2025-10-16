#pragma once

#include <mutex>
#include <thread>
#include <atomic>

#include <glim/util/concurrent_vector.hpp>
#include <glim/localization/localization_base.hpp>
#include <glim/preprocess/preprocessed_frame.hpp>
#include <glim/odometry/estimation_frame.hpp>

namespace spdlog {
class logger;
}

namespace glim {

/**
 * @brief Asynchronous localization executor to wrap and asynchronously run LocalizationBase
 * @note  All the exposed public methods are thread-safe
 *
 * This class provides asynchronous processing for localization, similar to AsyncOdometryEstimation.
 * It runs localization in a separate thread and provides thread-safe input/output queues.
 */
class AsyncLocalization {
public:
  /**
   * @brief Construct a new Async Localization object
   * @param localization  Localization implementation to be wrapped
   */
  explicit AsyncLocalization(const std::shared_ptr<LocalizationBase>& localization);

  /**
   * @brief Destroy the Async Localization object
   */
  ~AsyncLocalization();

  /**
   * @brief Insert a preprocessed point cloud frame for localization
   * @param frame  Preprocessed point cloud
   */
  void insert_frame(const PreprocessedFrame::Ptr& frame);

  /**
   * @brief Load global map for localization
   * @param map_path  Path to the global map file
   * @return true if map loaded successfully
   */
  bool load_global_map(const std::string& map_path);

  /**
   * @brief Set initial pose for localization
   * @param initial_pose  Initial pose estimate
   * @return true if pose set successfully
   */
  bool set_initial_pose(const Eigen::Isometry3d& initial_pose);

  /**
   * @brief Wait for the localization thread to finish
   */
  void join();

  /**
   * @brief Get the size of the input frame queue
   * @return Number of frames waiting to be processed
   */
  int workload() const;

  /**
   * @brief Get the localization results
   * @param localization_results  Localization results (frames with estimated poses)
   * @param marginalized_frames   Marginalized frames from optimization
   */
  void get_results(std::vector<EstimationFrame::ConstPtr>& localization_results,
                   std::vector<EstimationFrame::ConstPtr>& marginalized_frames);

  /**
   * @brief Get current localization confidence
   * @return Confidence score [0, 1]
   */
  double get_localization_confidence() const;

  /**
   * @brief Get current estimated pose
   * @return Current pose in world frame
   */
  Eigen::Isometry3d get_current_pose() const;

  /**
   * @brief Check if localization is initialized
   * @return true if initialized (map loaded and initial pose set)
   */
  bool is_initialized() const;

  /**
   * @brief Reset the localization system
   */
  void reset();

private:
  void run();

private:
  std::atomic_bool kill_switch;       ///< Flag to stop the thread immediately (Hard kill switch)
  std::atomic_bool end_of_sequence;   ///< Flag to stop the thread when input queue is empty (Soft kill switch)
  std::thread      thread;

  // Input queue
  ConcurrentVector<PreprocessedFrame::Ptr> input_frame_queue;

  // Output queues
  ConcurrentVector<EstimationFrame::ConstPtr> output_localization_results;
  ConcurrentVector<EstimationFrame::ConstPtr> output_marginalized_frames;

  std::atomic_int                      internal_frame_queue_size;
  std::shared_ptr<LocalizationBase>    localization;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim
