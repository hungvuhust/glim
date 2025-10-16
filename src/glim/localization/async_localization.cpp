#include <glim/localization/async_localization.hpp>

#include <spdlog/spdlog.h>
#include <glim/util/logging.hpp>

namespace glim {

AsyncLocalization::AsyncLocalization(
  const std::shared_ptr<LocalizationBase>& localization)
  : localization(localization),
    logger(create_module_logger("loc")) {
  kill_switch               = false;
  end_of_sequence           = false;
  internal_frame_queue_size = 0;
  thread                    = std::thread([this] { run(); });
}

AsyncLocalization::~AsyncLocalization() {
  kill_switch = true;
  join();
}

void AsyncLocalization::insert_frame(const PreprocessedFrame::Ptr& frame) {
  input_frame_queue.push_back(frame);
}

bool AsyncLocalization::load_global_map(const std::string& map_path) {
  if (!localization) {
    logger->error("Localization is not initialized");
    return false;
  }
  return localization->load_global_map(map_path);
}

bool AsyncLocalization::set_initial_pose(const Eigen::Isometry3d& initial_pose) {
  if (!localization) {
    logger->error("Localization is not initialized");
    return false;
  }
  return localization->set_initial_pose(initial_pose);
}

void AsyncLocalization::join() {
  end_of_sequence = true;
  if (thread.joinable()) {
    thread.join();
  }
}

int AsyncLocalization::workload() const {
  return input_frame_queue.size() + internal_frame_queue_size;
}

void AsyncLocalization::get_results(
  std::vector<EstimationFrame::ConstPtr>& localization_results,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  localization_results = output_localization_results.get_all_and_clear();
  marginalized_frames  = output_marginalized_frames.get_all_and_clear();
}

double AsyncLocalization::get_localization_confidence() const {
  if (!localization) {
    return 0.0;
  }
  return localization->get_localization_confidence();
}

Eigen::Isometry3d AsyncLocalization::get_current_pose() const {
  if (!localization) {
    return Eigen::Isometry3d::Identity();
  }
  return localization->get_current_pose();
}

bool AsyncLocalization::is_initialized() const {
  if (!localization) {
    return false;
  }
  return localization->is_initialized();
}

void AsyncLocalization::reset() {
  if (localization) {
    localization->reset();
  }
  // Clear input/output queues
  input_frame_queue.clear();
  output_localization_results.clear();
  output_marginalized_frames.clear();
  internal_frame_queue_size = 0;
}

void AsyncLocalization::run() {
  std::deque<PreprocessedFrame::Ptr> raw_frames;

  logger->info("AsyncLocalization thread started");

  while (!kill_switch) {
    // Get all new frames from input queue
    auto new_raw_frames = input_frame_queue.get_all_and_clear();
    raw_frames.insert(raw_frames.end(),
                      new_raw_frames.begin(),
                      new_raw_frames.end());
    internal_frame_queue_size = raw_frames.size();

    // Check if there's work to do
    if (raw_frames.empty()) {
      if (end_of_sequence) {
        logger->info("End of sequence reached, exiting AsyncLocalization thread");
        break;
      }

      // Sleep briefly to avoid busy waiting
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    // Check if localization is initialized
    if (!localization->is_initialized()) {
      logger->warn(
        "Localization not initialized (map not loaded or initial pose not set), "
        "skipping {} frames",
        raw_frames.size());
      raw_frames.clear();
      internal_frame_queue_size = 0;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    // Process frames
    while (!raw_frames.empty()) {
      const auto&                            frame = raw_frames.front();
      std::vector<EstimationFrame::ConstPtr> marginalized;

      try {
        // Perform localization
        auto localization_result =
          localization->localize_frame(frame, marginalized);

        if (localization_result) {
          output_localization_results.push_back(localization_result);
          logger->debug("Localized frame at time {:.3f}, confidence: {:.3f}",
                       frame->stamp,
                       localization->get_localization_confidence());
        } else {
          logger->warn("Localization failed for frame at time {:.3f}",
                      frame->stamp);
        }

        // Add marginalized frames to output
        if (!marginalized.empty()) {
          output_marginalized_frames.insert(marginalized);
          logger->debug("Marginalized {} frames", marginalized.size());
        }

      } catch (const std::exception& e) {
        logger->error("Exception during localization: {}", e.what());
      }

      raw_frames.pop_front();
      internal_frame_queue_size = raw_frames.size();
    }
  }

  // Process any remaining frames after receiving end_of_sequence
  if (!raw_frames.empty()) {
    logger->info("Processing {} remaining frames before shutdown",
                raw_frames.size());

    while (!raw_frames.empty() && localization->is_initialized()) {
      const auto&                            frame = raw_frames.front();
      std::vector<EstimationFrame::ConstPtr> marginalized;

      try {
        auto localization_result =
          localization->localize_frame(frame, marginalized);

        if (localization_result) {
          output_localization_results.push_back(localization_result);
        }

        if (!marginalized.empty()) {
          output_marginalized_frames.insert(marginalized);
        }

      } catch (const std::exception& e) {
        logger->error("Exception during final localization: {}", e.what());
      }

      raw_frames.pop_front();
    }
  }

  // Get any remaining frames from the localization system
  auto remaining = localization->get_remaining_frames();
  if (!remaining.empty()) {
    output_marginalized_frames.insert(remaining);
    logger->info("Retrieved {} remaining frames from localization",
                remaining.size());
  }

  logger->info("AsyncLocalization thread terminated");
}

}  // namespace glim
