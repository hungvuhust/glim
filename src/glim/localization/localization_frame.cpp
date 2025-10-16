#include <glim/localization/localization_frame.hpp>

namespace glim {

LocalizationFrame::Ptr LocalizationFrame::from_estimation_frame(
  const EstimationFrame::ConstPtr& est_frame) {
  auto loc_frame = std::make_shared<LocalizationFrame>();

  // Copy base EstimationFrame data
  loc_frame->id                  = est_frame->id;
  loc_frame->stamp               = est_frame->stamp;
  loc_frame->T_lidar_imu         = est_frame->T_lidar_imu;
  loc_frame->T_world_lidar       = est_frame->T_world_lidar;
  loc_frame->T_world_imu         = est_frame->T_world_imu;
  loc_frame->v_world_imu         = est_frame->v_world_imu;
  loc_frame->imu_bias            = est_frame->imu_bias;
  loc_frame->raw_frame           = est_frame->raw_frame;
  loc_frame->imu_rate_trajectory = est_frame->imu_rate_trajectory;
  loc_frame->frame_id            = est_frame->frame_id;
  loc_frame->frame               = est_frame->frame;
  loc_frame->voxelmaps           = est_frame->voxelmaps;
  loc_frame->custom_data         = est_frame->custom_data;

  // Initialize localization-specific data with defaults
  loc_frame->localization_confidence = 0.0;
  loc_frame->is_keyframe             = false;
  loc_frame->processing_time         = 0.0;
  loc_frame->total_time              = 0.0;

  return loc_frame;
}

}  // namespace glim
