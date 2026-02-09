# Hướng dẫn sử dụng các file JSON Config trong GLIM

## Tổng quan

GLIM sử dụng hệ thống config modular, cho phép thay đổi các module khác nhau mà không cần recompile.

```
config.json (Master config)
    ├── config_sensors.json
    ├── config_preprocess.json
    ├── config_odometry_*.json
    ├── config_sub_mapping_*.json
    ├── config_global_mapping_*.json
    └── config_viewer.json
```

---

## 1. `config.json` - Master Config

File chính để chọn các config modules khác.

```json
{
  "global": {
    "config_path": "",
    "config_sensors": "config_sensors.json",
    "config_preprocess": "config_preprocess.json",
    "config_odometry": "config_odometry_cpu.json",
    "config_sub_mapping": "config_sub_mapping_cpu.json",
    "config_global_mapping": "config_global_mapping_cpu.json",
    "config_viewer": "config_viewer.json"
  }
}
```

### Các preset phổ biến:

| Cấu hình | Odometry | SubMapping | GlobalMapping |
|----------|----------|------------|---------------|
| **CPU (cơ bản)** | `config_odometry_cpu.json` | `config_sub_mapping_cpu.json` | `config_global_mapping_cpu.json` |
| **GPU (nhanh)** | `config_odometry_gpu.json` | `config_sub_mapping_gpu.json` | `config_global_mapping_gpu.json` |
| **Lightweight** | `config_odometry_cpu.json` | `config_sub_mapping_passthrough.json` | `config_global_mapping_pose_graph.json` |
| **CT-ICP** | `config_odometry_ct.json` | `config_sub_mapping_passthrough.json` | `config_global_mapping_pose_graph.json` |

---

## 2. `config_sensors.json` - Cấu hình cảm biến

### IMU Config

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `imu_acc_noise` | Nhiễu accelerometer | `0.005` |
| `imu_gyro_noise` | Nhiễu gyroscope | `0.002` |
| `imu_int_noise` | Nhiễu integration | `0.001` |
| `imu_bias_noise` | Nhiễu bias random walk | `1e-5` |

### LiDAR Config

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `global_shutter_lidar` | `true` = không deskew | `false` |
| `T_lidar_imu` | Transform LiDAR→IMU [x,y,z,qx,qy,qz,qw] | `[0.006, -0.012, 0.008, 0, 0, 0, 1]` |
| `intensity_field` | Tên field intensity | `"intensity"` |
| `ring_field` | Tên field ring/laser ID | `"ring"` |

### Per-point Time Settings

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `autoconf_perpoint_times` | Tự động config timestamps | `true` |
| `perpoint_relative_time` | Timestamps tương đối | `true` |
| `perpoint_time_scale` | Scale (1.0=sec, 1e-9=ns) | `1.0` |

### Ví dụ T_lidar_imu cho các sensor:

| Sensor | T_lidar_imu |
|--------|-------------|
| Ouster OS0 | `[0.006, -0.012, 0.008, 0, 0, 0, 1]` |
| Livox Avia | `[0, 0, 0, 0, 0, 0, 1]` |
| Realsense L515 | `[-0.012, 0.016, 0.001, 0, 0, 0, 1]` |

---

## 3. `config_preprocess.json` - Tiền xử lý point cloud

### Distance Filter

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `distance_near_thresh` | Loại điểm gần hơn (m) | `0.5` |
| `distance_far_thresh` | Loại điểm xa hơn (m) | `100.0` |

### Downsampling

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `use_random_grid_downsampling` | Dùng random grid (khuyến nghị) | `true` |
| `downsample_resolution` | Kích thước voxel (m) | `1.0` |
| `random_downsample_target` | Số điểm mục tiêu | `10000` |
| `random_downsample_rate` | Tỷ lệ sampling (nếu target ≤ 0) | `0.1` |

### Outlier Removal (Optional)

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `enable_outlier_removal` | Bật/tắt | `false` |
| `outlier_removal_k` | Số neighbors | `10` |
| `outlier_std_mul_factor` | Ngưỡng std | `1.0` |

### Crop Box (Optional)

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `enable_cropbox_filter` | Bật/tắt | `false` |
| `crop_bbox_frame` | Frame tham chiếu | `"lidar"` |
| `crop_bbox_min` | Góc min [x,y,z] | `[-1, -1, -1]` |
| `crop_bbox_max` | Góc max [x,y,z] | `[1, 1, 1]` |

### Khác

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `k_correspondences` | Số neighbors cho covariance | `10` |
| `num_threads` | Số threads | `2` |

---

## 4. `config_odometry_*.json` - Ước lượng Odometry

### Có 3 versions:

| File | Mô tả |
|------|-------|
| `config_odometry_cpu.json` | GICP + iVox trên CPU |
| `config_odometry_gpu.json` | VGICP trên GPU |
| `config_odometry_ct.json` | Continuous-Time ICP |

### Initialization

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `initialization_mode` | `"LOOSE"` hoặc `"NAIVE"` | `"LOOSE"` |
| `initialization_window_size` | Thời gian khởi tạo (s) | `3.0` |
| `init_pose_damping_scale` | Damping cho pose đầu | `1e10` |

### Optimizer (iSAM2)

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `smoother_lag` | Cửa sổ smoothing (s) | `3.0` |
| `use_isam2_dogleg` | Dùng dogleg (robust hơn) | `true` |
| `isam2_relinearize_skip` | Relinearize mỗi N lần | `1` |
| `isam2_relinearize_thresh` | Ngưỡng relinearize | `0.1` |
| `fix_imu_bias` | Cố định IMU bias | `false` |

### Registration (CPU version)

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `registration_type` | `"GICP"` hoặc `"VGICP"` | `"GICP"` |
| `max_iterations` | Số vòng lặp max | `10` |
| `ivox_resolution` | Kích thước iVox (m) | `1.0` |
| `ivox_min_dist` | Khoảng cách min trong voxel | `0.1` |

### Registration (GPU version)

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `voxel_resolution` | Độ phân giải voxel | `0.25` |
| `voxelmap_levels` | Số tầng multi-resolution | `2` |
| `voxelmap_scaling_factor` | Hệ số scale giữa tầng | `2.0` |

### Keyframe Management (GPU version)

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `keyframe_update_strategy` | `"OVERLAP"` / `"DISPLACEMENT"` | `"OVERLAP"` |
| `max_num_keyframes` | Số keyframes tối đa | `15` |
| `keyframe_max_overlap` | Ngưỡng overlap để thêm keyframe | `0.7` |

---

## 5. `config_sub_mapping_*.json` - Sub Mapping

### Có 3 versions:

| File | Mô tả |
|------|-------|
| `config_sub_mapping_cpu.json` | Optimization trên CPU |
| `config_sub_mapping_gpu.json` | Optimization trên GPU |
| `config_sub_mapping_passthrough.json` | Không optimization |

### General

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `enable_imu` | Dùng IMU factors | `true` |
| `enable_optimization` | Bật optimization | `true` |

### Keyframe Strategy

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `max_num_keyframes` | Số keyframes/submap | `15` |
| `keyframe_update_strategy` | `"OVERLAP"` / `"DISPLACEMENT"` | `"OVERLAP"` |
| `keyframe_update_min_points` | Min points để làm keyframe | `500` |
| `max_keyframe_overlap` | Ngưỡng overlap | `0.6` |

### Registration Factors

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `create_between_factors` | Tạo between factors | `true` |
| `between_registration_type` | `"GICP"` / `"NONE"` | `"GICP"` |
| `registration_error_factor_type` | `"VGICP"` / `"VGICP_GPU"` | `"VGICP"` |
| `keyframe_voxel_resolution` | Voxel resolution | `0.25` |
| `keyframe_voxelmap_levels` | Số tầng | `2` |

### Post Processing

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `submap_downsample_resolution` | Downsample resolution | `0.3` |
| `submap_voxel_resolution` | Voxel cho global mapping | `0.5` |

---

## 6. `config_global_mapping_*.json` - Global Mapping

### Có 3 versions:

| File | Mô tả |
|------|-------|
| `config_global_mapping_cpu.json` | VGICP trên CPU |
| `config_global_mapping_gpu.json` | VGICP trên GPU |
| `config_global_mapping_pose_graph.json` | Pose graph với explicit loop detection |

### CPU/GPU Version - Implicit Loop

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `enable_imu` | Dùng IMU | `true` |
| `enable_optimization` | Bật optimization | `true` |
| `max_implicit_loop_distance` | Khoảng cách max cho loop (m) | `100.0` |
| `min_implicit_loop_overlap` | Overlap min để tạo factor | `0.2` |

### Pose Graph Version - Explicit Loop Detection

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `registration_type` | `"GICP"` / `"VGICP"` | `"VGICP"` |
| `min_travel_dist` | Quãng đường min giữa submaps (m) | `50.0` |
| `max_neighbor_dist` | Khoảng cách max để xét loop (m) | `10.0` |
| `min_inliear_fraction` | Tỷ lệ inlier min để accept | `0.5` |
| `gicp_max_correspondence_dist` | Correspondence distance (m) | `2.0` |
| `vgicp_voxel_resolution` | Voxel resolution (m) | `2.0` |

### Loop Factor Settings

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `loop_factor_stddev` | Độ lệch chuẩn | `0.1` |
| `loop_factor_robust_width` | Robust kernel width | `1.0` |
| `loop_candidate_buffer_size` | Buffer size | `100` |
| `num_threads` | Số threads | `2` |

### Optimizer

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `use_isam2_dogleg` | Dùng dogleg | `false` |
| `isam2_relinearize_skip` | Relinearize skip | `1` |
| `isam2_relinearize_thresh` | Relinearize threshold | `0.1` |

---

## 7. `config_viewer.json` - Viewer Settings

| Tham số | Mô tả | Giá trị mẫu |
|---------|-------|-------------|
| `viewer_width` | Chiều rộng cửa sổ | `1920` |
| `viewer_height` | Chiều cao cửa sổ | `1080` |
| `default_z_range` | Range Z mặc định [min, max] | `[-2.0, 4.0]` |
| `enable_partial_rendering` | Render từng phần (GPU yếu) | `false` |
| `partial_rendering_budget` | Số points/frame | `1024` |
| `point_shape_circle` | Điểm hình tròn | `true` |
| `point_size` | Kích thước điểm | `10.0` |
| `points_alpha` | Độ trong suốt points | `0.5` |
| `factors_alpha` | Độ trong suốt factors | `0.5` |

---

## Tips & Tricks

### Chạy nhanh trên máy yếu:

```json
// config.json
"config_odometry": "config_odometry_cpu.json",
"config_sub_mapping": "config_sub_mapping_passthrough.json",
"config_global_mapping": "config_global_mapping_pose_graph.json"
```

### Chạy chính xác nhất:

```json
// config.json
"config_odometry": "config_odometry_gpu.json",
"config_sub_mapping": "config_sub_mapping_gpu.json",
"config_global_mapping": "config_global_mapping_gpu.json"
```

### Xử lý large loops:

```json
// config_global_mapping_pose_graph.json
"min_travel_dist": 30.0,
"max_neighbor_dist": 25.0,
"min_inliear_fraction": 0.3
```

### Giảm drift:

```json
// config_odometry_*.json
"smoother_lag": 5.0,
"use_isam2_dogleg": true

// config_sub_mapping_*.json
"max_num_keyframes": 20,
"max_keyframe_overlap": 0.5
```

---

## So sánh các Preset

| Preset | CPU Usage | RAM | Accuracy | Loop Closure |
|--------|-----------|-----|----------|--------------|
| CPU + CPU + CPU | Cao | Trung bình | Cao | Implicit |
| GPU + GPU + GPU | Thấp (GPU) | Cao | Rất cao | Implicit |
| CPU + Passthrough + PoseGraph | Thấp | Thấp | Trung bình | Explicit |
| CT + Passthrough + PoseGraph | Trung bình | Thấp | Cao | Explicit |
