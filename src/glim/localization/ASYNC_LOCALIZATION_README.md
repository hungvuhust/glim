# AsyncLocalization - Asynchronous Localization Module

## Overview

`AsyncLocalization` is a thread-safe wrapper around `LocalizationBase` that enables asynchronous processing of point cloud frames for localization. It follows the same design pattern as `AsyncOdometryEstimation` to provide non-blocking, concurrent localization capabilities.

## Features

- **Asynchronous Processing**: Localization runs in a separate thread, allowing your main application to continue without blocking
- **Thread-Safe Interface**: All public methods are thread-safe and can be called from multiple threads
- **Queue Management**: Internal queues manage input frames and output results
- **Performance Monitoring**: Built-in workload tracking to monitor queue sizes
- **Graceful Shutdown**: Clean shutdown mechanism with thread joining

## Architecture

```
┌─────────────────┐
│  Main Thread    │
│                 │
│  insert_frame() │◄─── Point Cloud Data
│                 │
└────────┬────────┘
         │ Thread-safe queue
         ▼
┌─────────────────┐
│ Background      │
│ Thread          │
│                 │
│ Localization    │
│ Processing      │
└────────┬────────┘
         │ Thread-safe queue
         ▼
┌─────────────────┐
│  Main Thread    │
│                 │
│  get_results()  │───► Localization Results
│                 │
└─────────────────┘
```

## Usage

### Basic Setup

```cpp
#include <glim/localization/async_localization.hpp>
#include <glim/localization/localization_cpu.hpp>

// 1. Create localization parameters
glim::LocalizationCPUParams params;
params.registration_type = "GICP";
params.max_iterations = 20;
params.ivox_resolution = 0.5;

// 2. Create localization instance
auto localization = std::make_shared<glim::LocalizationCPU>(params);

// 3. Wrap with AsyncLocalization
auto async_loc = std::make_shared<glim::AsyncLocalization>(localization);

// 4. Load global map
if (!async_loc->load_global_map("/path/to/map.pcd")) {
    std::cerr << "Failed to load map" << std::endl;
    return;
}

// 5. Set initial pose
Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
initial_pose.translation() << x, y, z;  // Your initial position
async_loc->set_initial_pose(initial_pose);
```

### Processing Frames

```cpp
// In your sensor callback (ROS, custom loop, etc.)
void on_point_cloud_received(const PreprocessedFrame::Ptr& frame) {
    // Simply insert - processing happens asynchronously
    async_loc->insert_frame(frame);

    // Optional: Check workload
    int queue_size = async_loc->workload();
    if (queue_size > 20) {
        std::cout << "Warning: Queue size = " << queue_size << std::endl;
    }
}
```

### Retrieving Results

```cpp
// Periodically retrieve and process results
void process_localization_results() {
    std::vector<EstimationFrame::ConstPtr> results;
    std::vector<EstimationFrame::ConstPtr> marginalized;

    // Get all new results (clears internal queue)
    async_loc->get_results(results, marginalized);

    // Process results
    for (const auto& frame : results) {
        Eigen::Isometry3d pose = frame->T_world_sensor();
        double confidence = async_loc->get_localization_confidence();

        // Use pose and confidence...
        publish_pose(pose, confidence);
    }
}
```

### Shutdown

```cpp
// Graceful shutdown
async_loc->join();  // Waits for background thread to finish
```

## API Reference

### Constructor

```cpp
AsyncLocalization(const std::shared_ptr<LocalizationBase>& localization)
```

Creates an async localization wrapper. Starts the background processing thread.

### Input Methods

#### `insert_frame()`

```cpp
void insert_frame(const PreprocessedFrame::Ptr& frame)
```

Insert a preprocessed point cloud frame for localization. **Thread-safe**. The frame will be processed asynchronously in the background thread.

#### `load_global_map()`

```cpp
bool load_global_map(const std::string& map_path)
```

Load a global map from file. **Thread-safe**. Can be called while localization is running.

**Supported formats:**
- `.pcd` - Single PCD file
- Directory containing multiple PCD submaps

#### `set_initial_pose()`

```cpp
bool set_initial_pose(const Eigen::Isometry3d& initial_pose)
```

Set the initial pose estimate. **Thread-safe**. Should be called before processing frames.

### Output Methods

#### `get_results()`

```cpp
void get_results(
    std::vector<EstimationFrame::ConstPtr>& localization_results,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames)
```

Get all available localization results. **Thread-safe**. Clears the internal output queues.

- `localization_results`: Frames with estimated poses
- `marginalized_frames`: Frames that were marginalized from the optimization

#### `get_current_pose()`

```cpp
Eigen::Isometry3d get_current_pose() const
```

Get the most recent estimated pose. **Thread-safe**.

#### `get_localization_confidence()`

```cpp
double get_localization_confidence() const
```

Get the current localization confidence score [0, 1]. **Thread-safe**.

### Status Methods

#### `is_initialized()`

```cpp
bool is_initialized() const
```

Check if localization is ready (map loaded and initial pose set). **Thread-safe**.

#### `workload()`

```cpp
int workload() const
```

Get the number of frames waiting to be processed. **Thread-safe**. Useful for monitoring performance and detecting bottlenecks.

### Control Methods

#### `reset()`

```cpp
void reset()
```

Reset the localization system. **Thread-safe**. Clears all queues and resets the internal state.

#### `join()`

```cpp
void join()
```

Wait for the background thread to finish processing. Blocks until all queued frames are processed or thread is killed.

## Performance Considerations

### Queue Size Monitoring

Monitor the input queue size to ensure localization keeps up with incoming data:

```cpp
int queue_size = async_loc->workload();
if (queue_size > threshold) {
    // Localization is falling behind
    // Consider: reducing input rate, adjusting parameters, or dropping frames
}
```

### Typical Performance

- **Processing Rate**: Depends on point cloud density and registration parameters
- **Latency**: Minimal (queue-based) + processing time per frame
- **Memory**: Proportional to queue size

### Optimization Tips

1. **Adjust Parameters**:
   - Reduce `max_iterations` for faster convergence
   - Increase `ivox_resolution` for faster nearest neighbor search
   - Use GICP instead of VGICP if covariance estimation not needed

2. **Frame Rate Management**:
   - If workload grows, consider downsampling input frames
   - Use temporal filtering to skip similar consecutive frames

3. **Map Size**:
   - Larger maps require more memory and slower nearest neighbor search
   - Consider using hierarchical maps or local map extraction

## Integration Examples

### ROS 2 Integration

```cpp
class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode() : Node("localization_node") {
        // Create async localization
        auto loc = std::make_shared<glim::LocalizationCPU>(params_);
        async_loc_ = std::make_shared<glim::AsyncLocalization>(loc);

        // Load map
        async_loc_->load_global_map(map_path_);

        // Subscriber
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points", 10,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                auto frame = preprocess(msg);
                async_loc_->insert_frame(frame);
            });

        // Timer for publishing results
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { publish_results(); });
    }

    ~LocalizationNode() {
        async_loc_->join();
    }

private:
    void publish_results() {
        std::vector<EstimationFrame::ConstPtr> results, marg;
        async_loc_->get_results(results, marg);

        for (const auto& frame : results) {
            // Publish pose
            publish_pose(frame->T_world_sensor());
        }
    }

    std::shared_ptr<glim::AsyncLocalization> async_loc_;
};
```

### Custom Loop

```cpp
// Producer thread (sensor data)
void sensor_thread() {
    while (running) {
        auto frame = get_sensor_data();
        async_loc->insert_frame(frame);
    }
}

// Consumer thread (results)
void result_thread() {
    while (running) {
        std::vector<EstimationFrame::ConstPtr> results, marg;
        async_loc->get_results(results, marg);

        for (const auto& frame : results) {
            process_result(frame);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
```

## Comparison with Synchronous Localization

| Feature | Synchronous | Asynchronous |
|---------|-------------|--------------|
| Blocking | Yes | No |
| Thread-safe | No | Yes |
| Latency | Lower | Slightly higher (queuing) |
| Throughput | Limited by processing | Can buffer frames |
| Complexity | Simple | More complex |
| Use Case | Single-threaded apps | Multi-threaded, ROS, high-frequency sensors |

## Troubleshooting

### High Queue Size

**Symptom**: `workload()` returns large numbers (>20)

**Solutions**:
- Reduce input frame rate
- Optimize localization parameters
- Increase processing speed (fewer iterations, coarser voxel grid)
- Drop frames when queue is full

### Low Confidence

**Symptom**: `get_localization_confidence()` returns low values (<0.5)

**Solutions**:
- Check initial pose accuracy
- Verify map quality and coverage
- Adjust registration parameters
- Check for significant drift from initial pose

### Memory Growth

**Symptom**: Memory usage increases over time

**Solutions**:
- Call `get_results()` regularly to clear output queues
- Monitor and limit input queue size
- Check for frame reference leaks in your code

## Thread Safety

All public methods are thread-safe and can be called from multiple threads:

- ✅ `insert_frame()` - Thread-safe
- ✅ `get_results()` - Thread-safe
- ✅ `load_global_map()` - Thread-safe
- ✅ `set_initial_pose()` - Thread-safe
- ✅ `get_current_pose()` - Thread-safe
- ✅ `get_localization_confidence()` - Thread-safe
- ✅ `is_initialized()` - Thread-safe
- ✅ `workload()` - Thread-safe
- ✅ `reset()` - Thread-safe

## License

Same as the glim project.

## See Also

- `LocalizationBase` - Base interface for localization
- `LocalizationCPU` - CPU implementation
- `AsyncOdometryEstimation` - Similar async wrapper for odometry
- `ConcurrentVector` - Thread-safe queue implementation
