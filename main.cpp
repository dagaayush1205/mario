
auto setupRealsense(mem::Allocator& alloc, struct Config& config) -> struct RealsenseHandle*
{
  struct RealsenseHandle* handle = reinterpret_cast<struct RealsenseHandle*>(
    alloc.alloc(sizeof(struct RealsenseHandle)));

  new (&handle->pipe) rs2::pipeline();
  new (&handle->frame_q) rs2::frame_queue(); 
  new (&handle->pc) rs2::pointcloud(); 
  new (&handle->align) rs2::align(RS2_STREAM_COLOR);
  new (&handle->slam_cfg) stella_vslam::config("stellaconf.yaml");
  new (&handle->slam) stella_vslam::system(
    std::shared_ptr<stella_vslam::config>(&handle->slam_cfg, [](auto p) {}),
    "stellavocab.txt");

  handle->slam.startup();

  rs2::config stream_config;
  rs2::context ctx;
  float fov[2];

  auto devices = ctx.query_devices();
  if (devices.size() == 0) return nullptr;
  stream_config.enable_stream(rs2_stream::RS2_STREAM_COLOR,
                              0,
                              config.height,
                              config.width,
                              rs2_format::RS2_FORMAT_BGR8,
                              config.fps);
  stream_config.enable_stream(rs2_stream::RS2_STREAM_DEPTH,
                              0,
                              config.height,
                              config.width,
                              rs2_format::RS2_FORMAT_Z16,
                              config.fps);
  if (config.enable_imu) {
    stream_config.enable_stream(rs2_stream::RS2_STREAM_ACCEL,
                                RS2_FORMAT_MOTION_XYZ32F);
    stream_config.enable_stream(rs2_stream::RS2_STREAM_GYRO,
                                RS2_FORMAT_MOTION_XYZ32F);
  }
  rs2::pipeline_profile selection = handle->pipe.start(stream_config, handle->frame_q);
  // Get intrinsics from the stream: Depth Scale and FOV
  config.depth_scale = selection.get_device()
                           .query_sensors()
                           .front()
                           .as<rs2::depth_sensor>()
                           .get_depth_scale();
  auto depth_stream = selection.get_stream(rs2_stream::RS2_STREAM_DEPTH)
                          .as<rs2::video_stream_profile>();

  auto i = depth_stream.get_intrinsics();
  intrinsics = (rs2_intrinsics*) alloc.alloc(sizeof(i));
  *intrinsics = i;
  rs2_fov(&i, fov);
  config.fov[0] = (fov[0] * M_PI)/180.0f;
  config.fov[1] = (fov[1] * M_PI)/180.0f;

  int index = 0;
  for (rs2::sensor sensor : selection.get_device().query_sensors()) {
    if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
      ++index;
      if (index == 1) {
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
        // sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,50000);
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1); // emitter on for
        // depth information
      }
      if (index == 2) {
        // RGB camera
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
        // sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,50000);
      }

      // if (index == 3) {
      //   sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0);
      // }
    }
  }

  return handle;
}
