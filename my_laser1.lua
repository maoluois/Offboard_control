include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_tracked_pose=true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
--MAP_BUILDER.num_background_threads =4

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2 --积累几帧激光数据作为一个标准单位scan
--TRAJECTORY_BUILDER_2D.min_range = 0.2  --激光的最近有效距离
--TRAJECTORY_BUILDER_2D.max_range = 25.   --激光最远的有效距离
--TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5. --无效激光数据设置距离为该数值
TRAJECTORY_BUILDER_2D.use_imu_data = true  --是否使用imu数据

--TRAJECTORY_BUILDER_2D.submaps.num_range_data = 1  -- 每个子图只用一次扫描
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 50  -- 增大匹配权重

--TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.03  --//尽量小点  // 如果移动距离过小, 或者时间过短, 不进行地图的更新
--TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)
--TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 3
 
--POSE_GRAPH.optimization_problem.huber_scale = 1e2  --鲁棒核函数，去噪
 

--POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
--POSE_GRAPH.constraint_builder.sampling_ratio = 0 -- little
--POSE_GRAPH.constraint_builder.min_score = 0.73 -- 0.6 - 0.8
--POSE_GRAPH.global_constraint_search_after_n_seconds = 15.

return options
