# Monitor-only vertical coverage evidence from obstacle_sector_node. This is not a PX4/uORB interface.
uint64 timestamp              # [us] node publication time
uint64 sample_timestamp       # [us] map acquisition time

float32 mount_roll_rad        # [rad] calibrated sensor mount roll, sensor -> body FLU
float32 mount_pitch_down_rad  # [rad] positive boresight-down mount pitch
float32 mount_yaw_left_rad    # [rad] positive boresight-left mount yaw
float32 sensor_elevation_min_rad  # [rad] lowest ring in the sensor frame
float32 sensor_elevation_max_rad  # [rad] highest ring in the sensor frame
float32 height_up_m             # [m] upper edge of the collision band
float32 height_down_m           # [m] lower edge of the collision band
float32 attitude_allowance_rad  # [rad] band-test attitude allowance

uint32 required_rays           # rays required in each vertical half
bool packet_continuity_valid   # false means free-space certification is unavailable
uint32[72] up_ray_count        # free + obstacle rays in the upper half per sector
uint32[72] down_ray_count      # free + obstacle rays in the lower half per sector
bool[72] rear_only             # true when the sector only requires its upper half
bool[72] coverage_ok            # ray-count coverage decision before continuity gating
