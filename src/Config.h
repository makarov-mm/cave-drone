#pragma once

// World / voxel grid
inline constexpr int   WORLD_NX = 192;
inline constexpr int   WORLD_NY = 56;
inline constexpr int   WORLD_NZ = 192;
inline constexpr float VOXEL_SIZE = 0.5f;   // meters per voxel

// Observed map
inline constexpr int CHUNK_SIZE = 16;

// Lidar
inline constexpr float LIDAR_RANGE = 10.0f; // meters
inline constexpr int   LIDAR_RAYS_PER_FRAME = 384;

// Lidar noise model
inline constexpr float LIDAR_NOISE_SIGMA = 0.05f;  // Gaussian range noise, m
inline constexpr float LIDAR_DROPOUT_PROB = 0.01f; // ray returns nothing
inline constexpr float LIDAR_OUTLIER_PROB = 0.001f; // spurious short return (dust)

// Log-odds occupancy fusion (integer log-odds, OctoMap style)
inline constexpr int LOGODDS_HIT = 2;             // added on a measured return
inline constexpr int LOGODDS_MISS = -1;           // added when a ray passes through
inline constexpr int LOGODDS_OCC_THRESHOLD = 3;   // classified Occupied at or above
inline constexpr int LOGODDS_FREE_THRESHOLD = -2; // classified Free at or below
inline constexpr int LOGODDS_MIN = -8;            // clamp: bounds re-classification latency
inline constexpr int LOGODDS_MAX = 16;

// Drone dynamics
inline constexpr float DRONE_CRUISE_SPEED = 3.0f;   // m/s
inline constexpr float DRONE_LOOKAHEAD = 1.3f;      // pursuit distance, m
inline constexpr float DRONE_VEL_GAIN = 3.5f;       // velocity loop P gain
inline constexpr float DRONE_MAX_HORIZ_ACC = 8.0f;  // m/s^2
inline constexpr float DRONE_MAX_TOTAL_ACC = 24.0f; // specific force clamp
inline constexpr float DRONE_ATTITUDE_RATE = 8.0f;  // attitude tracking speed
inline constexpr float DRONE_DRAG = 0.10f;          // quadratic drag coeff
// Reflex safety layer: if any cell along the velocity vector within the
// probe distance is known occupied, the controller brakes hard. Catches
// the failure mode the pursuit chord gate cannot: momentum or slow drift
// carrying the drone off the chord into a wall. The probe distance is the
// larger of a time horizon and a fixed minimum, so it works both at cruise
// (braking from 3 m/s takes ~0.19 s over ~0.56 m) and at hover drift.
inline constexpr float DRONE_BRAKE_HORIZON = 0.35f;
inline constexpr float DRONE_BRAKE_MIN_PROBE = 0.7f;
inline constexpr float GRAVITY = 9.81f;

// Planner
inline constexpr double REPLAN_INTERVAL = 0.35;     // seconds
inline constexpr int    PLANNER_MAX_EXPANSIONS = 400000;

// Stuck-goal blacklist: unresolvable frontiers are excluded from goal
// selection for a while instead of being burned into the shared map,
// which would risk sealing off whole corridors
inline constexpr double BLACKLIST_TTL = 45.0;   // seconds
inline constexpr float  BLACKLIST_RADIUS = 3.0f;

// Multi-drone fleet
inline constexpr int   DRONE_COUNT = 3;
inline constexpr float CLAIM_RADIUS = 6.0f;      // goal separation between agents, m
inline constexpr float SEPARATION_DIST = 2.2f;   // drone-drone repulsion range, m
inline constexpr float SEPARATION_GAIN = 2.5f;
// Cap on the separation velocity command. Must stay well below cruise
// speed: an uncapped push can shove a drone off its safe corridor into a
// wall, which showed up as rare trajectory-dependent collisions.
inline constexpr float SEPARATION_MAX = 1.0f;

// Rendering
inline constexpr int MESH_REBUILDS_PER_FRAME = 32;
