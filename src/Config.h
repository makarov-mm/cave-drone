#pragma once

// World / voxel grid
constexpr int   WORLD_NX = 192;
constexpr int   WORLD_NY = 56;
constexpr int   WORLD_NZ = 192;
constexpr float VOXEL_SIZE = 0.5f;   // meters per voxel

// Observed map
constexpr int CHUNK_SIZE = 16;

// Lidar
constexpr float LIDAR_RANGE = 10.0f; // meters
constexpr int   LIDAR_RAYS_PER_FRAME = 384;

// Lidar noise model
constexpr float LIDAR_NOISE_SIGMA = 0.05f;  // Gaussian range noise, m
constexpr float LIDAR_DROPOUT_PROB = 0.01f; // ray returns nothing
constexpr float LIDAR_OUTLIER_PROB = 0.001f; // spurious short return (dust)

// Log-odds occupancy fusion (integer log-odds, OctoMap style)
constexpr int LOGODDS_HIT = 2;             // added on a measured return
constexpr int LOGODDS_MISS = -1;           // added when a ray passes through
constexpr int LOGODDS_OCC_THRESHOLD = 3;   // classified Occupied at or above
constexpr int LOGODDS_FREE_THRESHOLD = -2; // classified Free at or below
constexpr int LOGODDS_MIN = -8;            // clamp: bounds re-classification latency
constexpr int LOGODDS_MAX = 16;

// Drone dynamics
constexpr float DRONE_CRUISE_SPEED = 3.0f;   // m/s
constexpr float DRONE_LOOKAHEAD = 1.3f;      // pursuit distance, m
constexpr float DRONE_VEL_GAIN = 3.5f;       // velocity loop P gain
constexpr float DRONE_MAX_HORIZ_ACC = 8.0f;  // m/s^2
constexpr float DRONE_MAX_TOTAL_ACC = 24.0f; // specific force clamp
constexpr float DRONE_ATTITUDE_RATE = 8.0f;  // attitude tracking speed
constexpr float DRONE_DRAG = 0.10f;          // quadratic drag coeff
constexpr float GRAVITY = 9.81f;

// Planner
constexpr double REPLAN_INTERVAL = 0.35;     // seconds
constexpr int    PLANNER_MAX_EXPANSIONS = 400000;

// Stuck-goal blacklist: unresolvable frontiers are excluded from goal
// selection for a while instead of being burned into the shared map,
// which would risk sealing off whole corridors
constexpr double BLACKLIST_TTL = 45.0;   // seconds
constexpr float  BLACKLIST_RADIUS = 3.0f;

// Multi-drone fleet
constexpr int   DRONE_COUNT = 3;
constexpr float CLAIM_RADIUS = 6.0f;      // goal separation between agents, m
constexpr float SEPARATION_DIST = 2.2f;   // drone-drone repulsion range, m
constexpr float SEPARATION_GAIN = 2.5f;

// Rendering
constexpr int MESH_REBUILDS_PER_FRAME = 32;
