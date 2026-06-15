#ifndef MEASUREMENT_TYPES_H
#define MEASUREMENT_TYPES_H

#include "raylib.h"
#include <inttypes.h>
#include <math.h>
#include <stdint.h>

#define PLANE_INITIAL_X 0.0f
#define PLANE_INITIAL_Y 0.0f
#define PLANE_INITIAL_Z 50.0f

#define RADAR_MEASUREMENT 0
#define ODOMETRY_MEASUREMENT 1
#define GNSS_MEASUREMENT 2
#define MAP_UPDATE 3

#define ODOM_HZ 30.0f
#define RADAR_HZ 10.0f
#define GNSS_HZ 1.0f

#define _ODOMETRY_NOISE_STD_DEV (2.0f / sqrt(ODOM_HZ))
#define ODOMETRY_X_NOISE_STD_DEV _ODOMETRY_NOISE_STD_DEV
#define ODOMETRY_Y_NOISE_STD_DEV _ODOMETRY_NOISE_STD_DEV
#define ODOMETRY_Z_NOISE_STD_DEV _ODOMETRY_NOISE_STD_DEV

#define ODOMETRY_ANGULAR_NOISE_STD_DEV (0.01f * DEG2RAD / sqrt(ODOM_HZ))

#define _GNSS_NOISE_STD_DEV 5.0f
#define GNSS_X_NOISE_STD_DEV _GNSS_NOISE_STD_DEV
#define GNSS_Y_NOISE_STD_DEV _GNSS_NOISE_STD_DEV
#define GNSS_Z_NOISE_STD_DEV _GNSS_NOISE_STD_DEV

#define _RADAR_ANGLE_STD_DEV (0.5f * DEG2RAD)
#define RADAR_RANGE_NOISE_STD_DEV 0.5f
#define RADAR_AZIMUTH_NOISE_STD_DEV _RADAR_ANGLE_STD_DEV
#define RADAR_ELEVATION_NOISE_STD_DEV _RADAR_ANGLE_STD_DEV

#define RADAR_MAX_RANGE 300.0f
#define RADAR_FOV_RAD PI / 3.0f
#define RADAR_MAX_LANDMARKS 10

#define N_LANDMARKS 3
#define LANDMARK_1_ID 1
#define LANDMARK_2_ID 2
#define LANDMARK_3_ID 3
#define LANDMARK_1_POSITION {160.0f, 30.0f, 55.0f}
#define LANDMARK_2_POSITION {100.0f, 0.0f, 10.0f}
#define LANDMARK_3_POSITION {80.0f, 100.0f, 30.0f}

#define MAX_PATH_LENGTH 2048

#ifndef NDEBUG
#define ASSERT_EX(condition, statement)                                        \
    do {                                                                       \
        if (!(condition)) {                                                    \
            statement;                                                         \
            assert(condition);                                                 \
        }                                                                      \
    } while (false)
#else
#define ASSERT_EX(condition, statement) ((void)0)
#endif

typedef struct Landmark {
    uint8_t id;
    Vector3 position;
} Landmark;

typedef struct RadarReturnData {
    uint8_t landmark_id;
    float range;
    float azimuth;
    float elevation;
} RadarReturnData;

typedef struct RadarData {
    int num_returns;
    RadarReturnData returns[RADAR_MAX_LANDMARKS];
} RadarData;

typedef struct OdometryData {
    Vector3 delta_position;
    Quaternion delta_orientation;
} OdometryData;

typedef struct GNSSData {
    Vector3 position;
} GNSSData;

typedef struct SensorMessage {
    int32_t type;
    union {
        RadarData radar;
        OdometryData odom;
        GNSSData gnss;
    } data;
} SensorMessage;

typedef struct PoseWithCovariance {
    Vector3 position;
    Quaternion orientation;
    float covariance[6][6]; // 6x6 covariance matrix
} PoseWithCovariance;

typedef struct Map {
    Landmark landmarks[N_LANDMARKS];
    float covariance[N_LANDMARKS][3][3]; 
} Map;

typedef struct SlamMessage {
    PoseWithCovariance pose_with_covariance;
    Map map;
    Vector3 predicted_path[MAX_PATH_LENGTH];
    int path_length;
} SlamMessage;

#endif // MEASUREMENT_TYPES_H
