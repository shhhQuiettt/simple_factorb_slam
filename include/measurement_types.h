#ifndef MEASUREMENT_TYPES_H
#define MEASUREMENT_TYPES_H

#include "raylib.h"
#include <inttypes.h>
#include <stdint.h>

#define RADAR_MEASUREMENT 0
#define ODOMETRY_MEASUREMENT 1
#define GNSS_MEASUREMENT 2

#define ODOM_HZ 60.0f
#define RADAR_HZ 10.0f
#define GNSS_HZ 1.0f

#define _ODOMETRY_NOISE_STD_DEV 1.0f
#define ODOMETRY_X_NOISE_STD_DEV _ODOMETRY_NOISE_STD_DEV
#define ODOMETRY_Y_NOISE_STD_DEV _ODOMETRY_NOISE_STD_DEV
#define ODOMETRY_Z_NOISE_STD_DEV _ODOMETRY_NOISE_STD_DEV

#define _ODOMETRY_ANGULAR_NOISE_STD_DEV 0.001f * PI
#define ODOMETRY_PITCH_NOISE_STD_DEV _ODOMETRY_ANGULAR_NOISE_STD_DEV
#define ODOMETRY_YAW_NOISE_STD_DEV _ODOMETRY_ANGULAR_NOISE_STD_DEV
#define ODOMETRY_ROLL_NOISE_STD_DEV _ODOMETRY_ANGULAR_NOISE_STD_DEV

#define _GNSS_NOISE_STD_DEV 0.01f
#define GNSS_X_NOISE_STD_DEV _GNSS_NOISE_STD_DEV
#define GNSS_Y_NOISE_STD_DEV _GNSS_NOISE_STD_DEV
#define GNSS_Z_NOISE_STD_DEV _GNSS_NOISE_STD_DEV

#define _RADAR_ANGLE_STD_DEV PI / 90.0f
#define RADAR_RANGE_NOISE_STD_DEV 1.0f
#define RADAR_ROT_X_NOISE_STD_DEV _RADAR_ANGLE_STD_DEV
#define RADAR_ROT_Y_NOISE_STD_DEV _RADAR_ANGLE_STD_DEV

#define RADAR_MAX_RANGE 200.0f
#define RADAR_FOV_RAD PI / 2.0f
#define RADAR_MAX_LANDMARKS 10

typedef struct RadarReturnData {
    short landmark_id;
    float range;
    float rot_x;
    float rot_z;
} RadarReturnData;

typedef struct RadarData {
    int num_returns;
    RadarReturnData returns[RADAR_MAX_LANDMARKS];
} RadarData;

typedef struct OdometryData {
    Vector3 velocity;
    Vector3 twist;
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

#endif // MEASUREMENT_TYPES_H
