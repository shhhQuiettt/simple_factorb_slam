#ifndef MEASUREMENT_TYPES_H
#define MEASUREMENT_TYPES_H

#include "raylib.h"
#include <inttypes.h>
#include <stdint.h>

#define RADAR_MEASUREMENT 0
#define ODOMETRY_MEASUREMENT 1
#define GNSS_MEASUREMENT 2

#define ODOMETRY_X_NOISE_STD_DEV 0.1f
#define ODOMETRY_Y_NOISE_STD_DEV 0.1f
#define ODOMETRY_Z_NOISE_STD_DEV 0.1f
#define ODOMETRY_PITCH_NOISE_STD_DEV 0.1f
#define ODOMETRY_YAW_NOISE_STD_DEV 0.1f
#define ODOMETRY_ROLL_NOISE_STD_DEV 0.1f

#define GNSS_X_NOISE_STD_DEV 0.5f
#define GNSS_Y_NOISE_STD_DEV 0.5f
#define GNSS_Z_NOISE_STD_DEV 0.5f
enum MeasurementType {
    RADAR = RADAR_MEASUREMENT,
    ODOMETRY = ODOMETRY_MEASUREMENT,
    GNSS = GNSS_MEASUREMENT
};

typedef struct Landmark {
    uint8_t id;
    Vector3 position;
} Landmark;

typedef struct RadarReturnData {
    short landmark_id;
    float range;
    float azimuth;
    float elevation;
} RadarReturnData;

typedef struct OdometryData {
    Vector3 velocity;
    Vector3 twist; // angular velocity
} OdometryData;

typedef struct GNSSData {
    Vector3 position;
} GNSSData;

typedef struct SensorMessage {
    int32_t type;
    union {
        RadarReturnData radar;
        OdometryData odom;
        GNSSData gnss;
    } data;
} SensorMessage;

#endif // MEASUREMENT_TYPES_H
