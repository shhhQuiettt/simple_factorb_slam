#include "airplane.h"
#include "measurement_types.h"
#include <assert.h>
#include <error.h>
#include <raylib.h>
#include <raymath.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

double generate_gaussian(double mean, double std_dev) {
    static int has_spare = 0;
    static double spare;

    if (has_spare) {
        has_spare = 0;
        return mean + std_dev * spare;
    }

    has_spare = 1;
    double u, v, s;

    do {
        u = (rand() / ((double)RAND_MAX)) * 2.0 - 1.0;
        v = (rand() / ((double)RAND_MAX)) * 2.0 - 1.0;
        s = u * u + v * v;
    } while (s >= 1.0 || s == 0.0);

    s = sqrt(-2.0 * log(s) / s);

    spare = v * s;
    return mean + std_dev * (u * s);
}

Vector3 generate_gaussian_v3(double mean, double std_dev) {
    return (Vector3){
        generate_gaussian(mean, std_dev),
        generate_gaussian(mean, std_dev),
        generate_gaussian(mean, std_dev),
    };
}

Quaternion generate_gaussian_q(double mean, double std_dev) {
    Vector3 omega = generate_gaussian_v3(mean, std_dev);

    float theta = Vector3Length(omega);

    if (theta < 1e-6f) {
        return QuaternionIdentity();
    }

    Vector3 axis = Vector3Scale(omega, 1.0f / theta);

    Quaternion q_noise = QuaternionFromAxisAngle(axis, theta);

    return q_noise;
}

SensorMessage measure_odometry(Airplane *plane) {
    static bool first_call = true;
    static Vector3 last_position = {PLANE_INITIAL_X, PLANE_INITIAL_Y,
                                    PLANE_INITIAL_Z};

    static Quaternion last_rotation = {0, 0, 0, 1};

    if (first_call) {
        last_position = plane->position;
        last_rotation = plane->rotation;
        first_call = false;
    }

    Vector3 world_delta_position =
        Vector3Subtract(plane->position, last_position);

    Quaternion inv_last_rotation = QuaternionInvert(last_rotation);
    Vector3 delta_position =
        Vector3RotateByQuaternion(world_delta_position, inv_last_rotation);

    SensorMessage msg;
    msg.type = ODOMETRY_MEASUREMENT;
    msg.data.odom.delta_position = (Vector3){
        delta_position.x + generate_gaussian(0, ODOMETRY_X_NOISE_STD_DEV),
        delta_position.y + generate_gaussian(0, ODOMETRY_Y_NOISE_STD_DEV),
        delta_position.z + generate_gaussian(0, ODOMETRY_Z_NOISE_STD_DEV),
    };

    Quaternion noise_rotation =
        generate_gaussian_q(0, ODOMETRY_ANGULAR_NOISE_STD_DEV);
    Quaternion true_delta_rotation =
        QuaternionMultiply(inv_last_rotation, plane->rotation);

    msg.data.odom.delta_orientation =
        QuaternionMultiply(noise_rotation, true_delta_rotation);

    last_position = plane->position;
    last_rotation = plane->rotation;
    return msg;
}

SensorMessage measure_gnss(Airplane *plane) {
    SensorMessage msg;
    msg.type = GNSS_MEASUREMENT;
    msg.data.gnss.position =
        (Vector3){generate_gaussian(plane->position.x, GNSS_X_NOISE_STD_DEV),
                  generate_gaussian(plane->position.y, GNSS_Y_NOISE_STD_DEV),
                  generate_gaussian(plane->position.z, GNSS_Z_NOISE_STD_DEV)};
    return msg;
}

SensorMessage measure_radar(Airplane *plane, Landmark *landmarks,
                            const int num_landmarks) {
    assert(num_landmarks <= RADAR_MAX_LANDMARKS);
    SensorMessage msg;
    msg.type = RADAR_MEASUREMENT;
    msg.data.radar.num_returns = 0;
    for (int i = 0; i < num_landmarks; i++) {
        Landmark current_landmark = landmarks[i];

        Vector3 to_landmark =
            Vector3Subtract(current_landmark.position, plane->position);

        float distance = Vector3Length(to_landmark);

        if (distance > RADAR_MAX_RANGE) {
            continue;
        }

        Vector3 forward =
            Vector3RotateByQuaternion((Vector3){1, 0, 0}, plane->rotation);

        float angle_to_landmark =
            acosf(Vector3DotProduct(Vector3Normalize(to_landmark), forward));

        if (angle_to_landmark > RADAR_FOV_RAD / 2.0f) {
            continue;
        }

        Vector3 local_to_landmark = Vector3RotateByQuaternion(to_landmark, QuaternionInvert(plane->rotation));
        float azimuth = atan2f(local_to_landmark.y, local_to_landmark.x);
        float elevation = asinf(local_to_landmark.z / distance);

        size_t idx = msg.data.radar.num_returns;
        msg.data.radar.returns[idx].landmark_id = current_landmark.id;
        msg.data.radar.returns[idx].range =
            distance + generate_gaussian(0, RADAR_RANGE_NOISE_STD_DEV);
        msg.data.radar.returns[idx].azimuth =
            azimuth + generate_gaussian(0, RADAR_AZIMUTH_NOISE_STD_DEV);
        msg.data.radar.returns[idx].elevation =
            elevation + generate_gaussian(0, RADAR_ELEVATION_NOISE_STD_DEV);

        msg.data.radar.num_returns++;

        ASSERT_EX((msg.data.radar.returns[idx].landmark_id >= 1) &&
                      (msg.data.radar.returns[idx].landmark_id <= 3),
                  error(1, 0, "Invalid landmark ID: %d\n",
                        msg.data.radar.returns[idx].landmark_id););
        printf("Sending a landmark with id %d\n",
               msg.data.radar.returns[idx].landmark_id);
    }

    return msg;
}
