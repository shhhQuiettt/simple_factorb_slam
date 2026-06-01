#include "measurement_types.h"
#include <assert.h>
#include <raylib.h>
#include <raymath.h>
#include <stdlib.h>
#include "airplane.h"


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


SensorMessage measure_odometry(Airplane *plane) {
    SensorMessage msg;
    msg.type = ODOMETRY_MEASUREMENT;
    msg.data.odom.velocity = (Vector3){0, 0, plane->forward_speed};
    msg.data.odom.velocity.x += generate_gaussian(0, ODOMETRY_X_NOISE_STD_DEV);
    msg.data.odom.velocity.y += generate_gaussian(0, ODOMETRY_Y_NOISE_STD_DEV);
    msg.data.odom.velocity.z += generate_gaussian(0, ODOMETRY_Z_NOISE_STD_DEV);

    msg.data.odom.twist = (Vector3){
        generate_gaussian(plane->pitch_dot, ODOMETRY_PITCH_NOISE_STD_DEV),
        generate_gaussian(plane->yaw_dot, ODOMETRY_YAW_NOISE_STD_DEV),
        generate_gaussian(0.0, ODOMETRY_ROLL_NOISE_STD_DEV),
    };

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

SensorMessage measure_radar(Airplane *plane, Landmark *landmarks, const int num_landmarks) {
    assert(num_landmarks <= RADAR_MAX_LANDMARKS);
    SensorMessage msg;
    msg.type = RADAR_MEASUREMENT;
    msg.data.radar.num_returns = 0;
    for (int i = 0; i < num_landmarks; i++) {
        Vector3 to_landmark = Vector3Subtract(landmarks[i].position, plane->position);
        float distance = Vector3Length(to_landmark);
        if (distance <= RADAR_MAX_RANGE) {
            Vector3 forward = Vector3RotateByQuaternion((Vector3){0, 0, 1}, plane->rotation);
            float angle_to_landmark = acosf(Vector3DotProduct(Vector3Normalize(to_landmark), forward));
            if (angle_to_landmark > RADAR_FOV_RAD / 2.0f) {
                continue;
            }

            msg.data.radar.returns[msg.data.radar.num_returns].landmark_id = i;
            msg.data.radar.returns[msg.data.radar.num_returns].range = distance + generate_gaussian(0, RADAR_RANGE_NOISE_STD_DEV);
            msg.data.radar.returns[msg.data.radar.num_returns].rot_x = angle_to_landmark + generate_gaussian(0, RADAR_ROT_X_NOISE_STD_DEV);

        }

    }

    return msg;
}
