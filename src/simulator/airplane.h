#ifndef AIRPLANE_H
#define AIRPLANE_H

#include <raylib.h>
#include <inttypes.h>

typedef struct Airplane {
    Vector3 position;
    float forward_speed;

    float pitch_dot;
    float yaw_dot;
    float pitch;
    float yaw;

    Quaternion rotation;
} Airplane;


typedef struct Landmark {
    uint8_t id;
    Vector3 position;
} Landmark;

#endif // AIRPLANE_H
