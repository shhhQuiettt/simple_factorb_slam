#ifndef AIRPLANE_H
#define AIRPLANE_H

#include <raylib.h>
#include <inttypes.h>

typedef struct Airplane {
    Vector3 position;
    Vector3 velocity;
    Vector3 twist;

    Quaternion rotation;


} Airplane;



#endif // AIRPLANE_H
