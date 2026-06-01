#ifndef MAKE_MEASUREMENTS_H
#define MAKE_MEASUREMENTS_H

#include "measurement_types.h"
#include <raylib.h>
#include <raymath.h>
#include "airplane.h"

double generate_gaussian(double mean, double std_dev);

SensorMessage measure_odometry(Airplane *plane);

SensorMessage measure_gnss(Airplane *plane);

SensorMessage measure_radar(Airplane *plane, Landmark *landmarks, const int n_landmarks);

#endif // MAKE_MEASUREMENTS_H
