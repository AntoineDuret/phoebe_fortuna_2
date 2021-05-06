#ifndef PROXIMITY_SENSORS_H
#define PROXIMITY_SENSORS_H

#include <stdint.h>

#define MIN_DIST_OBST		1100	// Proportional to returned signal by objects. (Intensité?)

#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the wheel
#define WHEEL_DISTANCE      5.1f    // cm 5.35
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define WHEEL_PERIMETER     12.5f // [cm]

void obstacle_det_start(void);

void statusObstDetection(bool status);

void obstacle_detection(void);

#endif /* PROXIMITY_SENSORS_H */
