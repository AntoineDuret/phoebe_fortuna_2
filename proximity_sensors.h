#ifndef PROXIMITY_SENSORS_H
#define PROXIMITY_SENSORS_H

#include <stdint.h>

#define MIN_DIST_OBST		1100	// Proportional to returned signal by objects. (Intensit�?)


void obstacle_det_start(void);

void statusObstDetection(bool status);

void obstacle_detection(void);

#endif /* PROXIMITY_SENSORS_H */
