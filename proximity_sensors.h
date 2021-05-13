#ifndef PROXIMITY_SENSORS_H
#define PROXIMITY_SENSORS_H

#include <stdint.h>

#define MIN_DIST_OBST		1100	// Proportional to returned signal by objects. (Intensité?)

void obstacle_det_start(void);
void obstacle_detection(void);
void statusObstDetection(bool status);

#endif /* PROXIMITY_SENSORS_H */
