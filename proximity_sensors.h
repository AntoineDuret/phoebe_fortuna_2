#ifndef PROXIMITY_SENSORS_H
#define PROXIMITY_SENSORS_H

#include <stdint.h>

#define MIN_DIST_OBST		1000	// Experimental value

void obstacle_det_start(void);
void obstacle_detection(void);
void status_obst_detection(bool status);

#endif /* PROXIMITY_SENSORS_H */
