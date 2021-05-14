#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#include <stdbool.h>

// Parameters for line detection with embedded camera
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			20
#define GOAL_DIST_MIN			30
#define GOAL_DIST_MAX			120
#define MIN_GOAL_LINES			3

// Geometrical parameters of the e-puck2
#define WHEEL_PERIMETER     12.5f 					// in [cm]
#define WHEEL_DISTANCE      5.1f    				// e-puck2 diameter in [cm]
#define PERIMETER_EPUCK     (PI*WHEEL_DISTANCE)		// e-puck2 perimeter in [cm]
#define NSTEP_ONE_TURN      1000 					// number of steps for 1 turn of the wheel

// Setting used for the automatic return
#define RETURN_LINE_DETECTION_DISTANCE		160		// in [mm]


void detect_line(uint8_t *buffer);
void process_image_start(void);


/*======================================================================================*/
/* 								NEW FUNCTIONS DEFINED									*/
/*======================================================================================*/
bool verify_finish_line(void);
void status_goal_detection(bool status);
void return_to_start_line(void);
void turn_right_degrees(uint8_t degrees);
void turn_left_degrees(uint8_t degrees);
void go_forward_cm(uint8_t cm);

#endif /* PROCESS_IMAGE_H */
