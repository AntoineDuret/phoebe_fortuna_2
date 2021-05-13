#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#include <stdbool.h>

#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				10 		//pixel (before 5)
#define MIN_LINE_WIDTH			155
#define GOAL_DIST_MIN			140


void detectLine(uint8_t *buffer);
void process_image_start(void);


/*======================================================================================*/
/* 								NEW FUNCTIONS DEFINED									*/
/*======================================================================================*/
bool verify_finish_line(void);
void statusGoalDetection(bool status);
void return_to_start_line(void);
void turnRightDegrees(uint8_t degrees);
void turnLeftDegrees(uint8_t degrees);
void goForwardCM(uint8_t cm);

#endif /* PROCESS_IMAGE_H */
