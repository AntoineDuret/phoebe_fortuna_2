#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#include <stdbool.h>

#define IMAGE_BUFFER_SIZE		640
#define GOAL_DISTANCE			23.0f //cm
#define PXTOCM					1486.0f //pixel * cm --> Distance = h * D_lense / width  (D_lense est une constante) donc PXTOCM = h (2cm) * D_lense (772.55 pxl)
#define WIDTH_SLOPE				10 //pixel (before 5)
#define MIN_LINE_WIDTH			165
#define GOAL_DIST_MIN			100
#define MAX_DISTANCE			13.0f //cm
#define MOTOR_SPEED_LIMIT		1100
#define KP_LINE					800.0f
#define KI_LINE					3.5f	//must not be zero
#define MAX_SUM_ERROR_LINE		(MOTOR_SPEED_LIMIT/KI_LINE)
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2





float get_distance_cm(void);
uint16_t get_line_position(void);
uint16_t extract_line_width(uint8_t *buffer);
void process_image_start(void);
bool verify_finish_line(void);
void statusGoalDetection(bool status);
void positionningGoal(void);

#endif /* PROCESS_IMAGE_H */
