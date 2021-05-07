#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#include <stdbool.h>

#define IMAGE_BUFFER_SIZE		640
#define GOAL_DISTANCE			10.0f //cm
#define PXTOCM					1486.0f //pixel * cm --> Distance = h * D_lense / width  (D_lense est une constante) donc PXTOCM = h (2cm) * D_lense (772.55 pxl)
#define WIDTH_SLOPE				10 //pixel (before 5)
#define MIN_LINE_WIDTH			150
#define GOAL_DIST_MIN			120
#define MAX_DISTANCE			13.0f //cm


float get_distance_cm(void);
uint16_t get_line_position(void);
void extract_line_width(uint8_t *buffer); //uint16_t
void process_image_start(void);
bool verify_finish_line(void);

#endif /* PROCESS_IMAGE_H */
