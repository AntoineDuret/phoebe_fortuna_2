/*
  \file   	process_image.c
  \author 	Antoine Duret & Carla Schmid (Group G08)
  \date   	13.05.2021
  \version	2.0
  \brief  	Code for image processing related tasks
*/

#include "ch.h"
#include "hal.h"

#include <chprintf.h>
#include <usbcfg.h>
#include <stdbool.h>
#include <math.h>

#include "audio_processing.h"
#include "main.h"
#include "process_image.h"
#include "proximity_sensors.h"

#include "camera/po8030.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "motors.h"

static bool linesFound = FALSE;
static bool goalDetection = FALSE;

// Semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE); // @suppress("Field cannot be resolved")

/*======================================================================================*/
/* 						 	     REUSED CODE FROM THE TP4 				    			*/
/* 						  (with small additions and corrections)						*/
/*======================================================================================*/

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	// Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2,
															SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	while(1) {
        // Starts a capture
		dcmi_capture_start();

		// Waits for the capture to be done
		wait_image_ready();

		// Signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 2048);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

    while(1) {
    	// Waits until an image has been captured
        chBSemWait(&image_ready_sem);

        // Gets the pointer to the array filled with the last image in RGB565
        img_buff_ptr = dcmi_get_last_image_ptr();

  		for(uint16_t i = 0; i < 2 * IMAGE_BUFFER_SIZE; i+=2) {
   			image[i/2] = ((uint8_t) img_buff_ptr[i] & 0xF8); // red
   		}

  		// Search for line in the image and gets its width in pixels
  		detect_line(image);
    }
}


void process_image_start(void) {
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}


/*
* 	Counts the amount of lines found in picture and changes bool if enough lines are in detected.
*/
void detect_line(uint8_t *buffer) {
	volatile uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0, wrongLine = 0, lineNotFound = 0;
	uint32_t mean = 0;
	uint8_t counterLines = 0;

	// Performs an average
	for(uint32_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++) {
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	for(i = 0; i < IMAGE_BUFFER_SIZE ; i++) {
		do {
			wrongLine = 0;

			// Search for a begin
			while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)) {
				if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean) {
					begin = i;
					stop = 1;
				}

				i++;//+= WIDTH_SLOPE;
			}

			// If begin was found, search for an end.
			if((i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)) && begin) {
				stop = 0;

				while (stop == 0 && (i < IMAGE_BUFFER_SIZE)) {
					if((buffer[i] > mean) && (buffer[i-WIDTH_SLOPE] < mean)) {
						end = i;
						stop = 1;
					}

					i++;
				}

				if((i >= IMAGE_BUFFER_SIZE) || !end) { 	// If no end was found
					lineNotFound = 1;
				}
			} else {									// If no begin was found
				lineNotFound = 1;
			}

			// If a too small line has been detected, continue the search.
			if(!lineNotFound && ((end-begin) < MIN_LINE_WIDTH)) {
				i = end;
				begin = 0;
				end = 0;
				stop = 0;
				wrongLine = 1;
			}
		} while(wrongLine);

		if(!lineNotFound) {
			counterLines++;
		}
	}

	if(counterLines >= MIN_GOAL_LINES) {
		linesFound = TRUE;
	} else {
		linesFound = FALSE;
	}
}

/*======================================================================================*/
/* 									 NEW FUNCTIONS										*/
/*======================================================================================*/

/*
*	Function used to check if the finish line is reached.
*/
bool verify_finish_line(void) {
	bool goalDetected = FALSE;
	if(goalDetection) {
		if((VL53L0X_get_dist_mm() <= GOAL_DIST_MAX) && (VL53L0X_get_dist_mm() >= GOAL_DIST_MIN)) {
			if(linesFound) {
				goalDetected = TRUE;

				status_audio_command(FALSE);
				status_obst_detection(FALSE);
				status_goal_detection(FALSE);

				left_motor_set_speed(0);
				right_motor_set_speed(0);

			}
		}
	}

	return goalDetected;
}


/*
*	Function to control the goal detection.
*
*	params :
*	bool status		status value TRUE or FALSE
*/
void status_goal_detection(bool status) {
	goalDetection = status;
}


/*
*	Function used to return to the start line when a player is done with the game
*	(when the finish line is reached).
*/
void return_to_start_line(void) {
	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t proxValues;
	int16_t leftSpeed = 0, rightSpeed = 0;
	systime_t time;

	left_motor_set_speed(0);
	right_motor_set_speed(0);

	// Go towards hallway
	go_forward_cm(2);
	turn_left_degrees(50);

	// Goes through hallway till detects lines.
	time = chVTGetSystemTime();
	while ((linesFound == FALSE) || (VL53L0X_get_dist_mm() > RETURN_LINE_DETECTION_DISTANCE) ||
		   ((chVTGetSystemTime() - time) < MINIMAL_TIME_RETURN)) {
		messagebus_topic_wait(prox_topic, &proxValues, sizeof(proxValues));
		leftSpeed = MOTOR_SPEED_LIMIT - proxValues.delta[0]*3 - 2*proxValues.delta[1];
		rightSpeed = MOTOR_SPEED_LIMIT - proxValues.delta[7]*3 - 2*proxValues.delta[6];
		right_motor_set_speed(rightSpeed);
		left_motor_set_speed(leftSpeed);
		chThdSleepUntilWindowed(time, time + MS2ST(15)); // Refresh @ 100 Hz.
	}

	left_motor_set_speed(0);
	right_motor_set_speed(0);

	// Goes to starting position.
	go_forward_cm(7);
	turn_right_degrees(77);
	go_forward_cm(28);
	turn_right_degrees(80);

	status_audio_command(FALSE);
	status_voice_calibration(FALSE);
}


/*
*	Function to turn right of a given angle in degrees.
*
*	params :
*	uint8_t degrees		value of the desired rotation angle in degrees
*/
void turn_right_degrees(uint8_t degrees) {
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	left_motor_set_speed(  GAME_SPEED/2);
	right_motor_set_speed(-GAME_SPEED/2);

	while(abs(right_motor_get_pos()) <=
			(PERIMETER_EPUCK * NSTEP_ONE_TURN/360 / WHEEL_PERIMETER) * degrees) {
		chThdSleepMilliseconds(100);
	}

	left_motor_set_speed(0);
	right_motor_set_speed(0);
}


/*
*	Function to turn left of a given angle in degrees.
*
*	params :
*	uint8_t degrees		value of the desired rotation angle in degrees
*/
void turn_left_degrees(uint8_t degrees) {
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	left_motor_set_speed(-GAME_SPEED/2);
	right_motor_set_speed(GAME_SPEED/2);

	while(abs(right_motor_get_pos()) <=
			(PERIMETER_EPUCK * NSTEP_ONE_TURN/360 / WHEEL_PERIMETER) * degrees) {
		chThdSleepMilliseconds(100);
	}

	left_motor_set_speed(0);
	right_motor_set_speed(0);
}


/*
*	Function to move forward a given distance in cm.
*
*	params :
*	uint8_t cm		value of the desired distance in cm
*/
void go_forward_cm(uint8_t cm) {
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	left_motor_set_speed(GAME_SPEED);
	right_motor_set_speed(GAME_SPEED);

	while(abs(right_motor_get_pos()) <= (cm * NSTEP_ONE_TURN / WHEEL_PERIMETER)) {
		chThdSleepMilliseconds(100);
	}

	left_motor_set_speed(0);
	right_motor_set_speed(0);
}
