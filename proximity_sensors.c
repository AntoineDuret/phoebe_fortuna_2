/*
  \file   	proximity_sensors.c
  \author 	Antoine Duret & Carla Schmid (Group G08)
  \date   	13.05.2021
  \version	2.0
  \brief  	Code for obstacle avoidance with proximity sensors
*/

#include <stdbool.h>
#include <arm_math.h>

#include "ch.h"
#include "hal.h"

#include "audio_processing.h"
#include "main.h"
#include "process_image.h"
#include "proximity_sensors.h"

#include "sensors/proximity.h"
#include "leds.h"
#include "motors.h"

static bool obstacleDet = FALSE;

// Thread to detect if an obstacle is close to the IR Sensors in the front of the e-puck.
static THD_WORKING_AREA(prox_sens_thd_wa, 512);
static THD_FUNCTION(prox_sens_thd, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);

    while(1) {
    	if(obstacleDet) {
    		if((get_prox(0) > MIN_DIST_OBST) || (get_prox(1) > MIN_DIST_OBST) ||
    		   (get_prox(6) > MIN_DIST_OBST) || (get_prox(7) > MIN_DIST_OBST)) {
    			obstacle_detection();
    		}
    	}

    	chThdSleepMilliseconds(100);
    }
}


/*
*	Function to start the THREAD controlling the obstacle detection.
*/
void obstacle_det_start(void) {
	chThdCreateStatic(prox_sens_thd_wa, sizeof(prox_sens_thd_wa), NORMALPRIO, prox_sens_thd, NULL);
}


/*
*	Function to control the e-puck when an obstacle was detected.
*/
void obstacle_detection(void) {
	// Stop e-puck and turn off audio command / obstacle detection / goal detection.
	status_audio_command(FALSE);
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	status_goal_detection(FALSE);
	obstacleDet = FALSE;

	// Show 4 red LEDs to indicate that the minimal distance to objects were not kept.
	set_led(LED1,1);
    set_led(LED3,1);
    set_led(LED5,1);
    set_led(LED7,1);

    // Turn 180° in order to have a free path in front of robot.
    turn_left_degrees(180);

    // Wait 2 seconds additionally as penalty
    chThdSleepMilliseconds(2000);

    // Turn off LEDs to indicate that player can continue to play.
    set_led(LED1,0);
    set_led(LED3,0);
    set_led(LED5,0);
    set_led(LED7,0);

    // Turn on audio command
    status_audio_command(TRUE);
    status_goal_detection(TRUE);
   	obstacleDet = TRUE;
}


/*
*	Function to control the obstacle detection command.
*/
void status_obst_detection(bool status) {
	obstacleDet = status;
}
