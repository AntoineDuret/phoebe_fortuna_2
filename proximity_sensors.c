#include <main.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <arm_math.h>

#include <proximity_sensors.h>
#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "sensors/proximity.h"
#include "leds.h"
#include <motors.h>
#include <audio_processing.h>
#include <process_image.h>
//#include <process_image.h>

static bool obstacle_det = FALSE;
//static proximity_msg_t prox_values;

static THD_WORKING_AREA(prox_sens_thd_wa, 512);
static THD_FUNCTION(prox_sens_thd, arg) {
	(void) arg;
    chRegSetThreadName(__FUNCTION__);

    while(1) {
    	if(obstacle_det) {
    		if((get_prox(0) > MIN_DIST_OBST) || (get_prox(1) > MIN_DIST_OBST) ||
    		   (get_prox(6) > MIN_DIST_OBST) || (get_prox(7) > MIN_DIST_OBST)) {
    			obstacle_detection();
    		}
    	}
    	chThdSleepMilliseconds(100); // capteurs 100 Hz, PID 10Hz,
    }
}

void obstacle_det_start(void) {
	chThdCreateStatic(prox_sens_thd_wa, sizeof(prox_sens_thd_wa), NORMALPRIO, prox_sens_thd, NULL);
}

void statusObstDetection(bool status) {
	obstacle_det = status;
}

void obstacle_detection(void) {
	// Stop e-puck
	statusAudioCommand(FALSE);
	statusGoalDetection(FALSE);
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	// Stop obstacle detection
	obstacle_det = FALSE;

	// Show LEDs to indicate that minimal distance to objects were not kept
	set_led(LED1,1);
    set_led(LED3,1);
    set_led(LED5,1);
    set_led(LED7,1);

    // Turn 180° in order to have a free path in front of robot
    left_motor_set_pos(0);
    right_motor_set_pos(0);

    left_motor_set_speed(600);
	right_motor_set_speed(-600);

	while(abs(right_motor_get_pos()) <= (PERIMETER_EPUCK * NSTEP_ONE_TURN/2 / WHEEL_PERIMETER)) {
		chThdSleepMilliseconds(100);
	}

	left_motor_set_speed(0);
	right_motor_set_speed(0);

    // Wait 2 seconds additionally as penalty
    chThdSleepMilliseconds(2000);

    // Turn off LEDs to indicate that player can continue to play
    set_led(LED1,0);
    set_led(LED3,0);
    set_led(LED5,0);
    set_led(LED7,0);

    // Turn on audio command and & obstacle detection
    statusAudioCommand(TRUE);
    statusGoalDetection(TRUE);
    chThdSleepMilliseconds(2000); // Uncomment for 2s  till obstacle detection is turned on again
	obstacle_det = TRUE;
}
