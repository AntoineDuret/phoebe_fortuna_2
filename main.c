#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <arm_math.h>

#include "ch.h"
#include "hal.h"

#include "audio_processing.h"
#include "communications.h"
#include "fft.h"
#include "main.h"
#include "process_image.h"
#include "proximity_sensors.h"

#include "audio/microphone.h"
#include "camera/po8030.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "ir_remote.h"
#include "leds.h"
#include "memory_protection.h"
#include "motors.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include "selector.h"
#include "usbcfg.h"
#include "uc_usage.h"
#include "cmd.h"

#include <chmtx.h>


messagebus_t bus;
MUTEX_DECL(bus_lock); // @suppress("Field cannot be resolved")
CONDVAR_DECL(bus_condvar);
parameter_namespace_t parameter_root, aseba_ns;


int main(void) {
	// System initialization
    halInit();
    chSysInit();
    mpu_init();

    // Inter Process Communication bus initialization
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Peripherals initialization
    clear_leds();
	dcmi_start();					// camera
	po8030_start();					// clock generation for the po8030 (camera)
	motors_init();
	proximity_start();
	obstacle_det_start();
	VL53L0X_start();				// ToF init
    mic_start(&processAudioData);
    mic_input_start();
    process_image_start();

    uint8_t nbPlayers = 0;
    uint8_t currentPlayer = 0;
    uint tabPlayers[NB_PLAYERS_MAX];

    /* Infinite loop. */
    while(1) {
    	if(nbPlayers == 0) {
        	nbPlayers = game_setting();
        	currentPlayer = nbPlayers;

        	// Main update loop
        	while(currentPlayer != 0) {
        		currentPlayer--;
        		player_voice_config();
        		tabPlayers[currentPlayer] = game_running();
        		set_body_led(1);

        		if(currentPlayer > 0) {
        			positionningGoal();

        			// Wait till next player is ready  						// add wait for x seconds till confirmation set
        			while(get_selector() != currentPlayer) {
        				chThdSleepMilliseconds(100);
        				led_selector_management(get_selector());
        			}

        			// Confirm selector state
        			body_led_confirm();

        		} else { // Display the winner!
        			for(uint8_t i = 1; i < nbPlayers; i++) {
        				if(tabPlayers[i] < tabPlayers[currentPlayer]) {
        					currentPlayer = i;
        				}
        			}

        			led_selector_management(currentPlayer + 1);
        			set_body_led(1);
        			currentPlayer = 0;
        		}
        	}
        }

        chThdSleepMilliseconds(5000);

        while(get_selector() != 0) {
        	chThdSleepMilliseconds(100);
        }

        nbPlayers = 0;
    }
}



/*======================================================================================*/
/* 									 NEW FUNCTIONS										*/
/*======================================================================================*/

/*
*	Function used to start a game configuration and manage the early LED
*	communication with the player (see the report, appendix A2).
*/
uint8_t game_setting(void) {
	uint8_t selector_state = 0;
	uint8_t i = 0;

	// Waiting for the user to turn the game setting ON
	while(get_selector() != 0) {
		set_led(LED1, 2);
		set_led(LED3, 2);
		set_led(LED5, 2);
		set_led(LED7, 2);
		chThdSleepMilliseconds(1000);
	}

	if(get_selector() == 0) {
		set_led(LED1, 0);
		set_led(LED3, 0);
		set_led(LED5, 0);
		set_led(LED7, 0);
	}

	// Ready to start the player configuration and selector management
	set_body_led(1);

	// Wait for the user to select the number of players
	do {
		selector_state = get_selector();

		do {
			led_selector_management(get_selector());
			chThdSleepMilliseconds(100);
			i++;
		} while(i < 25); 									// /!\ MAGIC NUMBER

		i = 0;
	} while((selector_state != get_selector()) || (get_selector() == 0));

	body_led_confirm();

	return selector_state;
}


/*
*	Simple function used to manage the desired LED display corresponding to a
*	given selector position. Please see the report, appendix A4.
*/
void led_selector_management(int selector_pos) {
	switch(selector_pos) {
		case 0: // waiting state
			set_player_led_configuration(FULL, 0, 0, 0);
		break;

		case 1: // player 1
			set_player_led_configuration(FULL, BLUE);
		break;

		case 2: // player 2
			set_player_led_configuration(FULL, PINK);
		break;

		case 3: // ...
			set_player_led_configuration(FULL, LIGHT_BLUE);
		break;

		case 4:
			set_player_led_configuration(FULL, YELLOW);
		break;

		case 5:
			set_player_led_configuration(FULL, ORANGE);
		break;

		case 6:
			set_player_led_configuration(HALF, BLUE);
		break;

		case 7:
			set_player_led_configuration(HALF, PINK);
		break;

		case 8:
			set_player_led_configuration(HALF, LIGHT_BLUE);
		break;

		case 9:
			set_player_led_configuration(HALF, YELLOW);
		break;

		case 10:
			set_player_led_configuration(HALF, ORANGE);
		break;

		case 11:
			set_player_led_configuration(DIAG, BLUE);
		break;

		case 12:
			set_player_led_configuration(DIAG, PINK);
		break;

		case 13:
			set_player_led_configuration(DIAG, LIGHT_BLUE);
		break;

		case 14:
			set_player_led_configuration(DIAG, YELLOW);
		break;

		case 15:
			set_player_led_configuration(DIAG, ORANGE);
		break;
	}
}


/*
*	Function to configure the mid frequency of a player before a game
*	(voice calibration for player).
*/
void player_voice_config(void) {
	chThdSleepMilliseconds(500);
    set_front_led(1);
    statusVoiceCalibration(TRUE);

    while(getStatusVoiceCalibration()) {
    	chThdSleepMilliseconds(300);
    }

    set_front_led(0);
}


/*
*	This function is active during the game. It comes to an end when the finish line is detected.
*	It returns the time taken by each player to complete the game.
*/
uint game_running(void) {
	systime_t time;

	set_led(LED1, 1);
	set_led(LED3, 1);
	set_led(LED5, 1);
	set_led(LED7, 1);

	chThdSleepMilliseconds(1000);
	set_led(LED3, 0);
	chThdSleepMilliseconds(1000);
	set_led(LED5, 0);
	chThdSleepMilliseconds(1000);
	set_led(LED7, 0);
	chThdSleepMilliseconds(1000);
	set_led(LED1, 0);

	statusAudioCommand(TRUE);
    statusObstDetection(TRUE);
    statusGoalDetection(TRUE);

    time = chVTGetSystemTime();

    while (!verify_finish_line()) {
    	chThdSleepMilliseconds(200);
    }

    time = chVTGetSystemTime() - time;

	return time;
}


/*
*	Simple function used to show the LED ID of a player associated with a selector position
*
*	params :
*	led_conf_name_t led_conf	one of the three RGB LED configurations defined
*	uint8_t red_i				*
*	uint8_t green_i				* values corresponding with one of the five RGB colors defined
*	uint8_t blue_i				*
*/
void set_player_led_configuration(led_conf_name_t led_conf,
										uint8_t red_i, uint8_t green_i, uint8_t blue_i) {
	if(led_conf == 0) {
		set_rgb_led(0, red_i, green_i, blue_i);
    	set_rgb_led(1, red_i, green_i, blue_i);
    	set_rgb_led(2, red_i, green_i, blue_i);
    	set_rgb_led(3, red_i, green_i, blue_i);
	} else if(led_conf == 1) {
		set_rgb_led(0, red_i, green_i, blue_i);
		set_rgb_led(1, 0, 0, 0);
		set_rgb_led(2, 0, 0, 0);
    	set_rgb_led(3, red_i, green_i, blue_i);
	} else if(led_conf == 2) {
		set_rgb_led(0, 0, 0, 0);
		set_rgb_led(1, red_i, green_i, blue_i);
		set_rgb_led(2, 0, 0, 0);
    	set_rgb_led(3, red_i, green_i, blue_i);
	}
}


/*
*	Simple function defined to apply the abstraction and reuse principle.
*	Desired display to confirm something to the player with the body led.
*/
void body_led_confirm(void) {
	set_body_led(0);
	chThdSleepMilliseconds(1000);
	set_body_led(1);
	chThdSleepMilliseconds(1000);
	set_body_led(0);
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void) {
    chSysHalt("Stack smashing detected");
}
