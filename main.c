#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"
#include "communications.h"
#include <arm_math.h>

#include "aseba_vm/aseba_node.h"
#include "aseba_vm/skel_user.h"
#include "aseba_vm/aseba_can_interface.h"
#include "aseba_vm/aseba_bridge.h"
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "audio/microphone.h"
#include "camera/po8030.h"
#include "epuck1x/Asercom.h"
#include "epuck1x/Asercom2.h"
#include "epuck1x/a_d/advance_ad_scan/e_acc.h"
#include "sensors/battery_level.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "cmd.h"
#include "config_flash_storage.h"
#include "exti.h"
#include "i2c_bus.h"
#include "ir_remote.h"
#include "leds.h"
#include <main.h>
#include "memory_protection.h"
#include "motors.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include "sdio.h"
#include "selector.h"
#include "spi_comm.h"
#include "usbcfg.h"

#include "uc_usage.h"
#include <proximity_sensors.h>
#include <audio_processing.h>
#include <process_image.h>
//#include "../ChibiOS/os/rt/include/chmtx.h"


#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

messagebus_t bus; // verifier si besoin les 4 lignes pour IR sensor
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);
parameter_namespace_t parameter_root, aseba_ns;

static void serial_start(void) {
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

//static void timer12_start(void) {
//    // General Purpose Timer configuration
//    // timer 12 is a 16 bit timer so we can measure time to about 65ms with a 1Mhz counter
//    static const GPTConfig gpt12cfg = {
//        1000000,        /* 1MHz timer clock in order to measure uS.*/
//        NULL,           /* Timer callback.*/
//        0,
//        0
//    };
//
//    gptStart(&GPTD12, &gpt12cfg);
//    // Let the timer count to max value
//    gptStartContinuous(&GPTD12, 0xFFFF);
//}


int main(void) {
	// System initialization
    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */ // ??
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    // Peripherals initialization
    clear_leds();
	usb_start();			// starts the USB communication
	dcmi_start();				// camera
	po8030_start();			// camera
	motors_init();			// motors
	proximity_start();			// IR proximity detection
	obstacle_det_start();
	exti_start();
	spi_comm_start();
	VL53L0X_start();			// ToF init
	serial_start();			// starts the serial communication
	sdio_start();
//    timer12_start();		// starts timer 12
    mic_start(&process_audio_data);
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
        	while(currentPlayer != 0) {
        		currentPlayer--;
        		player_voice_config(); //
        		tabPlayers[currentPlayer] = game_running();

        		set_body_led(1);
        		if(currentPlayer > 0) {
        			return_to_start_line();

        			// Wait till next player is ready  // add wait for x seconds till confirmation set
        			while(get_selector() != currentPlayer) {
        				chThdSleepMilliseconds(300);
        				led_selector_management(get_selector());
        			}

        			// Confirm selector state
        			body_led_confirm();

        		} else {
        			for(uint8_t i = 1; i < nbPlayers; i++) {
        				if(tabPlayers[i] < tabPlayers[currentPlayer])
        					currentPlayer = i;
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


/*
*	Function used to start a game configuration and manage the early LED
*	communication with the user
*/
uint8_t game_setting(void) {
	uint8_t selector_state = 0; 												// vérifier cast ??

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

	uint8_t i = 0;
	// Wait for the user to select the number of players
	do {
		selector_state = get_selector();
		do {
			led_selector_management(get_selector());
			chThdSleepMilliseconds(100);
			i++;
		} while (i < 25);
		i = 0;
	} while((selector_state != get_selector()) || (get_selector() == 0));
	body_led_confirm();
	return selector_state;
}


/*
*	Simple function used to manage the desired LED display corresponding to a
*	given selector position
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
* Simple function defined to apply the abstraction and reuse principle.
* Desired display to confirm something to the player with the body led.
*/
void body_led_confirm(void) {
 set_body_led(0);
 chThdSleepMilliseconds(1000);
 set_body_led(1);
 chThdSleepMilliseconds(1000);
 set_body_led(0);
}


/*
*	Function to initialize all the settings before the game starts which are relevant for the player
*	(voice calibration for player)
*/
void player_voice_config(void) {
	chThdSleepMilliseconds(500);
    set_front_led(1);
    status_voice_calibration(TRUE);
    while(get_status_voice_calibration()) {
    	chThdSleepMilliseconds(300);
    }
    set_front_led(0);
}

/*
*	This function is active during the game. It comes to an end when the finish line is detected.
*
*	Returns the time for the player
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
	status_audio_command(TRUE);
    status_obst_detection(TRUE);
    status_goal_detection(TRUE);
    time = chVTGetSystemTime();

    while (!verify_finish_line()) {
    	chThdSleepMilliseconds(200); //
    }
    time = chVTGetSystemTime() - time;
	return time;
}

/*
*	Simple function used to show the LED ID of a player associated with a selector position
*
*	params :
*	led_conf_name_t led_conf		one of the three RGB LED configurations defined
*	uint8_t red_i					*
*	uint8_t green_i					* values corresponding with one of the five RGB colors defined
*	uint8_t blue_i					*
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


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
