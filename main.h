#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <arm_math.h>

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

// The game can be played with 1 to 15 players
#define	NB_PLAYERS_MAX			15

// User has a delay to stay on a selector position before nbPlayers is set and saved.
// This delay is given by: 		delay = (PLAYER_SELECT_DELAY*0.1) [s]	(see game_setting function)
#define	PLAYER_SELECT_DELAY		17

// List of the RGB LED configurations
typedef enum {
	FULL,
	HALF,
	DIAG,
	NUM_RGB_CONF,
} led_conf_name_t;

// List of the RGB LED colors
#define		NO_LIGHT			  0,   0,   0
#define		BLUE				  0,   0, 100
#define		PINK				100,   0, 100
#define		LIGHT_BLUE			  0, 100, 100
#define		YELLOW				100, 100,   0
#define		ORANGE				100,  30,   0


/*======================================================================================*/
/* 								NEW FUNCTIONS DEFINED									*/
/*======================================================================================*/
uint8_t game_setting(void);
void led_selector_management(int selector_pos);
void player_voice_config(void);
uint  game_running(void);
void set_player_led_configuration(led_conf_name_t led_conf,
										uint8_t red_i, uint8_t green_i, uint8_t blue_i);
void body_led_confirm(void);


// Robot wide IPC bus
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif


#endif
