#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <arm_math.h>

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define		NB_PLAYERS_MAX			15

#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the wheel
#define WHEEL_DISTANCE      5.1f    // cm 5.35
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define WHEEL_PERIMETER     12.5f // [cm]



// List of the RGB LED configurations
typedef enum {
	FULL,
	HALF,
	DIAG,
	NUM_RGB_CONF,
} led_conf_name_t;

// List of the RGB LED colors
#define		BLUE					  0,   0, 100
#define		PINK					100,   0, 100
#define		LIGHT_BLUE				  0, 100, 100
#define		YELLOW					100, 100,   0
#define		ORANGE					100,  30,   0


/*
 * NEW FUNCTIONS DEFINED
 */
uint8_t game_setting(void);
void LED_selector_management(int selector_pos);
void player_voice_config(void);
uint  game_running(void);
void set_player_led_configuration(led_conf_name_t led_conf,
										uint8_t red_i, uint8_t green_i, uint8_t blue_i);


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif


#endif
