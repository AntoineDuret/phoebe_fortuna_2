#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#include <stdbool.h>

#define GAME_SPEED				1100 	// Speed of the motors (max 1100)
#define NB_SAMPLES				40		// Samples for voice calibration (max 256)

// Frequency domain parameters & FFT parameters
#define FFT_SIZE 				1024
#define MIN_VALUE_THRESHOLD		10000	// Threshold for audio command intensity
#define MID_FREQ				15		// Frequency used when voice calibration is turned off
#define MIN_FREQ				8		// Lowest acceptable frequency for voice calibration
#define MAX_FREQ				22		// Highest acceptable frequency for voice calibration
#define HALF_BW					5		// Half of the voice command range
#define	ERROR_THRESHOLD			0.1f

// PID regulator parameters (tuned manually)
#define KP 						200
#define KD						2
#define KI 						2.25f
#define MAX_SUM_ERROR			(GAME_SPEED/KI)		// ARW implementation

typedef enum {
	// 2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,

	// Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;


void process_audio_data(int16_t *data, uint16_t num_samples);
void doFFT_optimized(uint16_t size, float* complex_buffer);

/*======================================================================================*/
/* 								NEW FUNCTIONS DEFINED									*/
/*======================================================================================*/
void player_voice_calibration(float* data);
void sound_remote(float* data);
void status_audio_command(bool status);
void status_voice_calibration(bool status);
bool get_status_voice_calibration(void);


#endif /* AUDIO_PROCESSING_H */
