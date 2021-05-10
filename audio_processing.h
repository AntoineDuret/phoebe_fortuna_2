#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#define GAME_SPEED				1100 				// speed of the motors (max 1100)
#define NB_SAMPLES				40		//max 256			// samples for voice calibration

// Frequency domain parameters & FFT parameters
#define FFT_SIZE 				1024
#define MIN_VALUE_THRESHOLD		10000				// to avoid noisy effects
#define MID_FREQ				15
#define MIN_FREQ				8
#define MAX_FREQ				22
#define HALF_BW					5

// PID regulator parameters
#define KP 						200.0f
#define KD						2.0f
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


void mic_input_start(void);
void sound_remote(float* data);
void processAudioData(int16_t *data, uint16_t num_samples);
void wait_send_to_computer(void);
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

/*
 * NEW FUNCTIONS DEFINED
 */
void statusAudioCommand(bool status);
void statusVoiceCalibration(bool status);
bool getStatusVoiceCalibration(void);
void player_voice_calibration(float* data);


#endif /* AUDIO_PROCESSING_H */
