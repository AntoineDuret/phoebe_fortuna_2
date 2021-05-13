#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#define GAME_SPEED				1100 	// speed of the motors (max 1100)
#define NB_SAMPLES				40		// samples for voice calibration (max 256)

// Frequency domain parameters & FFT parameters
#define FFT_SIZE 				1024
#define MIN_VALUE_THRESHOLD		10000	// to avoid noisy effects
#define MID_FREQ				15
#define MIN_FREQ				8
#define MAX_FREQ				22
#define HALF_BW					5
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
//void wait_send_to_computer(void);
float* get_audio_buffer_ptr(BUFFER_NAME_t name);


/*======================================================================================*/
/* 								NEW FUNCTIONS DEFINED									*/
/*======================================================================================*/
//void mic_input_start(void);
void player_voice_calibration(float* data);
void sound_remote(float* data);
void status_audio_command(bool status);
void status_voice_calibration(bool status);
bool get_status_voice_calibration(void);


#endif /* AUDIO_PROCESSING_H */
