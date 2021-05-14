/*
  \file   	audio_processing.c
  \author 	Antoine Duret & Carla Schmid (Group G08)
  \date   	13.05.2021
  \version	2.1
  \brief  	Code for audio processing related tasks
*/

#include <arm_math.h>
#include <arm_const_structs.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"

#include "audio_processing.h"
#include "communications.h"
#include "main.h"
#include "process_image.h"
#include "proximity_sensors.h"

#include "audio/microphone.h"
#include "motors.h"
#include "usbcfg.h"


/*======================================================================================*/
/* 						 	     REUSED CODE FROM THE TP5				    			*/
/* 						  (with small additions and corrections)						*/
/*======================================================================================*/

// Semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE); // @suppress("Field cannot be resolved")

// 2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];

// Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];
static float four_mics_output[FFT_SIZE];

static bool audio_command_on = 0;
static bool voice_calibration_on = 0;
static uint8_t mid_freq = MID_FREQ;


/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void process_audio_data(int16_t *data, uint16_t num_samples) {
	/*
	*	We get 160 samples per mic every 10 ms.
	*	So we fill the samples buffers to reach 1024 samples, then we compute the FFTs.
	*/
	static uint16_t nb_samples = 0;

	// Loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4) {
		// Construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		// Stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)) {
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)) {
		/*
		*	- FFT processing -
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*	We use only the left microphone.
		*/
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);

		/*
		*	- Magnitude processing -
		*	Computes the magnitude of the complex numbers and stores them
		*	in a buffer of FFT_SIZE because it only contains real numbers.
		*/
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);


		if((voice_calibration_on)) {
			for(uint16_t i = 0; i < FFT_SIZE; i++) {
				four_mics_output[i] = (micLeft_output[i] + micRight_output[i]
										+ micFront_output[i] + micBack_output[i])/4.0;
			}

			player_voice_calibration(four_mics_output);
		}

		nb_samples = 0;

		if(audio_command_on) {
			for(uint16_t i = 0; i < FFT_SIZE; i++) {
				four_mics_output[i] = (micLeft_output[i] + micRight_output[i]
										+ micFront_output[i] + micBack_output[i])/4.0;
			}

			sound_remote(four_mics_output);
		}
	}
}


/*
*	Wrapper to call a very optimized fft function provided by ARM
*	which uses a lot of tricks to optimize the computations
*/
void doFFT_optimized(uint16_t size, float* complex_buffer) {
	if(size == 1024) {
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
	}
}


/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name) {
	if(name == LEFT_CMPLX_INPUT) {
		return micLeft_cmplx_input;
	}
//	}
//	else if(name == RIGHT_CMPLX_INPUT) {
//		return micRight_cmplx_input;
//	}
//	else if(name == FRONT_CMPLX_INPUT) {
//		return micFront_cmplx_input;
//	}
//	else if(name == BACK_CMPLX_INPUT) {
//		return micBack_cmplx_input;
//	}
	else if(name == LEFT_OUTPUT) {
		return micLeft_output;
	}
//	else if(name == RIGHT_OUTPUT) {
//		return micRight_output;
//	}
//	else if(name == FRONT_OUTPUT) {
//		return micFront_output;
//	}
//	else if(name == BACK_OUTPUT) {
//		return micBack_output;
//	}
	else {
		return NULL;
	}
}



/*======================================================================================*/
/* 									 NEW FUNCTIONS										*/
/*======================================================================================*/

/*
*	Function defined to do the voice calibration for each player before their game.
*
*	params :
*	float* data			pointer to an array containing the computed magnitude of the cpx numbers
*						for one mic
*/
void player_voice_calibration(float* data) {
	uint16_t max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;
	static uint16_t ind_sample = 0;
	static uint16_t average_freq = 0;

	// Search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++) {
		if(data[i] > max_norm) {
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	if((max_norm_index != -1) && (max_norm_index >= MIN_FREQ) && (max_norm_index <= MAX_FREQ)) {
		average_freq += max_norm_index;
		ind_sample++;
	}

	if(ind_sample == NB_SAMPLES) {
		average_freq = (average_freq/ind_sample);
		mid_freq = average_freq;
		voice_calibration_on = FALSE;

		// Reset the average_freq to 0 for next calibration.
		average_freq = 0;
		ind_sample = 0;
	}
}


/*
*	Simple function used to detect the highest value in a buffer and to execute a motor
*	command depending on it. PID control for fine audio command.
*
*	params :
*	float* data			pointer to an array containing the computed magnitude of the cpx numbers
*						for one mic
*/
void sound_remote(float* data) {
	int16_t error = 0;
	float speed = 0;

	uint16_t max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	static int16_t sum_error = 0;
	static int16_t previous_error = 0;
	int16_t deriv_error = 0;

	// Search for the highest peak
	for(uint16_t i = mid_freq - HALF_BW ; i <= mid_freq + HALF_BW ; i++) {
		if(data[i] > max_norm) {
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	// PID regulator implementation on the speed for fine audio control.
	// Defined with the error between the mid_freq (voice calibration) and the control freq.
	if((max_norm_index == -1) || (audio_command_on == 0)) {
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		sum_error = 0;
	} else {
		error = max_norm_index - mid_freq;
		sum_error += error;
		deriv_error = error - previous_error;
		previous_error = error;

		// ARW
		if(abs(sum_error) > MAX_SUM_ERROR) {
			if(sum_error > 0) {
				sum_error = MAX_SUM_ERROR;
			} else {
				sum_error = - MAX_SUM_ERROR;
			}
		}

		speed = KP * error + KI * sum_error + KD * deriv_error;

		// Peak at a lower frequency than middle frequency : turn left / higher : turn right
		if(abs(error) < ERROR_THRESHOLD) {
			left_motor_set_speed(GAME_SPEED);
			right_motor_set_speed(GAME_SPEED);
			sum_error = 0;
		} else if(error < 0) {
			left_motor_set_speed(GAME_SPEED + speed);
			right_motor_set_speed(GAME_SPEED);
		} else {
			left_motor_set_speed(GAME_SPEED);
			right_motor_set_speed(GAME_SPEED - speed);
		}
	}
}


/*
*	Function to control the audio command.
*
*	params :
*	bool status		status value TRUE or FALSE
*/
void status_audio_command(bool status) {
	audio_command_on = status;
}


/*
*	Function to control the voice calibration.
*
*	params :
*	bool status		status value TRUE or FALSE
*/
void status_voice_calibration(bool status) {
	voice_calibration_on = status;
}


/*
*	Function to get the voice calibration control status.
*/
bool get_status_voice_calibration(void) {
	return voice_calibration_on;
}
