#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include "chbsem.h"

//debugg
#include "leds.h"


// Semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

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

static bool audio_command_on = 0;
static bool voice_calibration_on = 0;
static uint8_t mid_freq = MID_FREQ;

// Thread for microphone input
static THD_WORKING_AREA(micInput_thd_wa, 2048); // watch taille pour ce thread et autre thread gestion de selcteur
static THD_FUNCTION(micInput_thd, arg) {
    (void)arg;
    chRegSetThreadName(__FUNCTION__);

    while(1) {
      	float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);
        float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT);

        uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, FFT_SIZE);

        if(size == FFT_SIZE) {
            /*   Optimized FFT   */
        	chSysLock();
            doFFT_optimized(FFT_SIZE, bufferCmplxInput);
            arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);
            chSysUnlock();
        }

        chThdSleepMilliseconds(100); // capteurs 100 Hz, PID 10Hz,
    }
}


void mic_input_start(void) {
	chThdCreateStatic(micInput_thd_wa, sizeof(micInput_thd_wa), NORMALPRIO, micInput_thd, NULL);
}


/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data) {
	float error = 0;
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	float speed = 0;
	static float sum_error = 0;
	static float previous_error = 0;
	float deriv_error = 0;

	// Search for the highest peak
	for(uint16_t i = mid_freq - HALF_BW ; i <= mid_freq + HALF_BW ; i++) {
		if(data[i] > max_norm) {
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	// PID regulator implementation for fine audio control
	if((max_norm_index == -1) || (audio_command_on == 0)) {
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		sum_error = 0;
	} else {
		error = max_norm_index - mid_freq;
		sum_error += error;
		deriv_error = error - previous_error;
		previous_error = error;

		if(abs(sum_error) > MAX_SUM_ERROR) {
			if(sum_error > 0)
				sum_error = MAX_SUM_ERROR;
			else sum_error = - MAX_SUM_ERROR;
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
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples) {
	/*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
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
		*	FFT processing
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/
		//doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		//doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		//doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*
		*	Magnitude processing
		*	Computes the magnitude of the complex numbers and stores them
		*	in a buffer of FFT_SIZE because it only contains real numbers.
		*/
		//arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		// Sends only one FFT result over 8 for 1 mic
		if((voice_calibration_on)) {
			player_voice_calibration(micLeft_output);
		}

		nb_samples = 0;

		if(audio_command_on) {
			sound_remote(micLeft_output);
		}
	}
}


/*
*	Simple function to put the invoking thread into sleep until it can process the audio data
*/
void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}


/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name) {
	if(name == LEFT_CMPLX_INPUT) {
		return micLeft_cmplx_input;
	}
	else if(name == RIGHT_CMPLX_INPUT) {
		return micRight_cmplx_input;
	}
	else if(name == FRONT_CMPLX_INPUT) {
		return micFront_cmplx_input;
	}
	else if(name == BACK_CMPLX_INPUT) {
		return micBack_cmplx_input;
	}
	else if(name == LEFT_OUTPUT) {
		return micLeft_output;
	}
	else if(name == RIGHT_OUTPUT) {
		return micRight_output;
	}
	else if(name == FRONT_OUTPUT) {
		return micFront_output;
	}
	else if(name == BACK_OUTPUT) {
		return micBack_output;
	}
	else {
		return NULL;
	}
}


void statusAudioCommand(bool status) {
	audio_command_on = status;
}


void statusVoiceCalibration(bool status) {
	voice_calibration_on = status;
}

bool getStatusVoiceCalibration(void) {
	return voice_calibration_on;
}



void player_voice_calibration(float* data) {
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;
	static uint16_t ind_sample = 0;
	//static uint16_t ind_average[NB_SAMPLES];
	static uint16_t average_freq = 0;

	// Search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++) {
		if(data[i] > max_norm){
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
	}
}

