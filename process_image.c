#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <stdbool.h>

#include <main.h>
#include "camera/po8030.h"
#include "sensors/VL53L0X/VL53L0X.h"

#include <process_image.h>

//only for debugging
#include "leds.h"

static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2; //middle
static bool line_found = FALSE;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);



/* Returns the line width extracted from the image buffer given
* Returns 0 if line not found
*/
void extract_line_width(uint8_t *buffer) { //uint16_t

	volatile uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	//static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint32_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;


	do {
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)) {
			if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean){
				begin = i;
				stop = 1;
			}
			i++;
		}
		//if begin was found, search for an end
		if(i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin) {
			stop = 0;
			while (stop == 0 && i < IMAGE_BUFFER_SIZE) {
				if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean){
					end = i;
					stop = 1;
				}
				i++;
			}
			//if an end was not found
			if(i > IMAGE_BUFFER_SIZE || !end) {
				line_not_found = 1;
			}
		} else //if no begin was found
			line_not_found = 1;

		//if too small has been detected, continue the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH) {
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	} while(wrong_line);

	if(line_not_found) {
		//begin = 0;
		//end = 0;
		//width = last_width;
		line_found = FALSE;
	} else {
		line_found = TRUE;
		//last_width = width = (end-begin);
		//line_position = (begin + end)/2;
	}

	/*//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	} else {
		return width;
	}*/
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	//systime_t time;
	while(1){
		//time = chVTGetSystemTime();
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//chprintf((BaseSequentialStream *)&SDU1, "capture time = %d\n", chVTGetSystemTime()-time); //cable USB doit etre branché, sinon il restera stuck ici source: Antoine Duret
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint16_t lineWidth = 0;

	//bool send_to_computer = true;

    while(1) {
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
        //gets the pointer to the array filled with the last image in RGB565
        img_buff_ptr = dcmi_get_last_image_ptr();

  		for(uint16_t i = 0; i < 2 * IMAGE_BUFFER_SIZE; i+=2) {
   			//image[i/2] = ((uint8_t) img_buff_ptr[i+1] & 0x1F);	//blue
   			image[i/2] = ((uint8_t) img_buff_ptr[i] & 0xF8); //red
   		}

  		//search for line in the image and gets its width in pixels
  		//lineWidth =
  		extract_line_width(image);
  		if(lineWidth)
  			distance_cm = PXTOCM/lineWidth;

/*  		if(send_to_computer) {
  		  	SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
  		  	//chprintf((BaseSequentialStream *)&SDU1, "Distance = %f\n", get_distance_cm());
  		}
  		send_to_computer = ! send_to_computer;*/

    }
}




float get_distance_cm(void){
	return distance_cm;
}

uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

bool verify_finish_line(void) {
	//chprintf((BaseSequentialStream *)&SD3, "Distance = %d\r", VL53L0X_get_dist_mm()); //SDU1
	 if(VL53L0X_get_dist_mm() <= 150) {
		  //chprintf((BaseSequentialStream *)&SD3, "line testing"); //SDU1
		 if(line_found) {
			// chprintf((BaseSequentialStream *)&SD3, "line found"); //SDU1
		 	 set_body_led(1);
			 chThdSleepMilliseconds(400);
			 set_body_led(0);
			 chThdSleepMilliseconds(400);
		 }
	 }
	return line_found;
}
