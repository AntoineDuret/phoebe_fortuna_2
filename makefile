# Project name
PROJECT = phoebe_fortuna_2

# Path to the e-puck2_main-processor folder
GLOBAL_PATH = ../lib/e-puck2_main-processor

# Source files to include
CSRC += ./main.c \
		./audio_processing.c \
		./communications.c \
		./proximity_sensors.c \
		./process_image.c \

# Header folders to include
INCDIR += 

# Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile