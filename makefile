#Project name
PROJECT = BallBalance

#Path to the e-puck2_main-processor folder
GLOBAL_PATH = ../../lib/e-puck2_main-processor

#Source files
CSRC += ./main.c \
		./motors_speed.c \

#Header folders
INCDIR += 

#Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile