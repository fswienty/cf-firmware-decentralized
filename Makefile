# enable app support
APP=1
APP_STACKSIZE=300

VPATH += src/
PROJ_OBJ += decentralized_main.o
PROJ_OBJ += vector3.o

CRAZYFLIE_BASE=crazyflie-firmware
include $(CRAZYFLIE_BASE)/Makefile