# enable app support
APP=1
APP_STACKSIZE=300

VPATH += src/
PROJ_OBJ += random.o

CRAZYFLIE_BASE=crazyflie-firmware
include $(CRAZYFLIE_BASE)/Makefile