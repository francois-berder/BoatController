APP=mpu6050_calibration
APP_SRCS = src/main.c

# Disable debug print from fat16 driver
CFLAGS += -DNDEBUG
LIBS = fat16
DRIVERS = mbr mpu6050 sdcard sdcard_cache

BOARD=boat_controller
FRAMEWORK_ROOT_DIR=$(CURDIR)/../..

include $(FRAMEWORK_ROOT_DIR)/Makefile.include