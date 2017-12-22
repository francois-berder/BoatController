APP=mpu6050_collect
APP_SRCS = src/main.c \
           src/mbr.c

# Disable debug print from fat16 driver
CFLAGS += -DNDEBUG
LIBS = fat16
DRIVERS = mpu6050 mpu6050_fifo sdcard sdcard_cache

BOARD=boat_controller
FRAMEWORK_ROOT_DIR=$(CURDIR)/../..

include $(FRAMEWORK_ROOT_DIR)/Makefile.include