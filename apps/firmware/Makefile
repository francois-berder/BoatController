APP=firmware
APP_SRCS = src/block_storage.c \
           src/controller.c \
           src/main.c \
           src/mbr.c \
           src/output.c \
           src/radio.c \
           src/status.c

# Disable debug print from fat16 driver
CFLAGS += -DNDEBUG
DRIVERS = mpu6050 imu sdcard
LIBS = fat16

BOARD=boat_controller
FRAMEWORK_ROOT_DIR=$(CURDIR)/../..

include $(FRAMEWORK_ROOT_DIR)/Makefile.include