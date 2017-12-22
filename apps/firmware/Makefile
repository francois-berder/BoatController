APP=firmware
APP_SRCS = src/controller.c \
           src/main.c \
           src/mbr.c \
           src/output.c \
           src/radio.c \
           src/status.c

# Disable debug print from fat16 driver
CFLAGS += -DNDEBUG
DRIVERS = mpu6050 mpu6050_fifo sdcard sdcard_cache
LIBS = fat16

BOARD=boat_controller
FRAMEWORK_ROOT_DIR=$(CURDIR)/../..

include $(FRAMEWORK_ROOT_DIR)/Makefile.include