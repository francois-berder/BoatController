APP=mpu6050_collect
APP_SRCS = src/block_storage.c \
           src/main.c \
           src/mbr.c \
           src/mpu6050.c \
           src/sdcard.c

# Disable debug print from fat16 driver
CFLAGS += -DNDEBUG
LIBS = fat16

BOARD=boat_controller
FRAMEWORK_ROOT_DIR=$(CURDIR)/../..

include $(FRAMEWORK_ROOT_DIR)/Makefile.include