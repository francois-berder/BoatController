APP=mpu6050_collect
APP_SRCS = src/block_storage.c \
           src/fat16/fat16_priv.c \
           src/fat16/fat16.c \
           src/fat16/path.c \
           src/fat16/rootdir.c \
           src/fat16/subdir.c \
           src/main.c \
           src/mbr.c \
           src/mpu6050.c \
           src/sdcard.c

# Disable debug print from fat16 driver
CFLAGS += -DNDEBUG

BOARD=boat_controller
FRAMEWORK_ROOT_DIR=$(CURDIR)/../..

include $(FRAMEWORK_ROOT_DIR)/Makefile.include