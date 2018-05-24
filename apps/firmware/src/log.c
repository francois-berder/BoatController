#include <stdio.h>
#include "fat16/fat16.h"
#include "log.h"
#include "periph/crypto.h"
#include "sdcard_cache/sdcard_cache.h"

static int fd = -1;

void log_init(void)
{
    char filepath[13];
    unsigned int i;

    /* Create filename: 8 random letters */
    crypto_power_up();
    crypto_enable();
    crypto_get_random(filepath, 8);
    crypto_disable();
    crypto_power_down();
    for (i = 0; i < 8; ++i)
        filepath[i] = 'A' + (filepath[i] & 0xF);
    filepath[8] = '.';
    filepath[9] = 'T';
    filepath[10] = 'X';
    filepath[11] = 'T';
    filepath[12] = '\0';

    /* Open log file */
    fd = fat16_open(filepath, 'w');
    if (fd < 0)
        printf("Cannot log I/O to file %s\n", filepath);
    else
        printf("Logging I/O to file %s\n", filepath);

    /*
     * Flush cache now to ensure that file will exist on the SD card
     * even if no radio frames are found later.
     */
    sdcard_cache_flush();
}

void log_radio_frame(struct radio_frame_t rf)
{
    char buffer[64];
    if (fd >= 0) {
        int ret = sprintf(buffer, "%lu,0,%u,%u\n",
                        rf.t, rf.dir, rf.speed);
        if (ret >= 0)
            fat16_write(fd, buffer, ret);
    }
}

void log_output_frame(uint32_t t, struct output_frame_t of)
{
    char buffer[64];
    if (fd >= 0) {
        int ret = sprintf(buffer, "%lu,1,%u,%u,%u,%u\n",
                        t, of.left_rudder, of.right_rudder, of.left_motor, of.right_motor);
        if (ret >= 0)
            fat16_write(fd, buffer, ret);
    }
}

void log_mpu6050_frame(struct mpu6050_sample_t s)
{
    char buffer[64];
    if (fd >= 0) {
        int ret = sprintf(buffer, "%lu,2,%d,%d,%d,%d,%d,%d\n",
                        s.t,
                        s.accel.x, s.accel.y, s.accel.z,
                        s.gyro.x, s.gyro.y, s.gyro.z);
        if (ret >= 0)
            fat16_write(fd, buffer, ret);
    }
}

void log_flush(void)
{
    sdcard_cache_flush();
}

void log_stop(void)
{
    if (fd >= 0) {
        fat16_close(fd);
        fd = -1;
    }
}

int log_is_running(void)
{
    return fd >= 0;
}
