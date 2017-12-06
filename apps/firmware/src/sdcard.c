/*
 * Copyright (C) 2017  Francois Berder <fberder@outlook.fr>
 *
 * This file is part of pic24-framework.
 *
 * pic24-framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * pic24-framework is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with pic24-framework.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stddef.h>
#include <xc.h>
#include "mcu.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "periph_conf.h"
#include "sdcard.h"

#define MOSI_PIN            (GPIO_PIN(PORT_B, 13))
#define MISO_PIN            (GPIO_PIN(PORT_B, 4))
#define SCK_PIN             (GPIO_PIN(PORT_B, 6))
#define CS_PIN              (GPIO_PIN(PORT_B, 7))

#define INIT_ATTEMPT_COUNT_MAX          (100)
#define RESPONSE_ATTEMPT_COUNT_MAX      (128)
#define DATA_START_TOKEN                (0xFE)
#define CMD_TOKEN                       (0x40)
#define DATA_ACCEPTED                   (0x05)
#define DUMMY_CRC                       (0xFF)

enum SD_COMMAND {
    CMD0_GO_IDLE_STATE = 0,
    CMD1_SEND_OP_COND = 1,
    CMD8_SEND_IF_COND = 8,
    CMD16_SET_BLOCKLEN = 16,
    CMD17_READ_SINGLE_BLOCK = 17,
    CMD24_WRITE_SINGLE_BLOCK = 24,
    ACMD41_SEND_OP_COND = 41,
    CMD55_APP_CMD = 55,
    CMD59_CRC_ON_OFF = 59
};

enum SD_STATUS {
    IDLE_STATE = 0x1,
    ERASE_RESET = 0x2,
    ILLEGAL_COMMAND = 0x4,
    CRC_ERROR = 0x08,
    ERASE_SEQ_ERROR = 0x10,
    ADDRESS_ERROR = 0x20,
    PARAM_ERROR = 0x40
};

#define SD_STATUS_ERROR         (ILLEGAL_COMMAND | CRC_ERROR \
                                | ERASE_SEQ_ERROR | ADDRESS_ERROR \
                                | PARAM_ERROR)

static int wait_for(uint8_t byte)
{
    uint8_t attempt_count = RESPONSE_ATTEMPT_COUNT_MAX;
    uint8_t data;

    do {
        --attempt_count;
        spi_transfer(SPI_1, NULL, &data, 1);
    } while (attempt_count && data != byte);

    if (attempt_count == 0)
        return -1;

    return 0;
}

static int send_cmd(uint8_t *response, uint8_t cmd, uint32_t arg, uint8_t crc)
{
    int ret = 0;
    uint8_t buffer[] = {
        CMD_TOKEN | (cmd & 0x3F),
        arg >> 24,
        arg >> 16,
        arg >> 8,
        arg,
        crc
    };

    /* Flush spi buffer by sending dummy byte */
    spi_transfer(SPI_1, NULL, NULL, 1);

    gpio_write(CS_PIN, 0);

    /* Flush spi buffer by sending dummy byte */
    spi_transfer(SPI_1, NULL, NULL, 1);

    spi_transfer(SPI_1, buffer, NULL, sizeof(buffer));

    /* Wait for response from SD card */
    {
        uint8_t response_attempt_count = RESPONSE_ATTEMPT_COUNT_MAX;
        do {
            --response_attempt_count;
            spi_transfer(SPI_1, NULL, response, 1);
        } while (response_attempt_count && *response == 0xFF);

        if (response_attempt_count == 0 && *response == 0xFF)
            ret = -1;
    }

    /* Flush spi buffer by sending dummy byte */
    spi_transfer(SPI_1, NULL, NULL, 1);

    gpio_write(CS_PIN, 1);

    /* Flush spi buffer by sending dummy byte */
    spi_transfer(SPI_1, NULL, NULL, 1);

    return ret;
}

/* Send CMD1 until the card returns 0 or times out */
static int old_wakeup(void)
{
    uint8_t response;
    uint16_t attempt_count = INIT_ATTEMPT_COUNT_MAX;

    do {
        attempt_count--;

        if (send_cmd(&response, CMD1_SEND_OP_COND, 0, 0xF9) < 0)
            return -1;

        if (response == 0)
            break;

        /* Let's try again 10 ticks later if the card is not ready */
        mcu_delay(10);
    } while (attempt_count > 0 && response != 0);

    if (attempt_count == 0 && response != 0)
        return -1;

    return 0;
}

/* Send CMD55/ACMD41 until the card returns 0 or times out */
static int wakeup(uint8_t is_sdhc)
{
    uint8_t response;
    uint16_t attempt_count = INIT_ATTEMPT_COUNT_MAX;

    do {
        attempt_count--;

        if (send_cmd(&response, CMD55_APP_CMD, 0, 0x65) < 0)
            return -1;

        if (is_sdhc) {
            if (send_cmd(&response, ACMD41_SEND_OP_COND, 0x40000000, 0x77) < 0)
                return -1;
        }
        else {
            if (send_cmd(&response, ACMD41_SEND_OP_COND, 0, 0xE5) < 0)
                return -1;
        }

        if (response == 0)
            break;

        /* Let's try again 10 ticks later if the card is not ready */
        mcu_delay(10);
    } while (attempt_count > 0 && response != 0);

    if (attempt_count == 0 && response != 0)
        return -1;

    return 0;
}

int sdcard_init(void)
{
    uint8_t response;
    uint8_t is_sdhc = 0;
    int ret;

    gpio_init_out(MOSI_PIN, 0);
    gpio_init_in(MISO_PIN);
    gpio_init_out(SCK_PIN, 0);
    gpio_init_out(CS_PIN, 1);

    RPOR6bits.RP13R = 0x0007;
    RPINR20bits.SDI1R = 0x0004;
    RPOR3bits.RP6R = 0x0008;

    spi_power_up(SPI_1);
    spi_configure(SPI_1, 400000, SPI_MODE_0);
    spi_enable(SPI_1);

    /* Send 160 clock cycles */
    spi_transfer(SPI_1, NULL, NULL, 20);

    /* Send CMD0 */
    if (send_cmd(&response, CMD0_GO_IDLE_STATE, 0, 0x95) < 0
    ||  (response & SD_STATUS_ERROR))
        return -1;

    /* Send CMD8 */
    if (send_cmd(&response, CMD8_SEND_IF_COND, 0x000001AA, 0x87) < 0)
        return -1;

    if (response == 0x1)
        is_sdhc = 1;

    /* Send CMD55 to find whether we need to send CMD1 or CMD55/ACMD41 to init card */
    if (send_cmd(&response, CMD55_APP_CMD, 0, 0x65) < 0)
        return -1;

    /* Attempt to put SD card out of idle state */
    if (response == 0x5)
        ret = old_wakeup();
    else
        ret = wakeup(is_sdhc);

    if (ret)
        return ret;

    if (send_cmd(&response, CMD59_CRC_ON_OFF, 0, DUMMY_CRC) < 0
    ||  (response & SD_STATUS_ERROR))
        return -1;

    if (!is_sdhc) {
        if (send_cmd(&response, CMD16_SET_BLOCKLEN, SDCARD_BLOCK_LENGTH, DUMMY_CRC) < 0
        ||  (response & SD_STATUS_ERROR))
            return -1;
    }

    /*
     * Increase clock frequency:
     *  - Use system clock for SPI module
     *  - Set BRG to 0
     */
    PMD4 &= ~_PMD4_REFOMD_MASK;
    REFOCONL = _REFOCONL_ROEN_MASK;
    spi_disable(SPI_1);
    SPI1BRGL = 0;
    SPI1CON1 |= _SPI1CON1_MCLKEN_MASK;
    spi_enable(SPI_1);

    return 0;
}

int sdcard_read_block(void *block, uint32_t sector)
{
    int ret = 0;
    uint8_t cmd[] = {
        CMD_TOKEN | CMD17_READ_SINGLE_BLOCK,
        sector >> 15,
        sector >> 7,
        sector << 1,
        0x00,
        DUMMY_CRC
    };

    /* Flush spi buffer by sending dummy byte */
    spi_transfer(SPI_1, NULL, NULL, 1);

    gpio_write(CS_PIN, 0);

    spi_transfer(SPI_1, cmd, NULL, sizeof(cmd));

    if (wait_for(0) < 0                     /* Wait for SD card to be ready */
    ||  wait_for(DATA_START_TOKEN) < 0) {
        ret = -1;
        goto sdcard_read_block_end;
    }

    spi_fast_read(SPI_1, block, SDCARD_BLOCK_LENGTH);

    /* Discard CRC */
    spi_transfer(SPI_1, NULL, NULL, 2);

sdcard_read_block_end:
    gpio_write(CS_PIN, 1);
    return 0;
}

int sdcard_write_block(const void *block, uint32_t sector)
{
    int ret = 0;
    uint8_t cmd[] = {
        CMD_TOKEN | CMD24_WRITE_SINGLE_BLOCK,
        sector >> 15,
        sector >> 7,
        sector << 1,
        0x00,
        DUMMY_CRC
    };

    /* Flush spi buffer by sending dummy byte */
    spi_transfer(SPI_1, NULL, NULL, 1);

    gpio_write(CS_PIN, 0);

    spi_transfer(SPI_1, cmd, NULL, sizeof(cmd));

    /* Wait for SD card to be ready */
    if (wait_for(0)) {
        ret = -1;
        goto sdcard_write_block_end;
    }

    /* Start sending data */
    {
        uint8_t token = DATA_START_TOKEN;
        spi_transfer(SPI_1, &token, NULL, 1);
    }

    spi_fast_write(SPI_1, block, SDCARD_BLOCK_LENGTH);

    /* Send dummy CRC */
    spi_transfer(SPI_1, NULL, NULL, 2);

    /* Check response from SD card */
    {
        uint8_t data_response;
        spi_transfer(SPI_1, NULL, &data_response, 1);
        if ((data_response & 0x1F) != DATA_ACCEPTED) {
            ret = -1;
            goto sdcard_write_block_end;
        }
    }

    /* Wait while the SD card is busy */
    {
        uint8_t status;
        do {
            spi_transfer(SPI_1, NULL, &status, 1);
        } while (status == 0);
    }

sdcard_write_block_end:
    gpio_write(CS_PIN, 1);
    return ret;
}
