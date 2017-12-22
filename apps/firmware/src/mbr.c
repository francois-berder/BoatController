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

#include <stdint.h>
#include <stdio.h>
#include "mbr.h"
#include "sdcard/sdcard.h"
#include "sdcard_cache/sdcard_cache.h"

#define BOOT_SIGNATURE          (0xAA55)
#define BOOTSTRAP_CODE_SIZE     (446U)
#define PARTITION_ENTRY_SIZE    (16U)
#define PARTITION_TABLE_SIZE    (PARTITION_ENTRY_COUNT * PARTITION_ENTRY_SIZE)
#define SECTOR_SIZE             (512U)

static struct partition_info_t partitions[PARTITION_ENTRY_COUNT];

static uint32_t to_le32(uint8_t *buffer)
{
    uint32_t res;

    res = buffer[3];
    res <<= 8;
    res |= buffer[2];
    res <<= 8;
    res |= buffer[1];
    res <<= 8;
    res |= buffer[0];
    return res;
}

void mbr_read_partition_table(void)
{
    unsigned int i;
    uint8_t partition_table[PARTITION_TABLE_SIZE];
    uint16_t boot_sig;

    /* Skip boostrap code */
    sdcard_cache_seek(BOOTSTRAP_CODE_SIZE);

    sdcard_cache_read(partition_table, PARTITION_TABLE_SIZE);
    for (i = 0; i < PARTITION_ENTRY_COUNT; ++i) {
        uint8_t *entry = &partition_table[i * PARTITION_ENTRY_SIZE];

        partitions[i].status = entry[0];
        partitions[i].type = entry[4];
        partitions[i].start_sector = to_le32(&entry[8]);
        partitions[i].size = to_le32(&entry[12]);
        partitions[i].size *= SECTOR_SIZE;
    }

    /* Check boot signature */
    sdcard_cache_read(&boot_sig, sizeof(boot_sig));
    if (boot_sig != BOOT_SIGNATURE)
        printf("Invalid boot signature in MBR\n");
}

struct partition_info_t mbr_get_partition_info(unsigned int index)
{
    return partitions[index];
}
