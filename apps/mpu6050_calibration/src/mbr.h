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

#ifndef __MBR_H__
#define __MBR_H__

#include <stdint.h>

#define BOOTABLE_PARTITION      (0x80)
#define INACTIVE_PARTITION      (0x00)
#define PARTITION_ENTRY_COUNT   (4U)
#define FAT16_PARTITION_TYPE    (0x06)

struct partition_info_t {
    uint8_t status;
    uint8_t type;
    uint32_t start_sector;       /**< Index of the first sector of the partition */
    uint32_t size;               /**< Size of the parition in bytes */
};

void mbr_read_partition_table(void);

struct partition_info_t mbr_get_partition_info(unsigned int index);

#endif
