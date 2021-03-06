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

#include <string.h>
#include "sdcard_cache/sdcard_cache.h"
#include "sdcard/sdcard.h"

#define CACHE_ENTRY_COUNT           (8U)
#define CACHE_ENTRY_VALID_FLAG      (0x8000)
#define CACHE_ENTRY_DIRTY_FLAG      (0x4000)
#define CACHE_ENTRY_COUNTER_MASK    (0x3FFF)

static struct sdcard_spi_dev_t dev;
struct cache_entry_t {
    uint16_t status;            /**< [15..14]: valid & dirty flags, [0..13]: 14bit saturating counter */
    uint32_t sector;            /**< sector index */
    uint8_t block[SDCARD_BLOCK_LENGTH];
};
static struct cache_entry_t cache[CACHE_ENTRY_COUNT];
static uint32_t current_address;
static struct sdcard_cache_stats_t stats;

static unsigned int load_block(void)
{
    uint32_t sector = current_address >> 9;
    unsigned int cache_index = 0;
    unsigned int i;

    /* Check if the block is already in cache */
    for (i = 0; i < CACHE_ENTRY_COUNT; ++i) {
        if (cache[i].status & CACHE_ENTRY_VALID_FLAG
        &&  cache[i].sector == sector)
            return i;
    }

    /* The block we need is not in cache, let's find an empty entry */
    for (i = 0; i < CACHE_ENTRY_COUNT; ++i) {
        if (!(cache[i].status & CACHE_ENTRY_VALID_FLAG)) {
            cache_index = i;
            break;
        }
    }

    /* No empty space in cache, let's evict an entry */
    if (i == CACHE_ENTRY_COUNT) {
        uint16_t counter = cache[0].status & CACHE_ENTRY_COUNTER_MASK;
        cache_index = 0;

        /*
         * Find the cache entry with the lowest counter, this indicates
         * that it is the least used entry since the last flush.
         */
        for (i = 1; i < CACHE_ENTRY_COUNT; ++i) {
            uint16_t c = cache[i].status & CACHE_ENTRY_COUNTER_MASK;
            if (c < counter) {
                cache_index = i;
                counter = c;
            }
        }

        ++stats.evictions;

        /* Write back evicted block to SD card */
        if (cache[cache_index].status & CACHE_ENTRY_DIRTY_FLAG) {
            if (sdcard_write_block(&dev, cache[cache_index].block, cache[cache_index].sector) < 0)
                ++stats.write_error;
            else
                ++stats.write_success;
        }

        /*
         * Reset counters of all valid entries.
         * This aims at preventing old entries to stay in cache forever.
         */
        for (i = 0; i < CACHE_ENTRY_COUNT; ++i) {
            if (cache[cache_index].status & CACHE_ENTRY_VALID_FLAG)
                cache[cache_index].status &= ~CACHE_ENTRY_COUNTER_MASK;
        }
    }

    /* Initialise cache entry */
    cache[cache_index].status = CACHE_ENTRY_VALID_FLAG;
    cache[cache_index].sector = sector;
    if (sdcard_read_block(&dev, cache[cache_index].block, sector) < 0)
        ++stats.read_error;
    else
        ++stats.read_success;

    return cache_index;
}

void sdcard_cache_init(struct sdcard_spi_dev_t _dev)
{
    unsigned int i;

    dev = _dev;

    /* Mark all entries as invalid */
    for (i = 0; i < CACHE_ENTRY_COUNT; ++i)
        cache[i].status &= ~CACHE_ENTRY_VALID_FLAG;

    current_address = 0;

    /* Set to 0 all stats */
    stats.evictions = 0;
    stats.read_success = 0;
    stats.write_success = 0;
    stats.read_error = 0;
    stats.write_error = 0;
}

int sdcard_cache_read(void *buffer, uint32_t length)
{
    uint8_t *data = (uint8_t *)buffer;

    while (length) {
        unsigned int cache_index = load_block();
        uint16_t offset = current_address & (SDCARD_BLOCK_LENGTH - 1);
        uint16_t len = SDCARD_BLOCK_LENGTH - offset;
        if (len > length)
            len = length;

        memcpy(data, &cache[cache_index].block[offset], len);

        length -= len;
        data += len;
        current_address += len;
    }

    return 0;
}

int sdcard_cache_read_byte(void *data)
{
    sdcard_cache_read(data, 1);
    return 0;
}

int sdcard_cache_write(const void *buffer, uint32_t length)
{
    uint8_t *data = (uint8_t *)buffer;

    while (length) {
        unsigned int cache_index = load_block();
        uint16_t offset = current_address & (SDCARD_BLOCK_LENGTH - 1);
        uint16_t len = SDCARD_BLOCK_LENGTH - offset;
        if (len > length)
            len = length;

        memcpy(&cache[cache_index].block[offset], data, len);

        cache[cache_index].status |= CACHE_ENTRY_DIRTY_FLAG;

        /* Increase saturating counter */
        if ((cache[cache_index].status & CACHE_ENTRY_COUNTER_MASK) != CACHE_ENTRY_COUNTER_MASK)
            cache[cache_index].status++;

        length -= len;
        data += len;
        current_address += len;
    }

    return 0;
}

int sdcard_cache_seek(uint32_t address)
{
    current_address = address;
    return 0;
}

void sdcard_cache_flush(void)
{
    unsigned int i;

    /* Write to SD card all valid & dirty entries */
    for (i = 0; i < CACHE_ENTRY_COUNT; ++i) {
        if (cache[i].status & CACHE_ENTRY_VALID_FLAG
        &&  cache[i].status & CACHE_ENTRY_DIRTY_FLAG) {
            if (sdcard_write_block(&dev, cache[i].block, cache[i].sector) < 0)
                ++stats.write_error;
            else
                ++stats.write_success;

            cache[i].status &= ~CACHE_ENTRY_DIRTY_FLAG;
        }
    }

    /* Reset all counters */
    for (i = 0; i < CACHE_ENTRY_COUNT; ++i)
        cache[i].status &= ~CACHE_ENTRY_COUNTER_MASK;
}

struct sdcard_cache_stats_t sdcard_cache_get_stats(void)
{
    return stats;
}
