/*
 * Copyright (C) 2018 Francois Berder <fberder@outlook.fr>
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

#ifndef __RADIO_FILTER_H__
#define __RADIO_FILTER_H__

#include <stdbool.h>
#include "radio.h"

/**
 * @brief This function is called whenever the
 * radio contact with the emitter is lost or re-established.
 *
 * @param[in] lost True if radio contact is lost, false otherwise
 */
void radio_contact_status_callback(bool lost);

/**
 * @brief
 *
 * @retval 0 Output radio frame is invalid
 * @retval 1 Output radio frame is valid
 */
int radio_filter(struct radio_frame_t *out, struct radio_frame_t *in);

#endif
