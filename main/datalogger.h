/*
 * This file is part of the WiCAN project.
 *
 * Copyright (C) 2022  Meatpi Electronics.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Time-series datalogger.
 *
 * Samples values that the autopid engine has already computed from the CAN
 * bus, at a per-PID frequency chosen by the user (1..60 s). Samples are
 * formatted as InfluxDB line protocol and buffered in memory (optionally
 * gzip-compressed). When the buffer fills, or a maximum delay elapses, the
 * batch is either uploaded to an InfluxDB v1 instance over HTTPS (when Wi-Fi
 * is available) or flushed to the SD card (WICAN_PRO only) when offline.
 */

#pragma once

#include <stdbool.h>
#include "hw_config.h"      /* FS_MOUNT_POINT */

/* Path of the on-flash JSON configuration file. */
#define DATALOGGER_CONFIG_PATH      FS_MOUNT_POINT"/datalogger.json"

/**
 * @brief Initialise the datalogger. Loads configuration from
 *        DATALOGGER_CONFIG_PATH and, if enabled, starts the sampling task.
 *        Safe to call even when no configuration exists (becomes a no-op).
 *
 * @param device_id Device unique id, used as the "device" Influx tag.
 */
void datalogger_init(const char *device_id);

/**
 * @brief Re-read the configuration file and (re)start or stop the task.
 *        Called by the config server after the user saves new settings.
 */
void datalogger_reload(void);

/**
 * @brief Whether the datalogger is currently enabled and running.
 */
bool datalogger_is_running(void);
