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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_netif_sntp.h"
#include "cJSON.h"
#include "zlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <math.h>

#include "hw_config.h"
#include "datalogger.h"
#include "autopid.h"
#include "wifi_network.h"
#include "wc_timer.h"

#if HARDWARE_VER == WICAN_PRO
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include "soc/soc_caps.h"
#define DL_SD_MOUNT_POINT       "/sdcard"
#endif

#define TAG "DATALOGGER"

/* How often the sampling task wakes up to check per-PID timers (ms). */
#define DL_SAMPLE_TICK_MS           200
/* Minimum spacing between flush attempts, to avoid hammering a failing sink. */
#define DL_MIN_FLUSH_INTERVAL_S     3
/* Retry cadence for resolving autopid period overrides (see below). */
#define DL_OVERRIDE_RETRY_S         30
/* HTTP timeout for an Influx write. */
#define DL_HTTP_TIMEOUT_MS          8000

/* Defaults (overridable from the JSON config). */
#define DL_DEFAULT_FLUSH_BYTES      4096
#define DL_DEFAULT_MAX_DELAY_S      30
#define DL_DEFAULT_MAX_MEM_BYTES    32768
#define DL_DEFAULT_MEASUREMENT      "obd"

typedef struct
{
    char *name;             /* autopid sensor name */
    uint32_t freq_s;        /* logging frequency, 1..60 s */
    wc_timer_t timer;       /* next-due timer */
    uint32_t orig_period_ms;/* autopid period before our override */
    bool overridden;        /* true if we tightened the autopid period */
    bool resolved;          /* autopid period lookup succeeded */
} dl_pid_t;

typedef struct
{
    bool enabled;
    bool compression;

    /* InfluxDB v1 connection */
    char influx_url[256];   /* e.g. https://host:8086 (no trailing /write) */
    char influx_db[64];
    char influx_user[64];
    char influx_pass[64];
    char influx_measurement[64];
    bool insecure_tls;      /* relax CN check for self-hosted certs */

    /* Buffering */
    uint32_t flush_bytes;
    uint32_t max_delay_s;
    uint32_t max_mem_bytes;

    /* SD card (WICAN_PRO only) */
    bool sd_enabled;
    char sd_dir[128];

    dl_pid_t *pids;
    uint32_t pid_count;
} dl_config_t;

static dl_config_t s_cfg;
static char s_device_id[24] = {0};

/* Plaintext line-protocol accumulation buffer (owned by the task). */
static char *s_buf = NULL;
static size_t s_buf_len = 0;
static size_t s_buf_cap = 0;

static volatile bool s_reload = false;
static volatile bool s_running = false;
static bool s_sd_ok = false;
static bool s_sntp_started = false;

/* Start SNTP once, after Wi-Fi is up, so logged samples carry a real UTC
 * timestamp (needed for meaningful time-series and offline SD logs). */
static void maybe_start_sntp(void)
{
    if (s_sntp_started || !wifi_network_is_connected())
    {
        return;
    }
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    if (esp_netif_sntp_init(&cfg) == ESP_OK)
    {
        s_sntp_started = true;
        ESP_LOGI(TAG, "SNTP started");
    }
}

/* ----------------------------------------------------------------------- */
/* Configuration                                                            */
/* ----------------------------------------------------------------------- */

static void cfg_free(dl_config_t *cfg)
{
    if (cfg->pids)
    {
        for (uint32_t i = 0; i < cfg->pid_count; i++)
        {
            free(cfg->pids[i].name);
        }
        free(cfg->pids);
    }
    memset(cfg, 0, sizeof(*cfg));
}

static void cfg_copy_str(char *dst, size_t dst_size, const cJSON *obj, const char *key)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsString(item) && item->valuestring)
    {
        strlcpy(dst, item->valuestring, dst_size);
    }
}

static uint32_t cfg_get_uint(const cJSON *obj, const char *key, uint32_t def)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsNumber(item) && item->valuedouble >= 0)
    {
        return (uint32_t)item->valuedouble;
    }
    return def;
}

static bool cfg_get_bool(const cJSON *obj, const char *key, bool def)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsBool(item))
    {
        return cJSON_IsTrue(item);
    }
    return def;
}

/* Load the JSON config file into *cfg. Returns true if a valid file existed. */
static bool cfg_load(dl_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->flush_bytes = DL_DEFAULT_FLUSH_BYTES;
    cfg->max_delay_s = DL_DEFAULT_MAX_DELAY_S;
    cfg->max_mem_bytes = DL_DEFAULT_MAX_MEM_BYTES;
    strlcpy(cfg->influx_measurement, DL_DEFAULT_MEASUREMENT, sizeof(cfg->influx_measurement));
#if HARDWARE_VER == WICAN_PRO
    strlcpy(cfg->sd_dir, DL_SD_MOUNT_POINT"/wican", sizeof(cfg->sd_dir));
    cfg->sd_enabled = true;
#endif

    FILE *f = fopen(DATALOGGER_CONFIG_PATH, "r");
    if (!f)
    {
        ESP_LOGI(TAG, "no config file, datalogger disabled");
        return false;
    }

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 16384)
    {
        fclose(f);
        ESP_LOGW(TAG, "config file size invalid (%ld)", sz);
        return false;
    }

    char *txt = malloc((size_t)sz + 1);
    if (!txt)
    {
        fclose(f);
        return false;
    }
    size_t rd = fread(txt, 1, (size_t)sz, f);
    fclose(f);
    txt[rd] = '\0';

    cJSON *root = cJSON_Parse(txt);
    free(txt);
    if (!root)
    {
        ESP_LOGE(TAG, "config parse error");
        return false;
    }

    cfg->enabled = cfg_get_bool(root, "enabled", false);
    cfg->compression = cfg_get_bool(root, "compression", true);

    const cJSON *influx = cJSON_GetObjectItemCaseSensitive(root, "influx");
    if (cJSON_IsObject(influx))
    {
        cfg_copy_str(cfg->influx_url, sizeof(cfg->influx_url), influx, "url");
        cfg_copy_str(cfg->influx_db, sizeof(cfg->influx_db), influx, "db");
        cfg_copy_str(cfg->influx_user, sizeof(cfg->influx_user), influx, "username");
        cfg_copy_str(cfg->influx_pass, sizeof(cfg->influx_pass), influx, "password");
        cfg_copy_str(cfg->influx_measurement, sizeof(cfg->influx_measurement), influx, "measurement");
        cfg->insecure_tls = cfg_get_bool(influx, "insecure_tls", false);
    }

    const cJSON *buffer = cJSON_GetObjectItemCaseSensitive(root, "buffer");
    if (cJSON_IsObject(buffer))
    {
        cfg->flush_bytes = cfg_get_uint(buffer, "flush_bytes", DL_DEFAULT_FLUSH_BYTES);
        cfg->max_delay_s = cfg_get_uint(buffer, "max_delay_s", DL_DEFAULT_MAX_DELAY_S);
        cfg->max_mem_bytes = cfg_get_uint(buffer, "max_mem_bytes", DL_DEFAULT_MAX_MEM_BYTES);
    }
    if (cfg->flush_bytes < 512) cfg->flush_bytes = 512;
    if (cfg->max_delay_s < 1) cfg->max_delay_s = 1;
    if (cfg->max_mem_bytes < cfg->flush_bytes) cfg->max_mem_bytes = cfg->flush_bytes * 4;

    const cJSON *sd = cJSON_GetObjectItemCaseSensitive(root, "sd");
    if (cJSON_IsObject(sd))
    {
        cfg->sd_enabled = cfg_get_bool(sd, "enabled", cfg->sd_enabled);
        cfg_copy_str(cfg->sd_dir, sizeof(cfg->sd_dir), sd, "dir");
    }

    const cJSON *pids = cJSON_GetObjectItemCaseSensitive(root, "pids");
    if (cJSON_IsArray(pids))
    {
        int n = cJSON_GetArraySize(pids);
        if (n > 0)
        {
            cfg->pids = calloc((size_t)n, sizeof(dl_pid_t));
            if (cfg->pids)
            {
                for (int i = 0; i < n; i++)
                {
                    const cJSON *p = cJSON_GetArrayItem(pids, i);
                    const cJSON *name = cJSON_GetObjectItemCaseSensitive(p, "name");
                    if (!cJSON_IsString(name) || !name->valuestring || !name->valuestring[0])
                    {
                        continue;
                    }
                    uint32_t freq = cfg_get_uint(p, "freq", 5);
                    if (freq < 1) freq = 1;
                    if (freq > 60) freq = 60;

                    dl_pid_t *dst = &cfg->pids[cfg->pid_count];
                    dst->name = strdup(name->valuestring);
                    dst->freq_s = freq;
                    dst->timer = 0; /* due immediately */
                    if (dst->name)
                    {
                        cfg->pid_count++;
                    }
                }
            }
        }
    }

    cJSON_Delete(root);

    ESP_LOGI(TAG, "config loaded: enabled=%d pids=%lu compression=%d influx=%s",
             cfg->enabled, (unsigned long)cfg->pid_count, cfg->compression, cfg->influx_url);
    return true;
}

/* Ensure the CAN bus is actually polled at least as often as each PID's
 * logging frequency: tighten the autopid parameter period where needed,
 * remembering the original so it can be restored on config change.
 *
 * The autopid task can hold its mutex for a whole polling pass, so lookups
 * may time out; entries stay unresolved and this is retried from the task
 * loop until every PID is handled. Returns true when all are resolved. */
static bool apply_period_overrides(dl_config_t *cfg)
{
    bool all_resolved = true;

    for (uint32_t i = 0; i < cfg->pid_count; i++)
    {
        dl_pid_t *p = &cfg->pids[i];
        if (p->resolved)
        {
            continue;
        }

        uint32_t want_ms = p->freq_s * 1000;
        uint32_t cur_ms = 0;

        if (!autopid_get_param_period(p->name, &cur_ms))
        {
            all_resolved = false;
            continue;
        }
        p->resolved = true;

        if (cur_ms > want_ms)
        {
            if (autopid_set_param_period(p->name, want_ms))
            {
                p->orig_period_ms = cur_ms;
                p->overridden = true;
                ESP_LOGI(TAG, "PID '%s': autopid period %lu -> %lu ms",
                         p->name, (unsigned long)cur_ms, (unsigned long)want_ms);
            }
        }
    }

    return all_resolved;
}

static void restore_period_overrides(dl_config_t *cfg)
{
    for (uint32_t i = 0; i < cfg->pid_count; i++)
    {
        dl_pid_t *p = &cfg->pids[i];
        if (p->overridden)
        {
            autopid_set_param_period(p->name, p->orig_period_ms);
            p->overridden = false;
        }
    }
}

/* ----------------------------------------------------------------------- */
/* Buffer + line protocol                                                   */
/* ----------------------------------------------------------------------- */

static bool buf_reserve(size_t extra)
{
    if (s_buf_len + extra + 1 <= s_buf_cap)
    {
        return true;
    }
    size_t newcap = s_buf_cap ? s_buf_cap : 1024;
    while (newcap < s_buf_len + extra + 1)
    {
        newcap *= 2;
    }
    char *nb = realloc(s_buf, newcap);
    if (!nb)
    {
        return false;
    }
    s_buf = nb;
    s_buf_cap = newcap;
    return true;
}

/* Append a string with line-protocol escaping for measurement/tag context
 * (escapes spaces, commas and equals signs). */
static void buf_append_escaped(const char *s)
{
    for (const char *p = s; *p; p++)
    {
        if (*p == ' ' || *p == ',' || *p == '=')
        {
            s_buf[s_buf_len++] = '\\';
        }
        s_buf[s_buf_len++] = *p;
    }
}

/* Append one sample as an InfluxDB line-protocol record:
 *   <measurement>,device=<id>,sensor=<name> value=<v> [<unix_seconds>]
 */
static void buf_append_sample(const char *name, float value)
{
    /* Skip non-finite values: they are invalid line protocol and would cause
     * InfluxDB to reject the entire batch. */
    if (!isfinite(value))
    {
        return;
    }

    char numbuf[32];
    int nlen = snprintf(numbuf, sizeof(numbuf), "%.6g", value);
    if (nlen < 0) nlen = 0;

    time_t now = time(NULL);
    char tsbuf[16] = {0};
    /* Only emit a timestamp once the clock has been set (post-2021). */
    if (now > 1609459200)
    {
        snprintf(tsbuf, sizeof(tsbuf), " %lld", (long long)now);
    }

    /* Worst case size estimate (everything escaped). */
    size_t need = strlen(s_cfg.influx_measurement) * 2 + 8 +
                  strlen(s_device_id) * 2 + strlen(name) * 2 +
                  (size_t)nlen + sizeof(tsbuf) + 32;
    if (!buf_reserve(need))
    {
        ESP_LOGE(TAG, "out of memory buffering sample");
        return;
    }

    buf_append_escaped(s_cfg.influx_measurement);
    if (s_device_id[0])
    {
        /* An empty tag value is invalid line protocol and would make the
         * server reject the whole batch, so only tag when an id is set. */
        memcpy(s_buf + s_buf_len, ",device=", 8); s_buf_len += 8;
        buf_append_escaped(s_device_id);
    }
    memcpy(s_buf + s_buf_len, ",sensor=", 8); s_buf_len += 8;
    buf_append_escaped(name);
    memcpy(s_buf + s_buf_len, " value=", 7); s_buf_len += 7;
    memcpy(s_buf + s_buf_len, numbuf, (size_t)nlen); s_buf_len += (size_t)nlen;
    if (tsbuf[0])
    {
        size_t tl = strlen(tsbuf);
        memcpy(s_buf + s_buf_len, tsbuf, tl); s_buf_len += tl;
    }
    s_buf[s_buf_len++] = '\n';
    s_buf[s_buf_len] = '\0';
}

/* Drop the oldest whole lines so the buffer fits under max_mem_bytes. */
static void buf_trim_to_cap(void)
{
    if (s_buf_len <= s_cfg.max_mem_bytes)
    {
        return;
    }
    size_t drop = s_buf_len - (s_cfg.max_mem_bytes / 2);
    /* advance to the next line boundary so we never split a record */
    while (drop < s_buf_len && s_buf[drop] != '\n')
    {
        drop++;
    }
    if (drop < s_buf_len) drop++; /* skip the newline itself */
    if (drop >= s_buf_len)
    {
        s_buf_len = 0;
    }
    else
    {
        memmove(s_buf, s_buf + drop, s_buf_len - drop);
        s_buf_len -= drop;
    }
    s_buf[s_buf_len] = '\0';
    ESP_LOGW(TAG, "buffer full, dropped %u oldest bytes", (unsigned)drop);
}

/* ----------------------------------------------------------------------- */
/* gzip                                                                     */
/* ----------------------------------------------------------------------- */

static uint8_t *gzip_compress(const uint8_t *in, size_t in_len, size_t *out_len)
{
    z_stream strm;
    memset(&strm, 0, sizeof(strm));
    /* windowBits 10 (+16 for a gzip wrapper) and memLevel 2 keep deflate's
     * heap usage around 6 KB instead of the ~260 KB of the defaults, which
     * the ESP32 cannot spare. Line-protocol records repeat every few tens of
     * bytes, so a 1 KB window still compresses them well. */
    if (deflateInit2(&strm, Z_BEST_SPEED, Z_DEFLATED, 10 + 16, 2,
                     Z_DEFAULT_STRATEGY) != Z_OK)
    {
        return NULL;
    }

    size_t bound = deflateBound(&strm, in_len);
    uint8_t *out = malloc(bound);
    if (!out)
    {
        deflateEnd(&strm);
        return NULL;
    }

    strm.next_in = (Bytef *)in;
    strm.avail_in = (uInt)in_len;
    strm.next_out = out;
    strm.avail_out = (uInt)bound;

    int r = deflate(&strm, Z_FINISH);
    if (r != Z_STREAM_END)
    {
        free(out);
        deflateEnd(&strm);
        return NULL;
    }
    *out_len = strm.total_out;
    deflateEnd(&strm);
    return out;
}

/* ----------------------------------------------------------------------- */
/* InfluxDB v1 upload (HTTPS)                                                */
/* ----------------------------------------------------------------------- */

static void url_encode_append(char *dst, size_t dst_size, const char *src)
{
    static const char hex[] = "0123456789ABCDEF";
    size_t len = strlen(dst);
    for (const char *p = src; *p && len + 4 < dst_size; p++)
    {
        unsigned char c = (unsigned char)*p;
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~')
        {
            dst[len++] = (char)c;
        }
        else
        {
            dst[len++] = '%';
            dst[len++] = hex[c >> 4];
            dst[len++] = hex[c & 0x0F];
        }
    }
    dst[len] = '\0';
}

static bool influx_upload(const uint8_t *body, size_t body_len, bool gz)
{
    char url[1024];
    int n = snprintf(url, sizeof(url), "%s/write?db=", s_cfg.influx_url);
    if (n < 0 || n >= (int)sizeof(url))
    {
        ESP_LOGE(TAG, "influx url too long");
        return false;
    }
    url_encode_append(url, sizeof(url), s_cfg.influx_db);
    strlcat(url, "&precision=s", sizeof(url));
    if (s_cfg.influx_user[0])
    {
        strlcat(url, "&u=", sizeof(url));
        url_encode_append(url, sizeof(url), s_cfg.influx_user);
        strlcat(url, "&p=", sizeof(url));
        url_encode_append(url, sizeof(url), s_cfg.influx_pass);
    }

    esp_http_client_config_t hcfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = DL_HTTP_TIMEOUT_MS,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .skip_cert_common_name_check = s_cfg.insecure_tls,
    };

    esp_http_client_handle_t client = esp_http_client_init(&hcfg);
    if (!client)
    {
        return false;
    }

    esp_http_client_set_header(client, "Content-Type", "text/plain");
    if (gz)
    {
        esp_http_client_set_header(client, "Content-Encoding", "gzip");
    }
    esp_http_client_set_post_field(client, (const char *)body, body_len);

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "influx upload failed: %s", esp_err_to_name(err));
        return false;
    }
    if (status != 204 && status != 200)
    {
        ESP_LOGW(TAG, "influx upload http status %d", status);
        return false;
    }
    ESP_LOGI(TAG, "influx upload ok (%u bytes%s)", (unsigned)body_len, gz ? ", gz" : "");
    return true;
}

/* ----------------------------------------------------------------------- */
/* SD card flush (WICAN_PRO only)                                           */
/* ----------------------------------------------------------------------- */

#if HARDWARE_VER == WICAN_PRO
static sdmmc_card_t *s_card = NULL;

static bool sd_mount(void)
{
    if (s_card)
    {
        return true;
    }

    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
#ifdef SOC_SDMMC_USE_GPIO_MATRIX
    slot_config.clk = SDCARD_CLK;
    slot_config.cmd = SDCARD_CMD;
    slot_config.d0 = SDCARD_D0;
    slot_config.d1 = SDCARD_D1;
    slot_config.d2 = SDCARD_D2;
    slot_config.d3 = SDCARD_D3;
#endif
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(DL_SD_MOUNT_POINT, &host, &slot_config,
                                            &mount_config, &s_card);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "SD mount failed: %s", esp_err_to_name(ret));
        s_card = NULL;
        return false;
    }
    ESP_LOGI(TAG, "SD card mounted at %s", DL_SD_MOUNT_POINT);
    return true;
}

/* Unmount after an I/O failure so a re-inserted card gets a fresh mount. */
static void sd_recover(void)
{
    if (s_card)
    {
        esp_vfs_fat_sdcard_unmount(DL_SD_MOUNT_POINT, s_card);
        s_card = NULL;
    }
    s_sd_ok = false;
}

static bool sd_append(const uint8_t *data, size_t len, bool gz)
{
    if (!s_sd_ok && !sd_mount())
    {
        return false;
    }
    s_sd_ok = true;

    /* Ensure the target directory exists. */
    mkdir(s_cfg.sd_dir, 0775);

    /* One rolling file per day keeps individual writes large (low wear) while
     * remaining easy to manage. gzip members concatenate into a valid stream. */
    char fname[200];
    time_t now = time(NULL);
    struct tm tmv;
    localtime_r(&now, &tmv);
    if (now > 1609459200)
    {
        snprintf(fname, sizeof(fname), "%s/obd-%04d%02d%02d.lp%s",
                 s_cfg.sd_dir, tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                 gz ? ".gz" : "");
    }
    else
    {
        snprintf(fname, sizeof(fname), "%s/obd-boot.lp%s",
                 s_cfg.sd_dir, gz ? ".gz" : "");
    }

    FILE *f = fopen(fname, "ab");
    if (!f)
    {
        ESP_LOGW(TAG, "SD open %s failed", fname);
        sd_recover();
        return false;
    }

    size_t w = fwrite(data, 1, len, f);
    fclose(f);

    if (w != len)
    {
        ESP_LOGW(TAG, "SD short write (%u/%u)", (unsigned)w, (unsigned)len);
        sd_recover();
        return false;
    }
    ESP_LOGI(TAG, "SD flushed %u bytes to %s", (unsigned)len, fname);
    return true;
}
#endif /* WICAN_PRO */

/* ----------------------------------------------------------------------- */
/* Flush                                                                    */
/* ----------------------------------------------------------------------- */

/* Returns true if the buffer was successfully handed off (and cleared). */
static bool flush_buffer(void)
{
    if (s_buf_len == 0)
    {
        return true;
    }

    const uint8_t *payload = (const uint8_t *)s_buf;
    size_t payload_len = s_buf_len;
    bool gz = false;
    uint8_t *gzbuf = NULL;

    if (s_cfg.compression)
    {
        size_t gzlen = 0;
        gzbuf = gzip_compress((const uint8_t *)s_buf, s_buf_len, &gzlen);
        if (gzbuf)
        {
            payload = gzbuf;
            payload_len = gzlen;
            gz = true;
        }
    }

    bool done = false;

    if (wifi_network_is_connected() && s_cfg.influx_url[0] && s_cfg.influx_db[0])
    {
        done = influx_upload(payload, payload_len, gz);
    }

#if HARDWARE_VER == WICAN_PRO
    if (!done && s_cfg.sd_enabled)
    {
        done = sd_append(payload, payload_len, gz);
    }
#endif

    free(gzbuf);

    if (done)
    {
        s_buf_len = 0;
        if (s_buf)
        {
            s_buf[0] = '\0';
        }
    }
    else
    {
        ESP_LOGW(TAG, "no sink available (wifi=%d), retaining %u bytes",
                 wifi_network_is_connected(), (unsigned)s_buf_len);
        buf_trim_to_cap();
    }

    return done;
}

/* ----------------------------------------------------------------------- */
/* Task                                                                     */
/* ----------------------------------------------------------------------- */

static void datalogger_task(void *arg)
{
    (void)arg;

    cfg_load(&s_cfg);
    bool overrides_done = !s_cfg.enabled || apply_period_overrides(&s_cfg);
    wc_timer_t flush_timer;
    wc_timer_t attempt_timer;
    wc_timer_t override_retry_timer;
    wc_timer_set(&flush_timer, s_cfg.max_delay_s * 1000);
    wc_timer_set(&attempt_timer, DL_MIN_FLUSH_INTERVAL_S * 1000);
    wc_timer_set(&override_retry_timer, DL_OVERRIDE_RETRY_S * 1000);

    for (;;)
    {
        if (s_reload)
        {
            s_reload = false;
            /* Try to flush what we have before swapping config. */
            flush_buffer();
            restore_period_overrides(&s_cfg);
            cfg_free(&s_cfg);
            cfg_load(&s_cfg);
            overrides_done = !s_cfg.enabled || apply_period_overrides(&s_cfg);
            wc_timer_set(&flush_timer, s_cfg.max_delay_s * 1000);
        }

        s_running = s_cfg.enabled && s_cfg.pid_count > 0;

        if (!s_running)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!overrides_done && wc_timer_is_expired(&override_retry_timer))
        {
            wc_timer_set(&override_retry_timer, DL_OVERRIDE_RETRY_S * 1000);
            overrides_done = apply_period_overrides(&s_cfg);
            if (!overrides_done)
            {
                for (uint32_t i = 0; i < s_cfg.pid_count; i++)
                {
                    if (!s_cfg.pids[i].resolved)
                    {
                        ESP_LOGW(TAG, "PID '%s' not found in autopid config yet",
                                 s_cfg.pids[i].name);
                    }
                }
            }
        }

        maybe_start_sntp();

        /* Sample any PIDs whose timer is due. */
        for (uint32_t i = 0; i < s_cfg.pid_count; i++)
        {
            dl_pid_t *p = &s_cfg.pids[i];
            if (wc_timer_is_expired(&p->timer))
            {
                float v;
                if (autopid_get_value(p->name, &v))
                {
                    buf_append_sample(p->name, v);
                }
                wc_timer_set(&p->timer, p->freq_s * 1000);
            }
        }

        /* Flush when the buffer is full or the max delay has elapsed. */
        bool want_flush = (s_buf_len >= s_cfg.flush_bytes) ||
                          (s_buf_len > 0 && wc_timer_is_expired(&flush_timer));

        if (want_flush && wc_timer_is_expired(&attempt_timer))
        {
            wc_timer_set(&attempt_timer, DL_MIN_FLUSH_INTERVAL_S * 1000);
            if (flush_buffer())
            {
                wc_timer_set(&flush_timer, s_cfg.max_delay_s * 1000);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(DL_SAMPLE_TICK_MS));
    }
}

/* ----------------------------------------------------------------------- */
/* Public API                                                               */
/* ----------------------------------------------------------------------- */

void datalogger_init(const char *device_id)
{
    if (device_id)
    {
        strlcpy(s_device_id, device_id, sizeof(s_device_id));
    }

    static bool started = false;
    if (started)
    {
        datalogger_reload();
        return;
    }
    started = true;

    /* The TLS handshake for the Influx upload runs on this task's stack,
     * so it needs headroom beyond a typical worker task. */
    xTaskCreate(datalogger_task, "datalogger", 8192, NULL, 4, NULL);
    ESP_LOGI(TAG, "datalogger task started");
}

void datalogger_reload(void)
{
    s_reload = true;
}

bool datalogger_is_running(void)
{
    return s_running;
}
