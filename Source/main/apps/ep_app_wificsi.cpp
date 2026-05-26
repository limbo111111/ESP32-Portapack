#include "ep_app_wificsi.hpp"
#include "pp_commands.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char* TAG = "WIFICSI";
static EPAppWifiCsi* csi_app_instance = nullptr;

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

EPAppWifiCsi::EPAppWifiCsi() {
    csi_app_instance = this;
    // Bug fix: do NOT call enable_csi() here.
    // current_mode defaults to 0 (standby); CSI starts only when the user
    // explicitly selects mode 1 via OnWebData / OnPPData.
}

EPAppWifiCsi::~EPAppWifiCsi() {
    disable_csi();
    csi_app_instance = nullptr;
}

// ---------------------------------------------------------------------------
// enable_csi / disable_csi
// ---------------------------------------------------------------------------

void EPAppWifiCsi::enable_csi(uint32_t currentMillis) {
    ESP_LOGI(TAG, "Enabling CSI");

    // Reset state before activating the callback so the callback never sees
    // a partially-initialised prev_amplitudes array.
    esp_wifi_set_csi(false);
    esp_wifi_set_csi_rx_cb(nullptr, nullptr);

    // Safe to write without lock: callback is disabled above.
    is_calibrating         = true;
    calibration_start_time = currentMillis;
    last_calib_refresh     = currentMillis;
    calib_seconds_left     = CALIBRATION_DURATION_MS / 1000;
    motion_detected        = false;
    last_motion_time       = 0;
    for (int i = 0; i < MAX_SUBCARRIERS; i++) {
        prev_amplitudes[i] = 0.0f;
    }

    esp_wifi_set_promiscuous(true);

    wifi_promiscuous_filter_t filter = { .filter_mask = WIFI_PROMIS_FILTER_MASK_ALL };
    esp_wifi_set_promiscuous_filter(&filter);

    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = false,
        .manu_scale        = false,
        .shift             = 0,
    };
    esp_wifi_set_csi_config(&csi_config);

    // Register callback and enable AFTER state is clean.
    esp_wifi_set_csi_rx_cb(&EPAppWifiCsi::csi_cb, this);
    esp_wifi_set_csi(true);

    SetDisplayDirty();
}

void EPAppWifiCsi::disable_csi() {
    ESP_LOGI(TAG, "Disabling CSI");
    esp_wifi_set_csi(false);
    esp_wifi_set_csi_rx_cb(nullptr, nullptr);
    esp_wifi_set_promiscuous(false);
    // Safe to write without lock: callback is disabled above.
    is_calibrating  = false;
    motion_detected = false;
}

// ---------------------------------------------------------------------------
// CSI callback — runs in WiFi task context (different core possible)
// ---------------------------------------------------------------------------

void EPAppWifiCsi::csi_cb(void *ctx, wifi_csi_info_t *data) {
    if (csi_app_instance) {
        csi_app_instance->process_csi_data(data);
    }
}

void EPAppWifiCsi::process_csi_data(wifi_csi_info_t *data) {
    if (current_mode == 0) return;
    if (data->buf == nullptr || data->len <= 0) return;

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);

    // Skip first I/Q pair if invalid (= 2 bytes, not 4).
    int start_idx = data->first_word_invalid ? 2 : 0;

    float total_diff = 0.0f;
    int   count      = 0;

    for (int i = start_idx; i < data->len - 1; i += 2) {
        int subcarrier_idx = (i - start_idx) / 2;
        if (subcarrier_idx >= MAX_SUBCARRIERS) break;

        float im  = (float)(int8_t)data->buf[i];
        float re  = (float)(int8_t)data->buf[i + 1];
        float mag = sqrtf(im * im + re * re);

        if (prev_amplitudes[subcarrier_idx] > 0.1f) {
            total_diff += fabsf(mag - prev_amplitudes[subcarrier_idx]);
        }

        prev_amplitudes[subcarrier_idx] =
            (prev_amplitudes[subcarrier_idx] * LPF_ALPHA) + (mag * (1.0f - LPF_ALPHA));
        count++;
    }

    if (count == 0 || is_calibrating) return;

    float avg_diff = total_diff / (float)count;
    ESP_LOGD(TAG, "avg_diff: %.4f", avg_diff);

    // Bug fix: last_motion_time must be written inside the spinlock so Loop
    // always reads a consistent pair of (motion_detected, last_motion_time).
    portENTER_CRITICAL(&mux);
    if (avg_diff > MOTION_THRESHOLD) {
        motion_detected  = true;
        last_motion_time = now;
    }
    portEXIT_CRITICAL(&mux);

    SetDisplayDirty();
}

// ---------------------------------------------------------------------------
// Loop — runs in App task context
// ---------------------------------------------------------------------------

void EPAppWifiCsi::Loop(uint32_t currentMillis) {
    bool dirty = false;

    if (is_calibrating) {
        uint32_t elapsed = currentMillis - calibration_start_time;

        if (elapsed >= CALIBRATION_DURATION_MS) {
            is_calibrating     = false;
            calib_seconds_left = 0;
            ESP_LOGI(TAG, "Calibration complete");
            dirty = true;
        } else if (currentMillis - last_calib_refresh >= CALIB_REFRESH_MS) {
            last_calib_refresh = currentMillis;
            uint32_t remaining = CALIBRATION_DURATION_MS - elapsed;
            calib_seconds_left = remaining / 1000;
            dirty = true;
        }
    }

    // Bug fix: read motion_detected AND last_motion_time together under the
    // spinlock — they are written as a pair in process_csi_data.
    portENTER_CRITICAL(&mux);
    bool     snap_motion      = motion_detected;
    uint32_t snap_motion_time = last_motion_time;
    portEXIT_CRITICAL(&mux);

    if (snap_motion && (currentMillis - snap_motion_time > MOTION_TIMEOUT_MS)) {
        portENTER_CRITICAL(&mux);
        motion_detected = false;
        portEXIT_CRITICAL(&mux);
        dirty = true;
    }

    if (dirty) {
        SetDisplayDirty();
    }
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

void EPAppWifiCsi::OnDisplayRequest(DisplayGeneric* display) {
    if (current_mode == 0) {
        display->showTitle("WiFi CSI");
        display->showMainText("Mode: Standby");
        return;
    }

    display->showTitle("WiFi CSI Motion");

    if (is_calibrating) {
        char buf[64];
        snprintf(buf, sizeof(buf),
                 "Calibrating...\nPlease stand still.\n%u seconds left.",
                 (unsigned)calib_seconds_left);
        display->showMainText(buf);
        return;
    }

    portENTER_CRITICAL(&mux);
    bool m = motion_detected;
    portEXIT_CRITICAL(&mux);

    if (m) {
        display->showMainText("MOTION DETECTED!\n!!! Someone is moving !!!");
    } else {
        display->showMainText("Scanning for motion...\n(Quiet)");
    }
}

// ---------------------------------------------------------------------------
// Protocol handlers
// ---------------------------------------------------------------------------

bool EPAppWifiCsi::OnWebData(std::string& data) {
    if (data.compare(APP_CSI_PRE_STR "0\r\n") == 0) {
        current_mode = 0;
        disable_csi();
        SetDisplayDirty();
        return true;
    }
    if (data.compare(APP_CSI_PRE_STR "1\r\n") == 0) {
        current_mode = 1;
        enable_csi((uint32_t)(esp_timer_get_time() / 1000ULL));
        SetDisplayDirty();
        return true;
    }
    // Mode 2 (breathing) removed — not supported.
    return false;
}

bool EPAppWifiCsi::OnPPData(uint16_t command, std::vector<uint8_t>& data) {
    if (command == PPCMD_APPMGR_APPCMD) {
        if (data.size() >= 2) {
            uint16_t new_mode = *reinterpret_cast<uint16_t*>(data.data());
            // Bug fix: clamp to 1 (breathing mode 2 removed).
            if (new_mode <= 1) {
                current_mode = static_cast<uint8_t>(new_mode);
                if (current_mode == 0) {
                    disable_csi();
                } else {
                    enable_csi((uint32_t)(esp_timer_get_time() / 1000ULL));
                }
                SetDisplayDirty();
            }
            return true;
        }
    }
    return false;
}

bool EPAppWifiCsi::OnPPReqData(uint16_t command, std::vector<uint8_t>& data) {
    if (command == PPCMD_APPMGR_APPCMD) {
        // 3 bytes: [mode (uint16_t)] [motion_detected]
        data.resize(3);
        *reinterpret_cast<uint16_t*>(data.data()) = current_mode;

        portENTER_CRITICAL(&mux);
        data[2] = motion_detected ? 1 : 0;
        portEXIT_CRITICAL(&mux);

        return true;
    }
    return false;
}
