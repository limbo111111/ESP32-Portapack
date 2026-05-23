#include "ep_app_wificsi.hpp"
#include "pp_commands.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char* TAG = "WIFICSI";
static EPAppWifiCsi* csi_app_instance = nullptr;

EPAppWifiCsi::EPAppWifiCsi() {
    csi_app_instance = this;
    enable_csi();
}

EPAppWifiCsi::~EPAppWifiCsi() {
    disable_csi();
    csi_app_instance = nullptr;
}

void EPAppWifiCsi::enable_csi() {
    ESP_LOGI(TAG, "Enabling CSI");

    // Reset calibration state
    is_calibrating = true;
    calibration_start_time = esp_timer_get_time() / 1000;
    for(int i = 0; i < MAX_SUBCARRIERS; i++) {
        prev_amplitudes[i] = 0.0f;
    }
    motion_detected = false;
    breathing_detected = false;

    // For CSI to work correctly and receive packets, we need promiscuous mode
    esp_wifi_set_promiscuous(true);

    wifi_csi_config_t csi_config = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = 0,
    };

    wifi_promiscuous_filter_t filter = { .filter_mask = WIFI_PROMIS_FILTER_MASK_ALL };
    esp_wifi_set_promiscuous_filter(&filter);

    esp_wifi_set_csi_config(&csi_config);
    esp_wifi_set_csi_rx_cb(&EPAppWifiCsi::csi_cb, this);
    esp_wifi_set_csi(true);

    SetDisplayDirty();
}

void EPAppWifiCsi::disable_csi() {
    ESP_LOGI(TAG, "Disabling CSI");
    esp_wifi_set_csi(false);
    esp_wifi_set_csi_rx_cb(nullptr, nullptr);
    esp_wifi_set_promiscuous(false);
    is_calibrating = false;
}

void EPAppWifiCsi::csi_cb(void *ctx, wifi_csi_info_t *data) {
    if (csi_app_instance) {
        csi_app_instance->process_csi_data(data);
    }
}

void EPAppWifiCsi::process_csi_data(wifi_csi_info_t *data) {
    if (current_mode == 0) return;

    uint32_t now = esp_timer_get_time() / 1000;

    if (data->len > 0) {
        // Calculate the difference in amplitude per subcarrier
        float total_diff = 0.0f;
        int count = 0;

        // Skip first word if invalid
        int start_idx = 0;
        if (data->first_word_invalid) {
            start_idx = 4;
        }

        // CSI data is usually stored as pairs of (imaginary, real) for each subcarrier
        for (int i = start_idx; i < data->len - 1; i += 2) {
            int subcarrier_idx = (i - start_idx) / 2;
            if (subcarrier_idx >= MAX_SUBCARRIERS) {
                break;
            }

            int8_t imaginary = data->buf[i];
            int8_t real = data->buf[i+1];
            float mag = sqrt(imaginary * imaginary + real * real);

            // Calculate absolute difference from this subcarrier's historical amplitude
            if (prev_amplitudes[subcarrier_idx] > 0.1f) {
                total_diff += fabs(mag - prev_amplitudes[subcarrier_idx]);
            }

            // Simple low pass filter for this subcarrier (acts as a slower baseline)
            prev_amplitudes[subcarrier_idx] = (prev_amplitudes[subcarrier_idx] * 0.99f) + (mag * 0.01f);
            count++;
        }

        if (count > 0 && !is_calibrating) {
            float avg_diff = total_diff / count;

            ESP_LOGD(TAG, "avg_diff: %f", avg_diff);

            if (current_mode == 1) { // Motion Mode
                if (avg_diff > 1.2f) {
                    motion_detected = true;
                    last_motion_time = now;
                    SetDisplayDirty();
                }
            } else if (current_mode == 2) { // Breathing Mode
                if (avg_diff > 1.2f) {
                    motion_detected = true; // Still track large motion to warn user
                    last_motion_time = now;
                    SetDisplayDirty();
                } else if (avg_diff > 0.3f && avg_diff <= 1.2f) { // Lower threshold for micro-motions/breathing
                    breathing_detected = true;
                    last_breathing_time = now;
                    SetDisplayDirty();
                }
            }
        }
    }
}

void EPAppWifiCsi::Loop(uint32_t currentMillis) {
    bool dirty = false;

    if (is_calibrating) {
        if (currentMillis - calibration_start_time > CALIBRATION_DURATION_MS) {
            is_calibrating = false;
            ESP_LOGI(TAG, "Calibration complete");
            dirty = true;
        } else {
            // Force refresh every ~1 second during calibration to update countdown
            static uint32_t last_calib_refresh = 0;
            if (currentMillis - last_calib_refresh > 1000) {
                last_calib_refresh = currentMillis;
                dirty = true;
            }
        }
    }

    // Clear motion status after 2 seconds
    if (motion_detected && (currentMillis - last_motion_time > 2000)) {
        motion_detected = false;
        dirty = true;
    }

    // Clear breathing status after 2 seconds
    if (breathing_detected && (currentMillis - last_breathing_time > 2000)) {
        breathing_detected = false;
        dirty = true;
    }

    if (dirty) {
        SetDisplayDirty();
    }
}

void EPAppWifiCsi::OnDisplayRequest(DisplayGeneric* display) {
    if (current_mode == 2) {
        display->showTitle("WiFi CSI Breathing");
    } else {
        display->showTitle("WiFi CSI Motion");
    }

    if (current_mode == 0) {
        display->showMainText("Mode: Standby");
    } else if (is_calibrating) {
        uint32_t now = esp_timer_get_time() / 1000;
        int32_t elapsed = now - calibration_start_time;
        int seconds = (CALIBRATION_DURATION_MS - elapsed) / 1000 + 1;
        if (seconds < 0) seconds = 0;

        char buf[64];
        snprintf(buf, sizeof(buf), "Calibrating...\nPlease stand still.\n%d seconds left.", seconds);
        display->showMainText(buf);
    } else {
        if (current_mode == 1) { // Motion Mode
            if (motion_detected) {
                display->showMainText("MOTION DETECTED!\n!!! Someone is moving !!!");
            } else {
                display->showMainText("Scanning for motion...\n(Quiet)");
            }
        } else if (current_mode == 2) { // Breathing Mode
            if (motion_detected) {
                display->showMainText("TOO MUCH MOTION!\nPlease sit still to\ndetect breathing.");
            } else if (breathing_detected) {
                display->showMainText("Breathing / Micro-motion\ndetected.");
            } else {
                display->showMainText("Scanning for breathing...\n(Quiet)");
            }
        }
    }
}

bool EPAppWifiCsi::OnWebData(std::string& data) {
    if (data.compare(APP_CSI_PRE_STR "0\r\n") == 0) {
        current_mode = 0;
        disable_csi();
        SetDisplayDirty();
        return true;
    }
    if (data.compare(APP_CSI_PRE_STR "1\r\n") == 0) {
        current_mode = 1;
        enable_csi();
        SetDisplayDirty();
        return true;
    }
    if (data.compare(APP_CSI_PRE_STR "2\r\n") == 0) {
        current_mode = 2;
        enable_csi();
        SetDisplayDirty();
        return true;
    }
    return false;
}

bool EPAppWifiCsi::OnPPData(uint16_t command, std::vector<uint8_t>& data) {
    if (command == PPCMD_APPMGR_APPCMD) {
        if (data.size() >= 2) {
            uint16_t new_mode = *reinterpret_cast<uint16_t*>(data.data());
            if (new_mode <= 1) {
                current_mode = static_cast<uint8_t>(new_mode);
                if (current_mode == 1) {
                    enable_csi();
                } else {
                    disable_csi();
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
        data.resize(3);
        *reinterpret_cast<uint16_t*>(data.data()) = current_mode;
        data[2] = motion_detected ? 1 : 0;
        return true;
    }
    return false;
}
