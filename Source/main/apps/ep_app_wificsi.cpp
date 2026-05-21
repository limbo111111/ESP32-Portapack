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
}

void EPAppWifiCsi::disable_csi() {
    ESP_LOGI(TAG, "Disabling CSI");
    esp_wifi_set_csi(false);
    esp_wifi_set_csi_rx_cb(nullptr, nullptr);
    esp_wifi_set_promiscuous(false);
}

void EPAppWifiCsi::csi_cb(void *ctx, wifi_csi_info_t *data) {
    if (csi_app_instance) {
        csi_app_instance->process_csi_data(data);
    }
}

void EPAppWifiCsi::process_csi_data(wifi_csi_info_t *data) {
    if (current_mode == 0) return;

    if (data->len > 0) {
        // Calculate average amplitude of CSI data
        float avg_amp = 0.0f;
        int count = 0;

        // Skip first word if invalid
        int start_idx = 0;
        if (data->first_word_invalid) {
            start_idx = 4;
        }

        // CSI data is usually stored as pairs of (imaginary, real) for each subcarrier
        // We'll calculate a simple average magnitude
        for (int i = start_idx; i < data->len - 1; i += 2) {
            int8_t imaginary = data->buf[i];
            int8_t real = data->buf[i+1];
            float mag = sqrt(imaginary * imaginary + real * real);
            avg_amp += mag;
            count++;
        }

        if (count > 0) {
            avg_amp /= count;

            // Calculate absolute difference from previous average amplitude
            float diff = fabs(avg_amp - prev_amplitude);

            // Threshold for motion detection. This might need tuning based on environment.
            if (diff > 2.0f && prev_amplitude > 0.1f) {
                motion_detected = true;
                last_motion_time = esp_timer_get_time() / 1000;
                SetDisplayDirty();
            }

            // Simple low pass filter for background
            prev_amplitude = (prev_amplitude * 0.9f) + (avg_amp * 0.1f);
        }
    }
}

void EPAppWifiCsi::Loop(uint32_t currentMillis) {
    // Clear motion status after 2 seconds
    if (motion_detected && (currentMillis - last_motion_time > 2000)) {
        motion_detected = false;
        SetDisplayDirty();
    }
}

void EPAppWifiCsi::OnDisplayRequest(DisplayGeneric* display) {
    display->showTitle("WiFi Motion (CSI)");
    if (current_mode == 0) {
        display->showMainText("Mode: Standby");
    } else {
        if (motion_detected) {
            display->showMainText("MOTION DETECTED!\n!!! Someone is moving !!!");
        } else {
            display->showMainText("Scanning for motion...\n(Quiet)");
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
