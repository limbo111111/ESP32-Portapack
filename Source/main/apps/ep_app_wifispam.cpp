#include "ep_app_wifispam.hpp"
#include "pp_commands.hpp"
#include <esp_wifi.h>
#include <esp_timer.h>
#include <math.h>

static EPAppWifiSpam* wifispam_csi_instance = nullptr;

void EPAppWifiSpam::enable_csi() {
    ESP_LOGI("WIFICSI", "Enabling CSI in WiFi Spam App");
    esp_wifi_set_promiscuous(true);

    wifi_csi_config_t csi_config = {};
    csi_config.lltf_en = true;
    csi_config.htltf_en = true;
    csi_config.stbc_htltf2_en = true;
    csi_config.ltf_merge_en = true;
    csi_config.channel_filter_en = false;
    csi_config.manu_scale = false;
    csi_config.shift = 0;

    wifi_promiscuous_filter_t filter = { .filter_mask = WIFI_PROMIS_FILTER_MASK_ALL };
    esp_wifi_set_promiscuous_filter(&filter);

    esp_wifi_set_csi_config(&csi_config);
    wifispam_csi_instance = this;
    esp_wifi_set_csi_rx_cb(&EPAppWifiSpam::csi_cb, this);
    esp_wifi_set_csi(true);
}

void EPAppWifiSpam::disable_csi() {
    ESP_LOGI("WIFICSI", "Disabling CSI in WiFi Spam App");
    esp_wifi_set_csi(false);
    esp_wifi_set_csi_rx_cb(nullptr, nullptr);
    esp_wifi_set_promiscuous(false);
    wifispam_csi_instance = nullptr;
}

void EPAppWifiSpam::csi_cb(void *ctx, wifi_csi_info_t *data) {
    if (wifispam_csi_instance) {
        wifispam_csi_instance->process_csi_data(data);
    }
}

void EPAppWifiSpam::process_csi_data(wifi_csi_info_t *data) {
    if (current_mode != 4) return;
    if (data->len > 0) {
        float total_diff = 0.0f;
        int count = 0;
        int start_idx = data->first_word_invalid ? 4 : 0;
        for (int i = start_idx; i < data->len - 1; i += 2) {
            int subcarrier_idx = (i - start_idx) / 2;
            if (subcarrier_idx >= MAX_SUBCARRIERS) break;
            int8_t imaginary = data->buf[i];
            int8_t real = data->buf[i+1];
            float mag = sqrt(imaginary * imaginary + real * real);
            if (prev_amplitudes[subcarrier_idx] > 0.1f) total_diff += fabs(mag - prev_amplitudes[subcarrier_idx]);
            prev_amplitudes[subcarrier_idx] = (prev_amplitudes[subcarrier_idx] * 0.9f) + (mag * 0.1f);
            count++;
        }
        if (count > 0 && (total_diff / count) > 1.2f) {
            motion_detected = true;
            last_motion_time = esp_timer_get_time() / 1000;
            SetDisplayDirty();
        }
    }
}



void EPAppWifiSpam::randomizeBeaconSrcMac(uint8_t* beacon_raw, uint8_t macid) {
    if (macid == 0) {
        uint8_t random_byte;
        random_byte = (esp_random() % 13) + 1;
        for (int i = 0; i < 6; i++) {
            random_byte = esp_random() & 0xFF;
            beacon_raw[10 + i] = random_byte;  // Source Address (bytes 10–15)
            beacon_raw[16 + i] = random_byte;  // BSSID (bytes 16–21)
        }
    } else {
        beacon_raw[10 + 0] = beacon_raw[16 + 0] = 0xab;
        beacon_raw[10 + 1] = beacon_raw[16 + 1] = 0xba;
        beacon_raw[10 + 2] = beacon_raw[16 + 2] = 0xde;
        beacon_raw[10 + 3] = beacon_raw[16 + 3] = 0xad;
        beacon_raw[10 + 4] = beacon_raw[16 + 4] = 0xbf;
        beacon_raw[10 + 5] = beacon_raw[16 + 5] = macid;
    }
    // Ensure the MAC address is valid (set locally administered bit, clear multicast bit)
    beacon_raw[10] = (beacon_raw[10] & 0xFE) | 0x02;
    beacon_raw[16] = beacon_raw[10];  // First byte of BSSID matches Source Address
}

void EPAppWifiSpam::sendBeacon(std::string ssid, uint8_t macid) {
    randomizeBeaconSrcMac(packet, macid);
    // Build the beacon frame
    packet[37] = ssid.size();
    memcpy(&packet[38], ssid.data(), ssid.size());
    packet[50 + ssid.size()] = 3;  // set channel

    uint8_t postSSID[13] = {0x01, 0x08, 0x82, 0x84, 0x8b, 0x96, 0x24, 0x30, 0x48, 0x6c,  // supported rate
                            0x03, 0x01, 0x04 /*DSSS (Current Channel)*/};

    // Add everything that goes after the SSID
    for (int i = 0; i < 12; i++)
        packet[38 + ssid.size() + i] = postSSID[i];

    esp_wifi_80211_tx(WIFI_IF_AP, packet, 128, false);  // sizeof?!
}

void EPAppWifiSpam::Loop(uint32_t currentMillis) {
    if (current_mode == 0) return;  // Standby mode, do nothing
    if (currentMillis - lastBeaconTime > 10) {
        if (current_mode == 1) {  // Random characters mode
            std::string ssid;
            uint8_t len = esp_random() % 31 + 1;
            for (int i = 0; i < len; i++) {
                ssid += charset[esp_random() % (sizeof(charset) - 1)];
            }
            sendBeacon(ssid, 0);  // Use macid 0 for random mode
        }
        if (current_mode == 2) {  // Rick Roll mode
            sendBeacon(std::string(rick_ssids[ricknum]), ricknum % 8 + 1);
            ricknum = (ricknum + 1) % 8;
        }
        if (current_mode == 4) {
            // motion_detected logic inside Loop
            if (motion_detected && (currentMillis - last_motion_time > 2000)) {
                motion_detected = false;
                SetDisplayDirty();
            }
        }
        if (current_mode == 3) {  // Emoji spam mode
            std::string ssid = "";
            uint8_t len = esp_random() % 5 + 1;
            for (int i = 0; i < len; i++) {
                uint8_t emoji_type = esp_random() % 4;  // Choose from 4 different emoji categories
                switch (emoji_type) {                   // a lot may fail, maybe need a better solution, like a list of emojis
                    case 0:
                        // Add a random emoji from U+1F600 (😀) to U+1F636 (😶)
                        {
                            uint8_t emoji_offset = esp_random() % (0xB7 - 0x80);  // 0xB6 - 0x80 + 1 = 0x37
                            char emoji[5] = {0};
                            emoji[0] = '\xF0';
                            emoji[1] = '\x9F';
                            emoji[2] = '\x98';
                            emoji[3] = static_cast<char>(0x80 + emoji_offset);
                            ssid += emoji;
                        }
                        break;
                    case 1: {
                        // Add a random emoji from U+1F601 (😁) to U+1F64F (🙏)
                        uint32_t emoji_start = 0x1F601;
                        uint32_t emoji_end = 0x1F64F;
                        uint32_t emoji_code = emoji_start + (esp_random() % (emoji_end - emoji_start + 1));
                        char emoji[5] = {0};
                        // Encode as UTF-8
                        emoji[0] = 0xF0;
                        emoji[1] = 0x9F;
                        emoji[2] = static_cast<char>(0x80 | ((emoji_code >> 6) & 0x3F));
                        emoji[3] = static_cast<char>(0x80 | (emoji_code & 0x3F));
                        // For code points above U+1F600, adjust the second byte
                        if (emoji_code >= 0x1F600) {
                            emoji[1] = 0x9F;
                            emoji[2] = static_cast<char>(0x80 | ((emoji_code >> 6) & 0x3F));
                            emoji[3] = static_cast<char>(0x80 | (emoji_code & 0x3F));
                        }
                        ssid += emoji;
                    } break;
                    case 2: {
                        // Add a random emoji from U+1F680 (🚀) to U+1F6C0 (🛀)
                        uint32_t emoji_start = 0x1F680;
                        uint32_t emoji_end = 0x1F6C0;
                        uint32_t emoji_code = emoji_start + (esp_random() % (emoji_end - emoji_start + 1));
                        char emoji[5] = {0};
                        // Encode as UTF-8
                        emoji[0] = 0xF0;
                        emoji[1] = 0x9F;
                        emoji[2] = static_cast<char>(0x80 | ((emoji_code >> 6) & 0x3F));
                        emoji[3] = static_cast<char>(0x80 | (emoji_code & 0x3F));
                        ssid += emoji;
                    } break;
                    case 3: {
                        // Add a random emoji from U+1F440 (👀) to U+1F54B (🕋)
                        uint32_t emoji_start = 0x1F440;
                        uint32_t emoji_end = 0x1F54B;
                        uint32_t emoji_code = emoji_start + (esp_random() % (emoji_end - emoji_start + 1));
                        char emoji[5] = {0};
                        emoji[0] = 0xF0;
                        emoji[1] = 0x9F;
                        emoji[2] = static_cast<char>(0x80 | ((emoji_code >> 6) & 0x3F));
                        emoji[3] = static_cast<char>(0x80 | (emoji_code & 0x3F));
                        ssid += emoji;
                    } break;
                }
            }
            sendBeacon(ssid, 0);  // Use macid 0 for emoji mode
            lastBeaconTime = currentMillis;
        }
    }
}

void EPAppWifiSpam::OnDisplayRequest(DisplayGeneric* display) {
    display->showTitle("WiFi Spam App");
    if (current_mode == 0) {
        display->showMainText("Mode: Standby");
    } else if (current_mode == 1) {
        display->showMainText("Mode: Random Chars");
    } else if (current_mode == 2) {
        display->showMainText("Mode: Rick Roll");
    } else if (current_mode == 3) {
        display->showMainText("Mode: Emoji Spam");
    } else if (current_mode == 4) {
        if (motion_detected) {
            display->showMainText("MOTION DETECTED!\n!!! Someone is moving !!!");
        } else {
            display->showMainText("Mode: CSI Radar Scan...\n(Quiet)");
        }
    } else {
        display->showMainText("Unknown Mode");
    }
}

bool EPAppWifiSpam::OnWebData(std::string& data) {
    if (data.compare(APP_1_PRE_STR "0\r\n") == 0) {
        current_mode = 0;
        disable_csi();
        SetDisplayDirty();
        return true;
    }
    if (data.compare(APP_1_PRE_STR "1\r\n") == 0) {
        current_mode = 1;
        disable_csi();
        SetDisplayDirty();
        return true;
    }
    if (data.compare(APP_1_PRE_STR "2\r\n") == 0) {
        current_mode = 2;
        disable_csi();
        SetDisplayDirty();
        return true;
    }
    if (data.compare(APP_1_PRE_STR "3\r\n") == 0) {
        current_mode = 3;
        disable_csi();
        SetDisplayDirty();
        return true;
    }
    if (data.compare(APP_1_PRE_STR "4\r\n") == 0) {
        current_mode = 4;
        enable_csi();
        SetDisplayDirty();
        return true;
    }
    return false;
}

bool EPAppWifiSpam::OnPPData(uint16_t command, std::vector<uint8_t>& data) {
    if (command == PPCMD_APPMGR_APPCMD) {
        if (data.size() >= 2) {
            uint16_t new_mode = *reinterpret_cast<uint16_t*>(data.data());
            if (new_mode <= 4) {
                current_mode = static_cast<uint8_t>(new_mode);
                if (current_mode == 4) enable_csi();
                else disable_csi();
                SetDisplayDirty();
            }
            return true;
        }
    }
    return false;
}

bool EPAppWifiSpam::OnPPReqData(uint16_t command, std::vector<uint8_t>& data) {
    if (command == PPCMD_APPMGR_APPCMD) {
        data.resize(2);
        *reinterpret_cast<uint16_t*>(data.data()) = current_mode;
        return true;
    }
    return false;
}