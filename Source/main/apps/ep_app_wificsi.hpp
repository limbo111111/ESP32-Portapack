#ifndef EP_APP_WIFICSI_HPP
#define EP_APP_WIFICSI_HPP

#include "ep_app.hpp"
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#define APP_CSI_PRE_STR "#$$#$$$4"

class EPAppWifiCsi : public EPApp {
   public:
    EPAppWifiCsi();
    ~EPAppWifiCsi();

    bool OnPPData(uint16_t command, std::vector<uint8_t>& data) override;
    bool OnPPReqData(uint16_t command, std::vector<uint8_t>& data) override;

    bool OnWebData(std::string& data) override;

    void OnDisplayRequest(DisplayGeneric* display) override;
    void Loop(uint32_t currentMillis) override;

    static void csi_cb(void *ctx, wifi_csi_info_t *data);

   private:
    // Thresholds
    static constexpr float    MOTION_THRESHOLD    = 1.2f;
    static constexpr float    LPF_ALPHA           = 0.99f;
    static constexpr uint32_t MOTION_TIMEOUT_MS   = 2000;
    static constexpr uint32_t CALIB_REFRESH_MS    = 1000;

    static constexpr int MAX_SUBCARRIERS = 128;

    // Spinlock protecting all fields written by csi_cb / read by Loop+Display
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    // Bug fix: default was 1 (active) — must be 0 (standby) so CSI is not
    // running before the user selects a mode.
    uint8_t  current_mode       = 0;   // 0=standby, 1=motion

    // These are written inside the spinlock in csi_cb and read under the
    // spinlock in Loop/Display — no separate access outside the lock.
    bool     motion_detected    = false;
    uint32_t last_motion_time   = 0;

    bool     is_calibrating         = false;
    uint32_t calibration_start_time = 0;   // in currentMillis units
    static const uint32_t CALIBRATION_DURATION_MS = 10000;

    // Cached display value updated each Loop tick (avoids live timer call in OnDisplayRequest)
    uint32_t calib_seconds_left = CALIBRATION_DURATION_MS / 1000;

    uint32_t last_calib_refresh = 0;

    float prev_amplitudes[MAX_SUBCARRIERS] = {};

    void enable_csi(uint32_t currentMillis);
    void disable_csi();

    void process_csi_data(wifi_csi_info_t *data);
};

#endif  // EP_APP_WIFICSI_HPP
