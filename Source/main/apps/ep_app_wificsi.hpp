#ifndef EP_APP_WIFICSI_HPP
#define EP_APP_WIFICSI_HPP

#include "ep_app.hpp"
#include <esp_wifi.h>

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
    uint8_t current_mode = 1;  // 0 = standby, 1 = active
    uint32_t last_motion_time = 0;
    bool motion_detected = false;

    // For variance calculation per subcarrier
    static const int MAX_SUBCARRIERS = 128;
    float prev_amplitudes[MAX_SUBCARRIERS] = {0.0f};

    void enable_csi();
    void disable_csi();

    void process_csi_data(wifi_csi_info_t *data);
};

#endif  // EP_APP_WIFICSI_HPP
