#ifndef EP_APP_WIFICSI_HPP
#define EP_APP_WIFICSI_HPP

#include "ep_app.hpp"
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <esp_http_server.h>
#include <vector>
#include <string>

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

    // Called by the framework to register HTTP handlers
    void RegisterHttpHandlers(httpd_handle_t server);

    static void csi_cb(void *ctx, wifi_csi_info_t *data);

   private:
    // -----------------------------------------------------------------------
    // Tunable thresholds (can be updated at runtime via web UI)
    // -----------------------------------------------------------------------
    float    motion_threshold    = 1.2f;
    // Alpha deliberately low so the IIR tracks fast enough for the diff to
    // be non-zero. 0.3–0.5 gives a good balance between noise suppression
    // and responsiveness. Users can raise it in the web UI.
    float    lpf_alpha           = 0.4f;

    static constexpr uint32_t MOTION_TIMEOUT_MS       = 2000;
    static constexpr uint32_t CALIB_REFRESH_MS        = 500;
    static constexpr uint32_t CALIB_DURATION_MS       = 10000;
    static constexpr uint32_t WS_PUSH_INTERVAL_MS     = 200;
    static constexpr int      MAX_SUBCARRIERS          = 128;

    // How many frames a new MAC slot must warm up before we count its diff.
    // During warm-up the IIR baseline is being established.
    static constexpr uint8_t  WARMUP_FRAMES            = 5;

    // -----------------------------------------------------------------------
    // Per-MAC state — tracks each unique transmitter independently so that
    // frames from different APs/STAs are not mixed together, which would
    // produce false diffs when different MACs are interleaved.
    // -----------------------------------------------------------------------
    static constexpr int MAX_MAC_SLOTS = 8;
    struct MacSlot {
        bool     active              = false;
        uint8_t  mac[6]             = {};
        uint8_t  warmup             = WARMUP_FRAMES;
        uint32_t last_seen          = 0;
        float    prev_amp[MAX_SUBCARRIERS] = {};
    };
    MacSlot mac_slots[MAX_MAC_SLOTS];

    // -----------------------------------------------------------------------
    // Spinlock — protects all fields shared with csi_cb (WiFi task)
    // -----------------------------------------------------------------------
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    uint8_t  current_mode    = 0;
    bool     motion_detected = false;
    uint32_t last_motion_time = 0;

    bool     is_calibrating         = false;
    uint32_t calibration_start_time = 0;
    uint32_t calib_seconds_left     = CALIB_DURATION_MS / 1000;
    uint32_t last_calib_refresh     = 0;

    // CSI history ring buffer
    static constexpr int CSI_HISTORY_LEN = 64;
    float    csi_history[CSI_HISTORY_LEN] = {};
    uint16_t history_head = 0;
    uint32_t frame_count  = 0;
    float    last_avg_diff = 0.0f;

    // WebSocket client list
    static constexpr int MAX_WS_CLIENTS = 4;
    int ws_fds[MAX_WS_CLIENTS];
    int ws_fd_count = 0;
    httpd_handle_t http_server = nullptr;
    uint32_t last_ws_push_ms  = 0;

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------
    void enable_csi(uint32_t currentMillis);
    void disable_csi();
    void process_csi_data(wifi_csi_info_t *data);

    void push_ws_update();
    std::string build_json_state();

    void ws_add_client(int fd);
    void ws_remove_client(int fd);

    static esp_err_t http_get_root(httpd_req_t *req);
    static esp_err_t http_get_state(httpd_req_t *req);
    static esp_err_t http_post_cmd(httpd_req_t *req);
    static esp_err_t http_ws_handler(httpd_req_t *req);
};

#endif  // EP_APP_WIFICSI_HPP
