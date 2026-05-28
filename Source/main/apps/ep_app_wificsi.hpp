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

// History depth for the web graph (ring buffer)
#define CSI_HISTORY_LEN 64

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
    float    motion_threshold    = 1.2f;    // avg-diff that counts as motion
    float    lpf_alpha           = 0.97f;   // IIR smoothing (0=fast, 1=no smooth)

    static constexpr uint32_t MOTION_TIMEOUT_MS       = 2000;
    static constexpr uint32_t CALIB_REFRESH_MS        = 500;
    static constexpr uint32_t CALIB_DURATION_MS       = 10000;
    static constexpr uint32_t WS_PUSH_INTERVAL_MS     = 200;  // max WebSocket push rate
    static constexpr int      MAX_SUBCARRIERS          = 128;

    // -----------------------------------------------------------------------
    // Spinlock — protects all fields shared with csi_cb (WiFi task)
    // -----------------------------------------------------------------------
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    uint8_t  current_mode    = 0;   // 0=standby, 1=motion
    bool     motion_detected = false;
    uint32_t last_motion_time = 0;

    bool     is_calibrating         = false;
    uint32_t calibration_start_time = 0;
    uint32_t calib_seconds_left     = CALIB_DURATION_MS / 1000;
    uint32_t last_calib_refresh     = 0;

    float prev_amplitudes[MAX_SUBCARRIERS] = {};

    // -----------------------------------------------------------------------
    // CSI history ring buffer — written by csi_cb, read by HTTP handler
    // -----------------------------------------------------------------------
    float    csi_history[CSI_HISTORY_LEN] = {};   // avg_diff per frame
    uint16_t history_head = 0;                     // next write index
    uint32_t frame_count  = 0;                     // total frames received

    // Last avg_diff (for live display)
    float    last_avg_diff = 0.0f;

    // -----------------------------------------------------------------------
    // WebSocket descriptor list (simple; supports a small number of clients)
    // -----------------------------------------------------------------------
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

    void push_ws_update();   // sends JSON state to all connected WS clients
    std::string build_json_state();

    void ws_add_client(int fd);
    void ws_remove_client(int fd);

    // HTTP handler statics (httpd_uri_handler_t needs static/free functions)
    static esp_err_t http_get_root(httpd_req_t *req);
    static esp_err_t http_get_state(httpd_req_t *req);
    static esp_err_t http_post_cmd(httpd_req_t *req);
    static esp_err_t http_ws_handler(httpd_req_t *req);
};

#endif  // EP_APP_WIFICSI_HPP
