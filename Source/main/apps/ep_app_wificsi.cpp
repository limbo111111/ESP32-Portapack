#include "ep_app_wificsi.hpp"
#include "pp_commands.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "wifim.h"
#include "lwip/netif.h"
#include "lwip/sockets.h"
#include "lwip/icmp.h"
#include "lwip/inet_chksum.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

static const char* TAG = "WIFICSI";
static EPAppWifiCsi* csi_app_instance = nullptr;

// ---------------------------------------------------------------------------
// Inline HTML/JS web interface (stored in flash via string literal)
// ---------------------------------------------------------------------------
// Served as the root page. Uses EventSource (SSE) for polling-free updates.
// The JS connects to /ws (WebSocket) when available, falls back to /state poll.
// ---------------------------------------------------------------------------

static const char* WEB_PAGE = R"RAWHTML(<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 CSI Radar</title>
<style>
  :root{--bg:#0d1117;--card:#161b22;--border:#30363d;--accent:#58a6ff;
        --green:#3fb950;--red:#f85149;--yellow:#d29922;--text:#e6edf3;--muted:#8b949e}
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:var(--text);font-family:system-ui,sans-serif;min-height:100vh;padding:1rem}
  h1{font-size:1.4rem;color:var(--accent);letter-spacing:.05em;margin-bottom:1rem}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:1rem;max-width:900px;margin:0 auto}
  @media(max-width:600px){.grid{grid-template-columns:1fr}}
  .card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:1rem}
  .card h2{font-size:.85rem;text-transform:uppercase;letter-spacing:.08em;color:var(--muted);margin-bottom:.75rem}
  /* Status indicator */
  #status-box{grid-column:1/-1;display:flex;align-items:center;gap:1rem;padding:1.2rem}
  #dot{width:24px;height:24px;border-radius:50%;background:var(--muted);flex-shrink:0;transition:background .2s,box-shadow .2s}
  #dot.motion{background:var(--red);box-shadow:0 0 16px var(--red)}
  #dot.quiet{background:var(--green);box-shadow:0 0 10px var(--green)}
  #dot.standby{background:var(--muted)}
  #dot.calibrating{background:var(--yellow);animation:pulse 1s ease-in-out infinite}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
  #status-text{font-size:1.3rem;font-weight:600}
  #calib-bar-wrap{margin-top:.5rem;display:none}
  #calib-bar{height:6px;background:var(--yellow);border-radius:3px;width:0%;transition:width .5s linear}
  /* Controls */
  .btn{display:inline-block;padding:.5rem 1.2rem;border-radius:6px;border:1px solid var(--border);
       background:var(--card);color:var(--text);cursor:pointer;font-size:.9rem;transition:background .15s}
  .btn:hover{background:#21262d}
  .btn.active{background:var(--accent);border-color:var(--accent);color:#000;font-weight:600}
  .btn-danger{border-color:var(--red);color:var(--red)}
  .btn-danger:hover{background:rgba(248,81,73,.1)}
  .row{display:flex;gap:.6rem;flex-wrap:wrap}
  /* Slider */
  label{display:flex;flex-direction:column;gap:.3rem;font-size:.85rem;color:var(--muted)}
  input[type=range]{width:100%;accent-color:var(--accent)}
  .val{color:var(--text);font-weight:600}
  /* Canvas graph */
  canvas{width:100%;height:120px;display:block;border-radius:4px;background:#0d1117}
  /* Stats */
  .stat-row{display:flex;justify-content:space-between;font-size:.85rem;padding:.25rem 0;border-bottom:1px solid var(--border)}
  .stat-row:last-child{border-bottom:none}
  .stat-val{font-weight:600;color:var(--accent)}
  /* Connection badge */
  #conn{position:fixed;top:.6rem;right:.8rem;font-size:.75rem;padding:.2rem .6rem;
        border-radius:20px;background:var(--card);border:1px solid var(--border)}
  #conn.ok{border-color:var(--green);color:var(--green)}
  #conn.err{border-color:var(--red);color:var(--red)}
</style>
</head>
<body>
<div style="max-width:900px;margin:0 auto">
  <h1>&#x1F4F6; ESP32 CSI Radar</h1>
  <span id="conn">&#x25CF; connecting...</span>

  <div class="grid">
    <!-- Status -->
    <div class="card" id="status-box">
      <div id="dot" class="standby"></div>
      <div>
        <div id="status-text">Standby</div>
        <div id="calib-bar-wrap"><div id="calib-bar"></div></div>
        <div id="calib-label" style="font-size:.8rem;color:var(--yellow);margin-top:.3rem"></div>
      </div>
    </div>

    <!-- Controls -->
    <div class="card">
      <h2>Steuerung</h2>
      <div class="row" style="margin-bottom:.8rem">
        <button class="btn active" id="btn-standby" onclick="setMode(0)">&#x23F9; Standby</button>
        <button class="btn" id="btn-motion" onclick="setMode(1)">&#x1F6A8; Motion</button>
      </div>
      <label>
        Empfindlichkeit (threshold)
        <input type="range" id="thresh" min="0.2" max="5.0" step="0.1" value="1.2" oninput="updateThresh(this.value)">
        <span>Wert: <span class="val" id="thresh-val">1.2</span></span>
      </label>
      <div style="margin-top:.8rem">
        <label>
          LPF Glättung (alpha) — niedrig=reaktiv, hoch=glatt
          <input type="range" id="alpha" min="0.1" max="0.9" step="0.05" value="0.4" oninput="updateAlpha(this.value)">
          <span>Wert: <span class="val" id="alpha-val">0.40</span></span>
        </label>
      </div>
    </div>

    <!-- Live graph -->
    <div class="card">
      <h2>&#x1F4C8; CSI Signalverlauf (avg_diff)</h2>
      <canvas id="graph"></canvas>
      <div style="font-size:.75rem;color:var(--muted);margin-top:.4rem">
        <span style="color:var(--red)">&#x2014;</span> Threshold &nbsp;
        <span style="color:var(--accent)">&#x2014;</span> avg_diff
      </div>
    </div>

    <!-- Stats -->
    <div class="card">
      <h2>Statistiken</h2>
      <div class="stat-row"><span>Modus</span><span class="stat-val" id="s-mode">Standby</span></div>
      <div class="stat-row"><span>Letzte avg_diff</span><span class="stat-val" id="s-diff">—</span></div>
      <div class="stat-row"><span>Frames empfangen</span><span class="stat-val" id="s-frames">0</span></div>
      <div class="stat-row"><span>Motion erkannt</span><span class="stat-val" id="s-motion">Nein</span></div>
      <div class="stat-row"><span>Threshold</span><span class="stat-val" id="s-thresh">1.20</span></div>
      <div class="stat-row"><span>LPF Alpha</span><span class="stat-val" id="s-alpha">0.97</span></div>
    </div>
  </div>
</div>

<script>
const HISTORY = 64;
let history = new Array(HISTORY).fill(0);
let threshold = 1.2;
let wsConn = null;
let pollTimer = null;

// ---- WebSocket ----
function connectWS() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  wsConn = new WebSocket(`${proto}://${location.host}/ws`);
  wsConn.onopen = () => {
    setConn(true);
    if (pollTimer) { clearInterval(pollTimer); pollTimer = null; }
  };
  wsConn.onmessage = e => applyState(JSON.parse(e.data));
  wsConn.onerror = wsConn.onclose = () => {
    setConn(false);
    wsConn = null;
    // Fallback to REST polling
    if (!pollTimer) pollTimer = setInterval(fetchState, 500);
    setTimeout(connectWS, 3000);
  };
}

function fetchState() {
  fetch('/state').then(r=>r.json()).then(applyState).catch(()=>setConn(false));
}

function setConn(ok) {
  const el = document.getElementById('conn');
  el.textContent = ok ? '● live' : '● offline';
  el.className = ok ? 'ok' : 'err';
}

// ---- Apply state from JSON ----
function applyState(s) {
  setConn(true);
  threshold = s.threshold;

  // Merge history
  if (s.history && s.history.length) {
    history = [...history, ...s.history].slice(-HISTORY);
  }
  if (typeof s.last_diff === 'number') {
    document.getElementById('s-diff').textContent = s.last_diff.toFixed(4);
  }
  document.getElementById('s-frames').textContent = s.frames || 0;
  document.getElementById('s-thresh').textContent = threshold.toFixed(2);
  document.getElementById('s-alpha').textContent = (s.alpha||0.4).toFixed(2);

  const modeNames = ['Standby', 'Motion'];
  document.getElementById('s-mode').textContent = modeNames[s.mode] || '?';

  // Buttons
  [0,1].forEach(i => {
    document.getElementById(i===0?'btn-standby':'btn-motion')
      .classList.toggle('active', s.mode === i);
  });

  // Status dot + text
  const dot = document.getElementById('dot');
  const txt = document.getElementById('status-text');
  const calibWrap = document.getElementById('calib-bar-wrap');
  const calibLbl  = document.getElementById('calib-label');

  if (s.mode === 0) {
    dot.className='standby'; txt.textContent='Standby';
    calibWrap.style.display='none'; calibLbl.textContent='';
  } else if (s.calibrating) {
    dot.className='calibrating';
    txt.textContent='Kalibrierung läuft...';
    const pct = Math.max(0, Math.min(100, 100 - (s.calib_left/10)*100));
    calibWrap.style.display='block';
    document.getElementById('calib-bar').style.width = pct+'%';
    calibLbl.textContent = `Bitte still stehen — noch ${s.calib_left}s`;
    document.getElementById('s-motion').textContent = '—';
  } else if (s.motion) {
    dot.className='motion'; txt.textContent='BEWEGUNG!';
    calibWrap.style.display='none'; calibLbl.textContent='';
    document.getElementById('s-motion').textContent = 'JA ⚠';
    document.getElementById('s-motion').style.color = 'var(--red)';
  } else {
    dot.className='quiet'; txt.textContent='Kein Motion';
    calibWrap.style.display='none'; calibLbl.textContent='';
    document.getElementById('s-motion').textContent = 'Nein';
    document.getElementById('s-motion').style.color = 'var(--green)';
  }

  drawGraph();
}

// ---- Canvas graph ----
function drawGraph() {
  const canvas = document.getElementById('graph');
  const W = canvas.offsetWidth || 400;
  const H = 120;
  canvas.width = W; canvas.height = H;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0,0,W,H);

  const max_val = Math.max(threshold * 2, ...history, 0.01);
  const toY = v => H - 8 - (v / max_val) * (H - 16);

  // Threshold line
  ctx.strokeStyle = '#f85149';
  ctx.setLineDash([4,3]);
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  const ty = toY(threshold);
  ctx.moveTo(0, ty); ctx.lineTo(W, ty);
  ctx.stroke();
  ctx.setLineDash([]);

  // Signal line
  ctx.strokeStyle = '#58a6ff';
  ctx.lineWidth = 2;
  ctx.beginPath();
  history.forEach((v, i) => {
    const x = (i / (HISTORY - 1)) * W;
    const y = toY(v);
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Fill under signal
  ctx.fillStyle = 'rgba(88,166,255,0.08)';
  ctx.beginPath();
  history.forEach((v, i) => {
    const x = (i / (HISTORY - 1)) * W;
    const y = toY(v);
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  });
  ctx.lineTo(W, H); ctx.lineTo(0, H); ctx.closePath();
  ctx.fill();
}

// ---- Commands ----
function setMode(m) {
  send('mode=' + m);
}
function updateThresh(v) {
  document.getElementById('thresh-val').textContent = parseFloat(v).toFixed(1);
  send('threshold=' + v);
}
function updateAlpha(v) {
  document.getElementById('alpha-val').textContent = parseFloat(v).toFixed(2);
  send('alpha=' + v);
}
function send(cmd) {
  fetch('/cmd', {method:'POST', headers:{'Content-Type':'text/plain'}, body: cmd})
    .then(r=>r.json()).then(applyState).catch(console.error);
}

// Init
connectWS();
window.addEventListener('resize', drawGraph);
</script>
</body>
</html>
)RAWHTML";

// ---------------------------------------------------------------------------
// Ping task — sends ICMP echo to the gateway to generate steady CSI frames
// ---------------------------------------------------------------------------

// Raw ICMP echo packet layout (no external ping component needed)
struct icmp_echo_hdr_simple {
    uint8_t  type;
    uint8_t  code;
    uint16_t chksum;
    uint16_t id;
    uint16_t seqno;
};

void EPAppWifiCsi::ping_task(void* arg) {
    EPAppWifiCsi* self = static_cast<EPAppWifiCsi*>(arg);

    // Wait briefly so CSI callback is fully registered
    vTaskDelay(pdMS_TO_TICKS(200));

    // Open a raw ICMP socket (no external component, pure lwIP)
    int sock = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    if (sock < 0) {
        ESP_LOGE(TAG, "ping_task: raw socket failed: %d", errno);
        self->ping_sock_fd = -1;
        vTaskDelete(nullptr);
        return;
    }
    self->ping_sock_fd = sock;

    // Set receive timeout so we don't block forever
    struct timeval tv = { .tv_sec = 0, .tv_usec = 500000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint16_t seq = 0;
    uint8_t  pkt[sizeof(icmp_echo_hdr_simple) + 32];  // 32 bytes payload

    while (true) {
        // Resolve gateway each iteration (handles reconnects)
        struct netif* nif = netif_default;
        if (!nif || !netif_is_up(nif)) {
            ESP_LOGW(TAG, "ping_task: netif down, waiting...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        uint32_t gw_addr = nif->gw.addr;
        if (gw_addr == 0) {
            ESP_LOGW(TAG, "ping_task: no gateway, waiting...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Build ICMP echo request
        memset(pkt, 0xAB, sizeof(pkt));   // payload pattern
        icmp_echo_hdr_simple* hdr = reinterpret_cast<icmp_echo_hdr_simple*>(pkt);
        hdr->type   = 8;                  // ICMP_ECHO
        hdr->code   = 0;
        hdr->chksum = 0;
        hdr->id     = htons(0xC5C1);     // arbitrary ID
        hdr->seqno  = htons(seq++);
        // Compute checksum over entire packet
        hdr->chksum = inet_chksum(pkt, sizeof(pkt));

        struct sockaddr_in dest = {};
        dest.sin_family      = AF_INET;
        dest.sin_addr.s_addr = gw_addr;

        sendto(sock, pkt, sizeof(pkt), 0,
               reinterpret_cast<struct sockaddr*>(&dest), sizeof(dest));

        // Drain receive buffer (we don't actually need the reply content —
        // the CSI callback fires on the incoming frame automatically)
        uint8_t rxbuf[128];
        int rlen = recv(sock, rxbuf, sizeof(rxbuf), 0);
        if (rlen < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGD(TAG, "ping_task: recv err %d", errno);
        }

        vTaskDelay(pdMS_TO_TICKS(self->ping_interval_ms));
    }
    // Should not be reached; socket cleanup handled in disable_csi().
    close(sock);
    self->ping_sock_fd = -1;
}

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

EPAppWifiCsi::EPAppWifiCsi() {
    csi_app_instance = this;
    for (int i = 0; i < MAX_WS_CLIENTS; i++) ws_fds[i] = -1;
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

    // Stop any previous state cleanly (no promiscuous — see below)
    esp_wifi_set_csi(false);
    esp_wifi_set_csi_rx_cb(nullptr, nullptr);

    if (ping_task_handle) {
        if (ping_sock_fd >= 0) {
            close(ping_sock_fd);
            ping_sock_fd = -1;
        }
        vTaskDelete(ping_task_handle);
        ping_task_handle = nullptr;
    }
    is_calibrating         = true;
    calibration_start_time = currentMillis;
    last_calib_refresh     = currentMillis;
    calib_seconds_left     = CALIB_DURATION_MS / 1000;
    motion_detected        = false;
    last_motion_time       = 0;
    last_avg_diff          = 0.0f;
    history_head           = 0;
    frame_count            = 0;
    for (int i = 0; i < MAX_MAC_SLOTS; i++) {
        mac_slots[i].active = false;
        mac_slots[i].warmup = WARMUP_FRAMES;
        memset(mac_slots[i].prev_amp, 0, sizeof(mac_slots[i].prev_amp));
    }
    for (int i = 0; i < CSI_HISTORY_LEN; i++) csi_history[i] = 0.0f;

    // KEY: Do NOT use promiscuous mode.
    // esp_wifi_set_csi() only fires callbacks for frames received in the
    // normal STA receive path (i.e. from the associated AP). In pure
    // promiscuous mode without an association the driver discards most
    // frames before the CSI hook runs — that is why frame_count stays 0.
    //
    // Correct approach:
    //   1. WiFi already connected as STA (handled by the main firmware).
    //   2. Configure CSI and register callback.
    //   3. Spawn a ping task → the AP's replies are guaranteed CSI frames
    //      from a fixed, known transmitter.

    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = false,
        .shift             = 0,
        .dump_ack_en       = false,
    };
    // -----------------------------------------------------------------------
    // WifiM::wifi_sta_ok is set true only on IP_EVENT_STA_GOT_IP.
    // This works correctly in APSTA mode where esp_wifi_sta_get_ap_info()
    // and esp_netif checks can falsely fail despite being connected.
    // -----------------------------------------------------------------------
    if (!WifiM::getWifiStaStatus()) {
        ESP_LOGW(TAG, "enable_csi: wifi_sta_ok=false, retry in 2s (IP: %s)",
                 WifiM::getStaIp().c_str());
        csi_init_pending = true;
        csi_retry_at_ms  = currentMillis + 2000;
        SetDisplayDirty();
        return;
    }
    ESP_LOGI(TAG, "enable_csi: STA ok, IP=%s", WifiM::getStaIp().c_str());

    esp_err_t err = esp_wifi_set_csi_config(&csi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_csi_config: %s", esp_err_to_name(err));
    }

    esp_wifi_set_csi_rx_cb(&EPAppWifiCsi::csi_cb, this);

    err = esp_wifi_set_csi(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_csi(true): %s", esp_err_to_name(err));
        csi_init_pending = true;
        csi_retry_at_ms  = currentMillis + 3000;
        SetDisplayDirty();
        return;
    }

    csi_init_failed  = false;
    csi_init_pending = false;
    ESP_LOGI(TAG, "CSI enabled OK");

    xTaskCreate(ping_task, "csi_ping", 4096, this, 5, &ping_task_handle);

    SetDisplayDirty();
}


void EPAppWifiCsi::disable_csi() {
    ESP_LOGI(TAG, "Disabling CSI");

    if (ping_task_handle) {
        // Close the socket first; this unblocks any pending recv() in the task
        // so vTaskDelete() is safe and the FD is not leaked.
        if (ping_sock_fd >= 0) {
            close(ping_sock_fd);
            ping_sock_fd = -1;
        }
        vTaskDelete(ping_task_handle);
        ping_task_handle = nullptr;
    }

    esp_wifi_set_csi(false);
    esp_wifi_set_csi_rx_cb(nullptr, nullptr);
    is_calibrating  = false;
    motion_detected = false;
    last_avg_diff   = 0.0f;
    csi_init_failed = false;
}

// ---------------------------------------------------------------------------
// CSI callback (WiFi task context)
// ---------------------------------------------------------------------------

void EPAppWifiCsi::csi_cb(void *ctx, wifi_csi_info_t *data) {
    if (csi_app_instance) csi_app_instance->process_csi_data(data);
}

void EPAppWifiCsi::process_csi_data(wifi_csi_info_t *data) {
    if (current_mode == 0) return;
    if (!data->buf || data->len <= 0) return;

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);

    // --- Per-MAC slot selection -------------------------------------------
    // Using per-MAC state means we compare a frame only against previous
    // frames from the *same transmitter*, so multipath is consistent and
    // we don't get spurious diffs from different APs interleaved.
    uint8_t mac_idx = 0;
    bool    found   = false;
    for (int m = 0; m < MAX_MAC_SLOTS; m++) {
        if (mac_slots[m].active &&
            memcmp(mac_slots[m].mac, data->mac, 6) == 0) {
            mac_idx = m; found = true; break;
        }
    }
    if (!found) {
        // Find a free slot or evict the oldest
        uint32_t oldest_ts = UINT32_MAX;
        uint8_t  oldest_m  = 0;
        for (int m = 0; m < MAX_MAC_SLOTS; m++) {
            if (!mac_slots[m].active) { oldest_m = m; oldest_ts = 0; break; }
            if (mac_slots[m].last_seen < oldest_ts) {
                oldest_ts = mac_slots[m].last_seen;
                oldest_m  = m;
            }
        }
        mac_idx = oldest_m;
        memcpy(mac_slots[mac_idx].mac, data->mac, 6);
        mac_slots[mac_idx].active    = true;
        mac_slots[mac_idx].warmup    = WARMUP_FRAMES;
        mac_slots[mac_idx].last_seen = now;
        memset(mac_slots[mac_idx].prev_amp, 0, sizeof(mac_slots[mac_idx].prev_amp));
    }
    MacSlot& slot = mac_slots[mac_idx];
    slot.last_seen = now;

    // --- Parse I/Q → magnitude per subcarrier ----------------------------
    int start_idx = data->first_word_invalid ? 2 : 0;
    float total_diff = 0.0f;
    int   count      = 0;

    for (int i = start_idx; i < data->len - 1; i += 2) {
        int sc = (i - start_idx) / 2;
        if (sc >= MAX_SUBCARRIERS) break;

        float im  = (float)(int8_t)data->buf[i];
        float re  = (float)(int8_t)data->buf[i + 1];
        float mag = sqrtf(im * im + re * re);

        // Only accumulate diff once the slot has a valid baseline
        if (slot.warmup == 0 && slot.prev_amp[sc] > 0.0f) {
            total_diff += fabsf(mag - slot.prev_amp[sc]);
        }
        // IIR update — always
        slot.prev_amp[sc] = slot.prev_amp[sc] * lpf_alpha + mag * (1.0f - lpf_alpha);
        count++;
    }

    if (slot.warmup > 0) { slot.warmup--; return; }  // skip diff during warm-up
    if (count == 0) return;

    float avg_diff = total_diff / (float)count;

    portENTER_CRITICAL(&mux);
    last_avg_diff = avg_diff;
    frame_count++;

    // Store in ring buffer always (even during calibration — useful for
    // showing the baseline "noise floor" in the graph)
    csi_history[history_head] = avg_diff;
    history_head = (history_head + 1) % CSI_HISTORY_LEN;

    if (!is_calibrating && avg_diff > motion_threshold) {
        motion_detected  = true;
        last_motion_time = now;
    }
    portEXIT_CRITICAL(&mux);

    SetDisplayDirty();
}

// ---------------------------------------------------------------------------
// Loop (App task context)
// ---------------------------------------------------------------------------

void EPAppWifiCsi::Loop(uint32_t currentMillis) {
    bool dirty = false;

    // Retry CSI init if a previous attempt was deferred (WiFi not ready)
    if (csi_init_pending && currentMillis >= csi_retry_at_ms) {
        csi_init_pending = false;
        enable_csi(currentMillis);
        return;
    }


    if (is_calibrating) {
        uint32_t elapsed = currentMillis - calibration_start_time;
        if (elapsed >= CALIB_DURATION_MS) {
            is_calibrating     = false;
            calib_seconds_left = 0;
            ESP_LOGI(TAG, "Calibration complete");
            dirty = true;
        } else if (currentMillis - last_calib_refresh >= CALIB_REFRESH_MS) {
            last_calib_refresh = currentMillis;
            calib_seconds_left = (CALIB_DURATION_MS - elapsed) / 1000;
            dirty = true;
        }
    }

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

    // Push WebSocket update at the configured rate
    if (http_server && currentMillis - last_ws_push_ms >= WS_PUSH_INTERVAL_MS) {
        last_ws_push_ms = currentMillis;
        push_ws_update();
    }

    if (dirty) SetDisplayDirty();
}

// ---------------------------------------------------------------------------
// Display (Portapack / ePaper)
// ---------------------------------------------------------------------------

void EPAppWifiCsi::OnDisplayRequest(DisplayGeneric* display) {
    if (current_mode == 0) {
        display->showTitle("WiFi CSI");
        display->showMainText("Mode: Standby");
        return;
    }
    display->showTitle("WiFi CSI Radar");
    if (csi_init_failed) {
        display->showMainText("FEHLER: CSI init\ngescheitert.");
        return;
    }
    if (csi_init_pending) {
        display->showMainText("Warte auf WiFi...\nRetry laeuft...");
        return;
    }
    if (is_calibrating) {
        char buf[64];
        snprintf(buf, sizeof(buf),
                 "Kalibrierung...\nBitte stillhalten\n%u Sek verbleibend",
                 (unsigned)calib_seconds_left);
        display->showMainText(buf);
        return;
    }
    portENTER_CRITICAL(&mux);
    bool     m    = motion_detected;
    float    diff = last_avg_diff;
    uint32_t fc   = frame_count;
    portEXIT_CRITICAL(&mux);

    char buf[128];
    if (m) {
        snprintf(buf, sizeof(buf), "BEWEGUNG!\n!!! Jemand bewegt sich !!!\ndiff=%.3f", diff);
    } else {
        snprintf(buf, sizeof(buf), "Kein Motion\ndiff=%.3f  #%lu", diff, (unsigned long)fc);
    }
    display->showMainText(buf);
}

// ---------------------------------------------------------------------------
// WebSocket helpers
// ---------------------------------------------------------------------------

void EPAppWifiCsi::ws_add_client(int fd) {
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (ws_fds[i] == -1) { ws_fds[i] = fd; ws_fd_count++; return; }
    }
    ESP_LOGW(TAG, "WS client list full, dropping fd=%d", fd);
}

void EPAppWifiCsi::ws_remove_client(int fd) {
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (ws_fds[i] == fd) {
            ws_fds[i] = -1;
            if (ws_fd_count > 0) ws_fd_count--;
            return;
        }
    }
}

std::string EPAppWifiCsi::build_json_state() {
    portENTER_CRITICAL(&mux);
    float    snap_diff   = last_avg_diff;
    bool     snap_motion = motion_detected;
    uint32_t snap_frames = frame_count;
    uint16_t snap_head   = history_head;
    float    snap_hist[CSI_HISTORY_LEN];
    memcpy(snap_hist, csi_history, sizeof(snap_hist));
    portEXIT_CRITICAL(&mux);

    char buf[2048];
    int n = snprintf(buf, sizeof(buf),
        "{\"mode\":%d,\"motion\":%s,\"calibrating\":%s,"
        "\"calib_left\":%u,\"last_diff\":%.4f,\"frames\":%lu,"
        "\"threshold\":%.2f,\"alpha\":%.2f,\"history\":[",
        (int)current_mode,
        snap_motion   ? "true"  : "false",
        is_calibrating? "true"  : "false",
        (unsigned)calib_seconds_left,
        snap_diff,
        (unsigned long)snap_frames,
        motion_threshold,
        lpf_alpha);

    // Emit history in chronological order
    for (int k = 0; k < CSI_HISTORY_LEN; k++) {
        int idx = (snap_head + k) % CSI_HISTORY_LEN;
        n += snprintf(buf + n, sizeof(buf) - n, "%.4f%s",
                      snap_hist[idx], k < CSI_HISTORY_LEN - 1 ? "," : "");
    }
    snprintf(buf + n, sizeof(buf) - n, "]}");
    return std::string(buf);
}

void EPAppWifiCsi::push_ws_update() {
    if (ws_fd_count == 0) return;
    std::string json = build_json_state();

    httpd_ws_frame_t pkt = {};
    pkt.type    = HTTPD_WS_TYPE_TEXT;
    pkt.payload = (uint8_t*)json.c_str();
    pkt.len     = json.size();

    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (ws_fds[i] < 0) continue;
        esp_err_t err = httpd_ws_send_frame_async(http_server, ws_fds[i], &pkt);
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "WS send failed fd=%d, removing", ws_fds[i]);
            ws_remove_client(ws_fds[i]);
        }
    }
}

// ---------------------------------------------------------------------------
// HTTP handlers (static → forward to instance)
// ---------------------------------------------------------------------------

esp_err_t EPAppWifiCsi::http_get_root(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, WEB_PAGE, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t EPAppWifiCsi::http_get_state(httpd_req_t *req) {
    if (!csi_app_instance) return ESP_FAIL;
    std::string json = csi_app_instance->build_json_state();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json.c_str(), json.size());
    return ESP_OK;
}

esp_err_t EPAppWifiCsi::http_post_cmd(httpd_req_t *req) {
    if (!csi_app_instance) return ESP_FAIL;

    char body[64] = {};
    int received = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received <= 0) return ESP_FAIL;
    body[received] = '\0';

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    if (strncmp(body, "mode=", 5) == 0) {
        int m = atoi(body + 5);
        csi_app_instance->current_mode = (uint8_t)(m & 0x01);
        if (m == 0) csi_app_instance->disable_csi();
        else        csi_app_instance->enable_csi(now_ms);
        csi_app_instance->SetDisplayDirty();
    } else if (strncmp(body, "threshold=", 10) == 0) {
        float v = atof(body + 10);
        if (v >= 0.1f && v <= 20.0f) csi_app_instance->motion_threshold = v;
    } else if (strncmp(body, "alpha=", 6) == 0) {
        float v = atof(body + 6);
        if (v >= 0.1f && v <= 0.999f) csi_app_instance->lpf_alpha = v;
    }

    // Respond with current state so UI can update immediately
    std::string json = csi_app_instance->build_json_state();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json.c_str(), json.size());
    return ESP_OK;
}

esp_err_t EPAppWifiCsi::http_ws_handler(httpd_req_t *req) {
    if (!csi_app_instance) return ESP_FAIL;

    if (req->method == HTTP_GET) {
        // New WebSocket handshake
        ESP_LOGI(TAG, "WS client connected, fd=%d", httpd_req_to_sockfd(req));
        csi_app_instance->ws_add_client(httpd_req_to_sockfd(req));

        // Send current state immediately
        std::string json = csi_app_instance->build_json_state();
        httpd_ws_frame_t pkt = {};
        pkt.type    = HTTPD_WS_TYPE_TEXT;
        pkt.payload = (uint8_t*)json.c_str();
        pkt.len     = json.size();
        httpd_ws_send_frame(req, &pkt);
        return ESP_OK;
    }

    // Receive frame (we ignore incoming messages for now)
    httpd_ws_frame_t pkt = {};
    pkt.type = HTTPD_WS_TYPE_TEXT;
    uint8_t buf[64] = {};
    pkt.payload = buf;
    esp_err_t err = httpd_ws_recv_frame(req, &pkt, sizeof(buf));
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "WS recv err %d, removing client", err);
        csi_app_instance->ws_remove_client(httpd_req_to_sockfd(req));
    }
    return err;
}

// ---------------------------------------------------------------------------
// RegisterHttpHandlers — call this after your httpd_start()
// ---------------------------------------------------------------------------

void EPAppWifiCsi::RegisterHttpHandlers(httpd_handle_t server) {
    http_server = server;

    httpd_uri_t uri_root = {
        .uri                     = "/",
        .method                  = HTTP_GET,
        .handler                 = http_get_root,
        .user_ctx                = nullptr,
        .is_websocket            = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol   = nullptr,
    };
    httpd_register_uri_handler(server, &uri_root);

    httpd_uri_t uri_state = {
        .uri                     = "/state",
        .method                  = HTTP_GET,
        .handler                 = http_get_state,
        .user_ctx                = nullptr,
        .is_websocket            = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol   = nullptr,
    };
    httpd_register_uri_handler(server, &uri_state);

    httpd_uri_t uri_cmd = {
        .uri                     = "/cmd",
        .method                  = HTTP_POST,
        .handler                 = http_post_cmd,
        .user_ctx                = nullptr,
        .is_websocket            = false,
        .handle_ws_control_frames = false,
        .supported_subprotocol   = nullptr,
    };
    httpd_register_uri_handler(server, &uri_cmd);

    // WebSocket endpoint
    httpd_uri_t uri_ws = {
        .uri                     = "/ws",
        .method                  = HTTP_GET,
        .handler                 = http_ws_handler,
        .user_ctx                = nullptr,
        .is_websocket            = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol   = nullptr,
    };
    httpd_register_uri_handler(server, &uri_ws);

    ESP_LOGI(TAG, "HTTP handlers registered");
}

// ---------------------------------------------------------------------------
// Protocol handlers (Portapack ↔ ESP32 serial protocol)
// ---------------------------------------------------------------------------

bool EPAppWifiCsi::OnWebData(std::string& data) {
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (data.compare(APP_CSI_PRE_STR "0\r\n") == 0) {
        current_mode = 0;
        disable_csi();
        SetDisplayDirty();
        return true;
    }
    if (data.compare(APP_CSI_PRE_STR "1\r\n") == 0) {
        current_mode = 1;
        enable_csi(now_ms);
        SetDisplayDirty();
        return true;
    }
    return false;
}

bool EPAppWifiCsi::OnPPData(uint16_t command, std::vector<uint8_t>& data) {
    if (command == PPCMD_APPMGR_APPCMD && data.size() >= 2) {
        uint16_t new_mode = *reinterpret_cast<uint16_t*>(data.data());
        if (new_mode <= 1) {
            current_mode = (uint8_t)new_mode;
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
            if (current_mode == 0) disable_csi();
            else                   enable_csi(now_ms);
            SetDisplayDirty();
        }
        return true;
    }
    return false;
}

bool EPAppWifiCsi::OnPPReqData(uint16_t command, std::vector<uint8_t>& data) {
    if (command == PPCMD_APPMGR_APPCMD) {
        // 3 bytes: [mode uint16_t][motion bool]
        data.resize(3);
        *reinterpret_cast<uint16_t*>(data.data()) = current_mode;
        portENTER_CRITICAL(&mux);
        data[2] = motion_detected ? 1 : 0;
        portEXIT_CRITICAL(&mux);
        return true;
    }
    return false;
}
