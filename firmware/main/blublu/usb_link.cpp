/*
 * BluBlu peripheral mode: framed request/response link over USB-Serial-JTAG.
 * See usb_link.h for the frame format.
 */
#include "usb_link.h"
#include "api_common.h"
#include "camera_pipeline.h"
#include "robot_control.h"
#include <driver/usb_serial_jtag.h>
#include <esp_rom_crc.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <cJSON.h>
#include <sdkconfig.h>
#if CONFIG_STACKCHAN_BLUBLU_WIFI
#include <wifi_manager.h>
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace blublu::usb_link {

static const char* TAG = "BluBluUsb";

namespace {

constexpr uint8_t kVersion        = 1;
constexpr uint8_t kTypeRequest    = 1;
constexpr uint8_t kTypeResponse   = 2;
constexpr uint8_t kTypeEvent      = 3;
constexpr size_t kHeaderLen       = 12;
constexpr size_t kCrcLen          = 4;
constexpr size_t kMaxRequest      = 8192;  // payload limit host -> device
constexpr size_t kTxChunk         = 4096;  // must not exceed the driver TX ring buffer
constexpr TickType_t kTxTimeout   = pdMS_TO_TICKS(200);
constexpr TickType_t kRxIdle      = pdMS_TO_TICKS(300);
constexpr int kProtocolVersion    = 1;

SemaphoreHandle_t s_tx_m = nullptr;
QueueHandle_t s_snap_q   = nullptr;
std::atomic<uint32_t> s_tx_drops{0};
std::atomic<uint32_t> s_rx_frames{0};
std::atomic<uint32_t> s_rx_bad{0};
std::atomic<uint32_t> s_stream_gen{0};  // bump to stop the running stream task

struct SnapReq {
    uint16_t seq;
    int quality;
    bool fresh;
};

struct StreamArgs {
    uint32_t gen;
    int fps;
    int quality;
};

/* --------------------------------- Framing --------------------------------- */
inline void put_u16(uint8_t* p, uint16_t v)
{
    p[0] = v & 0xFF;
    p[1] = v >> 8;
}

inline void put_u32(uint8_t* p, uint32_t v)
{
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF;
    p[3] = v >> 24;
}

inline uint16_t get_u16(const uint8_t* p)
{
    return p[0] | (p[1] << 8);
}

inline uint32_t get_u32(const uint8_t* p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

inline uint32_t crc32(uint32_t crc, const void* data, size_t len)
{
    return esp_rom_crc32_le(crc, (const uint8_t*)data, len);
}

// Never blocks for long: when the host stops reading, the rest of the frame is dropped
// (the host resyncs on the next magic).
bool write_raw(const void* data, size_t len)
{
    const uint8_t* p = (const uint8_t*)data;
    while (len > 0) {
        const size_t n = std::min(len, kTxChunk);
        const int w    = usb_serial_jtag_write_bytes(p, n, kTxTimeout);
        if (w <= 0) {
            return false;
        }
        p += w;
        len -= w;
    }
    return true;
}

bool send_frame(uint8_t type, uint16_t seq, const std::string& json, const uint8_t* bin = nullptr,
                size_t bin_len = 0)
{
    if (!usb_serial_jtag_is_connected()) {
        return false;
    }
    uint8_t jl[4];
    put_u32(jl, json.size());
    uint32_t crc = crc32(0, jl, sizeof(jl));
    crc          = crc32(crc, json.data(), json.size());
    if (bin_len) {
        crc = crc32(crc, bin, bin_len);
    }
    uint8_t tail[4];
    put_u32(tail, crc);

    uint8_t hdr[kHeaderLen];
    hdr[0] = 'S';
    hdr[1] = 'C';
    hdr[2] = kVersion;
    hdr[3] = type;
    put_u16(hdr + 4, seq);
    put_u32(hdr + 6, sizeof(jl) + json.size() + bin_len);
    put_u16(hdr + 10, crc32(0, hdr, 10) & 0xFFFF);

    xSemaphoreTake(s_tx_m, portMAX_DELAY);
    bool ok = write_raw(hdr, sizeof(hdr)) && write_raw(jl, sizeof(jl)) && write_raw(json.data(), json.size()) &&
              (bin_len == 0 || write_raw(bin, bin_len)) && write_raw(tail, sizeof(tail));
    xSemaphoreGive(s_tx_m);
    if (!ok) {
        s_tx_drops++;
    }
    return ok;
}

std::string to_string(cJSON* root)
{
    char* str = cJSON_PrintUnformatted(root);
    std::string s(str ? str : "{}");
    cJSON_free(str);
    cJSON_Delete(root);
    return s;
}

// Takes ownership of root. seq 0 = fire-and-forget request, no response.
void reply(uint16_t seq, cJSON* root, const uint8_t* bin = nullptr, size_t bin_len = 0)
{
    if (seq == 0) {
        cJSON_Delete(root);
        return;
    }
    send_frame(kTypeResponse, seq, to_string(root), bin, bin_len);
}

// msg must be a constant string (not user input)
void reply_error(uint16_t seq, const char* msg)
{
    if (seq == 0) {
        return;
    }
    send_frame(kTypeResponse, seq, std::string(R"({"ok":false,"error":")") + msg + "\"}");
}

cJSON* ok_object()
{
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    return root;
}

/* --------------------------------- Requests -------------------------------- */
cJSON* build_info()
{
    cJSON* root  = api::build_info();
    cJSON* links = cJSON_AddObjectToObject(root, "links");
    cJSON_AddStringToObject(links, "usb", "usb-serial-jtag");
#if CONFIG_STACKCHAN_BLUBLU_WIFI
    cJSON_AddStringToObject(links, "wifi", WifiManager::GetInstance().GetIpAddress().c_str());
#else
    cJSON_AddNullToObject(links, "wifi");
#endif
    cJSON* usb = cJSON_AddObjectToObject(root, "usb");
    cJSON_AddNumberToObject(usb, "proto", kProtocolVersion);
    cJSON_AddNumberToObject(usb, "rx_frames", s_rx_frames.load());
    cJSON_AddNumberToObject(usb, "rx_bad", s_rx_bad.load());
    cJSON_AddNumberToObject(usb, "tx_drops", s_tx_drops.load());
    return root;
}

void stream_task(void* arg);

void stream_stop()
{
    s_stream_gen++;
}

// Returns the stream id (> 0), 0 on failure
uint32_t stream_start(int fps, int quality)
{
    const uint32_t id = ++s_stream_gen;  // also stops the previous stream
    auto* args        = new StreamArgs{id, fps, quality};
    if (xTaskCreatePinnedToCore(stream_task, "bb_usb_stream", 6144, args, 4, nullptr, 0) != pdPASS) {
        delete args;
        return 0;
    }
    return id;
}

void handle_request(uint16_t seq, const uint8_t* payload, size_t len)
{
    const uint32_t json_len = get_u32(payload);
    if (json_len > len - 4) {
        reply_error(seq, "bad payload");
        return;
    }
    cJSON* req = cJSON_ParseWithLength((const char*)payload + 4, json_len);
    cJSON* op  = cJSON_GetObjectItemCaseSensitive(req, "op");
    if (!cJSON_IsObject(req) || !cJSON_IsString(op) || !op->valuestring) {
        cJSON_Delete(req);
        reply_error(seq, "invalid json or op");
        return;
    }
    const std::string o = op->valuestring;
    const char* err     = nullptr;

    if (o == "hello") {
        // New host session: stop leftovers of the previous one
        stream_stop();
        cJSON* resp = build_info();
        cJSON_AddNumberToObject(resp, "proto", kProtocolVersion);
        cJSON* nonce = cJSON_GetObjectItemCaseSensitive(req, "nonce");
        if (nonce) {
            cJSON_AddItemToObject(resp, "nonce", cJSON_Duplicate(nonce, true));
        }
        reply(seq, resp);
    } else if (o == "ping") {
        cJSON* resp = ok_object();
        cJSON_AddNumberToObject(resp, "uptime_ms", (double)(esp_timer_get_time() / 1000));
        reply(seq, resp);
    } else if (o == "info") {
        reply(seq, build_info());
    } else if (o == "snapshot") {
        if (!camera::ready()) {
            err = "camera not ready";
        } else {
            SnapReq r{seq, std::clamp(api::json_int(req, "quality", 80), 10, 95),
                      api::json_int(req, "fresh", 1) != 0};
            if (xQueueSend(s_snap_q, &r, 0) != pdTRUE) {
                err = "too many snapshot requests";
            }
        }
    } else if (o == "head.get") {
        auto s      = robot::get_head();
        cJSON* resp = ok_object();
        api::add_head_state(resp, s);
        cJSON_AddBoolToObject(resp, "moving", s.moving);
        reply(seq, resp);
    } else if (o == "head" || o == "look") {
        cJSON* target = cJSON_CreateObject();
        if (!api::apply_head(req, o == "look", target, &err)) {
            cJSON_Delete(target);
        } else {
            auto s      = robot::get_head();
            cJSON* resp = ok_object();
            cJSON_AddItemToObject(resp, "target", target);
            api::add_head_state(cJSON_AddObjectToObject(resp, "current"), s);
            cJSON_AddBoolToObject(resp, "moving", s.moving);
            reply(seq, resp);
        }
    } else if (o == "home") {
        robot::go_home(std::clamp(api::json_int(req, "speed", 500), 0, 1000));
        cJSON* resp   = ok_object();
        cJSON* target = cJSON_AddObjectToObject(resp, "target");
        cJSON_AddNumberToObject(target, "yaw", 0);
        cJSON_AddNumberToObject(target, "pitch", 0);
        reply(seq, resp);
    } else if (o == "emotion.get") {
        cJSON* resp = ok_object();
        cJSON_AddStringToObject(resp, "emotion", robot::get_emotion().c_str());
        const auto& emotions = robot::supported_emotions();
        cJSON_AddItemToObject(resp, "supported", cJSON_CreateStringArray(emotions.data(), emotions.size()));
        reply(seq, resp);
    } else if (o == "emotion") {
        cJSON* resp = ok_object();
        if (!api::apply_emotion(req, resp, &err)) {
            cJSON_Delete(resp);
        } else {
            reply(seq, resp);
        }
    } else if (o == "speaking") {
        if (api::apply_speaking(req, &err)) {
            cJSON* resp = ok_object();
            cJSON_AddBoolToObject(resp, "speaking", robot::is_speaking());
            reply(seq, resp);
        }
    } else if (o == "stream.start") {
        const int fps     = std::clamp(api::json_int(req, "fps", 10), 1, camera::kMaxFps);
        const int quality = std::clamp(api::json_int(req, "quality", 70), 10, 95);
        uint32_t id       = 0;
        if (!camera::ready()) {
            err = "camera not ready";
        } else if ((id = stream_start(fps, quality)) == 0) {
            err = "no memory";
        } else {
            cJSON* resp = ok_object();
            cJSON_AddNumberToObject(resp, "stream", id);
            cJSON_AddNumberToObject(resp, "fps", fps);
            cJSON_AddNumberToObject(resp, "width", camera::stream_width());
            cJSON_AddNumberToObject(resp, "height", camera::stream_height());
            reply(seq, resp);
        }
    } else if (o == "stream.stop") {
        stream_stop();
        reply(seq, ok_object());
    } else if (o == "state") {
        cJSON* state = cJSON_GetObjectItemCaseSensitive(req, "state");
        if (!cJSON_IsString(state) || !state->valuestring || !robot::set_state(state->valuestring)) {
            err = "state must be idle, listening or thinking";
        } else {
            cJSON* resp = ok_object();
            cJSON_AddStringToObject(resp, "state", state->valuestring);
            reply(seq, resp);
        }
    } else if (o == "cam.reg") {
        uint8_t v      = 0;
        const int reg  = api::json_int(req, "reg", -1);
        const int val  = api::json_int(req, "val", -1);
        if (reg < 0 || reg > 255 || val > 255) {
            err = "bad reg";
        } else if (camera::sensor_reg(reg, val, &v) != ESP_OK) {
            err = "i2c failed";
        } else {
            cJSON* resp = ok_object();
            cJSON_AddNumberToObject(resp, "val", v);
            reply(seq, resp);
        }
    } else {
        err = "unknown op";
    }

    cJSON_Delete(req);
    if (err) {
        reply_error(seq, err);
    }
}

/* ---------------------------------- Tasks ---------------------------------- */
void snapshot_worker(void*)
{
    while (true) {
        SnapReq r;
        xQueueReceive(s_snap_q, &r, portMAX_DELAY);
        camera::JpegPtr jpg;
        esp_err_t err = camera::snapshot(r.quality, r.fresh, jpg);
        if (err == ESP_OK && jpg) {
            cJSON* resp = ok_object();
            cJSON_AddStringToObject(resp, "format", "jpeg");
            cJSON_AddNumberToObject(resp, "width", jpg->width);
            cJSON_AddNumberToObject(resp, "height", jpg->height);
            cJSON_AddNumberToObject(resp, "ts", (double)jpg->ts_ms);
            cJSON_AddNumberToObject(resp, "encode_ms", jpg->encode_ms);
            cJSON_AddNumberToObject(resp, "size", jpg->len);
            reply(r.seq, resp, jpg->data, jpg->len);
        } else if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE) {
            reply_error(r.seq, "camera timeout");
        } else {
            reply_error(r.seq, "encode failed");
        }
    }
}

void stream_task(void* arg)
{
    const StreamArgs a = *static_cast<StreamArgs*>(arg);
    delete static_cast<StreamArgs*>(arg);

    const char* reason = "stopped";
    const int slot     = camera::stream_acquire(a.fps, a.quality);
    uint32_t sent      = 0;
    if (slot < 0) {
        reason = "too many stream clients";
    } else {
        const int64_t interval_ms = 1000 / a.fps;
        int64_t next_due_ms       = 0;
        uint32_t last_seq         = 0;
        int misses                = 0;
        int fails                 = 0;
        char meta[160];
        while (s_stream_gen.load() == a.gen) {
            auto frame = camera::stream_wait(last_seq, 1000);
            if (!frame) {
                if (++misses >= 3) {
                    reason = "camera timeout";
                    break;
                }
                continue;
            }
            misses   = 0;
            last_seq = frame->seq;

            // Per-client rate limit (the encoder runs at the highest requested fps)
            const int64_t ts = frame->ts_ms;
            if (ts + interval_ms / 4 < next_due_ms) {
                continue;
            }
            next_due_ms += interval_ms;
            if (next_due_ms < ts) {
                next_due_ms = ts + interval_ms;
            }

            // newlib nano printf has no %lld: format the 64-bit timestamp with std::to_string
            int n = snprintf(meta, sizeof(meta),
                             R"({"event":"frame","stream":%lu,"format":"jpeg","seq":%lu,"width":%d,"height":%d,"ts":)",
                             (unsigned long)a.gen, (unsigned long)frame->seq, frame->width, frame->height);
            std::string json(meta, n);
            json += std::to_string(frame->ts_ms);
            json += '}';
            if (send_frame(kTypeEvent, 0, json, frame->data, frame->len)) {
                fails = 0;
                sent++;
            } else if (++fails >= 3) {
                reason = "host not reading";
                break;
            }
        }
        camera::stream_release(slot);
    }

    ESP_LOGI(TAG, "stream ended (%s) after %lu frames", reason, (unsigned long)sent);
    char msg[96];
    int n = snprintf(msg, sizeof(msg), R"({"event":"stream.stopped","stream":%lu,"reason":"%s"})",
                     (unsigned long)a.gen, reason);
    send_frame(kTypeEvent, 0, std::string(msg, n));
    vTaskDelete(nullptr);
}

// Scans buf for complete request frames, dispatches them and drops consumed / garbage bytes.
void parse(std::vector<uint8_t>& buf)
{
    size_t pos = 0;
    while (true) {
        while (pos + 1 < buf.size() && !(buf[pos] == 'S' && buf[pos + 1] == 'C')) {
            pos++;
        }
        if (pos + kHeaderLen > buf.size()) {
            break;
        }
        const uint8_t* h   = buf.data() + pos;
        const uint32_t len = get_u32(h + 6);
        const bool hdr_ok  = h[2] == kVersion && h[3] == kTypeRequest && len >= 4 && len <= kMaxRequest &&
                            get_u16(h + 10) == (crc32(0, h, 10) & 0xFFFF);
        if (!hdr_ok) {
            pos++;
            s_rx_bad++;
            continue;
        }
        if (pos + kHeaderLen + len + kCrcLen > buf.size()) {
            break;  // wait for the rest
        }
        const uint8_t* p = h + kHeaderLen;
        if (get_u32(p + len) != crc32(0, p, len)) {
            pos++;
            s_rx_bad++;
            continue;
        }
        s_rx_frames++;
        handle_request(get_u16(h + 4), p, len);
        pos += kHeaderLen + len + kCrcLen;
    }
    buf.erase(buf.begin(), buf.begin() + pos);
}

void rx_task(void*)
{
    std::vector<uint8_t> buf;
    buf.reserve(kHeaderLen + kMaxRequest + kCrcLen + 512);
    uint8_t tmp[512];
    while (true) {
        const int n = usb_serial_jtag_read_bytes(tmp, sizeof(tmp), kRxIdle);
        if (n > 0) {
            buf.insert(buf.end(), tmp, tmp + n);
        } else if (!buf.empty()) {
            // Idle with a partial frame pending (host vanished mid-frame or garbage that looked
            // like a header): skip one byte so a later frame can be found.
            buf.erase(buf.begin());
        } else {
            continue;
        }
        parse(buf);
    }
}

}  // namespace

bool start()
{
    usb_serial_jtag_driver_config_t cfg = {};
    cfg.tx_buffer_size                  = 16384;
    cfg.rx_buffer_size                  = 4096;
    esp_err_t err                       = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_serial_jtag_driver_install failed: %s", esp_err_to_name(err));
        return false;
    }
    s_tx_m   = xSemaphoreCreateMutex();
    s_snap_q = xQueueCreate(2, sizeof(SnapReq));
    xTaskCreatePinnedToCore(rx_task, "bb_usb_rx", 6144, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(snapshot_worker, "bb_usb_snap", 8192, nullptr, 4, nullptr, 1);
    ESP_LOGI(TAG, "USB link started (USB-Serial-JTAG, protocol v%d)", kProtocolVersion);
    return true;
}

}  // namespace blublu::usb_link
