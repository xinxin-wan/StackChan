/*
 * BluBlu peripheral mode: HTTP / WebSocket API (v1, no auth).
 */
#include "http_api.h"
#include "api_common.h"
#include "camera_pipeline.h"
#include "robot_control.h"
#include <esp_http_server.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <cJSON.h>
#include <wifi_manager.h>
#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <vector>

namespace blublu::http {

static const char* TAG = "BluBluHttp";

namespace {

using api::apply_emotion;
using api::apply_head;
using api::apply_speaking;
using api::add_head_state;
using api::deg;
using api::json_int;

httpd_handle_t s_server     = nullptr;
QueueHandle_t s_snapshot_q  = nullptr;
constexpr int kMaxBodyLen   = 2048;

/* --------------------------------- Helpers -------------------------------- */
void add_cors(httpd_req_t* r)
{
    httpd_resp_set_hdr(r, "Access-Control-Allow-Origin", "*");
}

esp_err_t send_json(httpd_req_t* r, const char* status, const std::string& body)
{
    httpd_resp_set_status(r, status);
    httpd_resp_set_type(r, "application/json");
    add_cors(r);
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    return httpd_resp_send(r, body.data(), body.size());
}

esp_err_t send_cjson(httpd_req_t* r, cJSON* root)
{
    char* str = cJSON_PrintUnformatted(root);
    std::string body(str ? str : "{}");
    cJSON_free(str);
    cJSON_Delete(root);
    return send_json(r, "200 OK", body);
}

// msg must be a constant string (not user input)
esp_err_t send_error(httpd_req_t* r, const char* status, const char* msg)
{
    std::string body = std::string(R"({"ok":false,"error":")") + msg + "\"}";
    return send_json(r, status, body);
}

int query_int(httpd_req_t* r, const char* key, int def)
{
    char query[128];
    if (httpd_req_get_url_query_str(r, query, sizeof(query)) != ESP_OK) {
        return def;
    }
    char value[16];
    if (httpd_query_key_value(query, key, value, sizeof(value)) != ESP_OK) {
        return def;
    }
    char* end = nullptr;
    long v    = strtol(value, &end, 10);
    if (end == value) {
        return def;
    }
    return (int)v;
}

// Returns nullptr and sends an error response on failure.
cJSON* read_json(httpd_req_t* r, bool allow_empty)
{
    const size_t len = r->content_len;
    if (len == 0) {
        if (allow_empty) {
            return cJSON_CreateObject();
        }
        send_error(r, "400 Bad Request", "empty body");
        return nullptr;
    }
    if (len > kMaxBodyLen) {
        send_error(r, "400 Bad Request", "body too large");
        return nullptr;
    }
    std::string body(len, '\0');
    size_t got = 0;
    while (got < len) {
        int ret = httpd_req_recv(r, &body[got], len - got);
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (ret <= 0) {
            send_error(r, "400 Bad Request", "read body failed");
            return nullptr;
        }
        got += ret;
    }
    cJSON* root = cJSON_ParseWithLength(body.data(), body.size());
    if (!cJSON_IsObject(root)) {
        cJSON_Delete(root);
        send_error(r, "400 Bad Request", "invalid json");
        return nullptr;
    }
    return root;
}

/* --------------------------------- Handlers -------------------------------- */
#ifdef CONFIG_HTTPD_WS_SUPPORT
int ws_client_count();
#endif

esp_err_t info_handler(httpd_req_t* r)
{
    auto& wifi  = WifiManager::GetInstance();
    cJSON* root = api::build_info();
    cJSON_AddStringToObject(root, "hostname", CONFIG_STACKCHAN_BLUBLU_HOSTNAME ".local");

    cJSON* links = cJSON_AddObjectToObject(root, "links");
#if CONFIG_STACKCHAN_BLUBLU_USB_LINK
    cJSON_AddStringToObject(links, "usb", "usb-serial-jtag");
#else
    cJSON_AddNullToObject(links, "usb");
#endif
    cJSON_AddStringToObject(links, "wifi", wifi.GetIpAddress().c_str());
    cJSON_AddStringToObject(root, "ssid", wifi.GetSsid().c_str());
    cJSON_AddNumberToObject(root, "rssi", wifi.GetRssi());
#ifdef CONFIG_HTTPD_WS_SUPPORT
    cJSON_AddNumberToObject(root, "ws_clients", ws_client_count());
#endif
    return send_cjson(r, root);
}

/* Snapshot: handled by a worker task so the HTTP server is not blocked while encoding */
esp_err_t snapshot_handler(httpd_req_t* r)
{
    if (!camera::ready()) {
        return send_error(r, "503 Service Unavailable", "camera not ready");
    }
    httpd_req_t* async_req = nullptr;
    if (httpd_req_async_handler_begin(r, &async_req) != ESP_OK) {
        return send_error(r, "500 Internal Server Error", "async begin failed");
    }
    if (xQueueSend(s_snapshot_q, &async_req, 0) != pdTRUE) {
        send_error(async_req, "409 Conflict", "too many snapshot requests");
        httpd_req_async_handler_complete(async_req);
    }
    return ESP_OK;
}

void snapshot_worker(void*)
{
    while (true) {
        httpd_req_t* r = nullptr;
        xQueueReceive(s_snapshot_q, &r, portMAX_DELAY);

        const int quality = query_int(r, "quality", 80);
        const bool fresh  = query_int(r, "fresh", 1) != 0;
        camera::JpegPtr jpg;
        esp_err_t err = camera::snapshot(quality, fresh, jpg);

        if (err == ESP_OK && jpg) {
            char ts[24], w[8], h[8], enc[12];
            snprintf(ts, sizeof(ts), "%s", std::to_string(jpg->ts_ms).c_str());
            snprintf(w, sizeof(w), "%d", jpg->width);
            snprintf(h, sizeof(h), "%d", jpg->height);
            snprintf(enc, sizeof(enc), "%lu", (unsigned long)jpg->encode_ms);
            httpd_resp_set_type(r, "image/jpeg");
            add_cors(r);
            httpd_resp_set_hdr(r, "Cache-Control", "no-store");
            httpd_resp_set_hdr(r, "X-Frame-Timestamp", ts);
            httpd_resp_set_hdr(r, "X-Frame-Width", w);
            httpd_resp_set_hdr(r, "X-Frame-Height", h);
            httpd_resp_set_hdr(r, "X-Encode-Ms", enc);
            httpd_resp_send(r, (const char*)jpg->data, jpg->len);
        } else if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE) {
            send_error(r, "503 Service Unavailable", "camera timeout");
        } else {
            send_error(r, "500 Internal Server Error", "encode failed");
        }
        httpd_req_async_handler_complete(r);
    }
}

/* MJPEG stream: one task per client (max 2), all sharing one encoder */
struct StreamCtx {
    httpd_req_t* req;
    int slot;
    int fps;
};

void stream_task(void* arg)
{
    auto* ctx      = static_cast<StreamCtx*>(arg);
    httpd_req_t* r = ctx->req;

    httpd_resp_set_type(r, "multipart/x-mixed-replace; boundary=frame");
    add_cors(r);
    httpd_resp_set_hdr(r, "Cache-Control", "no-cache, no-store");

    const int64_t interval_ms = 1000 / ctx->fps;
    int64_t next_due_ms       = 0;
    uint32_t last_seq         = 0;
    char part_hdr[160];
    uint32_t sent = 0;

    while (true) {
        auto frame = camera::stream_wait(last_seq, 3000);
        if (!frame) {
            ESP_LOGW(TAG, "stream: no frame in 3s, closing");
            httpd_resp_send_chunk(r, nullptr, 0);
            break;
        }
        last_seq = frame->seq;

        // Per-client rate limit (encoder runs at the highest requested fps)
        const int64_t ts = frame->ts_ms;
        if (ts + interval_ms / 4 < next_due_ms) {
            continue;
        }
        next_due_ms += interval_ms;
        if (next_due_ms < ts) {
            next_due_ms = ts + interval_ms;
        }

        int n = snprintf(part_hdr, sizeof(part_hdr),
                         "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\nX-Frame-Timestamp: %s\r\n\r\n",
                         (unsigned)frame->len, std::to_string(frame->ts_ms).c_str());
        if (httpd_resp_send_chunk(r, part_hdr, n) != ESP_OK ||
            httpd_resp_send_chunk(r, (const char*)frame->data, frame->len) != ESP_OK ||
            httpd_resp_send_chunk(r, "\r\n", 2) != ESP_OK) {
            break;
        }
        sent++;
    }

    ESP_LOGI(TAG, "stream client (slot %d) closed after %lu frames", ctx->slot, (unsigned long)sent);
    camera::stream_release(ctx->slot);
    httpd_req_async_handler_complete(r);
    delete ctx;
    vTaskDelete(nullptr);
}

esp_err_t stream_handler(httpd_req_t* r)
{
    if (!camera::ready()) {
        return send_error(r, "503 Service Unavailable", "camera not ready");
    }
    const int fps     = std::clamp(query_int(r, "fps", 10), 1, camera::kMaxFps);
    const int quality = std::clamp(query_int(r, "quality", 70), 10, 95);
    const int slot    = camera::stream_acquire(fps, quality);
    if (slot < 0) {
        return send_error(r, "409 Conflict", "too many stream clients");
    }

    httpd_req_t* async_req = nullptr;
    if (httpd_req_async_handler_begin(r, &async_req) != ESP_OK) {
        camera::stream_release(slot);
        return send_error(r, "500 Internal Server Error", "async begin failed");
    }
    auto* ctx = new StreamCtx{async_req, slot, fps};
    if (xTaskCreatePinnedToCore(stream_task, "bb_stream", 4096, ctx, 4, nullptr, 0) != pdPASS) {
        camera::stream_release(slot);
        send_error(async_req, "500 Internal Server Error", "no memory");
        httpd_req_async_handler_complete(async_req);
        delete ctx;
    }
    return ESP_OK;
}

esp_err_t head_get_handler(httpd_req_t* r)
{
    auto s      = robot::get_head();
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    add_head_state(root, s);
    cJSON_AddBoolToObject(root, "moving", s.moving);
    return send_cjson(r, root);
}

esp_err_t head_post_handler(httpd_req_t* r)
{
    cJSON* body = read_json(r, false);
    if (!body) {
        return ESP_OK;
    }
    cJSON* target   = cJSON_CreateObject();
    const char* err = nullptr;
    bool ok         = apply_head(body, false, target, &err);
    cJSON_Delete(body);
    if (!ok) {
        cJSON_Delete(target);
        return send_error(r, "400 Bad Request", err);
    }

    auto s      = robot::get_head();
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddItemToObject(root, "target", target);
    add_head_state(cJSON_AddObjectToObject(root, "current"), s);
    cJSON_AddBoolToObject(root, "moving", s.moving);
    return send_cjson(r, root);
}

esp_err_t head_home_handler(httpd_req_t* r)
{
    cJSON* body = read_json(r, true);
    if (!body) {
        return ESP_OK;
    }
    const int speed = std::clamp(json_int(body, "speed", 500), 0, 1000);
    cJSON_Delete(body);
    robot::go_home(speed);
    return send_json(r, "200 OK", R"({"ok":true,"target":{"yaw":0,"pitch":0}})");
}

esp_err_t emotion_get_handler(httpd_req_t* r)
{
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddStringToObject(root, "emotion", robot::get_emotion().c_str());
    const auto& emotions = robot::supported_emotions();
    cJSON_AddItemToObject(root, "supported", cJSON_CreateStringArray(emotions.data(), emotions.size()));
    return send_cjson(r, root);
}

esp_err_t emotion_post_handler(httpd_req_t* r)
{
    cJSON* body = read_json(r, false);
    if (!body) {
        return ESP_OK;
    }
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    const char* err = nullptr;
    bool ok         = apply_emotion(body, root, &err);
    cJSON_Delete(body);
    if (!ok) {
        cJSON_Delete(root);
        return send_error(r, "400 Bad Request", err);
    }
    return send_cjson(r, root);
}

esp_err_t speaking_handler(httpd_req_t* r)
{
    cJSON* body = read_json(r, false);
    if (!body) {
        return ESP_OK;
    }
    const char* err = nullptr;
    bool ok         = apply_speaking(body, &err);
    cJSON_Delete(body);
    if (!ok) {
        return send_error(r, "400 Bad Request", err);
    }
    return send_json(r, "200 OK", robot::is_speaking() ? R"({"ok":true,"speaking":true})"
                                                       : R"({"ok":true,"speaking":false})");
}

esp_err_t options_handler(httpd_req_t* r)
{
    httpd_resp_set_status(r, "204 No Content");
    add_cors(r);
    httpd_resp_set_hdr(r, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(r, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_set_hdr(r, "Access-Control-Max-Age", "600");
    return httpd_resp_send(r, nullptr, 0);
}

esp_err_t not_found_handler(httpd_req_t* r, httpd_err_code_t)
{
    send_error(r, "404 Not Found", "not found");
    return ESP_OK;
}

/* -------------------------------- WebSocket -------------------------------- */
#ifdef CONFIG_HTTPD_WS_SUPPORT
constexpr int kMaxWsClients = 2;
std::mutex s_ws_m;
std::vector<int> s_ws_fds;
esp_timer_handle_t s_ws_timer = nullptr;

void ws_prune_locked()
{
    s_ws_fds.erase(std::remove_if(s_ws_fds.begin(), s_ws_fds.end(),
                                  [](int fd) {
                                      return httpd_ws_get_fd_info(s_server, fd) != HTTPD_WS_CLIENT_WEBSOCKET;
                                  }),
                   s_ws_fds.end());
}

int ws_client_count()
{
    std::lock_guard<std::mutex> lk(s_ws_m);
    ws_prune_locked();
    return (int)s_ws_fds.size();
}

// Runs in the HTTP server task
void ws_push_work(void*)
{
    std::vector<int> fds;
    {
        std::lock_guard<std::mutex> lk(s_ws_m);
        ws_prune_locked();
        fds = s_ws_fds;
    }
    if (fds.empty()) {
        return;
    }

    auto s = robot::get_head();
    char msg[192];
    int n = snprintf(msg, sizeof(msg),
                     R"({"type":"state","yaw":%.1f,"pitch":%.1f,"moving":%s,"emotion":"%s","speaking":%s})",
                     deg(s.yaw), deg(s.pitch), s.moving ? "true" : "false", robot::get_emotion().c_str(),
                     robot::is_speaking() ? "true" : "false");

    httpd_ws_frame_t frame = {};
    frame.type             = HTTPD_WS_TYPE_TEXT;
    frame.payload          = (uint8_t*)msg;
    frame.len              = n;
    for (int fd : fds) {
        httpd_ws_send_frame_async(s_server, fd, &frame);
    }
}

void ws_timer_cb(void*)
{
    {
        std::lock_guard<std::mutex> lk(s_ws_m);
        if (s_ws_fds.empty()) {
            return;
        }
    }
    httpd_queue_work(s_server, ws_push_work, nullptr);
}

esp_err_t ws_handler(httpd_req_t* r)
{
    if (r->method == HTTP_GET) {
        // Handshake done
        std::lock_guard<std::mutex> lk(s_ws_m);
        ws_prune_locked();
        if ((int)s_ws_fds.size() >= kMaxWsClients) {
            ESP_LOGW(TAG, "ws: too many clients");
            return ESP_FAIL;
        }
        s_ws_fds.push_back(httpd_req_to_sockfd(r));
        ESP_LOGI(TAG, "ws client connected, total %d", (int)s_ws_fds.size());
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {};
    frame.type             = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret          = httpd_ws_recv_frame(r, &frame, 0);
    if (ret != ESP_OK) {
        return ret;
    }
    if (frame.len == 0) {
        return ESP_OK;
    }
    if (frame.len > kMaxBodyLen) {
        // Payload is not consumed, close the connection
        return ESP_FAIL;
    }
    std::string buf(frame.len, '\0');
    frame.payload = (uint8_t*)buf.data();
    ret           = httpd_ws_recv_frame(r, &frame, frame.len);
    if (ret != ESP_OK) {
        return ret;
    }
    if (frame.type != HTTPD_WS_TYPE_TEXT) {
        return ESP_OK;
    }

    cJSON* root = cJSON_ParseWithLength(buf.data(), buf.size());
    if (!cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return ESP_OK;
    }
    cJSON* type     = cJSON_GetObjectItemCaseSensitive(root, "type");
    const char* err = nullptr;
    if (cJSON_IsString(type) && type->valuestring) {
        std::string t = type->valuestring;
        if (t == "look") {
            apply_head(root, true, nullptr, &err);
        } else if (t == "head") {
            apply_head(root, false, nullptr, &err);
        } else if (t == "emotion") {
            apply_emotion(root, nullptr, &err);
        } else if (t == "speaking") {
            apply_speaking(root, &err);
        } else if (t == "home") {
            robot::go_home(std::clamp(json_int(root, "speed", 500), 0, 1000));
        } else {
            err = "unknown type";
        }
    } else {
        err = "type is required";
    }
    cJSON_Delete(root);

    if (err) {
        std::string reply = std::string(R"({"type":"error","error":")") + err + "\"}";
        httpd_ws_frame_t out = {};
        out.type             = HTTPD_WS_TYPE_TEXT;
        out.payload          = (uint8_t*)reply.data();
        out.len              = reply.size();
        httpd_ws_send_frame(r, &out);
    }
    return ESP_OK;
}
#endif  // CONFIG_HTTPD_WS_SUPPORT

void register_uri(const char* uri, httpd_method_t method, esp_err_t (*handler)(httpd_req_t*), bool ws = false)
{
    httpd_uri_t u = {};
    u.uri         = uri;
    u.method      = method;
    u.handler     = handler;
#ifdef CONFIG_HTTPD_WS_SUPPORT
    u.is_websocket = ws;
#endif
    esp_err_t err = httpd_register_uri_handler(s_server, &u);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register %s failed: %s", uri, esp_err_to_name(err));
    }
}

}  // namespace

bool start(uint16_t port)
{
    s_snapshot_q = xQueueCreate(4, sizeof(httpd_req_t*));
    xTaskCreatePinnedToCore(snapshot_worker, "bb_snapshot", 8192, nullptr, 4, nullptr, 1);

    httpd_config_t config   = HTTPD_DEFAULT_CONFIG();
    config.server_port      = port;
    config.ctrl_port        = 32769;
    config.max_uri_handlers = 16;
    config.max_open_sockets = 10;
    config.lru_purge_enable = true;
    config.stack_size       = 8192;
    config.uri_match_fn     = httpd_uri_match_wildcard;
    config.send_wait_timeout = 3;
    config.recv_wait_timeout = 5;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return false;
    }

    register_uri("/api/v1/info", HTTP_GET, info_handler);
    register_uri("/api/v1/snapshot", HTTP_GET, snapshot_handler);
    register_uri("/api/v1/stream", HTTP_GET, stream_handler);
    register_uri("/api/v1/head", HTTP_GET, head_get_handler);
    register_uri("/api/v1/head", HTTP_POST, head_post_handler);
    register_uri("/api/v1/head/home", HTTP_POST, head_home_handler);
    register_uri("/api/v1/emotion", HTTP_GET, emotion_get_handler);
    register_uri("/api/v1/emotion", HTTP_POST, emotion_post_handler);
    register_uri("/api/v1/speaking", HTTP_POST, speaking_handler);
#ifdef CONFIG_HTTPD_WS_SUPPORT
    register_uri("/api/v1/ws", HTTP_GET, ws_handler, true);
    esp_timer_create_args_t targs = {};
    targs.callback                = ws_timer_cb;
    targs.name                    = "bb_ws_push";
    esp_timer_create(&targs, &s_ws_timer);
    esp_timer_start_periodic(s_ws_timer, 200 * 1000);
#endif
    register_uri("/*", HTTP_OPTIONS, options_handler);
    httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, not_found_handler);

    ESP_LOGI(TAG, "HTTP API started on port %u", port);
    return true;
}

}  // namespace blublu::http
