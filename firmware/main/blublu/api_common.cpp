/*
 * BluBlu peripheral mode: request handling shared by the USB link and the HTTP API.
 */
#include "api_common.h"
#include "camera_pipeline.h"
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <system_info.h>
#include <algorithm>
#include <cmath>
#include <string>

#ifndef BLUBLU_PERIPHERAL_VERSION
#define BLUBLU_PERIPHERAL_VERSION "0.2.0"
#endif

namespace blublu::api {

int json_int(cJSON* root, const char* key, int def)
{
    cJSON* item = cJSON_GetObjectItemCaseSensitive(root, key);
    // Clamp before the cast: out-of-range double -> int is undefined behaviour
    return cJSON_IsNumber(item) ? (int)std::clamp(item->valuedouble, -1e9, 1e9) : def;
}

double deg(int tenths)
{
    return tenths / 10.0;
}

int to_tenths(double degrees)
{
    return (int)std::lround(std::clamp(degrees, -1000.0, 1000.0) * 10.0);
}

bool apply_head(cJSON* root, bool ws_look, cJSON* target, const char** err)
{
    const int speed = std::clamp(json_int(root, "speed", 500), 0, 1000);

    cJSON* look = ws_look ? root : cJSON_GetObjectItemCaseSensitive(root, "look");
    if (cJSON_IsObject(look)) {
        cJSON* x = cJSON_GetObjectItemCaseSensitive(look, "x");
        cJSON* y = cJSON_GetObjectItemCaseSensitive(look, "y");
        if (!cJSON_IsNumber(x) || !cJSON_IsNumber(y)) {
            *err = "look.x and look.y are required";
            return false;
        }
        int ty = 0, tp = 0;
        robot::look_at((float)x->valuedouble, (float)y->valuedouble, speed, ty, tp);
        if (target) {
            cJSON_AddNumberToObject(target, "yaw", deg(ty));
            cJSON_AddNumberToObject(target, "pitch", deg(tp));
        }
        return true;
    }

    cJSON* yaw   = cJSON_GetObjectItemCaseSensitive(root, "yaw");
    cJSON* pitch = cJSON_GetObjectItemCaseSensitive(root, "pitch");
    const bool has_yaw   = cJSON_IsNumber(yaw);
    const bool has_pitch = cJSON_IsNumber(pitch);
    if (!has_yaw && !has_pitch) {
        *err = "yaw/pitch or look is required";
        return false;
    }
    int ty = has_yaw ? to_tenths(yaw->valuedouble) : 0;
    int tp = has_pitch ? to_tenths(pitch->valuedouble) : 0;
    robot::move_head(has_yaw, ty, has_pitch, tp, speed);
    if (target) {
        if (has_yaw) {
            cJSON_AddNumberToObject(target, "yaw", deg(ty));
        }
        if (has_pitch) {
            cJSON_AddNumberToObject(target, "pitch", deg(tp));
        }
    }
    return true;
}

bool apply_emotion(cJSON* root, cJSON* resp, const char** err)
{
    cJSON* emotion = cJSON_GetObjectItemCaseSensitive(root, "emotion");
    if (!cJSON_IsString(emotion) || !emotion->valuestring) {
        *err = "emotion is required";
        return false;
    }
    int duration = std::max(0, json_int(root, "duration_ms", 0));
    std::string canonical;
    bool known = robot::set_emotion(emotion->valuestring, (uint32_t)duration, canonical);
    if (resp) {
        cJSON_AddStringToObject(resp, "emotion", canonical.c_str());
        if (!known) {
            cJSON_AddStringToObject(resp, "mapped", canonical.c_str());
        }
    }
    return true;
}

bool apply_speaking(cJSON* root, const char** err)
{
    cJSON* on    = cJSON_GetObjectItemCaseSensitive(root, "on");
    cJSON* level = cJSON_GetObjectItemCaseSensitive(root, "level");
    if (!cJSON_IsBool(on) && !cJSON_IsNumber(level)) {
        *err = "on or level is required";
        return false;
    }
    bool is_on = cJSON_IsBool(on) ? cJSON_IsTrue(on) : true;
    int lv     = cJSON_IsNumber(level) ? (int)std::clamp(level->valuedouble, 0.0, 100.0) : -1;
    robot::set_speaking(is_on, lv);
    return true;
}

void add_head_state(cJSON* obj, const robot::HeadState& s)
{
    cJSON_AddNumberToObject(obj, "yaw", deg(s.yaw));
    cJSON_AddNumberToObject(obj, "pitch", deg(s.pitch));
}

cJSON* build_info()
{
    auto head = robot::get_head();

    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddStringToObject(root, "device", "stackchan-k151");
    cJSON_AddStringToObject(root, "firmware",
                            "blublu-peripheral " BLUBLU_PERIPHERAL_VERSION " (stackchan " FIRMWARE_VERSION ")");
    cJSON_AddStringToObject(root, "mac", SystemInfo::GetMacAddress().c_str());

    cJSON* cam = cJSON_AddObjectToObject(root, "camera");
    cJSON_AddStringToObject(cam, "sensor", "GC0308");
    cJSON_AddBoolToObject(cam, "ready", camera::ready());
    int snap[2]   = {camera::width(), camera::height()};
    int stream[2] = {camera::stream_width(), camera::stream_height()};
    cJSON_AddItemToObject(cam, "snapshot", cJSON_CreateIntArray(snap, 2));
    cJSON_AddItemToObject(cam, "stream", cJSON_CreateIntArray(stream, 2));
    cJSON_AddNumberToObject(cam, "max_fps", camera::kMaxFps);
    cJSON_AddNumberToObject(cam, "stream_clients", camera::stream_clients());

    cJSON* h     = cJSON_AddObjectToObject(root, "head");
    double yl[2] = {deg(head.yaw_min), deg(head.yaw_max)};
    double pl[2] = {deg(head.pitch_min), deg(head.pitch_max)};
    cJSON_AddItemToObject(h, "yaw", cJSON_CreateDoubleArray(yl, 2));
    cJSON_AddItemToObject(h, "pitch", cJSON_CreateDoubleArray(pl, 2));

    const auto& emotions = robot::supported_emotions();
    cJSON_AddItemToObject(root, "emotions", cJSON_CreateStringArray(emotions.data(), emotions.size()));

    cJSON_AddNumberToObject(root, "uptime_ms", (double)(esp_timer_get_time() / 1000));
    static const char* const kResetReasons[] = {"unknown", "poweron",  "ext",      "sw",       "panic",
                                                "int_wdt", "task_wdt", "wdt",      "deepsleep", "brownout",
                                                "sdio",    "usb",      "jtag",     "efuse",     "pwr_glitch",
                                                "cpu_lockup"};
    const int reason = esp_reset_reason();
    cJSON_AddStringToObject(root, "reset_reason",
                            reason >= 0 && reason < (int)(sizeof(kResetReasons) / sizeof(kResetReasons[0]))
                                ? kResetReasons[reason]
                                : "unknown");
    cJSON* heap = cJSON_AddObjectToObject(root, "heap");
    cJSON_AddNumberToObject(heap, "internal", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    cJSON_AddNumberToObject(heap, "psram", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    return root;
}

}  // namespace blublu::api
