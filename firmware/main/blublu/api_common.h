/*
 * BluBlu peripheral mode: request handling shared by the USB link and the HTTP API.
 */
#pragma once
#include "robot_control.h"
#include <cJSON.h>

namespace blublu::api {

int json_int(cJSON* root, const char* key, int def);
double deg(int tenths);
int to_tenths(double degrees);

// {yaw,pitch,speed} or {look:{x,y},speed}; with ws_look the root itself is {x,y,speed}.
// Returns false with err set on invalid input. target (may be null) receives the clamped target.
bool apply_head(cJSON* root, bool ws_look, cJSON* target, const char** err);
bool apply_emotion(cJSON* root, cJSON* resp, const char** err);
bool apply_speaking(cJSON* root, const char** err);
void add_head_state(cJSON* obj, const robot::HeadState& s);

// Device / camera / head / emotion description shared by "info" requests.
// The caller adds transport specific fields (links, clients ...).
cJSON* build_info();

}  // namespace blublu::api
