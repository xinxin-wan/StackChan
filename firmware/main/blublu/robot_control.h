/*
 * BluBlu peripheral mode: head / emotion / speaking control.
 * All functions are thread safe (they take the LVGL lock internally),
 * except tick() which must be called with the LVGL lock held.
 */
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace blublu::robot {

/* ---------------------------------- Head ---------------------------------- */
// Angles are in 0.1 degree units internally.
struct HeadState {
    int yaw       = 0;
    int pitch     = 0;
    bool moving   = false;
    int yaw_min   = 0;
    int yaw_max   = 0;
    int pitch_min = 0;
    int pitch_max = 0;
};

HeadState get_head();
// Returns the clamped target (0.1 deg). Axis not given keeps its current target.
void move_head(bool has_yaw, int& yaw, bool has_pitch, int& pitch, int speed);
// x/y in [-1, 1]. Returns the computed target (0.1 deg).
void look_at(float x, float y, int speed, int& yaw_out, int& pitch_out);
void go_home(int speed);

/* --------------------------------- Emotion -------------------------------- */
// Returns false when the name is unknown (then neutral is applied).
bool set_emotion(const std::string& name, uint32_t duration_ms, std::string& canonical_out);
std::string get_emotion();
const std::vector<const char*>& supported_emotions();

/* -------------------------------- Speaking -------------------------------- */
// level < 0: use the mouth animation; level 0-100: drive mouth opening directly.
void set_speaking(bool on, int level);
bool is_speaking();

/* ---------------------------------- Text ---------------------------------- */
// Show a speech bubble. duration_ms 0 = keep until replaced.
void show_text(const std::string& text, uint32_t duration_ms);

/* ---------------------------------- State --------------------------------- */
// Conversation state shown on the face: "idle", "listening" or "thinking".
// listening: neutral face + animated "Listening..." bubble.
// thinking:  doubt face + animated "..." bubble.
// Setting an emotion or starting to speak ends the state. Returns false for unknown names.
bool set_state(const std::string& name);

/* ---------------------------------- Tick ---------------------------------- */
// Timeouts (emotion revert, speaking watchdog, text). Call with LVGL lock held.
void tick();

}  // namespace blublu::robot
