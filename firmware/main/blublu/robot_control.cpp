/*
 * BluBlu peripheral mode: head / emotion / speaking control.
 */
#include "robot_control.h"
#include "emoji_face.h"
#include <hal/hal.h>
#include <stackchan/stackchan.h>
#include <stackchan/motion/motion_math.h>
#include <stackchan/avatar/decorators/decorators.h>
#include <board.h>
#include <display.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <memory>

using namespace stackchan;

namespace blublu::robot {

static const char* TAG = "BluBluRobot";

static constexpr int64_t kSpeakingTimeoutUs = 30LL * 1000 * 1000;
// Fallback in case the host never ends the state
static constexpr int64_t kStateTimeoutUs    = 60LL * 1000 * 1000;

namespace {

struct EmotionAlias {
    const char* name;
    const char* canonical;
};

// Canonical names: the 6 avatar emotions plus shy and speechless. Aliases follow xiaozhi's emotion names.
const EmotionAlias kAliases[] = {
    {"neutral", "neutral"},  {"relaxed", "neutral"}, {"cool", "neutral"},     {"confident", "neutral"},
    {"happy", "happy"},      {"laughing", "happy"},  {"funny", "happy"},      {"loving", "happy"},
    {"winking", "happy"},    {"delicious", "happy"}, {"kissy", "happy"},      {"silly", "happy"},
    {"angry", "angry"},      {"sad", "sad"},         {"crying", "sad"},       {"embarrassed", "shy"},
    {"doubt", "doubt"},      {"doubtful", "doubt"},  {"thinking", "doubt"},   {"confused", "doubt"},
    {"surprised", "doubt"},  {"shocked", "doubt"},   {"sleepy", "sleepy"},    {"shy", "shy"},
    {"speechless", "speechless"}, {"awkward", "speechless"},
};

// Names understood by StackChanAvatarDisplay::SetEmotion
const char* display_name(const std::string& canonical)
{
    if (canonical == "doubt" || canonical == "speechless") {
        return "doubtful";
    }
    if (canonical == "shy") {
        return "happy";
    }
    return canonical.c_str();
}

// State below is protected by the LVGL lock
std::string g_emotion            = "neutral";
int64_t g_emotion_revert_at      = 0;
bool g_speaking                  = false;
int g_speaking_modifier_id       = -1;
int64_t g_speaking_deadline      = 0;
int64_t g_text_clear_at          = 0;
bool g_text_shown                = false;
// Decorators added for the current emotion (anger mark, blush, sweat)
std::vector<int> g_emotion_decorators;

enum class State { Idle, Listening, Thinking };

State g_state             = State::Idle;
int64_t g_state_deadline  = 0;

// Leave listening/thinking, keep the current emotion.
void clear_state_locked()
{
    g_state = State::Idle;
}

// Emoji face only while listening/thinking; otherwise the original avatar (with its blink) shows the emotion.
void refresh_face_locked()
{
    if (g_speaking || g_text_shown || g_state == State::Idle) {
        emoji::hide();
    } else {
        emoji::show(g_state == State::Listening ? "listening" : "thinking");
    }
}

void apply_emotion_locked(const std::string& canonical)
{
    const bool was_sleepy = g_emotion == "sleepy";
    g_emotion             = canonical;
    auto& sc              = GetStackChan();
    if (!sc.hasAvatar()) {
        return;
    }
    auto& av = sc.avatar();
    Board::GetInstance().GetDisplay()->SetEmotion(display_name(canonical));
    // Sleepy shows "Zzz..." in the speech bubble; clear it when leaving sleepy
    if (was_sleepy && canonical != "sleepy" && !g_text_shown) {
        av.clearSpeech();
    }

    for (int id : g_emotion_decorators) {
        av.removeDecorator(id);
    }
    g_emotion_decorators.clear();
    std::unique_ptr<avatar::Decorator> deco;
    if (canonical == "angry") {
        deco = std::make_unique<avatar::AngryDecorator>(lv_screen_active());
    } else if (canonical == "shy") {
        deco = std::make_unique<avatar::ShyDecorator>(lv_screen_active());
    } else if (canonical == "speechless") {
        deco = std::make_unique<avatar::SweatDecorator>(lv_screen_active());
    }
    if (deco) {
        g_emotion_decorators.push_back(av.addDecorator(std::move(deco)));
    }
}

// Stops the mouth animation; after speaking the face goes back to neutral.
void stop_speaking_locked()
{
    auto& sc = GetStackChan();
    if (g_speaking_modifier_id >= 0) {
        sc.removeModifier(g_speaking_modifier_id);
        g_speaking_modifier_id = -1;
    }
    if (sc.hasAvatar()) {
        sc.avatar().mouth().setWeight(0);
    }
    if (g_speaking) {
        g_speaking          = false;
        g_emotion_revert_at = 0;
        apply_emotion_locked("neutral");
    }
}

}  // namespace

/* ---------------------------------- Head ---------------------------------- */
HeadState get_head()
{
    LvglLockGuard lock;
    auto& m = GetStackChan().motion();
    HeadState s;
    s.yaw       = m.getCurrentYawAngle();
    s.pitch     = m.getCurrentPitchAngle();
    s.moving    = m.isMoving();
    auto yl     = m.yawServo().getAngleLimit();
    auto pl     = m.pitchServo().getAngleLimit();
    s.yaw_min   = yl.x;
    s.yaw_max   = yl.y;
    s.pitch_min = pl.x;
    s.pitch_max = pl.y;
    return s;
}

void move_head(bool has_yaw, int& yaw, bool has_pitch, int& pitch, int speed)
{
    LvglLockGuard lock;
    auto& m = GetStackChan().motion();
    if (has_yaw) {
        auto l = m.yawServo().getAngleLimit();
        yaw    = std::clamp(yaw, l.x, l.y);
        m.yawServo().moveWithSpeed(yaw, speed);
    }
    if (has_pitch) {
        auto l = m.pitchServo().getAngleLimit();
        pitch  = std::clamp(pitch, l.x, l.y);
        m.pitchServo().moveWithSpeed(pitch, speed);
    }
}

void look_at(float x, float y, int speed, int& yaw_out, int& pitch_out)
{
    x = std::clamp(x, -1.0f, 1.0f);
    y = std::clamp(y, -1.0f, 1.0f);
    LvglLockGuard lock;
    auto& m     = GetStackChan().motion();
    auto yl     = m.yawServo().getAngleLimit();
    auto pl     = m.pitchServo().getAngleLimit();
    auto angles = motion::calculateNormalizedLookAngles(x, y, yl.x, yl.y, pl.x, pl.y);
    m.moveWithSpeed(angles.yaw, angles.pitch, speed);
    yaw_out   = angles.yaw;
    pitch_out = angles.pitch;
}

void go_home(int speed)
{
    LvglLockGuard lock;
    GetStackChan().motion().goHome(speed);
}

/* --------------------------------- Emotion -------------------------------- */
const std::vector<const char*>& supported_emotions()
{
    static const std::vector<const char*> list = {"neutral", "happy", "angry",      "sad",
                                                  "doubt",   "sleepy", "shy", "speechless"};
    return list;
}

bool set_emotion(const std::string& name, uint32_t duration_ms, std::string& canonical_out)
{
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });

    bool known    = false;
    canonical_out = "neutral";
    for (const auto& a : kAliases) {
        if (lower == a.name) {
            canonical_out = a.canonical;
            known         = true;
            break;
        }
    }

    LvglLockGuard lock;
    clear_state_locked();
    apply_emotion_locked(canonical_out);
    g_emotion_revert_at = duration_ms > 0 ? esp_timer_get_time() + (int64_t)duration_ms * 1000 : 0;
    ESP_LOGI(TAG, "emotion %s -> %s (duration %lu ms)", name.c_str(), canonical_out.c_str(),
             (unsigned long)duration_ms);
    return known;
}

std::string get_emotion()
{
    LvglLockGuard lock;
    return g_emotion;
}

/* -------------------------------- Speaking -------------------------------- */
void set_speaking(bool on, int level)
{
    LvglLockGuard lock;
    auto& sc = GetStackChan();
    if (!on) {
        stop_speaking_locked();
        return;
    }

    clear_state_locked();
    g_speaking          = true;
    g_speaking_deadline = esp_timer_get_time() + kSpeakingTimeoutUs;

    if (level >= 0) {
        // Level mode: drive the mouth directly
        if (g_speaking_modifier_id >= 0) {
            sc.removeModifier(g_speaking_modifier_id);
            g_speaking_modifier_id = -1;
        }
        if (sc.hasAvatar()) {
            sc.avatar().mouth().setWeight(std::clamp(level, 0, 100));
        }
    } else if (g_speaking_modifier_id < 0) {
        // No lifetime: the timeout is handled in tick()
        g_speaking_modifier_id = sc.addModifier(std::make_unique<SpeakingModifier>(0, 180, false));
    }
}

bool is_speaking()
{
    LvglLockGuard lock;
    return g_speaking;
}

/* ---------------------------------- Text ---------------------------------- */
void show_text(const std::string& text, uint32_t duration_ms)
{
    LvglLockGuard lock;
    auto& sc = GetStackChan();
    if (!sc.hasAvatar()) {
        return;
    }
    if (text.empty()) {
        sc.avatar().clearSpeech();
        g_text_shown    = false;
        g_text_clear_at = 0;
        return;
    }
    sc.avatar().setSpeech(text);
    g_text_shown    = true;
    g_text_clear_at = duration_ms > 0 ? esp_timer_get_time() + (int64_t)duration_ms * 1000 : 0;
}

/* ---------------------------------- State --------------------------------- */
bool set_state(const std::string& name)
{
    State state;
    if (name == "idle") {
        state = State::Idle;
    } else if (name == "listening") {
        state = State::Listening;
    } else if (name == "thinking") {
        state = State::Thinking;
    } else {
        return false;
    }

    LvglLockGuard lock;
    if (state == g_state) {
        return true;
    }
    if (state == State::Idle) {
        clear_state_locked();
        g_emotion_revert_at = 0;
        apply_emotion_locked("neutral");
        refresh_face_locked();
        return true;
    }

    stop_speaking_locked();
    g_emotion_revert_at = 0;
    apply_emotion_locked("neutral");
    g_state             = state;
    g_state_deadline    = esp_timer_get_time() + kStateTimeoutUs;
    refresh_face_locked();
    ESP_LOGI(TAG, "state %s", name.c_str());
    return true;
}

/* ---------------------------------- Tick ---------------------------------- */
void tick()
{
    const int64_t now = esp_timer_get_time();

    if (g_state != State::Idle && now >= g_state_deadline) {
        ESP_LOGW(TAG, "state timeout, back to idle");
        clear_state_locked();
        apply_emotion_locked("neutral");
    }

    if (g_emotion_revert_at != 0 && now >= g_emotion_revert_at) {
        g_emotion_revert_at = 0;
        apply_emotion_locked("neutral");
    }

    if (g_speaking && now >= g_speaking_deadline) {
        ESP_LOGW(TAG, "speaking timeout, stop");
        stop_speaking_locked();
    }

    if (g_text_clear_at != 0 && now >= g_text_clear_at) {
        g_text_clear_at = 0;
        g_text_shown    = false;
        if (GetStackChan().hasAvatar()) {
            GetStackChan().avatar().clearSpeech();
        }
    }

    refresh_face_locked();
}

}  // namespace blublu::robot
