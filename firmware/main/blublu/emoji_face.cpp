/*
 * BluBlu expression face: emoji-inspired faces drawn with LVGL primitives
 * (white eyes/mouth on black, plus gestures and effects such as tears or "Z"s).
 */
#include "emoji_face.h"
#include <lvgl.h>
#include <cstdint>
#include <cstring>

namespace blublu::emoji {

namespace {

constexpr uint32_t kWhite = 0xFFFFFF;
constexpr uint32_t kBlack = 0x000000;
constexpr uint32_t kRed   = 0xF44336;
constexpr uint32_t kBlue  = 0x4FC3F7;
constexpr uint32_t kPink  = 0xFF6F91;

constexpr int32_t kW = 320;
constexpr int32_t kH = 240;

// Dot eyes identical to the original avatar: 20 px circles at center +/-70, 16 px above center.
constexpr int32_t kEyeL = 90;
constexpr int32_t kEyeR = 230;
constexpr int32_t kEyeY = 104;
constexpr int32_t kEyeD = 20;

enum class Motion { None, Bob, Sway, Twitch };

using Builder = void (*)(lv_obj_t*);

struct Face {
    const char* name;
    Builder build;
    Motion motion;
    int32_t amplitude;
    uint32_t period_ms;
};

lv_obj_t* g_panel   = nullptr;
lv_obj_t* g_content = nullptr;
const Face* g_current = nullptr;
bool g_visible        = false;

// ---------- primitives ----------

lv_obj_t* blob(lv_obj_t* p, int32_t cx, int32_t cy, int32_t w, int32_t h, uint32_t color,
               lv_opa_t opa = LV_OPA_COVER)
{
    lv_obj_t* o = lv_obj_create(p);
    lv_obj_remove_style_all(o);
    lv_obj_set_size(o, w, h);
    lv_obj_set_pos(o, cx - w / 2, cy - h / 2);
    lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(o, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(o, opa, 0);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_CLICKABLE);
    return o;
}

lv_obj_t* ring(lv_obj_t* p, int32_t cx, int32_t cy, int32_t w, int32_t h, int32_t width, uint32_t color)
{
    lv_obj_t* o = blob(p, cx, cy, w, h, color, LV_OPA_TRANSP);
    lv_obj_set_style_border_color(o, lv_color_hex(color), 0);
    lv_obj_set_style_border_width(o, width, 0);
    lv_obj_set_style_border_opa(o, LV_OPA_COVER, 0);
    return o;
}

// Angles in degrees: 0 = 3 o'clock, clockwise (screen y grows downward).
lv_obj_t* arc(lv_obj_t* p, int32_t cx, int32_t cy, int32_t r, int32_t start, int32_t end, int32_t width,
              uint32_t color, bool rounded = true)
{
    lv_obj_t* a = lv_arc_create(p);
    lv_obj_remove_style_all(a);
    lv_obj_set_size(a, r * 2, r * 2);
    lv_obj_set_pos(a, cx - r, cy - r);
    lv_arc_set_bg_angles(a, start, end);
    lv_obj_set_style_arc_color(a, lv_color_hex(color), LV_PART_MAIN);
    lv_obj_set_style_arc_width(a, width, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(a, rounded, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(a, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(a, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(a, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_remove_flag(a, LV_OBJ_FLAG_CLICKABLE);
    return a;
}

// Points must have static storage: lv_line keeps the pointer.
lv_obj_t* line(lv_obj_t* p, const lv_point_precise_t* pts, uint32_t n, int32_t width, uint32_t color)
{
    lv_obj_t* l = lv_line_create(p);
    lv_line_set_points(l, pts, n);
    lv_obj_set_pos(l, 0, 0);
    lv_obj_set_style_line_width(l, width, 0);
    lv_obj_set_style_line_color(l, lv_color_hex(color), 0);
    lv_obj_set_style_line_opa(l, LV_OPA_COVER, 0);
    lv_obj_set_style_line_rounded(l, true, 0);
    lv_obj_remove_flag(l, LV_OBJ_FLAG_CLICKABLE);
    return l;
}

// ---------- animations ----------

void set_tx(void* o, int32_t v)
{
    lv_obj_set_style_translate_x(static_cast<lv_obj_t*>(o), v, 0);
}

void set_ty(void* o, int32_t v)
{
    lv_obj_set_style_translate_y(static_cast<lv_obj_t*>(o), v, 0);
}

void animate(lv_obj_t* o, lv_anim_exec_xcb_t cb, int32_t from, int32_t to, uint32_t duration,
             uint32_t reverse_ms, uint32_t delay_ms, uint32_t repeat_delay_ms,
             lv_anim_path_cb_t path = lv_anim_path_ease_in_out)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, o);
    lv_anim_set_exec_cb(&a, cb);
    lv_anim_set_values(&a, from, to);
    lv_anim_set_duration(&a, duration);
    if (reverse_ms > 0) {
        lv_anim_set_reverse_duration(&a, reverse_ms);
    }
    lv_anim_set_delay(&a, delay_ms);
    lv_anim_set_repeat_delay(&a, repeat_delay_ms);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_path_cb(&a, path);
    lv_anim_start(&a);
}

// Tear: falls and fades out. v = 0..100
void tear_cb(void* o, int32_t v)
{
    auto* obj = static_cast<lv_obj_t*>(o);
    lv_obj_set_style_translate_y(obj, v * 55 / 100, 0);
    lv_obj_set_style_opa(obj, (lv_opa_t)(v < 70 ? 255 : (100 - v) * 255 / 30), 0);
}

// "Z": drifts up-right, fades in then out. v = 0..100
void z_cb(void* o, int32_t v)
{
    auto* obj = static_cast<lv_obj_t*>(o);
    lv_obj_set_style_translate_x(obj, v * 18 / 100, 0);
    lv_obj_set_style_translate_y(obj, -v * 36 / 100, 0);
    int32_t opa = v < 25 ? v * 255 / 25 : (100 - v) * 255 / 75;
    lv_obj_set_style_opa(obj, (lv_opa_t)opa, 0);
}

// ---------- faces (320x240 canvas) ----------

void dot_eyes(lv_obj_t* p)
{
    blob(p, kEyeL, kEyeY, kEyeD, kEyeD, kWhite);
    blob(p, kEyeR, kEyeY, kEyeD, kEyeD, kWhite);
}

void build_neutral(lv_obj_t* p)
{
    dot_eyes(p);
    arc(p, 160, 120, 50, 40, 140, 8, kWhite);
}

void build_happy(lv_obj_t* p)
{
    dot_eyes(p);
    // open laughing mouth with tongue
    arc(p, 160, 140, 34, 0, 180, 34, kWhite, false);
    blob(p, 160, 164, 30, 14, kPink);
    // blush
    blob(p, kEyeL - 32, 140, 34, 18, kPink, LV_OPA_60);
    blob(p, kEyeR + 32, 140, 34, 18, kPink, LV_OPA_60);
}

void build_angry(lv_obj_t* p)
{
    static const lv_point_precise_t brow_l[] = {{62, 70}, {110, 86}};
    static const lv_point_precise_t brow_r[] = {{258, 70}, {210, 86}};
    line(p, brow_l, 2, 9, kWhite);
    line(p, brow_r, 2, 9, kWhite);
    dot_eyes(p);
    arc(p, 160, 205, 44, 220, 320, 8, kWhite);
    // anger mark
    constexpr int32_t cx = 274, cy = 40, d = 11, r = 9;
    arc(p, cx - d, cy - d, r, 0, 90, 5, kRed);
    arc(p, cx + d, cy - d, r, 90, 180, 5, kRed);
    arc(p, cx + d, cy + d, r, 180, 270, 5, kRed);
    arc(p, cx - d, cy + d, r, 270, 360, 5, kRed);
}

void build_sad(lv_obj_t* p)
{
    static const lv_point_precise_t brow_l[] = {{66, 84}, {110, 70}};
    static const lv_point_precise_t brow_r[] = {{254, 84}, {210, 70}};
    line(p, brow_l, 2, 7, kWhite);
    line(p, brow_r, 2, 7, kWhite);
    dot_eyes(p);
    arc(p, 160, 208, 42, 230, 310, 8, kWhite);
    lv_obj_t* tear = blob(p, kEyeL - 6, 130, 11, 16, kBlue);
    animate(tear, tear_cb, 0, 100, 1500, 0, 0, 400, lv_anim_path_ease_in);
}

// Surprise: the usual dot eyes and a big, tall hollow "O" mouth.
void build_surprise(lv_obj_t* p)
{
    dot_eyes(p);
    ring(p, 160, 172, 48, 60, 8, kWhite);
}

void build_doubt(lv_obj_t* p)
{
    dot_eyes(p);
    static const lv_point_precise_t mouth[] = {{122, 166}, {142, 157}, {162, 165}, {182, 157}, {200, 163}};
    line(p, mouth, 5, 7, kWhite);
    // question mark
    constexpr int32_t qx = 270, qy = 42, qr = 15;
    arc(p, qx, qy, qr, 180, 90, 6, kWhite);
    static const lv_point_precise_t stem[] = {{qx, qy + qr - 3}, {qx, qy + qr + 9}};
    line(p, stem, 2, 6, kWhite);
    blob(p, qx, qy + qr + 22, 9, 9, kWhite);
}

void build_sleepy(lv_obj_t* p)
{
    dot_eyes(p);
    // small "o" mouth
    ring(p, 160, 168, 24, 30, 6, kWhite);
    // Z z z
    static const lv_point_precise_t z_s[] = {{0, 0}, {14, 0}, {0, 14}, {14, 14}};
    static const lv_point_precise_t z_m[] = {{0, 0}, {20, 0}, {0, 20}, {20, 20}};
    static const lv_point_precise_t z_l[] = {{0, 0}, {28, 0}, {0, 28}, {28, 28}};
    struct {
        const lv_point_precise_t* pts;
        int32_t x, y, w;
    } zs[] = {{z_s, 226, 70, 4}, {z_m, 246, 44, 5}, {z_l, 270, 12, 6}};
    uint32_t delay = 0;
    for (auto& z : zs) {
        lv_obj_t* l = line(p, z.pts, 4, z.w, kWhite);
        lv_obj_set_pos(l, z.x, z.y);
        lv_obj_set_style_line_rounded(l, false, 0);
        lv_obj_set_style_opa(l, LV_OPA_TRANSP, 0);
        animate(l, z_cb, 0, 100, 2400, 0, delay, 0, lv_anim_path_linear);
        delay += 800;
    }
}

void blink_cb(void* o, int32_t v)
{
    lv_obj_set_height(static_cast<lv_obj_t*>(o), v);
}

// Capsule outline eye, kept vertically centered so it can blink by shrinking its height.
lv_obj_t* capsule_eye(lv_obj_t* p, int32_t cx, int32_t cy, int32_t w, int32_t h, int32_t border)
{
    lv_obj_t* o = ring(p, cx, cy, w, h, border, kWhite);
    lv_obj_align(o, LV_ALIGN_CENTER, cx - kW / 2, cy - kH / 2);
    animate(o, blink_cb, h, border * 2, 90, 90, 0, 3200);
    return o;
}

// NOMI listening (alternative): cupped hand by the ear plus two capsule outline eyes.
void build_listening_hand(lv_obj_t* p)
{
    // cupped hand behind the ear, opening toward the eyes: palm, four curled fingers, thumb
    lv_obj_t* hand = lv_obj_create(p);
    lv_obj_remove_style_all(hand);
    lv_obj_set_size(hand, 84, 84);
    lv_obj_set_pos(hand, 59, 80);
    lv_obj_remove_flag(hand, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(hand, LV_OBJ_FLAG_CLICKABLE);
    blob(hand, 30, 48, 50, 58, kWhite);
    static const lv_point_precise_t f1[] = {{24, 26}, {42, 9}};
    static const lv_point_precise_t f2[] = {{30, 33}, {55, 15}};
    static const lv_point_precise_t f3[] = {{36, 40}, {64, 27}};
    static const lv_point_precise_t f4[] = {{40, 47}, {67, 41}};
    line(hand, f1, 2, 15, kWhite);
    line(hand, f2, 2, 15, kWhite);
    line(hand, f3, 2, 15, kWhite);
    line(hand, f4, 2, 15, kWhite);
    blob(hand, 64, 57, 18, 12, kBlack);
    static const lv_point_precise_t thumb[] = {{38, 68}, {62, 67}};
    line(hand, thumb, 2, 16, kWhite);
    animate(hand, set_tx, 0, 4, 700, 700, 0, 300);
    capsule_eye(p, 175, 120, 26, 42, 5);
    capsule_eye(p, 239, 120, 26, 42, 5);
}

// Listening: ear icon plus two capsule outline eyes.
void build_listening(lv_obj_t* p)
{
    lv_obj_t* ear = lv_obj_create(p);
    lv_obj_remove_style_all(ear);
    lv_obj_set_size(ear, 57, 69);
    lv_obj_set_pos(ear, 76, 88);
    lv_obj_remove_flag(ear, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(ear, LV_OBJ_FLAG_CLICKABLE);
    // Mirrored hearing-icon ear, tilted 15 degrees to the left.
    arc(ear, 24, 26, 24, 115, 355, 7, kWhite);  // outer rim
    static const lv_point_precise_t rim[] = {{15, 44}, {26, 58}};
    line(ear, rim, 2, 7, kWhite);
    arc(ear, 32, 53, 9, 5, 165, 7, kWhite);  // lobe
    arc(ear, 24, 27, 12, 145, 345, 5, kWhite);  // inner fold
    static const lv_point_precise_t fold[] = {{17, 32}, {25, 40}};
    line(ear, fold, 2, 5, kWhite);
    animate(ear, set_ty, 0, -3, 700, 700, 0, 300);
    capsule_eye(p, 175, 120, 26, 42, 5);
    capsule_eye(p, 239, 120, 26, 42, 5);
}

// Heart: floats up-right from the lips and fades. v = 0..100
void heart_cb(void* o, int32_t v)
{
    auto* obj = static_cast<lv_obj_t*>(o);
    lv_obj_set_style_translate_x(obj, v * 50 / 100, 0);
    lv_obj_set_style_translate_y(obj, -v * 70 / 100, 0);
    int32_t opa = v < 20 ? v * 255 / 20 : (100 - v) * 255 / 80;
    lv_obj_set_style_opa(obj, (lv_opa_t)opa, 0);
}

// Kiss: one eye winking, puckered "3" lips, a heart floating away.
void build_kiss(lv_obj_t* p)
{
    dot_eyes(p);
    arc(p, 158, 150, 11, 270, 90, 7, kWhite);
    arc(p, 158, 172, 11, 270, 90, 7, kWhite);
    blob(p, 58, 145, 40, 22, kPink, LV_OPA_60);
    blob(p, 262, 145, 40, 22, kPink, LV_OPA_60);

    lv_obj_t* heart = lv_obj_create(p);
    lv_obj_remove_style_all(heart);
    lv_obj_set_size(heart, 40, 40);
    lv_obj_set_pos(heart, 176, 132);
    lv_obj_remove_flag(heart, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(heart, LV_OBJ_FLAG_CLICKABLE);
    blob(heart, 10, 10, 20, 20, kRed);
    blob(heart, 26, 10, 20, 20, kRed);
    static const lv_point_precise_t heart_l[] = {{18, 28}, {5, 15}};
    static const lv_point_precise_t heart_r[] = {{18, 28}, {31, 15}};
    line(heart, heart_l, 2, 12, kRed);
    line(heart, heart_r, 2, 12, kRed);
    lv_obj_set_style_opa(heart, LV_OPA_TRANSP, 0);
    animate(heart, heart_cb, 0, 100, 1800, 0, 300, 700, lv_anim_path_ease_out);
}

// Half-disc eye glancing sideways. The clipping box shows one half of the disc:
// v = 0 -> round side to the left (looking left), v = 1 -> round side to the right.
constexpr int32_t kGlanceD     = 26;
constexpr int32_t kGlanceShift = 6;

void glance_cb(void* o, int32_t v)
{
    auto* box    = static_cast<lv_obj_t*>(o);
    auto* disc   = lv_obj_get_child(box, 0);
    int32_t cx   = (int32_t)(intptr_t)lv_obj_get_user_data(box);
    bool right   = v != 0;
    lv_obj_set_x(box, right ? cx + kGlanceShift : cx - kGlanceD / 2 - kGlanceShift);
    lv_obj_set_x(disc, right ? -kGlanceD / 2 : 0);
}

void glance_eye(lv_obj_t* p, int32_t cx, int32_t cy)
{
    lv_obj_t* box = lv_obj_create(p);
    lv_obj_remove_style_all(box);
    lv_obj_set_size(box, kGlanceD / 2, kGlanceD);
    lv_obj_set_y(box, cy - kGlanceD / 2);
    lv_obj_remove_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(box, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_user_data(box, (void*)(intptr_t)cx);
    blob(box, kGlanceD / 2, kGlanceD / 2, kGlanceD, kGlanceD, kWhite);
    glance_cb(box, 0);
    // Step path: hold each side for the whole duration, then flip.
    animate(box, glance_cb, 0, 1, 1200, 1200, 0, 0, lv_anim_path_step);
}

// Thinking: the native neutral face (flat 90x6 mouth) with half-disc eyes looking left then right.
void build_thinking(lv_obj_t* p)
{
    glance_eye(p, kEyeL, kEyeY);
    glance_eye(p, kEyeR, kEyeY);
    lv_obj_t* mouth = blob(p, 160, 146, 90, 6, kWhite);
    lv_obj_set_style_radius(mouth, 0, 0);
}

const Face g_faces[] = {
    {"neutral", build_neutral, Motion::Bob, 3, 1600},
    {"happy", build_happy, Motion::Bob, 8, 450},
    {"angry", build_angry, Motion::Twitch, 4, 70},
    {"sad", build_sad, Motion::Bob, 3, 2000},
    {"surprise", build_surprise, Motion::Bob, 5, 600},
    {"doubt", build_doubt, Motion::Sway, 6, 1200},
    {"sleepy", build_sleepy, Motion::Bob, 3, 2400},
    {"listening", build_listening, Motion::None, 0, 0},
    {"listening_hand", build_listening_hand, Motion::None, 0, 0},
    {"kiss", build_kiss, Motion::Bob, 3, 1200},
    {"thinking", build_thinking, Motion::Bob, 3, 1800},
};

const Face* find(const char* name)
{
    for (const auto& f : g_faces) {
        if (strcmp(f.name, name) == 0) {
            return &f;
        }
    }
    return &g_faces[0];
}

void ensure_panel()
{
    if (g_panel != nullptr) {
        return;
    }
    g_panel = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(g_panel);
    lv_obj_set_size(g_panel, LV_PCT(100), LV_PCT(100));
    lv_obj_center(g_panel);
    lv_obj_set_style_bg_color(g_panel, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(g_panel, LV_OPA_COVER, 0);
    lv_obj_remove_flag(g_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(g_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(g_panel, LV_OBJ_FLAG_HIDDEN);

    g_content = lv_obj_create(g_panel);
    lv_obj_remove_style_all(g_content);
    lv_obj_set_size(g_content, kW, kH);
    lv_obj_center(g_content);
    lv_obj_remove_flag(g_content, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(g_content, LV_OBJ_FLAG_CLICKABLE);
}

void start_motion(const Face& f)
{
    lv_anim_delete(g_content, nullptr);
    lv_obj_set_style_translate_x(g_content, 0, 0);
    lv_obj_set_style_translate_y(g_content, 0, 0);
    if (f.motion == Motion::None) {
        return;
    }
    animate(g_content, f.motion == Motion::Bob ? set_ty : set_tx, -f.amplitude, f.amplitude, f.period_ms,
            f.period_ms, 0, f.motion == Motion::Twitch ? 1200 : 0);
}

}  // namespace

void show(const char* name)
{
    const Face* f = find(name);
    if (g_visible && f == g_current) {
        return;
    }
    ensure_panel();
    if (f != g_current) {
        lv_obj_clean(g_content);  // also deletes the children's animations
        f->build(g_content);
        g_current = f;
    }
    start_motion(*f);
    if (!g_visible) {
        lv_obj_remove_flag(g_panel, LV_OBJ_FLAG_HIDDEN);
        g_visible = true;
    }
}

void hide()
{
    if (g_panel == nullptr || !g_visible) {
        return;
    }
    lv_anim_delete(g_content, nullptr);
    lv_obj_clean(g_content);
    g_current = nullptr;
    lv_obj_add_flag(g_panel, LV_OBJ_FLAG_HIDDEN);
    g_visible = false;
}

}  // namespace blublu::emoji
