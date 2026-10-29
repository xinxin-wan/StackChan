/*
 * BluBlu expression face: emoji-inspired faces drawn with LVGL primitives over the avatar.
 * All functions must be called with the LVGL lock held.
 */
#pragma once

namespace blublu::emoji {

// neutral, happy, angry, sad, doubt, sleepy, listening, thinking. Unknown names show neutral.
void show(const char* name);
void hide();

}  // namespace blublu::emoji
