/*
 * BluBlu peripheral mode: camera capture + JPEG encoding.
 *
 * One capture task owns the V4L2 device. Snapshots are encoded straight from a
 * dequeued capture buffer; the stream uses a 2x decimated copy.
 */
#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <esp_err.h>

namespace blublu::camera {

struct Jpeg {
    uint8_t* data     = nullptr;
    size_t len        = 0;
    int width         = 0;
    int height        = 0;
    int64_t ts_ms     = 0;  // capture time, ms since boot
    uint32_t seq      = 0;
    uint32_t encode_ms = 0;
    ~Jpeg();
};
using JpegPtr = std::shared_ptr<Jpeg>;

bool start();
bool ready();
int width();         // snapshot width
int height();        // snapshot height
int stream_width();
int stream_height();

// Blocks until a new frame is captured and encoded.
esp_err_t snapshot(int quality, bool fresh, JpegPtr& out, int timeout_ms = 3000);

// Stream clients. Returns a slot id (>= 0) or -1 when all slots are in use.
static constexpr int kMaxStreamClients = 2;
static constexpr int kMaxFps           = 8;  // sensor runs at half clock, see start()
int stream_acquire(int fps, int quality);
void stream_release(int slot);
int stream_clients();
// Waits for a stream frame whose seq differs from last_seq. Returns nullptr on timeout.
JpegPtr stream_wait(uint32_t last_seq, int timeout_ms);

// Debug: write (val >= 0) and/or read a sensor register.
esp_err_t sensor_reg(uint8_t reg, int val, uint8_t* out);

}  // namespace blublu::camera
