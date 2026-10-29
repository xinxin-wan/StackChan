/*
 * BluBlu peripheral mode: camera capture + JPEG encoding.
 */
#include "camera_pipeline.h"
#include <hal/board/hal_bridge.h>
#include <esp_jpeg_common.h>
#include <esp_jpeg_enc.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_check.h>
#include <driver/i2c_master.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <vector>

namespace blublu::camera {

static const char* TAG = "BluBluCam";

Jpeg::~Jpeg()
{
    if (data) {
        heap_caps_free(data);
    }
}

namespace {

struct MmapBuf {
    uint8_t* start = nullptr;
    size_t length  = 0;
};

bool s_ready = false;
int s_fd     = -1;
int s_w = 0, s_h = 0;      // full frame
int s_qw = 0, s_qh = 0;    // stream frame
std::vector<MmapBuf> s_bufs;

std::mutex s_m;
std::condition_variable s_cv_jpeg;
uint32_t s_frame_seq = 0;

// Snapshot (full frame): the capture task lends the dequeued buffer to the
// encoder and requeues it when the encoder is done.
std::mutex s_vga_enc_m;
jpeg_enc_handle_t s_vga_enc = nullptr;
int s_vga_q                 = 80;
std::condition_variable s_cv_hold;
int s_hold_want          = 0;
uint32_t s_hold_target   = 0;
bool s_hold_lent         = false;
bool s_hold_done         = false;
const uint8_t* s_hold_ptr = nullptr;
int64_t s_hold_ts_us     = 0;

// Stream (half size), encoded block by block by the capture task. Each block
// is decimated into internal RAM before it is fed to the encoder.
int64_t s_next_q_due_us  = 0;
int s_stream_fps[kMaxStreamClients] = {0};
int s_stream_clients     = 0;
int s_stream_quality     = 70;
jpeg_enc_handle_t s_q_enc = nullptr;
int s_q_q                = 70;
uint8_t* s_q_block       = nullptr;
int s_q_block_size       = 0;
int s_q_block_rows       = 0;
JpegPtr s_latest;
uint32_t s_jpeg_seq = 0;

// Allocate the JPEG output buffer at its worst-case size; finish_jpeg()
// shrinks it in place once the real size is known.
JpegPtr new_jpeg(size_t max_size)
{
    auto frame  = std::make_shared<Jpeg>();
    frame->data = (uint8_t*)heap_caps_malloc(max_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return frame->data ? frame : nullptr;
}

void finish_jpeg(const JpegPtr& frame, int len)
{
    void* p = heap_caps_realloc(frame->data, len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p) {
        frame->data = (uint8_t*)p;
    }
    frame->len = len;
}

jpeg_enc_handle_t open_encoder(int w, int h, int quality)
{
    jpeg_enc_config_t cfg = DEFAULT_JPEG_ENC_CONFIG();
    cfg.width             = w;
    cfg.height            = h;
    cfg.src_type          = JPEG_PIXEL_FORMAT_YCbYCr;
    cfg.subsampling       = JPEG_SUBSAMPLE_420;
    cfg.quality           = quality;
    jpeg_enc_handle_t h_enc = nullptr;
    jpeg_error_t err        = jpeg_enc_open(&cfg, &h_enc);
    if (err != JPEG_ERR_OK) {
        ESP_LOGE(TAG, "jpeg_enc_open %dx%d failed: %d", w, h, err);
        return nullptr;
    }
    return h_enc;
}

// YUYV 2x decimation of output rows [y0, y0 + rows): keep every other row and
// every other macro pixel pair.
void downsample_yuyv_2x(const uint8_t* src, int w, int y0, int rows, uint8_t* dst)
{
    const int dw = w / 2;
    for (int y = 0; y < rows; y++) {
        const uint32_t* s = (const uint32_t*)(src + (size_t)(2 * (y0 + y)) * w * 2);
        uint32_t* d       = (uint32_t*)(dst + (size_t)y * dw * 2);
        for (int x = 0; x < dw / 2; x++) {
            uint32_t a = s[2 * x];      // Y0 U0 Y1 V0
            uint32_t b = s[2 * x + 1];  // Y2 U1 Y3 V1
            d[x]       = (a & 0xFF00FFFFu) | ((b & 0xFFu) << 16);  // Y0 U0 Y2 V0
        }
    }
}

JpegPtr encode_stream_frame(const uint8_t* src, int quality, int64_t ts)
{
    if (quality != s_q_q) {
        jpeg_enc_set_quality(s_q_enc, quality);
        s_q_q = quality;
    }
    const size_t max_size = (size_t)s_qw * s_qh;
    JpegPtr frame         = new_jpeg(max_size);
    if (!frame) {
        ESP_LOGE(TAG, "stream: out of memory");
        return nullptr;
    }
    int out_len      = 0;
    jpeg_error_t err = JPEG_ERR_FAIL;
    const int64_t t0 = esp_timer_get_time();
    for (int y = 0; y < s_qh; y += s_q_block_rows) {
        downsample_yuyv_2x(src, s_w, y, s_q_block_rows, s_q_block);
        err = jpeg_enc_process_with_block(s_q_enc, s_q_block, s_q_block_size, frame->data, max_size, &out_len);
        if (err < JPEG_ERR_OK) {
            break;
        }
    }
    if (err != JPEG_ERR_OK || out_len <= 0) {
        ESP_LOGE(TAG, "stream encode failed: %d", err);
        // the encoder may be stuck mid-image; start over with a fresh one
        jpeg_enc_close(s_q_enc);
        s_q_enc = open_encoder(s_qw, s_qh, s_q_q);
        return nullptr;
    }
    finish_jpeg(frame, out_len);
    frame->width     = s_qw;
    frame->height    = s_qh;
    frame->ts_ms     = ts / 1000;
    frame->encode_ms = (esp_timer_get_time() - t0) / 1000;
    return frame;
}

int64_t stream_interval_us_locked()
{
    int max_fps = 0;
    for (int f : s_stream_fps) {
        max_fps = std::max(max_fps, f);
    }
    if (max_fps <= 0) {
        max_fps = 10;
    }
    return 1000000LL / max_fps;
}

void capture_task(void*)
{
    const size_t frame_bytes = (size_t)s_w * s_h * 2;
    while (true) {
        struct v4l2_buffer buf = {};
        buf.type               = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory             = V4L2_MEMORY_MMAP;
        if (ioctl(s_fd, VIDIOC_DQBUF, &buf) != 0 || buf.index >= s_bufs.size()) {
            ESP_LOGE(TAG, "VIDIOC_DQBUF failed");
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        const int64_t ts     = esp_timer_get_time();
        const uint8_t* src   = s_bufs[buf.index].start;
        const size_t n       = buf.bytesused ? buf.bytesused : s_bufs[buf.index].length;
        const bool complete  = n >= frame_bytes;
        bool stream_due      = false;
        int stream_quality   = 0;
        bool lend            = false;

        {
            std::lock_guard<std::mutex> lk(s_m);
            s_frame_seq++;

            if (complete && s_stream_clients > 0 && s_q_enc && ts >= s_next_q_due_us) {
                stream_due       = true;
                stream_quality   = s_stream_quality;
                const int64_t iv = stream_interval_us_locked();
                s_next_q_due_us += iv;
                if (s_next_q_due_us < ts - iv) {
                    s_next_q_due_us = ts;
                }
            }

            if (complete && s_hold_want > 0 && !s_hold_lent && s_frame_seq >= s_hold_target) {
                s_hold_lent  = true;
                s_hold_done  = false;
                s_hold_ptr   = src;
                s_hold_ts_us = ts;
                lend         = true;
            }
        }

        if (lend) {
            s_cv_hold.notify_all();
        }

        if (stream_due) {
            JpegPtr frame = encode_stream_frame(src, stream_quality, ts);
            if (frame) {
                {
                    std::lock_guard<std::mutex> lk(s_m);
                    if (s_stream_clients > 0) {
                        frame->seq = ++s_jpeg_seq;
                        s_latest   = frame;
                    }
                }
                s_cv_jpeg.notify_all();
            }
        }

        if (lend) {
            // keep the buffer dequeued until the snapshot encoder is done with it
            std::unique_lock<std::mutex> lk(s_m);
            if (!s_cv_hold.wait_for(lk, std::chrono::milliseconds(2000), [] { return s_hold_done; })) {
                ESP_LOGW(TAG, "snapshot encoder did not return the buffer");
            }
            s_hold_lent = false;
            s_hold_ptr  = nullptr;
        }

        if (ioctl(s_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF failed");
        }
    }
}

}  // namespace

// GC0308 on the CoreS3 internal I2C bus (port 1, address 0x21). The camera
// driver keeps its own handle; a second device handle on the same bus is fine.
esp_err_t sensor_reg(uint8_t reg, int val, uint8_t* out)
{
    static std::mutex m;
    static i2c_master_dev_handle_t dev = nullptr;
    std::lock_guard<std::mutex> lk(m);
    if (!dev) {
        i2c_master_bus_handle_t bus = nullptr;
        ESP_RETURN_ON_ERROR(i2c_master_get_bus_handle((i2c_port_num_t)1, &bus), TAG, "no i2c bus");
        i2c_device_config_t cfg = {};
        cfg.dev_addr_length     = I2C_ADDR_BIT_LEN_7;
        cfg.device_address      = 0x21;
        cfg.scl_speed_hz        = 100000;
        ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &cfg, &dev), TAG, "add i2c device");
    }
    if (val >= 0) {
        uint8_t w[2] = {reg, (uint8_t)val};
        ESP_RETURN_ON_ERROR(i2c_master_transmit(dev, w, 2, 100), TAG, "i2c write");
    }
    return i2c_master_transmit_receive(dev, &reg, 1, out, 1, 100);
}

bool start()
{
    auto cam = hal_bridge::board_get_camera();
    if (!cam || cam->GetVideoFd() < 0 || !cam->IsStreaming()) {
        ESP_LOGE(TAG, "camera not available");
        return false;
    }
    const uint32_t fmt = cam->GetSensorFormat();
    if (fmt != V4L2_PIX_FMT_YUYV && fmt != V4L2_PIX_FMT_YUV422P) {
        ESP_LOGE(TAG, "unsupported sensor format 0x%08lx", (unsigned long)fmt);
        return false;
    }

    s_fd = cam->GetVideoFd();
    s_w  = cam->GetFrameWidth();
    s_h  = cam->GetFrameHeight();
    s_qw = s_w / 2;
    s_qh = s_h / 2;
    for (size_t i = 0; i < cam->GetMmapBufferCount(); i++) {
        size_t len  = 0;
        void* start = cam->GetMmapBuffer(i, &len);
        s_bufs.push_back({(uint8_t*)start, len});
    }
    ESP_LOGI(TAG, "camera %dx%d, %d buffers, stream %dx%d", s_w, s_h, (int)s_bufs.size(), s_qw, s_qh);

    // The DVP DMA writes VGA YUYV into quad PSRAM at the sensor's pixel clock
    // (20 MB/s), which leaves the shared flash/PSRAM bus almost no headroom:
    // JPEG encoding or USB transfers then make it drop bytes, which shows up
    // as colour-shifted (misaligned YUYV) rows. Halving the GC0308 clock
    // (P0:0x28 bits[7:4] = divider - 1) gives ~8 fps of clean frames.
    uint8_t id = 0;
    if (sensor_reg(0xfe, 0x00, &id) == ESP_OK && sensor_reg(0x00, -1, &id) == ESP_OK && id == 0x9b) {
        uint8_t v = 0;
        sensor_reg(0x28, 0x10, &v);
        ESP_LOGI(TAG, "GC0308 clock divider 0x%02x", v);
        // The VGA init table ends with CISCTL_mode1 (P0:0x14) = 0x10, but the
        // driver's mirror/flip read-modify-write runs before that write has
        // latched and stores 0x00. Without bit 4 the Bayer phase is off by one
        // pixel: green is demosaiced as red/blue and every picture is magenta.
        uint8_t m = 0;
        if (sensor_reg(0x14, -1, &m) == ESP_OK) {
            sensor_reg(0x14, (m & 0x03) | 0x10, &v);
            ESP_LOGI(TAG, "GC0308 CISCTL_mode1 0x%02x -> 0x%02x", m, (m & 0x03) | 0x10);
        }
    } else {
        ESP_LOGW(TAG, "sensor id 0x%02x: clock divider not set", id);
    }

    s_vga_enc = open_encoder(s_w, s_h, s_vga_q);
    s_q_enc   = open_encoder(s_qw, s_qh, s_q_q);
    if (!s_vga_enc || !s_q_enc) {
        return false;
    }

    s_q_block_size = jpeg_enc_get_block_size(s_q_enc);
    s_q_block_rows = s_q_block_size / (s_qw * 2);
    if (s_q_block_rows <= 0 || s_q_block_rows * s_qw * 2 != s_q_block_size || s_qh % s_q_block_rows != 0) {
        ESP_LOGE(TAG, "unexpected encoder block size %d", s_q_block_size);
        return false;
    }
    s_q_block = (uint8_t*)heap_caps_aligned_alloc(16, s_q_block_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_q_block) {
        ESP_LOGE(TAG, "buffer alloc failed");
        return false;
    }

    xTaskCreatePinnedToCore(capture_task, "bb_cam", 8192, nullptr, 5, nullptr, 1);
    s_ready = true;
    return true;
}

bool ready()
{
    return s_ready;
}

int width()
{
    return s_w;
}

int height()
{
    return s_h;
}

int stream_width()
{
    return s_qw;
}

int stream_height()
{
    return s_qh;
}

esp_err_t snapshot(int quality, bool fresh, JpegPtr& out, int timeout_ms)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    quality                  = std::clamp(quality, 10, 95);
    const size_t frame_bytes = (size_t)s_w * s_h * 2;

    // one snapshot at a time: the encoder and the lent capture buffer are shared
    std::lock_guard<std::mutex> enc_lock(s_vga_enc_m);

    const uint8_t* input = nullptr;
    int64_t ts_us        = 0;
    {
        std::unique_lock<std::mutex> lk(s_m);
        s_hold_target = s_frame_seq + (fresh ? 2 : 1);
        s_hold_want++;
        const bool ok = s_cv_hold.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                                           [] { return s_hold_lent && !s_hold_done; });
        s_hold_want--;
        if (!ok) {
            return ESP_ERR_TIMEOUT;
        }
        input = s_hold_ptr;
        ts_us = s_hold_ts_us;
    }

    if (quality != s_vga_q) {
        jpeg_enc_set_quality(s_vga_enc, quality);
        s_vga_q = quality;
    }
    const size_t max_size = (size_t)s_w * s_h;
    JpegPtr frame         = new_jpeg(max_size);
    int out_len           = 0;
    jpeg_error_t err      = JPEG_ERR_NO_MEM;
    const int64_t t0      = esp_timer_get_time();
    if (frame) {
        err = jpeg_enc_process(s_vga_enc, (uint8_t*)input, frame_bytes, frame->data, max_size, &out_len);
    }
    const uint32_t enc_ms = (esp_timer_get_time() - t0) / 1000;

    {
        std::lock_guard<std::mutex> lk(s_m);
        s_hold_done = true;
    }
    s_cv_hold.notify_all();

    if (!frame) {
        return ESP_ERR_NO_MEM;
    }
    if (err != JPEG_ERR_OK || out_len <= 0) {
        ESP_LOGE(TAG, "snapshot encode failed: %d", err);
        return ESP_FAIL;
    }
    finish_jpeg(frame, out_len);
    frame->width     = s_w;
    frame->height    = s_h;
    frame->ts_ms     = ts_us / 1000;
    frame->encode_ms = enc_ms;
    out              = frame;
    ESP_LOGI(TAG, "snapshot %dx%d q=%d %d bytes, encode %lu ms", s_w, s_h, quality, out_len, (unsigned long)enc_ms);
    return ESP_OK;
}

int stream_acquire(int fps, int quality)
{
    if (!s_ready) {
        return -1;
    }
    std::lock_guard<std::mutex> lk(s_m);
    if (s_stream_clients >= kMaxStreamClients) {
        return -1;
    }
    for (int i = 0; i < kMaxStreamClients; i++) {
        if (s_stream_fps[i] == 0) {
            s_stream_fps[i] = std::clamp(fps, 1, kMaxFps);
            if (s_stream_clients == 0) {
                s_stream_quality = std::clamp(quality, 10, 95);
                s_next_q_due_us  = 0;
            }
            s_stream_clients++;
            ESP_LOGI(TAG, "stream client +1 (slot %d, fps %d), total %d", i, s_stream_fps[i], s_stream_clients);
            return i;
        }
    }
    return -1;
}

void stream_release(int slot)
{
    if (slot < 0 || slot >= kMaxStreamClients) {
        return;
    }
    std::lock_guard<std::mutex> lk(s_m);
    if (s_stream_fps[slot] == 0) {
        return;
    }
    s_stream_fps[slot] = 0;
    s_stream_clients--;
    if (s_stream_clients == 0) {
        s_latest.reset();
    }
    ESP_LOGI(TAG, "stream client -1 (slot %d), total %d", slot, s_stream_clients);
}

int stream_clients()
{
    std::lock_guard<std::mutex> lk(s_m);
    return s_stream_clients;
}

JpegPtr stream_wait(uint32_t last_seq, int timeout_ms)
{
    std::unique_lock<std::mutex> lk(s_m);
    bool ok = s_cv_jpeg.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                                 [&] { return s_latest && s_latest->seq != last_seq; });
    if (!ok) {
        return nullptr;
    }
    return s_latest;
}

}  // namespace blublu::camera
