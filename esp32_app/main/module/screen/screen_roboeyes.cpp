#include "screen_roboeyes.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "screen_player.h"

using byte = uint8_t;

extern "C" unsigned long millis(void)
{
    return static_cast<unsigned long>(esp_timer_get_time() / 1000ULL);
}

static long random(long max_value)
{
    if (max_value <= 0) {
        return 0;
    }
    return static_cast<long>(esp_random() % static_cast<uint32_t>(max_value));
}

#ifdef DEFAULT
#undef DEFAULT
#endif
#include <FluxGarage_RoboEyes.h>

namespace {

constexpr uint16_t kScreenW = SCREEN_PLAYER_WIDTH;
constexpr uint16_t kScreenH = SCREEN_PLAYER_HEIGHT;
constexpr uint8_t kRoboEyesFps = 60;
constexpr uint16_t kDirtyPadPx = 5;
constexpr size_t kBytesPerPixel = 2;
constexpr size_t kBackgroundBytes = static_cast<size_t>(kScreenW) * kScreenH * kBytesPerPixel;
constexpr size_t kMaxRegionBytes = 16 * 1024;
constexpr size_t kMaxRegionPixels = kMaxRegionBytes / kBytesPerPixel;

struct DrawBounds {
    int16_t x0 = 0;
    int16_t y0 = 0;
    int16_t x1 = 0;
    int16_t y1 = 0;
    bool valid = false;
};

enum class DrawOpType : uint8_t {
    RoundRect,
    Triangle,
};

struct DrawOp {
    DrawOpType type = DrawOpType::RoundRect;
    int16_t x0 = 0;
    int16_t y0 = 0;
    int16_t x1 = 0;
    int16_t y1 = 0;
    int16_t x2 = 0;
    int16_t y2 = 0;
    uint8_t color = 0;
};

struct ExpressionProfile {
    uint8_t mood = DEFAULT;
    bool curiosity = true;
    bool sweat = false;
    bool auto_blink = true;
    int blink_interval_s = 4;
    int blink_variation_s = 3;
    bool idle = false;
    int idle_interval_s = 3;
    int idle_variation_s = 2;
    uint8_t eye_width = 112;
    uint8_t eye_height = 104;
    uint8_t eye_radius = 28;
    int eye_gap = 32;
    uint8_t position = DEFAULT;
    uint32_t reposition_min_ms = 2300;
    uint32_t reposition_jitter_ms = 1400;
    bool enter_blink = true;
};

void reset_bounds(DrawBounds &bounds);
void reset_draw_ops();
void include_bounds(DrawBounds &bounds, int16_t x0, int16_t y0, int16_t x1, int16_t y1);
DrawBounds expand_and_clip(const DrawBounds &src, int16_t pad);
void record_op(const DrawOp &op);
void background_pixel(int x, int y, int &r, int &g, int &b);
esp_err_t render_region(const DrawBounds &region);

DrawBounds g_prev_bounds;
DrawBounds g_curr_bounds;
DrawOp g_draw_ops[40];
uint8_t g_draw_op_count = 0;
uint8_t *g_background_buffer = nullptr;
uint8_t *g_region_buffer = nullptr;
bool g_ready = false;
bool g_active = false;
uint32_t g_next_reposition_ms = 0;
char g_expression[24] = "";

class RoboEyesQspiAdapter {
public:
    void clearDisplay()
    {
        reset_draw_ops();
        reset_bounds(g_curr_bounds);
    }

    void display()
    {
        DrawBounds current = expand_and_clip(g_curr_bounds, kDirtyPadPx);
        DrawBounds dirty;
        if (g_prev_bounds.valid) {
            include_bounds(dirty, g_prev_bounds.x0, g_prev_bounds.y0, g_prev_bounds.x1, g_prev_bounds.y1);
        }
        if (current.valid) {
            include_bounds(dirty, current.x0, current.y0, current.x1, current.y1);
        }

        if (dirty.valid) {
            const int64_t start_us = esp_timer_get_time();
            esp_err_t ret = render_region(dirty);
            const uint32_t elapsed_us = static_cast<uint32_t>(esp_timer_get_time() - start_us);
            screen_player_report_frame(ret, elapsed_us, "roboeyes_region_failed");
        }

        g_prev_bounds = current;
    }

    void fillRoundRect(int x, int y, int w, int h, int r, uint8_t color)
    {
        if (w <= 0 || h <= 0) {
            return;
        }

        DrawOp op;
        op.type = DrawOpType::RoundRect;
        op.x0 = static_cast<int16_t>(x);
        op.y0 = static_cast<int16_t>(y);
        op.x1 = static_cast<int16_t>(w);
        op.y1 = static_cast<int16_t>(h);
        op.x2 = static_cast<int16_t>(std::max(0, r));
        op.color = color;
        record_op(op);
        include_bounds(g_curr_bounds,
                       static_cast<int16_t>(x),
                       static_cast<int16_t>(y),
                       static_cast<int16_t>(x + w - 1),
                       static_cast<int16_t>(y + h - 1));
    }

    void fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint8_t color)
    {
        DrawOp op;
        op.type = DrawOpType::Triangle;
        op.x0 = static_cast<int16_t>(x0);
        op.y0 = static_cast<int16_t>(y0);
        op.x1 = static_cast<int16_t>(x1);
        op.y1 = static_cast<int16_t>(y1);
        op.x2 = static_cast<int16_t>(x2);
        op.y2 = static_cast<int16_t>(y2);
        op.color = color;
        record_op(op);

        include_bounds(g_curr_bounds,
                       static_cast<int16_t>(std::min(x0, std::min(x1, x2))),
                       static_cast<int16_t>(std::min(y0, std::min(y1, y2))),
                       static_cast<int16_t>(std::max(x0, std::max(x1, x2))),
                       static_cast<int16_t>(std::max(y0, std::max(y1, y2))));
    }
};

RoboEyesQspiAdapter g_adapter;
RoboEyes<RoboEyesQspiAdapter> g_eyes(g_adapter);

inline uint8_t clamp_u8(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 255) {
        return 255;
    }
    return static_cast<uint8_t>(value);
}

inline float clamp01(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

inline float smoothstep01(float value)
{
    const float t = clamp01(value);
    return t * t * (3.0f - 2.0f * t);
}

inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return static_cast<uint16_t>(((r & 0xF8U) << 8) |
                                 ((g & 0xFCU) << 3) |
                                 (b >> 3));
}

inline void put_pixel_be(uint8_t *dst, uint16_t color)
{
    dst[0] = static_cast<uint8_t>(color >> 8);
    dst[1] = static_cast<uint8_t>(color);
}

void reset_bounds(DrawBounds &bounds)
{
    bounds.valid = false;
}

void reset_draw_ops()
{
    g_draw_op_count = 0;
}

void include_bounds(DrawBounds &bounds, int16_t x0, int16_t y0, int16_t x1, int16_t y1)
{
    if (x1 < x0 || y1 < y0) {
        return;
    }

    x0 = std::clamp<int16_t>(x0, 0, kScreenW - 1);
    y0 = std::clamp<int16_t>(y0, 0, kScreenH - 1);
    x1 = std::clamp<int16_t>(x1, 0, kScreenW - 1);
    y1 = std::clamp<int16_t>(y1, 0, kScreenH - 1);

    if (!bounds.valid) {
        bounds.x0 = x0;
        bounds.y0 = y0;
        bounds.x1 = x1;
        bounds.y1 = y1;
        bounds.valid = true;
        return;
    }

    bounds.x0 = std::min(bounds.x0, x0);
    bounds.y0 = std::min(bounds.y0, y0);
    bounds.x1 = std::max(bounds.x1, x1);
    bounds.y1 = std::max(bounds.y1, y1);
}

DrawBounds expand_and_clip(const DrawBounds &src, int16_t pad)
{
    DrawBounds out;
    if (!src.valid) {
        return out;
    }
    include_bounds(out,
                   static_cast<int16_t>(src.x0 - pad),
                   static_cast<int16_t>(src.y0 - pad),
                   static_cast<int16_t>(src.x1 + pad),
                   static_cast<int16_t>(src.y1 + pad));
    return out;
}

void record_op(const DrawOp &op)
{
    if (g_draw_op_count >= sizeof(g_draw_ops) / sizeof(g_draw_ops[0])) {
        return;
    }
    g_draw_ops[g_draw_op_count++] = op;
}

bool ensure_region_buffer(size_t bytes)
{
    if (bytes == 0 || bytes > kMaxRegionBytes) {
        return false;
    }

    if (g_region_buffer != nullptr) {
        return true;
    }

    g_region_buffer = static_cast<uint8_t *>(
        heap_caps_malloc(kMaxRegionBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
    if (g_region_buffer == nullptr) {
        g_region_buffer = static_cast<uint8_t *>(
            heap_caps_malloc(kMaxRegionBytes, MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
    }
    if (g_region_buffer == nullptr) {
        g_region_buffer = static_cast<uint8_t *>(
            heap_caps_malloc(kMaxRegionBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    }
    if (g_region_buffer == nullptr) {
        g_region_buffer = static_cast<uint8_t *>(
            heap_caps_malloc(kMaxRegionBytes, MALLOC_CAP_8BIT));
    }
    return g_region_buffer != nullptr;
}

bool ensure_background_buffer(void)
{
    if (g_background_buffer != nullptr) {
        return true;
    }

    g_background_buffer = static_cast<uint8_t *>(
        heap_caps_malloc(kBackgroundBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (g_background_buffer == nullptr) {
        g_background_buffer = static_cast<uint8_t *>(
            heap_caps_malloc(kBackgroundBytes, MALLOC_CAP_8BIT));
    }
    if (g_background_buffer == nullptr) {
        return false;
    }

    uint8_t *dst = g_background_buffer;
    for (int y = 0; y < kScreenH; ++y) {
        for (int x = 0; x < kScreenW; ++x) {
            int r = 0;
            int g = 0;
            int b = 0;
            background_pixel(x, y, r, g, b);
            put_pixel_be(dst, rgb565(clamp_u8(r), clamp_u8(g), clamp_u8(b)));
            dst += kBytesPerPixel;
        }
    }
    return true;
}

void background_pixel(int x, int y, int &r, int &g, int &b)
{
    const float cx = static_cast<float>(kScreenW) * 0.5f;
    const float cy = static_cast<float>(kScreenH) * 0.5f;
    const float dx = static_cast<float>(x) - cx;
    const float dy = static_cast<float>(y) - cy;
    const float radial = sqrtf((dx * dx) + (dy * dy)) / 184.0f;
    const float edge = smoothstep01((1.02f - radial) / 0.16f);
    const float halo = expf(-radial * radial * 3.2f);
    const float y_norm = static_cast<float>(y) / static_cast<float>(kScreenH - 1);

    r = static_cast<int>((3.0f + halo * 10.0f + (1.0f - y_norm) * 4.0f) * edge);
    g = static_cast<int>((4.0f + halo * 12.0f + (1.0f - y_norm) * 5.0f) * edge);
    b = static_cast<int>((4.0f + halo * 5.0f + y_norm * 3.0f) * edge);
}

uint8_t *region_pixel_ptr(const DrawBounds &region, int y0, int width, int x, int y)
{
    const size_t offset =
        (static_cast<size_t>(y - y0) * static_cast<size_t>(width) +
         static_cast<size_t>(x - region.x0)) *
        kBytesPerPixel;
    return &g_region_buffer[offset];
}

void copy_background_span(const DrawBounds &region, int y0, int width, int x0, int x1, int y)
{
    if (x1 < x0) {
        return;
    }

    uint8_t *dst = region_pixel_ptr(region, y0, width, x0, y);
    const size_t span_bytes = static_cast<size_t>(x1 - x0 + 1) * kBytesPerPixel;
    if (g_background_buffer != nullptr) {
        const size_t bg_offset =
            (static_cast<size_t>(y) * kScreenW + static_cast<size_t>(x0)) * kBytesPerPixel;
        memcpy(dst, &g_background_buffer[bg_offset], span_bytes);
        return;
    }

    for (int x = x0; x <= x1; ++x) {
        int r = 0;
        int g = 0;
        int b = 0;
        background_pixel(x, y, r, g, b);
        put_pixel_be(dst, rgb565(clamp_u8(r), clamp_u8(g), clamp_u8(b)));
        dst += kBytesPerPixel;
    }
}

uint16_t eye_color_for_y(const DrawOp &op, int y)
{
    if (op.type == DrawOpType::Triangle) {
        return rgb565(255, 210, 46);
    }

    const float cy = static_cast<float>(op.y0) + static_cast<float>(op.y1) * 0.5f;
    const float shade = clamp01((static_cast<float>(y) - cy) /
                                std::max(1.0f, static_cast<float>(op.y1)) + 0.5f);
    return rgb565(255,
                  clamp_u8(static_cast<int>(228.0f - shade * 34.0f)),
                  clamp_u8(static_cast<int>(72.0f - shade * 34.0f)));
}

void fill_color_span(const DrawBounds &region, int y0, int width, int x0, int x1, int y, uint16_t color)
{
    if (x1 < x0) {
        return;
    }

    uint8_t *dst = region_pixel_ptr(region, y0, width, x0, y);
    for (int x = x0; x <= x1; ++x) {
        put_pixel_be(dst, color);
        dst += kBytesPerPixel;
    }
}

void draw_op_span(const DrawBounds &region,
                  int y0,
                  int width,
                  const DrawOp &op,
                  int x0,
                  int x1,
                  int y)
{
    x0 = std::max<int>(x0, region.x0);
    x1 = std::min<int>(x1, region.x1);
    if (x1 < x0) {
        return;
    }

    if (op.color != 0) {
        fill_color_span(region, y0, width, x0, x1, y, eye_color_for_y(op, y));
    } else {
        copy_background_span(region, y0, width, x0, x1, y);
    }
}

int rounded_rect_row_inset(int local_y, int height, int radius)
{
    if (radius <= 0 || (local_y >= radius && local_y < height - radius)) {
        return 0;
    }

    float dy = 0.0f;
    if (local_y < radius) {
        dy = static_cast<float>(radius) - (static_cast<float>(local_y) + 0.5f);
    } else {
        dy = (static_cast<float>(local_y) + 0.5f) - static_cast<float>(height - radius);
    }

    const float rr = static_cast<float>(radius * radius);
    const float dx = sqrtf(std::max(0.0f, rr - dy * dy));
    return std::max(0, static_cast<int>(ceilf(static_cast<float>(radius) - dx)));
}

void draw_round_rect_op(const DrawBounds &region, int y0, int rows, int width, const DrawOp &op)
{
    const int rect_w = op.x1;
    const int rect_h = op.y1;
    if (rect_w <= 0 || rect_h <= 0) {
        return;
    }

    const int y_start = std::max<int>(y0, op.y0);
    const int y_end = std::min<int>(y0 + rows - 1, op.y0 + rect_h - 1);
    if (y_end < y_start) {
        return;
    }

    const int radius = std::clamp<int>(op.x2, 0, std::min(rect_w, rect_h) / 2);
    for (int y = y_start; y <= y_end; ++y) {
        const int inset = rounded_rect_row_inset(y - op.y0, rect_h, radius);
        draw_op_span(region,
                     y0,
                     width,
                     op,
                     static_cast<int>(op.x0) + inset,
                     static_cast<int>(op.x0) + rect_w - 1 - inset,
                     y);
    }
}

void add_triangle_intersection(float scan_y,
                               int x0,
                               int y0,
                               int x1,
                               int y1,
                               float *intersections,
                               uint8_t &count)
{
    if (y0 == y1 || count >= 3) {
        return;
    }

    const int min_y = std::min(y0, y1);
    const int max_y = std::max(y0, y1);
    if (scan_y < static_cast<float>(min_y) || scan_y >= static_cast<float>(max_y)) {
        return;
    }

    const float t = (scan_y - static_cast<float>(y0)) / static_cast<float>(y1 - y0);
    intersections[count++] = static_cast<float>(x0) + t * static_cast<float>(x1 - x0);
}

void draw_triangle_op(const DrawBounds &region, int y0, int rows, int width, const DrawOp &op)
{
    const int tri_y0 = std::min<int>(op.y0, std::min<int>(op.y1, op.y2));
    const int tri_y1 = std::max<int>(op.y0, std::max<int>(op.y1, op.y2));
    const int y_start = std::max<int>(y0, tri_y0);
    const int y_end = std::min<int>(y0 + rows - 1, tri_y1);
    if (y_end < y_start) {
        return;
    }

    for (int y = y_start; y <= y_end; ++y) {
        float xs[3] = {};
        uint8_t count = 0;
        const float scan_y = static_cast<float>(y) + 0.5f;
        add_triangle_intersection(scan_y, op.x0, op.y0, op.x1, op.y1, xs, count);
        add_triangle_intersection(scan_y, op.x1, op.y1, op.x2, op.y2, xs, count);
        add_triangle_intersection(scan_y, op.x2, op.y2, op.x0, op.y0, xs, count);
        if (count < 2) {
            continue;
        }

        float min_x = xs[0];
        float max_x = xs[0];
        for (uint8_t i = 1; i < count; ++i) {
            min_x = std::min(min_x, xs[i]);
            max_x = std::max(max_x, xs[i]);
        }

        draw_op_span(region,
                     y0,
                     width,
                     op,
                     static_cast<int>(ceilf(min_x)),
                     static_cast<int>(floorf(max_x)),
                     y);
    }
}

esp_err_t render_region_rows(const DrawBounds &region, int y0, int rows)
{
    const int width = region.x1 - region.x0 + 1;
    const size_t bytes = static_cast<size_t>(width) * static_cast<size_t>(rows) * 2U;
    if (!ensure_region_buffer(bytes)) {
        return ESP_ERR_NO_MEM;
    }

    uint8_t *dst = g_region_buffer;
    for (int y = y0; y < y0 + rows; ++y) {
        if (g_background_buffer != nullptr) {
            const size_t offset =
                (static_cast<size_t>(y) * kScreenW + static_cast<size_t>(region.x0)) * kBytesPerPixel;
            const size_t row_bytes = static_cast<size_t>(width) * kBytesPerPixel;
            memcpy(dst, &g_background_buffer[offset], row_bytes);
            dst += row_bytes;
        } else {
            for (int x = region.x0; x <= region.x1; ++x) {
                int r = 0;
                int g = 0;
                int b = 0;
                background_pixel(x, y, r, g, b);
                put_pixel_be(dst, rgb565(clamp_u8(r), clamp_u8(g), clamp_u8(b)));
                dst += kBytesPerPixel;
            }
        }
    }

    for (uint8_t i = 0; i < g_draw_op_count; ++i) {
        const DrawOp &op = g_draw_ops[i];
        if (op.type == DrawOpType::RoundRect) {
            draw_round_rect_op(region, y0, rows, width, op);
        } else {
            draw_triangle_op(region, y0, rows, width, op);
        }
    }

    return screen_player_draw_rgb565_rect(region.x0, y0, width, rows, g_region_buffer, bytes);
}

esp_err_t render_region(const DrawBounds &region)
{
    if (!region.valid) {
        return ESP_OK;
    }

    const int width = region.x1 - region.x0 + 1;
    const int total_rows = region.y1 - region.y0 + 1;
    int rows_per_chunk = static_cast<int>(kMaxRegionPixels / static_cast<uint16_t>(std::max(1, width)));
    rows_per_chunk = std::clamp(rows_per_chunk, 1, total_rows);

    for (int y = region.y0; y <= region.y1; y += rows_per_chunk) {
        const int rows = std::min(rows_per_chunk, region.y1 - y + 1);
        const esp_err_t ret = render_region_rows(region, y, rows);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

void trim_lower_expression(const char *expression, char *out, size_t out_len)
{
    if (out_len == 0) {
        return;
    }
    const char *src = expression != nullptr && expression[0] != '\0' ? expression : "normal";
    size_t i = 0;
    while (i + 1 < out_len && src[i] != '\0') {
        char c = src[i++];
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c + ('a' - 'A'));
        }
        out[i - 1] = c;
    }
    out[i] = '\0';
}

ExpressionProfile profile_for_expression(const char *expression)
{
    ExpressionProfile p;
    if (std::strcmp(expression, "happy") == 0 || std::strcmp(expression, "glee") == 0) {
        p.mood = HAPPY;
        p.eye_height = 108;
        p.reposition_min_ms = 3400;
        p.enter_blink = false;
        return p;
    }
    if (std::strcmp(expression, "bad") == 0 || std::strcmp(expression, "angry") == 0) {
        p.mood = ANGRY;
        p.eye_height = 92;
        p.eye_radius = 24;
        p.blink_interval_s = 3;
        return p;
    }
    if (std::strcmp(expression, "sleepy") == 0 || std::strcmp(expression, "sad") == 0) {
        p.mood = TIRED;
        p.eye_height = 86;
        p.eye_radius = 22;
        p.blink_interval_s = 3;
        p.curiosity = false;
        return p;
    }
    if (std::strcmp(expression, "curious") == 0 || std::strcmp(expression, "watch") == 0) {
        p.eye_height = 106;
        p.reposition_min_ms = 1600;
        p.reposition_jitter_ms = 900;
        return p;
    }
    return p;
}

void apply_profile(const ExpressionProfile &profile, bool on_enter)
{
    g_eyes.setWidth(profile.eye_width, profile.eye_width);
    g_eyes.setHeight(profile.eye_height, profile.eye_height);
    g_eyes.setBorderradius(profile.eye_radius, profile.eye_radius);
    g_eyes.setSpacebetween(profile.eye_gap);
    g_eyes.setMood(profile.mood);
    g_eyes.setCuriosity(profile.curiosity ? ON : OFF);
    g_eyes.setSweat(profile.sweat ? ON : OFF);
    g_eyes.setAutoblinker(profile.auto_blink ? ON : OFF,
                          profile.blink_interval_s,
                          profile.blink_variation_s);
    g_eyes.setIdleMode(profile.idle ? ON : OFF,
                       profile.idle_interval_s,
                       profile.idle_variation_s);
    g_eyes.setPosition(profile.position);
    g_eyes.open();

    if (on_enter && profile.enter_blink) {
        g_eyes.blink();
    }

    const uint32_t jitter = profile.reposition_jitter_ms == 0 ? 0 : static_cast<uint32_t>(random(profile.reposition_jitter_ms));
    g_next_reposition_ms = millis() + profile.reposition_min_ms + jitter;
}

uint8_t pick_position()
{
    static constexpr uint8_t positions[] = {
        DEFAULT, DEFAULT, DEFAULT, E, W, NE, NW, N,
    };
    return positions[random(static_cast<long>(sizeof(positions)))];
}

void maybe_reposition(void)
{
    const uint32_t now = millis();
    if (g_next_reposition_ms == 0 || now < g_next_reposition_ms) {
        return;
    }

    g_eyes.setPosition(pick_position());
    g_next_reposition_ms = now + 2200U + static_cast<uint32_t>(random(1800));
}

void ensure_ready(void)
{
    if (g_ready) {
        return;
    }

    reset_bounds(g_prev_bounds);
    reset_bounds(g_curr_bounds);
    reset_draw_ops();
    ensure_background_buffer();
    g_eyes.begin(kScreenW, kScreenH, kRoboEyesFps);
    g_eyes.setDisplayColors(0, 255);
    g_eyes.setIdleMode(ON, 2, 2);
    g_eyes.setAutoblinker(ON, 3, 2);
    g_eyes.open();
    g_ready = true;
}

} // namespace

extern "C" esp_err_t screen_roboeyes_begin_expression(const char *expression)
{
    ensure_ready();

    char token[sizeof(g_expression)];
    trim_lower_expression(expression, token, sizeof(token));
    const bool changed = std::strcmp(token, g_expression) != 0;
    if (changed) {
        std::strncpy(g_expression, token, sizeof(g_expression));
        g_expression[sizeof(g_expression) - 1] = '\0';
    }

    apply_profile(profile_for_expression(g_expression), true);
    g_active = true;
    for (int i = 0; i < 8; ++i) {
        g_eyes.drawEyes();
        vTaskDelay(pdMS_TO_TICKS(18));
    }
    return ESP_OK;
}

extern "C" esp_err_t screen_roboeyes_update(void)
{
    if (!g_active) {
        return ESP_OK;
    }

    maybe_reposition();
    g_eyes.update();
    return ESP_OK;
}
