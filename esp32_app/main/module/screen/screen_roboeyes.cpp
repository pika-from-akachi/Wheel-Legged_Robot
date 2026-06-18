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
constexpr uint8_t kRoboEyesFps = 35;
constexpr uint32_t kStartupFrameIntervalMs = 40;
constexpr uint32_t kStartupTotalMs = 4200;
constexpr uint32_t kLogoFadeMs = 900;
constexpr uint32_t kRevealStartMs = 220;
constexpr uint32_t kRevealMs = 1700;
constexpr uint32_t kSweepStartMs = 900;
constexpr uint32_t kSweepEndMs = 2800;
constexpr uint16_t kDirtyPadPx = 5;
constexpr uint16_t kMaxRegionPixels = kScreenW * 120;

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
esp_err_t render_region(const DrawBounds &region);

DrawBounds g_prev_bounds;
DrawBounds g_curr_bounds;
DrawOp g_draw_ops[40];
uint8_t g_draw_op_count = 0;
uint8_t *g_region_buffer = nullptr;
uint16_t *g_startup_line = nullptr;
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

inline uint16_t swap16(uint16_t value)
{
    return static_cast<uint16_t>((value >> 8) | (value << 8));
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
    if (bytes == 0) {
        return false;
    }

    if (g_region_buffer != nullptr) {
        return true;
    }

    g_region_buffer = static_cast<uint8_t *>(
        heap_caps_malloc(kMaxRegionPixels * 2U, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (g_region_buffer == nullptr) {
        g_region_buffer = static_cast<uint8_t *>(
            heap_caps_malloc(kMaxRegionPixels * 2U, MALLOC_CAP_8BIT));
    }
    return g_region_buffer != nullptr;
}

float rounded_rect_distance(float px, float py, float hw, float hh, float radius)
{
    const float r = std::max(0.5f, std::min(radius, std::min(hw, hh) - 0.5f));
    const float qx = fabsf(px) - (hw - r);
    const float qy = fabsf(py) - (hh - r);
    const float ox = std::max(qx, 0.0f);
    const float oy = std::max(qy, 0.0f);
    return sqrtf((ox * ox) + (oy * oy)) + std::min(std::max(qx, qy), 0.0f) - r;
}

int triangle_edge(int x0, int y0, int x1, int y1, int px, int py)
{
    return (px - x0) * (y1 - y0) - (py - y0) * (x1 - x0);
}

bool point_in_triangle(int px, int py, const DrawOp &op)
{
    const int e0 = triangle_edge(op.x0, op.y0, op.x1, op.y1, px, py);
    const int e1 = triangle_edge(op.x1, op.y1, op.x2, op.y2, px, py);
    const int e2 = triangle_edge(op.x2, op.y2, op.x0, op.y0, px, py);
    return (e0 >= 0 && e1 >= 0 && e2 >= 0) || (e0 <= 0 && e1 <= 0 && e2 <= 0);
}

void blend_to(int &r, int &g, int &b, int tr, int tg, int tb, int alpha)
{
    if (alpha <= 0) {
        return;
    }
    if (alpha > 255) {
        alpha = 255;
    }
    r += ((tr - r) * alpha) / 255;
    g += ((tg - g) * alpha) / 255;
    b += ((tb - b) * alpha) / 255;
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

void render_roboeyes_pixel(int x, int y, int &r, int &g, int &b)
{
    background_pixel(x, y, r, g, b);

    for (uint8_t i = 0; i < g_draw_op_count; ++i) {
        const DrawOp &op = g_draw_ops[i];
        const bool is_eye = op.color != 0;

        if (op.type == DrawOpType::RoundRect) {
            if (x < op.x0 - 12 || x >= op.x0 + op.x1 + 12 ||
                y < op.y0 - 12 || y >= op.y0 + op.y1 + 12) {
                continue;
            }

            const float cx = static_cast<float>(op.x0) + static_cast<float>(op.x1) * 0.5f;
            const float cy = static_cast<float>(op.y0) + static_cast<float>(op.y1) * 0.5f;
            const float dist = rounded_rect_distance(static_cast<float>(x) + 0.5f - cx,
                                                     static_cast<float>(y) + 0.5f - cy,
                                                     static_cast<float>(op.x1) * 0.5f,
                                                     static_cast<float>(op.y1) * 0.5f,
                                                     static_cast<float>(op.x2));
            if (is_eye) {
                if (dist <= 10.0f) {
                    const float glow_q = clamp01((10.0f - dist) / 10.0f);
                    blend_to(r, g, b, 255, 176, 38, static_cast<int>(glow_q * glow_q * 115.0f));
                }
                if (dist <= 0.0f) {
                    const float shade = clamp01((static_cast<float>(y) - cy) /
                                                std::max(1.0f, static_cast<float>(op.y1)) + 0.5f);
                    blend_to(r,
                             g,
                             b,
                             255,
                             static_cast<int>(228.0f - shade * 34.0f),
                             static_cast<int>(72.0f - shade * 34.0f),
                             dist > -2.0f ? 255 : 230);
                }
            } else if (dist <= 0.0f) {
                background_pixel(x, y, r, g, b);
            }
            continue;
        }

        if (point_in_triangle(x, y, op)) {
            if (is_eye) {
                blend_to(r, g, b, 255, 210, 46, 235);
            } else {
                background_pixel(x, y, r, g, b);
            }
        }
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
        for (int x = region.x0; x <= region.x1; ++x) {
            int r = 0;
            int g = 0;
            int b = 0;
            render_roboeyes_pixel(x, y, r, g, b);
            put_pixel_be(dst, rgb565(clamp_u8(r), clamp_u8(g), clamp_u8(b)));
            dst += 2;
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

bool ensure_startup_line(void)
{
    if (g_startup_line != nullptr) {
        return true;
    }
    g_startup_line = static_cast<uint16_t *>(
        heap_caps_malloc(kScreenW * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT));
    if (g_startup_line == nullptr) {
        g_startup_line = static_cast<uint16_t *>(heap_caps_malloc(kScreenW * sizeof(uint16_t), MALLOC_CAP_8BIT));
    }
    return g_startup_line != nullptr;
}

void render_startup_pixel(uint32_t elapsed_ms, int x, int y, int &r, int &g, int &b)
{
    const int center_x = kScreenW / 2;
    const int center_y = kScreenH / 2;
    const float dx = static_cast<float>(x - center_x);
    const float dy = static_cast<float>(y - center_y);
    const float max_radius = sqrtf(static_cast<float>(center_x * center_x + center_y * center_y));
    const float bg_fade = smoothstep01(static_cast<float>(elapsed_ms) / static_cast<float>(kLogoFadeMs));
    const float radial = sqrtf((dx * dx) + (dy * dy)) / max_radius;
    const float vignette = clamp01(1.0f - radial);
    const float y_norm = static_cast<float>(y) / static_cast<float>(kScreenH - 1);

    r = static_cast<int>((4.0f + 12.0f * vignette + 2.0f * (1.0f - y_norm)) * bg_fade);
    g = static_cast<int>((6.0f + 17.0f * vignette + 2.0f * (1.0f - y_norm)) * bg_fade);
    b = static_cast<int>((5.0f + 7.0f * vignette + 2.0f * y_norm) * bg_fade);

    float reveal_t = (static_cast<float>(elapsed_ms) - static_cast<float>(kRevealStartMs)) /
                     static_cast<float>(kRevealMs);
    reveal_t = smoothstep01(reveal_t);
    const float reveal_radius = reveal_t * max_radius;
    const float reveal_inner = std::max(0.0f, reveal_radius - 18.0f);
    const float reveal_outer = reveal_radius + 18.0f;
    const float d = sqrtf((dx * dx) + (dy * dy));

    float ring_alpha = 0.0f;
    if (d >= reveal_inner && d <= reveal_outer && reveal_outer > reveal_inner) {
        ring_alpha = 1.0f - fabsf(d - reveal_radius) / 18.0f;
    }

    float core_alpha = 0.0f;
    const float core_x = dx / 88.0f;
    const float core_y = dy / 42.0f;
    const float core = (core_x * core_x) + (core_y * core_y);
    if (core < 1.0f && d < reveal_radius) {
        core_alpha = smoothstep01((1.0f - core) / 0.42f) * bg_fade;
    }

    float sweep_gain = 0.0f;
    if (elapsed_ms >= kSweepStartMs && elapsed_ms <= kSweepEndMs) {
        const float sweep_t = static_cast<float>(elapsed_ms - kSweepStartMs) /
                              static_cast<float>(kSweepEndMs - kSweepStartMs);
        const float sweep_pos = -60.0f + sweep_t * (static_cast<float>(kScreenW + kScreenH) + 120.0f);
        const float line_dist = fabsf(static_cast<float>(x + y) - sweep_pos);
        if (line_dist < 24.0f) {
            const float q = 1.0f - line_dist / 24.0f;
            sweep_gain = q * q;
        }
    }

    blend_to(r, g, b, 255, 183, 42, static_cast<int>(ring_alpha * 160.0f));
    blend_to(r, g, b, 255, 214, 64, static_cast<int>(core_alpha * 210.0f));
    blend_to(r, g, b, 255, 232, 130, static_cast<int>(sweep_gain * 150.0f));
}

esp_err_t render_startup_frame(uint32_t elapsed_ms)
{
    if (!ensure_startup_line()) {
        return ESP_ERR_NO_MEM;
    }

    const int64_t start_us = esp_timer_get_time();
    for (int y = 0; y < kScreenH; ++y) {
        for (int x = 0; x < kScreenW; ++x) {
            int r = 0;
            int g = 0;
            int b = 0;
            render_startup_pixel(elapsed_ms, x, y, r, g, b);
            g_startup_line[x] = swap16(rgb565(clamp_u8(r), clamp_u8(g), clamp_u8(b)));
        }

        const esp_err_t ret = screen_player_draw_rgb565_rect(0,
                                                             y,
                                                             kScreenW,
                                                             1,
                                                             g_startup_line,
                                                             kScreenW * sizeof(uint16_t));
        if (ret != ESP_OK) {
            screen_player_report_frame(ret,
                                       static_cast<uint32_t>(esp_timer_get_time() - start_us),
                                       "startup_frame_failed");
            return ret;
        }
    }

    screen_player_report_frame(ESP_OK,
                               static_cast<uint32_t>(esp_timer_get_time() - start_us),
                               nullptr);
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
    g_eyes.begin(kScreenW, kScreenH, kRoboEyesFps);
    g_eyes.setDisplayColors(0, 255);
    g_eyes.open();
    g_ready = true;
}

} // namespace

extern "C" esp_err_t screen_roboeyes_play_startup(void)
{
    const uint32_t begin_ms = millis();
    uint32_t last_frame_ms = 0;

    while (true) {
        const uint32_t now_ms = millis();
        uint32_t elapsed_ms = now_ms - begin_ms;
        if (elapsed_ms > kStartupTotalMs) {
            elapsed_ms = kStartupTotalMs;
        }

        if (last_frame_ms == 0 || now_ms - last_frame_ms >= kStartupFrameIntervalMs ||
            elapsed_ms >= kStartupTotalMs) {
            const esp_err_t ret = render_startup_frame(elapsed_ms);
            if (ret != ESP_OK) {
                return ret;
            }
            last_frame_ms = now_ms;
        }

        if (elapsed_ms >= kStartupTotalMs) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    return ESP_OK;
}

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
    g_eyes.drawEyes();
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
