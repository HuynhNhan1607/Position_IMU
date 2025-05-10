#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "bno055.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lpf.h"

static const char *TAG_LPF = "LPF";

#define LPF_DEFAULT_ALPHA 0.2f

bool lpf_init(lpf_filter_t *filter, float alpha, size_t prev_value_size)
{
    if (!filter)
    {
        ESP_LOGE(TAG_LPF, "Filter pointer is NULL");
        return false;
    }

    // Thiết lập hệ số cố định cho bộ lọc IIR
    filter->a1 = 1.142981f;
    filter->a2 = -0.412802f;
    filter->b0 = 0.067455f;
    filter->b1 = 0.134911f;
    filter->b2 = 0.067455f;

    // Khởi tạo cờ trước
    filter->initialized = false;

    // Khởi tạo từng trường riêng biệt thay vì dùng memset
    // Tránh dùng memset để ngăn truy cập bộ nhớ không hợp lệ
    filter->prev_inputs[0].x = 0.0f;
    filter->prev_inputs[0].y = 0.0f;
    filter->prev_inputs[0].z = 0.0f;

    filter->prev_inputs[1].x = 0.0f;
    filter->prev_inputs[1].y = 0.0f;
    filter->prev_inputs[1].z = 0.0f;

    filter->prev_outputs[0].x = 0.0f;
    filter->prev_outputs[0].y = 0.0f;
    filter->prev_outputs[0].z = 0.0f;

    filter->prev_outputs[1].x = 0.0f;
    filter->prev_outputs[1].y = 0.0f;
    filter->prev_outputs[1].z = 0.0f;

    // Đợi thêm một chút nữa để ổn định

    return true;
}

void lpf_apply_vec3(lpf_filter_t *filter, bno055_vec3_t *vec)
{
    if (!filter || !vec)
    {
        return;
    }

    if (!filter->initialized)
    {
        // Khởi tạo giá trị lịch sử
        filter->prev_inputs[0] = filter->prev_inputs[1] = *vec;
        filter->prev_outputs[0] = filter->prev_outputs[1] = *vec;
        filter->initialized = true;
        return;
    }

    // Lưu trữ tạm thời cho giá trị đã lọc
    bno055_vec3_t filtered;

    // Áp dụng công thức bộ lọc IIR bậc 2 cho từng thành phần
    // y[n] = a1*y[n-1] + a2*y[n-2] + b0*x[n] + b1*x[n-1] + b2*x[n-2]

    // Trục X
    filtered.x = filter->a1 * filter->prev_outputs[0].x +
                 filter->a2 * filter->prev_outputs[1].x +
                 filter->b0 * vec->x +
                 filter->b1 * filter->prev_inputs[0].x +
                 filter->b2 * filter->prev_inputs[1].x;

    // Trục Y
    filtered.y = filter->a1 * filter->prev_outputs[0].y +
                 filter->a2 * filter->prev_outputs[1].y +
                 filter->b0 * vec->y +
                 filter->b1 * filter->prev_inputs[0].y +
                 filter->b2 * filter->prev_inputs[1].y;

    // Trục Z
    filtered.z = filter->a1 * filter->prev_outputs[0].z +
                 filter->a2 * filter->prev_outputs[1].z +
                 filter->b0 * vec->z +
                 filter->b1 * filter->prev_inputs[0].z +
                 filter->b2 * filter->prev_inputs[1].z;

    // Cập nhật lịch sử
    filter->prev_inputs[1] = filter->prev_inputs[0]; // x[n-2] = x[n-1]
    filter->prev_inputs[0] = *vec;                   // x[n-1] = x[n]

    filter->prev_outputs[1] = filter->prev_outputs[0]; // y[n-2] = y[n-1]
    filter->prev_outputs[0] = filtered;                // y[n-1] = y[n]

    // Gán giá trị đã lọc vào đầu ra
    *vec = filtered;
}

bool lpf_set_alpha(lpf_filter_t *filter, float alpha)
{
    // Hàm này không còn cần thiết vì chúng ta đang sử dụng hệ số cố định
    // Giữ lại để tránh lỗi khi biên dịch các code đang gọi hàm này
    return true;
}

void lpf_reset(lpf_filter_t *filter, size_t prev_value_size)
{
    if (filter)
    {
        memset(filter->prev_inputs, 0, sizeof(filter->prev_inputs));
        memset(filter->prev_outputs, 0, sizeof(filter->prev_outputs));
        filter->initialized = false;
    }
}