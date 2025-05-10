#ifndef LPF_H
#define LPF_H

#include <stdbool.h>
#include <stdlib.h>
#include "bno055.h"

// Default filter coefficient
#define LPF_DEFAULT_ALPHA 0.2f
typedef struct
{
    // Hệ số bộ lọc IIR bậc 2
    ;
    double a1; // Hệ số cho y[n-1]
    double a2; // Hệ số cho y[n-2]
    double b0; // Hệ số cho x[n]
    double b1; // Hệ số cho x[n-1]
    double b2; // Hệ số cho x[n-2]

    bool initialized; // Cờ báo hiệu bộ lọc đã được khởi tạo chưa

    // Chỉ lưu trữ cho dữ liệu vec3
    bno055_vec3_t prev_inputs[2];  // Giá trị đầu vào trước đó: x[n-1], x[n-2]
    bno055_vec3_t prev_outputs[2]; // Giá trị đầu ra trước đó: y[n-1], y[n-2]

} lpf_filter_t;

bool lpf_init(lpf_filter_t *filter, float alpha, size_t prev_value_size);

void lpf_apply_vec3(lpf_filter_t *filter, bno055_vec3_t *vec);

void lpf_apply_euler(lpf_filter_t *filter, bno055_euler_t *euler);

void lpf_apply_quaternion(lpf_filter_t *filter, bno055_quaternion_t *quat);

bool lpf_set_alpha(lpf_filter_t *filter, float alpha);

void lpf_reset(lpf_filter_t *filter, size_t prev_value_size);

#endif /* LPF_H */