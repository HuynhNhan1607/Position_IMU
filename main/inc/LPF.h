#ifndef LPF_H
#define LPF_H

#include <stdbool.h>
#include <stdlib.h>
#include "bno055.h"

// Default filter coefficient
#define LPF_DEFAULT_ALPHA 0.2f

typedef struct
{
    float alpha;      // Filter coefficient (0.0-1.0), lower = more filtering
    bool initialized; // Flag indicating if the filter has been initialized
    void *prev_value; // Previous filtered value
} lpf_filter_t;

bool lpf_init(lpf_filter_t *filter, float alpha, size_t prev_value_size);

void lpf_apply_vec3(lpf_filter_t *filter, bno055_vec3_t *vec);

void lpf_apply_euler(lpf_filter_t *filter, bno055_euler_t *euler);

void lpf_apply_quaternion(lpf_filter_t *filter, bno055_quaternion_t *quat);

bool lpf_set_alpha(lpf_filter_t *filter, float alpha);

void lpf_reset(lpf_filter_t *filter, size_t prev_value_size);

#endif /* LPF_H */