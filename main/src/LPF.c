#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "bno055.h"
#include "esp_log.h"

#define LPF_DEFAULT_ALPHA 0.2f // Default filter coefficient

static const char *TAG_LPF = "LPF";

typedef struct
{
    float alpha;      // Filter coefficient (0.0-1.0), lower = more filtering
    bool initialized; // Flag indicating if the filter has been initialized
    void *prev_value; // Previous filtered value
} lpf_filter_t;

bool lpf_init(lpf_filter_t *filter, float alpha, size_t prev_value_size)
{
    if (!filter || prev_value_size == 0)
    {
        ESP_LOGE(TAG_LPF, "Invalid filter parameters");
        return false;
    }

    filter->alpha = (alpha > 0.0f && alpha <= 1.0f) ? alpha : LPF_DEFAULT_ALPHA;
    filter->initialized = false;

    // Allocate memory for previous value
    filter->prev_value = malloc(prev_value_size);
    if (!filter->prev_value)
    {
        ESP_LOGE(TAG_LPF, "Failed to allocate memory for filter");
        return false;
    }

    // Initialize memory to zero
    memset(filter->prev_value, 0, prev_value_size);

    return true;
}

void lpf_apply_vec3(lpf_filter_t *filter, bno055_vec3_t *vec)
{
    if (!filter || !filter->prev_value || !vec)
    {
        return;
    }

    bno055_vec3_t *prev = (bno055_vec3_t *)filter->prev_value;

    if (!filter->initialized)
    {
        *prev = *vec;
        filter->initialized = true;
        return;
    }

    // Apply filter to each component
    prev->x = filter->alpha * vec->x + (1.0f - filter->alpha) * prev->x;
    prev->y = filter->alpha * vec->y + (1.0f - filter->alpha) * prev->y;
    prev->z = filter->alpha * vec->z + (1.0f - filter->alpha) * prev->z;

    // Copy filtered values back
    *vec = *prev;
}

void lpf_apply_euler(lpf_filter_t *filter, bno055_euler_t *euler)
{
    if (!filter || !filter->prev_value || !euler)
    {
        return;
    }

    bno055_euler_t *prev = (bno055_euler_t *)filter->prev_value;

    if (!filter->initialized)
    {
        *prev = *euler;
        filter->initialized = true;
        return;
    }

    // Apply filter to each angle
    prev->heading = filter->alpha * euler->heading + (1.0f - filter->alpha) * prev->heading;
    prev->roll = filter->alpha * euler->roll + (1.0f - filter->alpha) * prev->roll;
    prev->pitch = filter->alpha * euler->pitch + (1.0f - filter->alpha) * prev->pitch;

    // Copy filtered values back
    *euler = *prev;
}

void lpf_apply_quaternion(lpf_filter_t *filter, bno055_quaternion_t *quat)
{
    if (!filter || !filter->prev_value || !quat)
    {
        return;
    }

    bno055_quaternion_t *prev = (bno055_quaternion_t *)filter->prev_value;

    if (!filter->initialized)
    {
        *prev = *quat;
        filter->initialized = true;
        return;
    }

    // Apply filter to each quaternion component
    prev->w = filter->alpha * quat->w + (1.0f - filter->alpha) * prev->w;
    prev->x = filter->alpha * quat->x + (1.0f - filter->alpha) * prev->x;
    prev->y = filter->alpha * quat->y + (1.0f - filter->alpha) * prev->y;
    prev->z = filter->alpha * quat->z + (1.0f - filter->alpha) * prev->z;

    // Copy filtered values back
    *quat = *prev;
}

bool lpf_set_alpha(lpf_filter_t *filter, float alpha)
{
    if (!filter || alpha <= 0.0f || alpha > 1.0f)
    {
        return false;
    }
    filter->alpha = alpha;
    return true;
}

void lpf_reset(lpf_filter_t *filter, size_t prev_value_size)
{
    if (filter && filter->prev_value)
    {
        memset(filter->prev_value, 0, prev_value_size);
        filter->initialized = false;
    }
}