#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

#include "bno055_handler.h"
#include "position_tracker.h"
#include "sys_config.h"

static const char *TAG_POS = "PosTracker";

// Position and velocity storage structure
typedef struct
{
    // Position (x, y) in meters
    float position[2];

    // Velocity (vx, vy) in m/s
    float velocity[2];

    // Mutex for data protection
    SemaphoreHandle_t data_mutex;
} pos_data_t;

// Buffer structure for Simpson integration
typedef struct
{
    // Acceleration buffer (4 sample points)
    bno055_vec3_t accel_buffer[4];

    // Corresponding timestamps for each sample (us)
    int64_t time_buffer[4];

    // Current count of points in buffer
    uint8_t count;
} simpson_buffer_t;

// Global variables
static pos_data_t pos_data = {0};
static simpson_buffer_t simpson_buffer = {0};
static bool is_initialized = false;

// Initialize position tracker
esp_err_t position_tracker_init()
{
    if (is_initialized)
    {
        return ESP_OK;
    }

    ESP_LOGI(TAG_POS, "Initializing X-Y position tracker");

    // Create mutex for data protection
    pos_data.data_mutex = xSemaphoreCreateMutex();
    if (pos_data.data_mutex == NULL)
    {
        ESP_LOGE(TAG_POS, "Failed to create mutex");
        return ESP_FAIL;
    }

    // Initialize starting values
    pos_data.position[0] = 0.0f; // X
    pos_data.position[1] = 0.0f; // Y
    pos_data.velocity[0] = 0.0f; // Vx
    pos_data.velocity[1] = 0.0f; // Vy

    // Initialize Simpson buffer
    simpson_buffer.count = 0;

    is_initialized = true;

    ESP_LOGI(TAG_POS, "Position tracker initialized");
    return ESP_OK;
}

// Reset position to (0,0)
void position_tracker_reset()
{
    if (!is_initialized)
    {
        return;
    }

    if (xSemaphoreTake(pos_data.data_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
    {
        pos_data.position[0] = 0.0f;
        pos_data.position[1] = 0.0f;
        pos_data.velocity[0] = 0.0f;
        pos_data.velocity[1] = 0.0f;
        xSemaphoreGive(pos_data.data_mutex);

        // Reset buffer
        simpson_buffer.count = 0;

        ESP_LOGI(TAG_POS, "Position reset to (0,0)");
    }
}

// Get current position
void position_tracker_get_position(float position[2])
{
    if (!is_initialized)
    {
        position[0] = 0.0f;
        position[1] = 0.0f;
        return;
    }

    if (xSemaphoreTake(pos_data.data_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        position[0] = pos_data.position[0];
        position[1] = pos_data.position[1];
        xSemaphoreGive(pos_data.data_mutex);
    }
    else
    {
        position[0] = 0.0f;
        position[1] = 0.0f;
    }
}

// Get current velocity
void position_tracker_get_velocity(float velocity[2])
{
    if (!is_initialized)
    {
        velocity[0] = 0.0f;
        velocity[1] = 0.0f;
        return;
    }

    if (xSemaphoreTake(pos_data.data_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        velocity[0] = pos_data.velocity[0];
        velocity[1] = pos_data.velocity[1];
        xSemaphoreGive(pos_data.data_mutex);
    }
    else
    {
        velocity[0] = 0.0f;
        velocity[1] = 0.0f;
    }
}

// Calculate integral using Simpson's 3/8 rule (for 4 points)
static void simpson_integrate(const simpson_buffer_t *buffer, float result[2])
{
    if (buffer->count != 4)
    {
        result[0] = 0.0f;
        result[1] = 0.0f;
        return;
    }

    // Calculate total time interval
    float total_time = (buffer->time_buffer[3] - buffer->time_buffer[0]) / 1000000.0f;

    // Check time validity
    if (total_time <= 0.0f || total_time > 0.2f)
    {
        result[0] = 0.0f;
        result[1] = 0.0f;
        return;
    }

    // Simpson's 3/8 rule weights: [1, 3, 3, 1]
    float weights[4] = {1.0f, 3.0f, 3.0f, 1.0f};

    // Calculate integral for each axis
    float weighted_sum[2] = {0.0f, 0.0f};

    for (int i = 0; i < 4; i++)
    {
        weighted_sum[0] += weights[i] * buffer->accel_buffer[i].x;
        weighted_sum[1] += weights[i] * buffer->accel_buffer[i].y;
    }

    // Multiply by factor 3h/8
    result[0] = weighted_sum[0] * (total_time / 8.0f);
    result[1] = weighted_sum[1] * (total_time / 8.0f);
}

// Add new acceleration data (called at 100Hz from sensor task)
void position_tracker_add_accel()
{
    if (!is_initialized)
    {
        return;
    }

    // Get current acceleration data from bno055_handler
    bno055_vec3_t accel;
    get_accel(&accel);

    // Current timestamp
    int64_t current_time = esp_timer_get_time();

    // Check motion status
    bool is_moving = get_motion_status();

    if (!is_moving)
    {
        // If not moving, reset velocity to 0
        if (xSemaphoreTake(pos_data.data_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            pos_data.velocity[0] = 0.0f;
            pos_data.velocity[1] = 0.0f;
            xSemaphoreGive(pos_data.data_mutex);
        }

        // Reset buffer
        simpson_buffer.count = 0;
        return;
    }

    // Add to buffer
    if (simpson_buffer.count < 4)
    {
        simpson_buffer.accel_buffer[simpson_buffer.count] = accel;
        simpson_buffer.time_buffer[simpson_buffer.count] = current_time;
        simpson_buffer.count++;
    }
    else
    {
        // If buffer is full, calculate integral to update position

        // 1. Integrate acceleration to get velocity delta
        float delta_velocity[2];
        simpson_integrate(&simpson_buffer, delta_velocity);

        if (xSemaphoreTake(pos_data.data_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            // 2. Update velocity
            pos_data.velocity[0] += delta_velocity[0];
            pos_data.velocity[1] += delta_velocity[1];

            // 3. Calculate position delta: v*t + 1/2*a*t^2
            float time_delta = (current_time - simpson_buffer.time_buffer[0]) / 1000000.0f;

            // Use formula s = v*t + 1/2*a*t^2, with v as updated velocity
            // and a as average acceleration
            float avg_accel_x = (accel.x + simpson_buffer.accel_buffer[3].x) / 2.0f;
            float avg_accel_y = (accel.y + simpson_buffer.accel_buffer[3].y) / 2.0f;

            float delta_pos_x = pos_data.velocity[0] * time_delta + 0.5f * avg_accel_x * time_delta * time_delta;
            float delta_pos_y = pos_data.velocity[1] * time_delta + 0.5f * avg_accel_y * time_delta * time_delta;

            // 4. Update position
            pos_data.position[0] += delta_pos_x;
            pos_data.position[1] += delta_pos_y;

            xSemaphoreGive(pos_data.data_mutex);
        }

        // Reset buffer and store first new point
        simpson_buffer.count = 1;
        simpson_buffer.accel_buffer[0] = accel;
        simpson_buffer.time_buffer[0] = current_time;
    }
}

// Position tracking task (runs at 100Hz)
void position_tracker_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const int UPDATE_RATE_MS = 10; // 100Hz

    ESP_LOGI(TAG_POS, "Position tracking task started");

    int print_counter = 0;

    char json_buffer[512];

    while (1)
    {
        // Add new acceleration data (position will be calculated automatically when 4 points are collected)
        position_tracker_add_accel();

        // Print position periodically (25Hz - every 4 updates)
        if (++print_counter >= 4)
        {
            print_counter = 0;

            float pos[2];
            float vel[2];
            position_tracker_get_position(pos);
            position_tracker_get_velocity(vel);

            snprintf(json_buffer, sizeof(json_buffer),
                     "{"
                     "\"id\":\"%s\","
                     "\"type\":\"state\","
                     "\"data\":{"
                     "\"position\":[%.4f,%.4f],"
                     "\"velocity\":[%.4f,%.4f]"
                     "}"
                     "}\n",
                     ID_ROBOT, pos[0], pos[1], vel[0], vel[1]);
            printf("%s", json_buffer);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(UPDATE_RATE_MS));
    }
}

// Start position tracker
void position_tracker_start()
{
    esp_err_t err = position_tracker_init();

    if (err == ESP_OK)
    {
        static TaskHandle_t tracker_task_handle = NULL;

        xTaskCreatePinnedToCore(
            position_tracker_task,
            "pos_tracker_task",
            4096,
            NULL,
            5, // Lower priority than sensor processing task
            &tracker_task_handle,
            0 // Run on core 0
        );

        ESP_LOGI(TAG_POS, "Position tracker started");
    }
    else
    {
        ESP_LOGE(TAG_POS, "Failed to start position tracker");
    }
}