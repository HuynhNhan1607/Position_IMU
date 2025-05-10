#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "bno055.h"
#include "bno055_handler.h"
#include "LPF.h"
#include "nvs_handler.h"
#include "sys_config.h"
#include "string.h"
#include "uart_handler.h"
#include <math.h>

#define BNO_MODE OPERATION_MODE_IMUPLUS

#define BNO055_CALIBRATED_BIT BIT0

#define MOTION_THRESHOLD 4.5f  // Ngưỡng phát hiện chuyển động từ gyro
#define MOTION_HYSTERESIS 0.3f // Độ trễ để tránh chuyển trạng thái liên tục

#define ACCEL_THRESHOLD 0.02f

static const char *TAG_IMU = "BNO055_Handler";

/*--------------------------------------------------
 * Task Handles
 *------------------------------------------------*/
TaskHandle_t blink_led_task_handle = NULL;
TaskHandle_t ndof_task_handle = NULL;
TaskHandle_t calib_task_handle = NULL;
TaskHandle_t offset_update_task_handle = NULL;
/*--------------------------------------------------
 * Sensor Data
 *------------------------------------------------*/
bno055_euler_t euler = {0.0f, 0.0f, 0.0f};           // Orientation (heading, roll, pitch)
bno055_quaternion_t quat = {0.0f, 0.0f, 0.0f, 0.0f}; // Quaternion representation
bno055_vec3_t lin_accel = {0.0f, 0.0f, 0.0f};        // Linear acceleration
bno055_vec3_t lin_accel_forOffset = {0.0f, 0.0f, 0.0f};

bno055_vec3_t gravity = {0.0f, 0.0f, 0.0f}; // Gravity vector

/*--------------------------------------------------
 * Configuration
 *------------------------------------------------*/
static bno055_config_t bno_conf;         // BNO055 configuration
static i2c_number_t i2c_num = 0;         // I2C bus number
static gpio_num_t led_gpio = GPIO_NUM_2; // LED GPIO pin

/*--------------------------------------------------
 * Calibration & State
 *------------------------------------------------*/
static bool calibration_complete = false;               // Calibration status flag
static float yaw_offset = 0.0f;                         // Heading reference offset
static bool apply_yaw_offset = false;                   // Whether to apply yaw offset
float adjusted_heading = 0.0f;                          // Adjusted heading value
static bno055_vec3_t accel_offset = {0.0f, 0.0f, 0.0f}; // Acceleration reference offset
static float accel_std_dev[3] = {0.0f, 0.0f, 0.0f};     // Standard deviation of acceleration
static volatile bool need_inheritance = false;          // Cờ báo hiệu cần thực hiện kế thừa 10%
static bool motion_detected = false;                    // Biến toàn cục lưu trạng thái chuyển động
/*--------------------------------------------------
 * Synchronization & Events
 *------------------------------------------------*/
static SemaphoreHandle_t heading_mutex = NULL; // Mutex for heading access
static SemaphoreHandle_t accel_data_mutex = NULL;
static SemaphoreHandle_t offset_mutex = NULL;

EventGroupHandle_t bno055_event_group = NULL; // Event group for state flags

/*--------------------------------------------------
 * Filters
 *------------------------------------------------*/
static lpf_filter_t lpf_gyro;  // Low-pass filter for gyroscope
static lpf_filter_t lpf_accel; // Low-pass filter for acceleration

/*--------------------------------------------------
 * Cờ và biến trạng thái
 *------------------------------------------------*/
static volatile bool should_update_offset = false; // Cờ điều khiển việc cập nhật offset
static volatile bool offset_task_running = false;  // Cờ kiểm tra task offset đang chạy

// Biến cho thuật toán Welford liên tục
static int welford_n = 0;                        // Số mẫu hiện tại
static float accel_mean[3] = {0.0f, 0.0f, 0.0f}; // Giá trị trung bình hiện tại
static float accel_M2[3] = {0.0f, 0.0f, 0.0f};   // Giá trị M2 của thuật toán Welford

esp_err_t bno055_get_accel_euler(i2c_number_t i2c_num, bno055_vec3_t *accel, bno055_euler_t *euler)
{
    esp_err_t err = bno055_get_euler(i2c_num, euler);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_IMU, "bno055_get_euler() returned error: %02x", err);
        return err;
    }

    err = bno055_get_lin_accel(i2c_num, accel);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_IMU, "bno055_get_lin_accel() returned error: %02x", err);
        return err;
    }
    return ESP_OK;
}

float get_heading()
{
    float result = 0.0f;
    if (heading_mutex != NULL && xSemaphoreTake(heading_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
    {
        result = adjusted_heading;
        xSemaphoreGive(heading_mutex);
    }
    else
    {
        ESP_LOGW(TAG_IMU, "get_heading() - Failed mutex");
    }
    return result;
}

// Add this function after get_heading()
void get_accel_for_offset(bno055_vec3_t *accel)
{
    if (offset_mutex != NULL && xSemaphoreTake(offset_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        accel->x = lin_accel_forOffset.x;
        accel->y = lin_accel_forOffset.y; // Ma trận xoay 180 độ
        accel->z = lin_accel_forOffset.z;
        xSemaphoreGive(offset_mutex);
    }
    else
    {
        ESP_LOGW(TAG_IMU, "get_heading() - Failed mutex");
    }
}

void get_accel(bno055_vec3_t *accel)
{
    if (accel_data_mutex != NULL && xSemaphoreTake(accel_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        accel->x = lin_accel.x;
        accel->y = lin_accel.y; // Ma trận xoay 180 độ
        accel->z = lin_accel.z;
        xSemaphoreGive(accel_data_mutex);
    }
    else
    {
        ESP_LOGW(TAG_IMU, "get_heading() - Failed mutex");
    }
}

bool get_motion_status()
{
    return motion_detected;
}

static void apply_accel_threshold(bno055_vec3_t *vec, float threshold)
{
    if (fabsf(vec->x) < threshold)
        vec->x = 0.0f;
    if (fabsf(vec->y) < threshold)
        vec->y = 0.0f;
    if (fabsf(vec->z) < threshold)
        vec->z = 0.0f;
}

void blink_led_task(void *pvParameters)
{
    gpio_num_t led_gpio = *(gpio_num_t *)pvParameters;
    gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(led_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(led_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void handle_sensor_error(i2c_number_t i2c_num, esp_err_t err_code)
{
    TaskHandle_t reinit_task_handle = NULL;

    ESP_LOGE(TAG_IMU, "BNO055 sensor error: %02x", err_code);

    // Đóng kết nối với cảm biến
    esp_err_t err = bno055_close(i2c_num);
    ESP_LOGW(TAG_IMU, "bno055_close() returned 0x%02X", err);

    // Tạo task reinit_sensor để khởi tạo lại cảm biến
    xTaskCreatePinnedToCore(
        reinit_sensor,
        "reinit_sensor",
        2048,
        NULL,
        10,
        &reinit_task_handle,
        1);

    if (ndof_task_handle != NULL)
    {
        vTaskSuspend(ndof_task_handle);
    }
}

void send_calibration_notification(calib_status_t status)
{
    char calib_json[256];
    int len = snprintf(calib_json, sizeof(calib_json),
                       "{"
                       "\"id\":\"%s\","
                       "\"type\":\"bno055\","
                       "\"data\":{"
                       "\"event\":\"calibration_complete\","
                       "\"status\":{\"sys\":%d,\"gyro\":%d,\"accel\":%d,\"mag\":%d}"
                       "}"
                       "}\n",
                       ID_ROBOT, status.sys, status.gyro, status.accel, status.mag);

    if (len >= sizeof(calib_json))
    {
        ESP_LOGE(TAG_IMU, "Calibration JSON buffer overflow");
        return;
    }

    // Thay thế gửi qua socket bằng in ra màn hình
    printf("Calibration notification: %s\n", calib_json);
}

void bno055_set_reference(void)
{
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay đủ lâu để scheduler hoạt động
    ESP_LOGI(TAG_IMU, "Setting reference point using Welford's algorithm...");

    esp_err_t err;
    const int SAMPLE_COUNT = 300; // Giảm kích thước mẫu
    const float STD_DEV_THRESHOLD_HEADING = 1.0f;
    const float STD_DEV_THRESHOLD_ACCEL = 0.05f;

    // Biến cho thuật toán Welford cho heading (vẫn dùng biến cục bộ)
    int heading_n = 0;
    float heading_mean = 0.0f, heading_M2 = 0.0f;

    // Reset biến Welford toàn cục dành cho acceleration
    welford_n = 0;
    accel_mean[0] = accel_mean[1] = accel_mean[2] = 0.0f;
    accel_M2[0] = accel_M2[1] = accel_M2[2] = 0.0f;

    ESP_LOGI(TAG_IMU, "Collecting %d samples for reference calculation...", SAMPLE_COUNT);

    // Thu thập mẫu và cập nhật thống kê theo thời gian thực
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        if (i % 10 == 0)
        {
            ESP_LOGI(TAG_IMU, "Collecting sample %d/%d...", i, SAMPLE_COUNT);
        }

        // Đọc dữ liệu từ cảm biến

        err = bno055_get_accel_euler(i2c_num, &lin_accel, &euler);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_IMU, "Error reading sensor data: 0x%02X", err);
            continue;
        }

        heading_n++;
        welford_n++;

        float heading_delta = euler.heading - heading_mean;
        heading_mean += heading_delta / heading_n;
        float heading_delta2 = euler.heading - heading_mean;
        heading_M2 += heading_delta * heading_delta2;

        // Cập nhật thống kê cho acceleration sử dụng biến toàn cục
        // Trục X
        float delta_x = lin_accel.x - accel_mean[0];
        accel_mean[0] += delta_x / welford_n;
        float delta2_x = lin_accel.x - accel_mean[0];
        accel_M2[0] += delta_x * delta2_x;

        // Trục Y
        float delta_y = lin_accel.y - accel_mean[1];
        accel_mean[1] += delta_y / welford_n;
        float delta2_y = lin_accel.y - accel_mean[1];
        accel_M2[1] += delta_y * delta2_y;

        // Trục Z
        float delta_z = lin_accel.z - accel_mean[2];
        accel_mean[2] += delta_z / welford_n;
        float delta2_z = lin_accel.z - accel_mean[2];
        accel_M2[2] += delta_z * delta2_z;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Tính độ lệch chuẩn cuối cùng
    float heading_std_dev = sqrtf(heading_M2 / heading_n);
    accel_std_dev[0] = sqrtf(accel_M2[0] / welford_n);
    accel_std_dev[1] = sqrtf(accel_M2[1] / welford_n);
    accel_std_dev[2] = sqrtf(accel_M2[2] / welford_n);

    ESP_LOGI(TAG_IMU, "Stats - Heading: mean=%.2f, std_dev=%.4f",
             heading_mean, heading_std_dev);
    ESP_LOGI(TAG_IMU, "Stats - Accel: mean=[%.4f, %.4f, %.4f], std_dev=[%.4f, %.4f, %.4f]",
             accel_mean[0], accel_mean[1], accel_mean[2],
             accel_std_dev[0], accel_std_dev[1], accel_std_dev[2]);

    // Xác định xem giá trị có đủ ổn định để thiết lập tham chiếu
    bool heading_stable = (heading_std_dev < STD_DEV_THRESHOLD_HEADING);
    bool accel_stable = (accel_std_dev[0] < STD_DEV_THRESHOLD_ACCEL &&
                         accel_std_dev[1] < STD_DEV_THRESHOLD_ACCEL &&
                         accel_std_dev[2] < STD_DEV_THRESHOLD_ACCEL);

    // Thiết lập giá trị tham chiếu
    if (heading_stable)
    {
        yaw_offset = heading_mean;
        apply_yaw_offset = true;
        ESP_LOGI(TAG_IMU, "Heading is stable (std_dev=%.4f), reference set: %.2f",
                 heading_std_dev, yaw_offset);
    }
    else
    {
        ESP_LOGW(TAG_IMU, "Heading not stable (std_dev=%.4f > threshold=%.4f)",
                 heading_std_dev, STD_DEV_THRESHOLD_HEADING);
        yaw_offset = euler.heading;
        apply_yaw_offset = true;
    }

    if (accel_stable)
    {
        accel_offset.x = accel_mean[0];
        accel_offset.y = accel_mean[1];
        accel_offset.z = accel_mean[2];
        ESP_LOGI(TAG_IMU, "Acceleration stable, reference: [%.4f, %.4f, %.4f]",
                 accel_offset.x, accel_offset.y, accel_offset.z);
        need_inheritance = true;
    }
    else
    {
        ESP_LOGW(TAG_IMU, "Acceleration not stable enough");
        accel_offset.x = lin_accel.x;
        accel_offset.y = lin_accel.y;
        accel_offset.z = lin_accel.z;
    }

    ESP_LOGI(TAG_IMU, "Reference point setting complete");
}

float apply_heading_offset(float raw_heading)
{
    if (!apply_yaw_offset)
    {
        return raw_heading;
    }
    float adjusted = raw_heading - yaw_offset;
    while (adjusted > 180.0f)
        adjusted -= 360.0f;
    while (adjusted < -180.0f)
        adjusted += 360.0f;

    return adjusted;
}

#define SCALE_ACCEL_OFFSET 0.2f
static void apply_accel_offset(bno055_vec3_t *vec)
{
    vec->x = vec->x - (SCALE_ACCEL_OFFSET * accel_offset.x);
    vec->y = vec->y - (SCALE_ACCEL_OFFSET * accel_offset.y);
    vec->z = vec->z - (SCALE_ACCEL_OFFSET * accel_offset.z);
}

void reinit_sensor(void *pvParameters)
{
    esp_err_t err;

    // Tạo task blink_led khi sensor lỗi
    if (blink_led_task_handle == NULL)
    {
        xTaskCreatePinnedToCore(blink_led_task,
                                "blink_led_task",
                                1024,
                                &led_gpio,
                                5,
                                &blink_led_task_handle,
                                0);
    }

    vTaskDelay(pdMS_TO_TICKS(REINIT_TIME));

    while (1)
    {
        err = bno055_open(i2c_num, &bno_conf, BNO_MODE);
        ESP_LOGI(TAG_IMU, "bno055_open() returned 0x%02X", err);
        if (err == ESP_OK)
        {

            if (blink_led_task_handle != NULL)
            {
                vTaskDelete(blink_led_task_handle);
                gpio_set_level(led_gpio, 0);
                blink_led_task_handle = NULL;
            }
            if (ndof_task_handle != NULL)
            {
                vTaskResume(ndof_task_handle);
            }
            vTaskDelete(NULL);
            break;
        }
        else
        {
            // Khởi tạo thất bại, thử lại sau 20 giây
            ESP_LOGW(TAG_IMU, "Failed to open BNO055, retrying......");
            vTaskDelay(pdMS_TO_TICKS(REINIT_TIME));
        }
    }
}
// Thêm task mới cho calibration
void calibration_task(void *pvParameters)
{
    esp_err_t err;
    calib_status_t calib_status;
    bno055_offsets_t offsets;
    bool was_calibrated = false;

    ESP_LOGI(TAG_IMU, "Calibration task started");

    // Khởi tạo LED và đảm bảo nó tắt
    gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(led_gpio, 0);

    // Lặp kiểm tra trạng thái hiệu chuẩn
    while (!was_calibrated)
    {
        // Kiểm tra xem cảm biến đã được hiệu chuẩn đầy đủ chưa
        if (bno055_is_fully_calibrated(i2c_num, &calib_status, BNO_MODE))
        {
            was_calibrated = true;

            ESP_LOGI(TAG_IMU, "Calib - Sys: %d, Gyro: %d, Accel: %d, Mag: %d",
                     calib_status.sys, calib_status.gyro,
                     calib_status.accel, calib_status.mag);

            // Đọc giá trị offset
            err = bno055_get_offsets(i2c_num, &offsets);
            if (err == ESP_OK)
            {
                ESP_LOGI(TAG_IMU, "Accel offset: %d %d %d    Magnet: %d %d %d    Gyro: %d %d %d Acc_Radius: %d    Mag_Radius: %d",
                         offsets.accel_offset_x, offsets.accel_offset_y, offsets.accel_offset_z,
                         offsets.mag_offset_x, offsets.mag_offset_y, offsets.mag_offset_z,
                         offsets.gyro_offset_x, offsets.gyro_offset_y, offsets.gyro_offset_z,
                         offsets.accel_radius, offsets.mag_radius);
            }

            // Lưu dữ liệu hiệu chuẩn vào NVS
            err = nvs_save_bno055_calibration(&offsets);
            if (err == ESP_OK)
            {
                ESP_LOGI(TAG_IMU, "Calibration data saved successfully");
            }
            else
            {
                ESP_LOGE(TAG_IMU, "Failed to save calibration data: %d", err);
            }
        }

        // Đợi một khoảng thời gian trước khi kiểm tra lại
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Chuẩn bị sẵn sàng trước khi đo

    bno055_set_reference();

    xTaskCreatePinnedToCore(ndof_task,
                            "ndof_task",
                            4096,
                            NULL,
                            10,
                            &ndof_task_handle,
                            0);

    if (offset_update_task_handle == NULL)
    {
        xTaskCreatePinnedToCore(
            dynamic_offset_update_task,
            "offset_update",
            4096,
            NULL,
            8, // Ưu tiên thấp hơn ndof_task nhưng cao hơn các task nền
            &offset_update_task_handle,
            0);

        // Tạm dừng task cho đến khi cần thiết
        if (offset_update_task_handle != NULL)
        {
            vTaskSuspend(offset_update_task_handle);
        }
    }
    // Task hoàn thành và xóa chính nó

    ESP_LOGI(TAG_IMU, "Calibration task complete");
    // Bật LED chỉ thị
    gpio_set_level(led_gpio, 1);
    // Gửi thông báo hiệu chuẩn hoàn tất qua màn hình
    send_calibration_notification(calib_status);

    if (bno055_event_group != NULL)
    {
        xEventGroupSetBits(bno055_event_group, BNO055_CALIBRATED_BIT);
        ESP_LOGI(TAG_IMU, "BNO055 calibration complete bit set");
    }
    calib_task_handle = NULL;
    vTaskDelete(NULL);
}

bool is_motion_detected(bno055_vec3_t *gyro)
{
    static bool previous_state = false;
    static int stable_count = 0;
    static bool offset_task_is_suspended = true; // Track task state
    const int STABLE_THRESHOLD = 3;              // Số lần đọc ổn định để thay đổi trạng thái

    // Tính độ lớn của vector gyro
    float magnitude = sqrtf(gyro->x * gyro->x + gyro->y * gyro->y + gyro->z * gyro->z);

    float current_threshold = previous_state ? (MOTION_THRESHOLD - MOTION_HYSTERESIS) : (MOTION_THRESHOLD + MOTION_HYSTERESIS);

    bool current_state = (magnitude > current_threshold);

    if (current_state == previous_state)
    {
        stable_count = 0;
    }
    else
    {
        stable_count++;
        if (stable_count >= STABLE_THRESHOLD)
        {
            // Phát hiện trạng thái chuyển từ di chuyển sang đứng yên
            bool moving_to_stationary = (previous_state == true && current_state == false);

            previous_state = current_state;
            stable_count = 0;
            ESP_LOGI(TAG_IMU, "Status: %s (mag: %.2f)",
                     current_state ? "Moving" : "Stationary", magnitude);

            // Quản lý task offset dựa vào trạng thái di chuyển
            if (offset_update_task_handle != NULL)
            {
                if (moving_to_stationary && offset_task_is_suspended)
                {
                    // Chỉ resume nếu task đang suspended
                    ESP_LOGI(TAG_IMU, "Resuming offset task (transitioned to stationary)");
                    vTaskResume(offset_update_task_handle);
                    offset_task_is_suspended = false;
                    need_inheritance = true;
                }
                else if (current_state && !offset_task_is_suspended)
                {
                    // Chỉ suspend nếu task đang running
                    ESP_LOGI(TAG_IMU, "Suspending offset task (transitioned to moving)");
                    vTaskSuspend(offset_update_task_handle);
                    offset_task_is_suspended = true;
                }
            }
        }
    }

    motion_detected = previous_state;

    return previous_state;
}
/**
 * Task cập nhật offset gia tốc động khi thiết bị đứng yên
 * Sử dụng thuật toán Welford để tính thống kê trực tuyến liên tục
 */
/**
 * Task for dynamic acceleration offset update when device is stationary
 * Uses Welford's algorithm for continuous online statistics
 */
void dynamic_offset_update_task(void *pvParameters)
{
    ESP_LOGI(TAG_IMU, "Dynamic offset update task started");

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    const float STD_DEV_THRESHOLD = 0.1f;  // Standard deviation threshold
    const int MIN_SAMPLES_FOR_UPDATE = 30; // Minimum samples required to update offset
    const int STD_DEV_CALC_INTERVAL = 50;  // Calculate std dev every 50 samples
    const float INHERITANCE_FACTOR = 0.1f; // Keep 10% of previous Welford statistics

    bno055_vec3_t lin_accel_copy;          // Local copy of acceleration data
    float std_dev[3] = {0.0f, 0.0f, 0.0f}; // Store standard deviation values

    offset_task_running = true;

    // Task starts in suspended state
    ESP_LOGI(TAG_IMU, "Dynamic offset update task starting in suspended state");

    while (1)
    {
        if (need_inheritance)
        {
            need_inheritance = false; // Reset cờ

            // LUÔN kế thừa 10% mẫu từ lần chạy trước
            int keep_samples = (int)(welford_n * INHERITANCE_FACTOR);
            if (keep_samples < 1)
                keep_samples = 1; // Giữ ít nhất 1 mẫu

            ESP_LOGI(TAG_IMU, "Inheriting %d samples (10%%) from previous calculation. Mean: [%.4f, %.4f, %.4f]",
                     keep_samples, accel_mean[0], accel_mean[1], accel_mean[2]);

            welford_n = keep_samples;

            accel_M2[0] *= INHERITANCE_FACTOR;
            accel_M2[1] *= INHERITANCE_FACTOR;
            accel_M2[2] *= INHERITANCE_FACTOR;
        }

        get_accel_for_offset(&lin_accel_copy);

        // Increment sample count
        welford_n++;

        // Update statistics for each axis using Welford's algorithm
        // X-axis
        float delta_x = lin_accel_copy.x - accel_mean[0];
        accel_mean[0] += delta_x / welford_n;
        float delta2_x = lin_accel_copy.x - accel_mean[0];
        accel_M2[0] += delta_x * delta2_x;

        // Y-axis
        float delta_y = lin_accel_copy.y - accel_mean[1];
        accel_mean[1] += delta_y / welford_n;
        float delta2_y = lin_accel_copy.y - accel_mean[1];
        accel_M2[1] += delta_y * delta2_y;

        // Z-axis
        float delta_z = lin_accel_copy.z - accel_mean[2];
        accel_mean[2] += delta_z / welford_n;
        float delta2_z = lin_accel_copy.z - accel_mean[2];
        accel_M2[2] += delta_z * delta2_z;

        // Calculate standard deviation only every 50 samples or on first sample
        bool calc_std_dev = (welford_n % STD_DEV_CALC_INTERVAL == 0 || welford_n == 1);

        if (calc_std_dev)
        {
            // Calculate standard deviation
            std_dev[0] = (welford_n > 1) ? sqrtf(accel_M2[0] / welford_n) : 0.0f;
            std_dev[1] = (welford_n > 1) ? sqrtf(accel_M2[1] / welford_n) : 0.0f;
            std_dev[2] = (welford_n > 1) ? sqrtf(accel_M2[2] / welford_n) : 0.0f;

            // Log periodic information
            ESP_LOGD(TAG_IMU, "Collected %d samples, mean: [%.4f, %.4f, %.4f], std_dev: [%.4f, %.4f, %.4f]",
                     welford_n, accel_mean[0], accel_mean[1], accel_mean[2],
                     std_dev[0], std_dev[1], std_dev[2]);

            // // Check if values are stable enough and have collected enough samples
            // bool accel_stable = (std_dev[0] < STD_DEV_THRESHOLD &&
            //                      std_dev[1] < STD_DEV_THRESHOLD &&
            //                      std_dev[2] < STD_DEV_THRESHOLD);

            // Update offset if acceleration is stable and we have enough samples
            if ((welford_n >= MIN_SAMPLES_FOR_UPDATE))
            {
                accel_offset.x = accel_mean[0];
                accel_offset.y = accel_mean[1];
                accel_offset.z = accel_mean[2];

                ESP_LOGD(TAG_IMU, "Acceleration stable after %d samples, offset updated: [%.4f, %.4f, %.4f]",
                         welford_n, accel_offset.x, accel_offset.y, accel_offset.z);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(BNO_POLLING_MS));
    }
}

/*--------------------------------------------------
 * MAIN TASK
 *--------------------------------------------------*/

void ndof_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    esp_err_t err;
    int64_t time_mks, time_mks_after;
    int time_bno;

    // Biến cục bộ để lưu giá trị tạm thời
    bno055_vec3_t local_lin_accel = {0.0f, 0.0f, 0.0f};
    bno055_euler_t local_euler = {0.0f, 0.0f, 0.0f};
    bno055_vec3_t local_gyro_raw = {0.0f, 0.0f, 0.0f}; // Chỉ dùng local, không cần global
    float local_adjusted_heading = 0.0f;
    bool is_moving = false;

    char json_buffer[255];
    int cycle_counter = 0;

    if (bno055_event_group != NULL)
    {
        xEventGroupSetBits(bno055_event_group, BNO055_TASK_RUNNING_BIT);
        ESP_LOGI(TAG_IMU, "ndof_task is now running");
    }

    while (1)
    {
        time_mks = esp_timer_get_time();

        // Đọc dữ liệu vào biến cục bộ
        err = bno055_get_accel_euler(i2c_num, &local_lin_accel, &local_euler);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_IMU, "bno055_get_accel_euler() returned error: %02x", err);
            handle_sensor_error(i2c_num, err);
            taskYIELD();
            continue;
        }

        // Đọc gyro vào biến cục bộ
        err = bno055_get_gyro_raw(i2c_num, &local_gyro_raw);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG_IMU, "bno055_get_gyro_raw() returned error: %02x", err);
            handle_sensor_error(i2c_num, err);
            taskYIELD();
            continue;
        }

        lpf_apply_vec3(&lpf_accel, &local_lin_accel);
        lpf_apply_vec3(&lpf_gyro, &local_gyro_raw);

        if (offset_mutex != NULL && xSemaphoreTake(offset_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            // Cập nhật lin_accel và euler toàn cục
            lin_accel_forOffset.x = local_lin_accel.x;
            lin_accel_forOffset.y = local_lin_accel.y;
            lin_accel_forOffset.z = local_lin_accel.z;

            xSemaphoreGive(offset_mutex);
        }

        // Áp dụng offset vào biến cục bộ
        apply_accel_offset(&local_lin_accel);

        apply_accel_threshold(&local_lin_accel, ACCEL_THRESHOLD); // Tính toán adjusted heading

        local_adjusted_heading = apply_heading_offset(local_euler.heading);

        time_mks_after = esp_timer_get_time();
        time_bno = time_mks_after - time_mks;

        // Áp dụng bộ lọc thấp thông vào dữ liệu cục bộ

        // Phát hiện chuyển động dựa trên dữ liệu gyro cục bộ
        is_moving = is_motion_detected(&local_gyro_raw);

        // Cập nhật biến toàn cục lin_accel và euler thông qua mutex
        if (accel_data_mutex != NULL && xSemaphoreTake(accel_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            // Cập nhật lin_accel và euler toàn cục
            lin_accel.x = local_lin_accel.x;
            lin_accel.y = local_lin_accel.y;
            lin_accel.z = local_lin_accel.z;

            euler.heading = local_euler.heading;
            euler.roll = local_euler.roll;
            euler.pitch = local_euler.pitch;

            xSemaphoreGive(accel_data_mutex);
        }

        // Cập nhật adjusted_heading riêng lẻ vì có mutex riêng
        if (heading_mutex != NULL && xSemaphoreTake(heading_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            adjusted_heading = local_adjusted_heading;
            xSemaphoreGive(heading_mutex);
        }

        // Tạo JSON và gửi dữ liệu - sử dụng biến cục bộ để tránh khóa mutex thêm lần nữa
#if SEND_ALL == 1
        int bytes_written = snprintf(json_buffer, sizeof(json_buffer),
                                     "{"
                                     "\"id\":\"%s\","
                                     "\"type\":\"bno055\","
                                     "\"data\":{"
                                     "\"time\":%10d,"
                                     "\"euler\":[%.4f,%.4f,%.4f],"
                                     "\"lin_accel\":[%.4f,%.4f,%.4f],"
                                     "\"gyro_raw\":[%.4f,%.4f,%.4f],"
                                     "\"status\":\"%s\""
                                     "}"
                                     "}\n",
                                     ID_ROBOT, time_bno,
                                     local_adjusted_heading, local_euler.pitch, local_euler.roll,
                                     local_lin_accel.x, local_lin_accel.y, local_lin_accel.z,
                                     local_gyro_raw.x, local_gyro_raw.y, local_gyro_raw.z,
                                     is_moving ? "moving" : "stationary");
#else
        snprintf(json_buffer, sizeof(json_buffer),
                 "{"
                 "\"id\":\"%s\","
                 "\"type\":\"bno055\","
                 "\"data\":{"
                 "\"time\":%10d,"
                 "\"lin_accel\":[%.4f,%.4f,%.4f],"
                 "\"gyro_raw\":[%.4f,%.4f,%.4f],"
                 "\"is_moving\":%s"
                 "}"
                 "}\n",
                 ID_ROBOT, time_bno,
                 local_lin_accel.x, local_lin_accel.y, local_lin_accel.z,
                 local_gyro_raw.x, local_gyro_raw.y, local_gyro_raw.z,
                 is_moving ? "true" : "false");
#endif

#if SEND_UART == 1
        cycle_counter++;
        if (cycle_counter % 10 == 0)
        {
            uart2_send_data(json_buffer, bytes_written);
            cycle_counter = 0;
        }
#else
        printf("%s", json_buffer);

#endif

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(BNO_POLLING_MS));
    }
}
void bno055_start()
{
    printf("\n\n\n");
    printf("********************\n");
    printf("  BNO055 NDOF test\n");
    printf("********************\n");

    esp_err_t err;
    err = nvs_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_IMU, "Failed to initialize NVS");
    }

    if (bno055_event_group == NULL)
    {
        bno055_event_group = xEventGroupCreate();
        if (bno055_event_group == NULL)
        {
            ESP_LOGE(TAG_IMU, "Failed to create event group");
        }
        xEventGroupClearBits(bno055_event_group, BNO055_TASK_RUNNING_BIT);
    }

    if (bno055_event_group == NULL)
    {
        bno055_event_group = xEventGroupCreate();
        if (bno055_event_group == NULL)
        {
            ESP_LOGE(TAG_IMU, "Failed to create event group");
        }
        xEventGroupClearBits(bno055_event_group, BNO055_CALIBRATED_BIT);
    }

    err = bno055_set_default_conf(&bno_conf);
    err = bno055_open(i2c_num, &bno_conf, BNO_MODE);
    ESP_LOGI(TAG_IMU, "bno055_open() returned 0x%02X", err);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_IMU, "Program terminated! returned 0x%02X", err);
        err = bno055_close(i2c_num);
        ESP_LOGW(TAG_IMU, "bno055_close() returned 0x%02X", err);
        ESP_LOGW(TAG_IMU, "Failed to open BNO055, starting reinit process");
        xTaskCreatePinnedToCore(reinit_sensor,
                                "reinit_sensor",
                                2048,
                                NULL,
                                10,
                                NULL,
                                1);
    }
    else
    {
        // Reset calibration status
        calibration_complete = false;

        // Tạo task hiệu chuẩn với ưu tiên cao hơn
        xTaskCreatePinnedToCore(calibration_task,
                                "calib_task",
                                4096,
                                NULL,
                                11, // Ưu tiên cao hơn ndof_task
                                &calib_task_handle,
                                1);
    }

    heading_mutex = xSemaphoreCreateMutex();
    accel_data_mutex = xSemaphoreCreateMutex();
    offset_mutex = xSemaphoreCreateMutex();
    /*-------- Low Pass Filter Init --------*/
    if (lpf_init(&lpf_accel, 0.4, sizeof(bno055_vec3_t)))
    {
        ESP_LOGI(TAG_IMU, "Accel LPF initialized successfully");
    }
    else
    {
        ESP_LOGE(TAG_IMU, "Failed to initialize Accel LPF");
    }

    // Khởi tạo LPF cho gyroscope
    if (lpf_init(&lpf_gyro, 0.6, sizeof(bno055_vec3_t)))
    {
        ESP_LOGI(TAG_IMU, "Gyro LPF initialized successfully");
    }
    else
    {
        ESP_LOGE(TAG_IMU, "Failed to initialize Gyro LPF");
    }
}