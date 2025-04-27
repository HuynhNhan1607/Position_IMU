#ifndef POSITION_TRACKER_H
#define POSITION_TRACKER_H

#include "esp_err.h"

/**
 * @brief Khởi tạo bộ theo dõi vị trí
 * @return esp_err_t ESP_OK nếu thành công
 */
esp_err_t position_tracker_init();

/**
 * @brief Đặt lại vị trí về (0,0)
 */
void position_tracker_reset();

/**
 * @brief Lấy vị trí hiện tại
 * @param position Mảng 2 phần tử để lưu vị trí [x, y] (m)
 */
void position_tracker_get_position(float position[2]);

/**
 * @brief Thêm dữ liệu gia tốc mới vào bộ theo dõi
 * Được gọi với tần số 100Hz
 */
void position_tracker_add_accel();

/**
 * @brief Khởi động bộ theo dõi vị trí
 */
void position_tracker_start();

#endif /* POSITION_TRACKER_H */