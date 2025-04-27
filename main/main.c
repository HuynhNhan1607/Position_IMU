#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sys_config.h"
#include "position_tracker.h"

#include "bno055_handler.h"

extern EventGroupHandle_t bno055_event_group;

void app_main(void)
{
    // vTaskDelay(pdMS_TO_TICKS(100));

    bno055_start();

    if (bno055_event_group != NULL)
    {
        // Wait indefinitely for the ndof_task to start
        EventBits_t bits = xEventGroupWaitBits(
            bno055_event_group,
            BNO055_TASK_RUNNING_BIT,
            pdFALSE,      // Don't clear the bits after retrieving
            pdTRUE,       // Wait for all bits
            portMAX_DELAY // Wait forever
        );

        ESP_LOGI("Main_Task", "BNO055 ndof_task is now running, starting position tracker");
        position_tracker_start();
    }
    else
    {
        ESP_LOGE("Main_Task", "BNO055 event group not initialized");
    }
}
