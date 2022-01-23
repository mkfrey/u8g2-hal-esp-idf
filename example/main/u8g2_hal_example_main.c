#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void task_test_SSD1306(void *ignore);

void task_test_SSD1306i2c(void *ignore);

_Noreturn void app_main() {
#if EXAMPLE_INTERFACE_I2C
    xTaskCreate(task_test_SSD1306i2c, "test", 4000, NULL, tskIDLE_PRIORITY, NULL);
#else
    xTaskCreate(task_test_SSD1306, "test", 4000, NULL, tskIDLE_PRIORITY, NULL);
#endif

    while (true) {
        vTaskDelay(1);
    }
}
