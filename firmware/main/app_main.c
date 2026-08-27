#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"

#include "app_config.h"
#include "framed_serial.h"

void app_comunication_task(void *arg);
TaskHandle_t app_comunication_task_handle = NULL;

void app_main(void)
{
    /* Memory check */
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI("BOARD_INIT", "Detected Flash Size: %lu MB", flash_size / (1024 * 1024));

    if (esp_psram_is_initialized())
    {
        ESP_LOGI("BOARD_INIT", "PSRAM Total Size: %d bytes", esp_psram_get_size());
        ESP_LOGI("BOARD_INIT", "PSRAM Free Size: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
    else
    {
        ESP_LOGE("BOARD_INIT", "PSRAM Failed to initialize!");
    }

    /* Initialize USB framed serial communication */
    static tinyusb_config_t usb_config;
    usb_config = (tinyusb_config_t)TINYUSB_DEFAULT_CONFIG();
    static tinyusb_config_cdcacm_t cdc_config = FRAMED_SERIAL_USB_CDC_DEFAULT_CONFIG();

    void** config = malloc(2 * sizeof(void*));
    assert(config != NULL && "Failed to allocate memory for USB framed serial configuration");
    config[0] = (void *)&usb_config;
    config[1] = (void *)&cdc_config;
    ESP_ERROR_CHECK(framed_serial_init(FRAMED_SERIAL_PROTOCOL_USB_CDC, config));

    xTaskCreate(app_comunication_task, "app_communication_task", APP_COMMUNICATION_TASK_STACK_SIZE,
                NULL, APP_COMMUNICATION_TASK_PRIORITY, &app_comunication_task_handle);
}

void app_comunication_task(void *arg)
{
    framed_serial_frame_t rx_frame;
    framed_serial_frame_t tx_frame;

    const size_t frame_prefix_size = sizeof(rx_frame) - FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH;
    size_t frame_size = 0;

    while (true)
    {
        rx_frame = retrieve_frame_from_buffer(rx_buffer, portMAX_DELAY);
        frame_size = frame_prefix_size + rx_frame.data_length;

        switch (rx_frame.type)
        {
            case APP_CMD_ECHO_CODE:
                ESP_LOGI("APP_COMMUNICATION_TASK", "Received ECHO command with data length: %d", rx_frame.data_length);
                if (rx_frame.data_length == 0)
                {
                    ESP_LOGW("APP_COMMUNICATION_TASK", "ECHO command received with no data to echo");
                    break;
                }

                memcpy(&tx_frame, &rx_frame, frame_size);        
                if (xRingbufferSend(tx_buffer, &tx_frame, frame_size, portMAX_DELAY) != pdTRUE)
                {
                    ESP_LOGE("APP_COMMUNICATION_TASK", "Failed to send ECHO response to TX buffer");
                }
                else
                {
                    ESP_LOGI("APP_COMMUNICATION_TASK", "ECHO response sent successfully");
                }

                break;

            default:
                ESP_LOGW("APP_COMMUNICATION_TASK", "Received unknown command type: 0x%02X", rx_frame.type);
                break;
        }
    }
}