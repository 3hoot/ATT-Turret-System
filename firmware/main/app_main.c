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
#include "stepper_driver.h"

static const char *TAG = "app_main";

/* -------------- Command executor -------------- */

/*
 * Task responsible for executing commands received via framed serial communication.
 */
static void app_executor_task(void *arg);
TaskHandle_t app_executor_task_handle = NULL;

/* -------------- Helper functions -------------- */

/*
 * Send an ACK frame in response to a received command.
 * 
 * @param command_type The type of the command being acknowledged.
 * @param status_code The status code to include in the ACK frame.
 * @return ESP_OK on success, non-zero error code on failure.
 */
static esp_err_t app_send_ack(const uint8_t command_type, const uint16_t status_code);

void app_main(void)
{
    /* ---------------- Memory check ---------------- */

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "Detected Flash Size: %lu MB", flash_size / (1024 * 1024));

    if (esp_psram_is_initialized())
    {
        ESP_LOGI(TAG, "PSRAM Total Size: %d bytes", esp_psram_get_size());
        ESP_LOGI(TAG, "PSRAM Free Size: %zu bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
    else
    {
        ESP_LOGE(TAG, "PSRAM Failed to initialize!");
    }

    /* ----------- USB communication init ----------- */
    
    static tinyusb_config_t usb_config;
    usb_config = (tinyusb_config_t)TINYUSB_DEFAULT_CONFIG();
    static tinyusb_config_cdcacm_t cdc_config = FRAMED_SERIAL_USB_CDC_DEFAULT_CONFIG();

    void** config = malloc(2 * sizeof(void*));
    assert(config != NULL && "Failed to allocate memory for USB framed serial configuration");
    config[0] = (void *)&usb_config;
    config[1] = (void *)&cdc_config;
    ESP_ERROR_CHECK(framed_serial_init(FRAMED_SERIAL_PROTOCOL_USB_CDC, config));

    /* ------------- Stepper driver init ------------ */

    stepper_drv_stepper_config_t stepper_configs[APP_STEPPER_COUNT] = {
        { .stepper_id = APP_STEPPER_A_ID, .pin_step = APP_STEPPER_STEP_PIN_A, .pin_dir = APP_STEPPER_DIR_PIN_A, .pin_enable = APP_STEPPER_EN_PIN_A },
        { .stepper_id = APP_STEPPER_B_ID, .pin_step = APP_STEPPER_STEP_PIN_B, .pin_dir = APP_STEPPER_DIR_PIN_B, .pin_enable = APP_STEPPER_EN_PIN_B }
    };
    ESP_ERROR_CHECK(stepper_drv_init(stepper_configs, APP_STEPPER_COUNT));

    /* ------------ Command executor init ----------- */

    xTaskCreate(app_executor_task, "app_executor_task", APP_EXECUTOR_TASK_STACK_SIZE,
                NULL, APP_EXECUTOR_TASK_PRIORITY, &app_executor_task_handle);
    if (app_executor_task_handle == NULL)
    {
        ESP_LOGE(TAG, "APP_EXECUTOR", "Failed to create app executor task");
    }
}

static void app_executor_task(void *arg)
{
    framed_serial_frame_t rx_frame = {0};

    const size_t frame_prefix_size = sizeof(rx_frame) - FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH;
    size_t rx_frame_size = 0;

    while (true)
    {
        memset(&rx_frame, 0, sizeof(rx_frame));
        rx_frame = framed_serial_get_frame_from_buffer(rx_buffer, portMAX_DELAY);
        rx_frame_size = frame_prefix_size + rx_frame.data_length;

        switch (rx_frame.type)
        {
            case APP_CMD_ECHO_CODE:
            {
                ESP_LOGI(TAG, "Received ECHO command with payload length: %d", rx_frame.data_length);
                if (rx_frame.data_length == 0)
                {
                    ESP_LOGW(TAG, "ECHO command received with no payload");
                    break;
                }

                framed_serial_frame_t tx_frame = {0};
                memcpy(&tx_frame, &rx_frame, rx_frame_size);        
                if (xRingbufferSend(tx_buffer, &tx_frame, rx_frame_size, portMAX_DELAY) != pdTRUE)
                {
                    ESP_LOGE(TAG, "Failed to send ECHO response to TX buffer");
                }
                else
                {
                    ESP_LOGI(TAG, "ECHO response sent with payload length: %d", tx_frame.data_length);
                }

                break;
            }

            case APP_CMD_STEP_ARM_CODE:
                ESP_LOGI(TAG, "Received STEP_ARM command for stepper ID: %d", rx_frame.data[0]);
                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, stepper_drv_enable(rx_frame.data[0])));
                break;

            case APP_CMD_STEP_DISARM_CODE:
                ESP_LOGI(TAG, "Received STEP_DISARM command for stepper ID: %d", rx_frame.data[0]);
                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, stepper_drv_disable(rx_frame.data[0])));
                break;

            case APP_CMD_STEP_START_CODE:
            {
                uint8_t stepper_id = rx_frame.data[0];
                stepper_drv_direction_t direction = (rx_frame.data[1] == 0) ? STEPPER_DRV_DIRECTION_FWD : STEPPER_DRV_DIRECTION_REV;

                ESP_LOGI(TAG, "Received STEP_START command for stepper ID: %d, direction: %s",
                         stepper_id, (direction == STEPPER_DRV_DIRECTION_FWD) ? "FWD" : "REV");

                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, stepper_drv_raw_step_start(stepper_id, direction)));
                break;
            }

            case APP_CMD_STEP_STOP_CODE:
                ESP_LOGI(TAG, "Received STEP_STOP command for stepper ID: %d", rx_frame.data[0]);
                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, stepper_drv_raw_step_stop(rx_frame.data[0])));
                break;

            case APP_CMD_STEP_SET_FREQ_CODE:
            {
                uint8_t stepper_id = rx_frame.data[0];
                uint16_t stepper_raw_frequency_hz = framed_serial_bytes_to_u16_be(rx_frame.data, 1);

                ESP_LOGI(TAG, "Received STEP_SET_FREQ command for stepper ID: %d, frequency: %u Hz",
                         stepper_id, stepper_raw_frequency_hz);

                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, stepper_drv_raw_step_set_frequency(stepper_id, stepper_raw_frequency_hz)));
                break;
            }

            case APP_CMD_STEP_GET_POS_CODE:
            {
                uint8_t stepper_id = rx_frame.data[0];
                int32_t position = stepper_drv_get_position(stepper_id);

                ESP_LOGI(TAG, "Received STEP_GET_POS command for stepper ID: %d, current position: %d", stepper_id, position);

                /* Manually construct the response frame */
                framed_serial_frame_t tx_frame = {.type = APP_CMD_ACK_CODE(rx_frame.type), .data_length = 4, .checksum = 0};
                framed_serial_u32_to_bytes_be((uint32_t)position, tx_frame.data, 0);
                const size_t tx_frame_size = frame_prefix_size + tx_frame.data_length;

                if (xRingbufferSend(tx_buffer, &tx_frame, tx_frame_size, portMAX_DELAY) != pdTRUE)
                {
                    ESP_LOGE(TAG, "Failed to send STEP_GET_POS response to TX buffer");
                }
                else
                {
                    ESP_LOGI(TAG, "STEP_GET_POS response sent successfully with payload length: %d", tx_frame.data_length);
                }
            }

            default:
                ESP_LOGW(TAG, "Received unknown command type: 0x%02X", rx_frame.type);
                break;
        }
    }
}

static esp_err_t app_send_ack(const uint8_t command_type, const uint16_t status_code)
{
    framed_serial_frame_t tx_frame = {.type = APP_CMD_ACK_CODE(command_type), .data_length = 2, .checksum = 0};
    framed_serial_u16_to_bytes_be(status_code, tx_frame.data, 0);
    const size_t tx_frame_size = sizeof(tx_frame) - FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH + tx_frame.data_length;

    if (xRingbufferSend(tx_buffer, &tx_frame, tx_frame_size, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to send ACK for command type: 0x%02X", command_type);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ACK sent for command type: 0x%02X with status code: %u", command_type, status_code);
    return ESP_OK;
}