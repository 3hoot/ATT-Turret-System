#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "driver/i2c_master.h"

#include "app_config.h"
#include "framed_serial.h"
#include "stepper_driver.h"
#include "utils.h"
#include "as5600.h"
#include "ina226.h"

static const char *TAG = "app_main";

/* Held as an array for easy access to all devices */
static stepper_drv_stepper_handle_t *steppers[APP_STEPPER_COUNT];
static as5600_handle_t *position_sensors[APP_AS5600_SENSOR_COUNT];
static ina226_handle_t *power_sensors[APP_INA226_SENSOR_COUNT];

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
static esp_err_t app_send_ack(const uint8_t command_type, const esp_err_t status_code);

/*
 * Send a response frame in response to a received command.
 * Preforms an internal memcpy of the data to be sent, so the caller can free or modify the data after return.
 *
 * @param command_type The type of the command for which a response is being sent.
 * @param data Pointer to the data to include in the response frame.
 * @param data_length The length of the data.
 * @return ESP_OK on success, non-zero error code on failure.
 */
static esp_err_t app_send_response(const uint8_t command_type, const uint8_t *data, const size_t data_length);

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

    stepper_drv_stepper_handle_t stepper_A_handle = {0};
    stepper_drv_stepper_config_t stepper_A_config = {
        .pin_step = APP_STEPPER_A_STEP_PIN,
        .pin_dir = APP_STEPPER_A_DIR_PIN,
        .pin_enable = APP_STEPPER_A_EN_PIN
    };
    ESP_ERROR_CHECK(stepper_drv_init(stepper_A_config, &stepper_A_handle));
    steppers[0] = &stepper_A_handle;

    stepper_drv_stepper_handle_t stepper_B_handle = {0};
    stepper_drv_stepper_config_t stepper_B_config = {
        .pin_step = APP_STEPPER_B_STEP_PIN,
        .pin_dir = APP_STEPPER_B_DIR_PIN,
        .pin_enable = APP_STEPPER_B_EN_PIN
    };
    ESP_ERROR_CHECK(stepper_drv_init(stepper_B_config, &stepper_B_handle));
    steppers[1] = &stepper_B_handle;

    /* ---------------- I2C bus init ---------------- */

    /*
     * Because the AS5600 position sensor has a fixed I2C address, 
     * we need to initialize two separate I2C buses to communicate with two sensors simultaneously.
     * Another thing to note, it is recommended by Espressif to use external pull-up whenever possible
     * (Internal pull-ups are not strong enough for high-speed I2C communication).
     */

    i2c_master_bus_config_t bus_1_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = 1,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_NUM_0,
        .intr_priority = APP_I2C_BUS_1_INTERRUPT_PRIORITY,
        .scl_io_num = APP_I2C_BUS_1_SCL_PIN,
        .sda_io_num = APP_I2C_BUS_1_SDA_PIN
    };
    i2c_master_bus_handle_t bus_1_handle = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_1_config, &bus_1_handle));
    
    i2c_master_bus_config_t bus_2_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = 0,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_NUM_1,
        .intr_priority = APP_I2C_BUS_2_INTERRUPT_PRIORITY,
        .scl_io_num = APP_I2C_BUS_2_SCL_PIN,
        .sda_io_num = APP_I2C_BUS_2_SDA_PIN
    };
    i2c_master_bus_handle_t bus_2_handle = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_2_config, &bus_2_handle));

    /* -------- AS5600 position sensors init -------- */

    as5600_config_t as5600_1_config = AS5600_DEFAULT_CONFIG;
    as5600_handle_t as5600_1_handle = {0};
    ESP_ERROR_CHECK(as5600_init(bus_1_handle, &as5600_1_config, &as5600_1_handle));
    position_sensors[0] = &as5600_1_handle;

    as5600_config_t as5600_2_config = AS5600_DEFAULT_CONFIG;
    as5600_handle_t as5600_2_handle = {0};
    ESP_ERROR_CHECK(as5600_init(bus_2_handle, &as5600_2_config, &as5600_2_handle));
    position_sensors[1] = &as5600_2_handle;

    /* ---------- INA226 power sensors init --------- */

    ina226_config_t ina226_1_config = INA226_DEFAULT_CONFIG;
    ina226_handle_t ina226_1_handle = {0};
    ESP_ERROR_CHECK(ina226_init(bus_1_handle, &ina226_1_config, APP_INA226_1_I2C_ADDRESS, &ina226_1_handle));
    power_sensors[0] = &ina226_1_handle;
    
    ina226_config_t ina226_2_config = INA226_DEFAULT_CONFIG;
    ina226_handle_t ina226_2_handle = {0};
    ESP_ERROR_CHECK(ina226_init(bus_2_handle, &ina226_2_config, APP_INA226_2_I2C_ADDRESS, &ina226_2_handle));
    power_sensors[1] = &ina226_2_handle;

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
        rx_frame = framed_serial_get_frame_from_buffer(framed_serial_rx_buffer, portMAX_DELAY);
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

                uint8_t *echo_payload = malloc(rx_frame.data_length);
                if (echo_payload == NULL)
                {
                    ESP_LOGE(TAG, "Failed to allocate memory for ECHO payload");
                    break;
                }
                memcpy(echo_payload, rx_frame.data, rx_frame.data_length);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, echo_payload, rx_frame.data_length));
                free(echo_payload);
                break;
            }

            case APP_CMD_STEP_ARM_CODE:
                ESP_LOGI(TAG, "Received STEP_ARM command for stepper ID: %d", rx_frame.data[0]);
                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, stepper_drv_enable(steppers[rx_frame.data[0]])));
                break;

            case APP_CMD_STEP_DISARM_CODE:
                ESP_LOGI(TAG, "Received STEP_DISARM command for stepper ID: %d", rx_frame.data[0]);
                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, stepper_drv_disable(steppers[rx_frame.data[0]])));
                break;

            case APP_CMD_STEP_START_CODE:
            {
                uint8_t stepper_id = rx_frame.data[0];
                stepper_drv_direction_t direction = (rx_frame.data[1] == 0) ? STEPPER_DRV_DIRECTION_FWD : STEPPER_DRV_DIRECTION_REV;

                ESP_LOGI(TAG, "Received STEP_START command for stepper ID: %d, direction: %s",
                         stepper_id, (direction == STEPPER_DRV_DIRECTION_FWD) ? "FWD" : "REV");

                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, stepper_drv_raw_step_start(steppers[stepper_id], direction)));
                break;
            }

            case APP_CMD_STEP_STOP_CODE:
                ESP_LOGI(TAG, "Received STEP_STOP command for stepper ID: %d", rx_frame.data[0]);
                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, stepper_drv_raw_step_stop(steppers[rx_frame.data[0]])));
                break;

            case APP_CMD_STEP_SET_FREQ_CODE:
            {
                uint8_t stepper_id = rx_frame.data[0];
                uint16_t stepper_raw_frequency_hz = utils_bytes_to_u16_be(rx_frame.data, 1);

                ESP_LOGI(TAG, "Received STEP_SET_FREQ command for stepper ID: %d, frequency: %u Hz",
                         stepper_id, stepper_raw_frequency_hz);

                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, stepper_drv_raw_step_set_frequency(steppers[stepper_id], stepper_raw_frequency_hz)));
                break;
            }

            case APP_CMD_STEP_GET_POS_CODE:
            {
                uint8_t stepper_id = rx_frame.data[0];
                int32_t position = 0;
                ESP_ERROR_CHECK_WITHOUT_ABORT(stepper_drv_get_position(steppers[stepper_id], &position));

                ESP_LOGI(TAG, "Received STEP_GET_POS command for stepper ID: %d, current position: %d", stepper_id, position);

                uint8_t position_bytes[4];
                utils_u32_to_bytes_be((uint32_t)position, position_bytes, 0);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, position_bytes, sizeof(position_bytes)));
                break;
            }

            case APP_CMD_SENSE_GET_POS_CODE:
            {
                uint8_t sensor_id = rx_frame.data[0];
                float angle = 0;
                ESP_ERROR_CHECK_WITHOUT_ABORT(as5600_read_angle(position_sensors[sensor_id], &angle));

                ESP_LOGI(TAG, "Received SENSE_GET_POS command for sensor ID: %d, current angle: %f", sensor_id, angle);

                uint16_t angle_raw = (uint16_t)(angle / AS5600_ANGLE_FACTOR);
                uint8_t angle_bytes[2];
                utils_u16_to_bytes_be(angle_raw, angle_bytes, 0);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, angle_bytes, sizeof(angle_bytes)));
                break;
            }

            case APP_CMD_SENSE_SET_ZERO_CODE:
            {
                uint8_t sensor_id = rx_frame.data[0];
                uint16_t current_angle = utils_bytes_to_u16_be(rx_frame.data, 1);

                ESP_LOGI(TAG, "Received SENSE_SET_ZERO command for sensor ID: %d, zero angle set to: %u", sensor_id, current_angle);

                ESP_ERROR_CHECK(app_send_ack(rx_frame.type, as5600_set_zero_angle(position_sensors[sensor_id], current_angle)));
                break;
            }

            case APP_CMD_SENSE_GET_PWR_CODE:
            {
                uint8_t sensor_id = rx_frame.data[0];
                float power = 0;
                ESP_ERROR_CHECK_WITHOUT_ABORT(ina226_read_power(power_sensors[sensor_id], &power));

                ESP_LOGI(TAG, "Received SENSE_GET_PWR command for sensor ID: %d, current power: %f", sensor_id, power);

                uint16_t power_raw = (uint16_t)(power / INA226_POWER_FACTOR);
                uint8_t power_bytes[2];
                utils_u16_to_bytes_be(power_raw, power_bytes, 0);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, power_bytes, sizeof(power_bytes)));
                break;
            }

            case APP_CMD_SENSE_GET_CURR_CODE:
            {
                uint8_t sensor_id = rx_frame.data[0];
                float current = 0;
                ESP_ERROR_CHECK_WITHOUT_ABORT(ina226_read_current(power_sensors[sensor_id], &current));

                ESP_LOGI(TAG, "Received SENSE_GET_CURR command for sensor ID: %d, current current: %f", sensor_id, current);

                uint16_t current_raw = (uint16_t)(current / INA226_CURRENT_FACTOR);
                uint8_t current_bytes[2];
                utils_u16_to_bytes_be(current_raw, current_bytes, 0);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, current_bytes, sizeof(current_bytes)));
                break;
            }

            case APP_CMD_SENSE_GET_VOLT_CODE:
            {
                uint8_t sensor_id = rx_frame.data[0];
                float voltage = 0;
                ESP_ERROR_CHECK(ina226_read_bus_voltage(power_sensors[sensor_id], &voltage));

                ESP_LOGI(TAG, "Received SENSE_GET_VOLT command for sensor ID: %d, current voltage: %f", sensor_id, voltage);

                uint16_t voltage_raw = (uint16_t)(voltage / INA226_BUS_VOLTAGE_FACTOR);
                uint8_t voltage_bytes[2];
                utils_u16_to_bytes_be(voltage_raw, voltage_bytes, 0);
                ESP_ERROR_CHECK(app_send_response(rx_frame.type, voltage_bytes, sizeof(voltage_bytes)));
                break;
            }

            default:
                ESP_LOGW(TAG, "Received unknown command type: 0x%02X", rx_frame.type);
                break;
        }
    }
}

static esp_err_t app_send_ack(const uint8_t command_type, const esp_err_t status_code)
{
    framed_serial_frame_t tx_frame = {.type = APP_CMD_ACK_CODE(command_type), .data_length = 2, .checksum = 0};
    utils_u16_to_bytes_be((uint16_t)status_code, tx_frame.data, 0);
    const size_t tx_frame_size = sizeof(tx_frame) - FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH + tx_frame.data_length;

    if (xRingbufferSend(framed_serial_tx_buffer, &tx_frame, tx_frame_size, APP_TX_BUFFER_TIMEOUT_MS) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to send ACK for command type: 0x%02X", command_type);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ACK sent for command type: 0x%02X with status code: %u", command_type, (unsigned int)status_code);
    return ESP_OK;
}

static esp_err_t app_send_response(const uint8_t command_type, const uint8_t *data, const size_t data_length)
{
    framed_serial_frame_t tx_frame = {.type = APP_CMD_ACK_CODE(command_type), .data_length = data_length, .checksum = 0};
    memcpy(tx_frame.data, data, data_length);
    const size_t tx_frame_size = sizeof(tx_frame) - FRAMED_SERIAL_MAX_FRAME_DATA_LENGTH + tx_frame.data_length;

    if (xRingbufferSend(framed_serial_tx_buffer, &tx_frame, tx_frame_size, APP_TX_BUFFER_TIMEOUT_MS) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to send response for command type: 0x%02X", command_type);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Response sent for command type: 0x%02X with data length: %zu", command_type, data_length);
    return ESP_OK;
}