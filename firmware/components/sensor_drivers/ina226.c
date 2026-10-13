#include <stdint.h>
#include <stdatomic.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "esp_intr_alloc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ina226.h"

static const char *TAG = "ina226";

/*
 * Interrupt Service Routine (ISR) for handling alert events from the INA226 sensor.
 * Upon receiving an alert, this ISR will invoke the registered callback function, if any, passing the device handle and user-defined argument.
 * 
 * @param arg Pointer to the INA226 device handle.
 */
static void IRAM_ATTR ina226_alert_isr_handler(void *arg);

/*
 * Telemetry Task for the INA226 sensor.
 * This task will periodically read the sensor values and update the device handle.
 *
 * @param arg Pointer to the INA226 device handle.
 */
static void ina226_telemetry_task(void *arg);

esp_err_t ina226_init(i2c_master_bus_handle_t bus_handle, const ina226_config_t *config, const uint8_t i2c_address, ina226_handle_t *dev_handle)
{
    ESP_RETURN_ON_FALSE(bus_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid I2C bus handle");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid INA226 configuration");
    
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_address,
        .scl_speed_hz = INA226_I2C_SPEED_HZ,
    };

    ESP_LOGI(TAG, "Adding device to I2C bus and probing for device presence");
    i2c_master_dev_handle_t bus_dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &bus_dev_handle));
    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, i2c_address, INA226_I2C_TIMEOUT_MS));
    
    dev_handle->bus_handle = bus_handle;
    dev_handle->bus_dev_handle = bus_dev_handle;
    dev_handle->config = *config;
    atomic_store(&dev_handle->shunt_voltage, 0);
    atomic_store(&dev_handle->bus_voltage, 0);
    atomic_store(&dev_handle->power, 0);
    atomic_store(&dev_handle->current, 0);
    dev_handle->alert_callback = NULL;
    dev_handle->alert_callback_arg = NULL;

    ESP_LOGI(TAG, "Writing configuration to INA226 sensor");
    uint16_t config_reg_value = ((config->averaging << INA226_REG_CONFIG_AVG_SHIFT) & INA226_REG_CONFIG_AVG_MASK) | 
                                ((config->vbus_conversion_time << INA226_REG_CONFIG_VBUSCT_SHIFT) & INA226_REG_CONFIG_VBUSCT_MASK) |
                                ((config->vsh_conversion_time << INA226_REG_CONFIG_VSHCT_SHIFT) & INA226_REG_CONFIG_VSHCT_MASK) |
                                ((config->operating_mode << INA226_REG_CONFIG_MODE_SHIFT) & INA226_REG_CONFIG_MODE_MASK);
    uint8_t config_write_buffer[3] = {INA226_REG_CONFIG, (uint8_t)(config_reg_value >> 8), (uint8_t)(config_reg_value & 0xFF)};
    ESP_ERROR_CHECK(i2c_master_transmit(bus_dev_handle, config_write_buffer, sizeof(config_write_buffer), INA226_I2C_TIMEOUT_MS));  

    ESP_LOGI(TAG, "Writing calibration value to INA226 sensor");
    uint8_t calibration_write_buffer[3] = {INA226_REG_CALIBRATION, (uint8_t)(config->calibration_value >> 8), (uint8_t)(config->calibration_value & 0xFF)};
    ESP_ERROR_CHECK(i2c_master_transmit(bus_dev_handle, calibration_write_buffer, sizeof(calibration_write_buffer), INA226_I2C_TIMEOUT_MS));

    ESP_LOGI(TAG, "Writing alert configuration to INA226 sensor");
    uint16_t alert_config_value = ((config->alert_function << INA226_REG_ALERT_FUNC_SHIFT) & INA226_REG_ALERT_FUNC_MASK) | 
                                  (((config->alert_active_high ? 1 : 0) << INA226_REG_ALERT_APOL_SHIFT) & INA226_REG_ALERT_APOL_MASK) | 
                                  (((config->alert_latch ? 1 : 0) << INA226_REG_ALERT_LEN_SHIFT) & INA226_REG_ALERT_LEN_MASK);
    uint8_t alert_config_write_buffer[3] = {INA226_REG_ALERT, (uint8_t)(alert_config_value >> 8), (uint8_t)(alert_config_value & 0xFF)};
    ESP_ERROR_CHECK(i2c_master_transmit(bus_dev_handle, alert_config_write_buffer, sizeof(alert_config_write_buffer), INA226_I2C_TIMEOUT_MS));

    ESP_LOGI(TAG, "Writing alert limit to INA226 sensor");
    uint8_t alert_limit_write_buffer[3] = {INA226_REG_ALERT_LIMIT, (uint8_t)(config->alert_limit >> 8), (uint8_t)(config->alert_limit & 0xFF)};
    ESP_ERROR_CHECK(i2c_master_transmit(bus_dev_handle, alert_limit_write_buffer, sizeof(alert_limit_write_buffer), INA226_I2C_TIMEOUT_MS));

    ESP_LOGI(TAG, "Creating telemetry task for INA226 sensor");
    BaseType_t task_creation_result = xTaskCreate(ina226_telemetry_task, "ina226_telemetry_task", INA226_TELEMETRY_TASK_STACK_SIZE,
                                                  (void *)dev_handle, INA226_TELEMETRY_TASK_PRIORITY, &dev_handle->telemetry_task_handle);
    ESP_RETURN_ON_FALSE(task_creation_result == pdPASS, ESP_ERR_NO_MEM, TAG, "Failed to create telemetry task for INA226 sensor");

    ESP_LOGI(TAG, "INA226 sensor initialized successfully");
    return ESP_OK;
}

esp_err_t ina226_register_callback(ina226_handle_t *dev_handle, const gpio_num_t alert_gpio_num, ina226_alert_callback_t callback, void *arg)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid INA226 device handle");
    ESP_RETURN_ON_FALSE(callback != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid callback function");

    ESP_LOGI(TAG, "Registering alert callback for INA226 sensor on GPIO %d", alert_gpio_num);
    dev_handle->alert_callback = callback;
    dev_handle->alert_callback_arg = arg;

    /* We can use alert_active_high INA226 flag to determine the interrupt type */
    gpio_int_type_t intr_type = dev_handle->config.alert_active_high 
                                ? GPIO_INTR_POSEDGE 
                                : GPIO_INTR_NEGEDGE;

    gpio_config_t io_config = {
        .pin_bit_mask = (1ULL << alert_gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = dev_handle->config.alert_active_high ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE,
        .pull_down_en = dev_handle->config.alert_active_high ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = intr_type,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_config), TAG, "Failed to configure alert GPIO");

    esp_err_t err = gpio_install_isr_service(INA226_INTR_PRIORITY);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(err));
        return err;
    }

    return gpio_isr_handler_add(alert_gpio_num, ina226_alert_isr_handler, (void *)dev_handle);
}

esp_err_t ina226_read_shunt_voltage(ina226_handle_t *dev_handle, float *shunt_voltage)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid INA226 device handle");
    ESP_RETURN_ON_FALSE(shunt_voltage != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid shunt voltage pointer");

    *shunt_voltage = atomic_load(&dev_handle->shunt_voltage) * INA226_SHUNT_VOLTAGE_FACTOR;
    return ESP_OK;
}

esp_err_t ina226_read_bus_voltage(ina226_handle_t *dev_handle, float *bus_voltage)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid INA226 device handle");
    ESP_RETURN_ON_FALSE(bus_voltage != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid bus voltage pointer");

    *bus_voltage = atomic_load(&dev_handle->bus_voltage) * INA226_BUS_VOLTAGE_FACTOR;
    return ESP_OK;
}

esp_err_t ina226_read_power(ina226_handle_t *dev_handle, float *power)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid INA226 device handle");
    ESP_RETURN_ON_FALSE(power != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid power pointer");

    *power = atomic_load(&dev_handle->power) * INA226_POWER_FACTOR;
    return ESP_OK;
}

esp_err_t ina226_read_current(ina226_handle_t *dev_handle, float *current)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid INA226 device handle");
    ESP_RETURN_ON_FALSE(current != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid current pointer");

    *current = atomic_load(&dev_handle->current) * INA226_CURRENT_FACTOR;
    return ESP_OK;
}

static void IRAM_ATTR ina226_alert_isr_handler(void *arg)
{
    ina226_handle_t *dev_handle = (ina226_handle_t *)arg;
    if (dev_handle == NULL || dev_handle->alert_callback == NULL)
    {
        return;
    }

    dev_handle->alert_callback(*dev_handle, dev_handle->alert_callback_arg);
}

static void ina226_telemetry_task(void *arg)
{
    ina226_handle_t *dev_handle = (ina226_handle_t *)arg;
    if (dev_handle == NULL)
    {
        ESP_LOGE(TAG, "Invalid INA226 device handle in telemetry task");
        vTaskDelete(NULL);
        return;
    }

    while (true)
    {
        /* ------------- Shunt voltage read ------------- */

        uint8_t shunt_voltage_write_buffer[1] = {INA226_REG_SHUNT_VOLTAGE};
        uint8_t shunt_voltage_read_buffer[2] = {0};
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle->bus_dev_handle, shunt_voltage_write_buffer, sizeof(shunt_voltage_write_buffer),
                                                    shunt_voltage_read_buffer, sizeof(shunt_voltage_read_buffer), INA226_I2C_TIMEOUT_MS));
        uint16_t shunt_voltage = (shunt_voltage_read_buffer[0] << 8) | shunt_voltage_read_buffer[1];
        atomic_store(&dev_handle->shunt_voltage, shunt_voltage);

        /* -------------- Bus voltage read -------------- */
        

        uint8_t bus_voltage_write_buffer[1] = {INA226_REG_BUS_VOLTAGE};
        uint8_t bus_voltage_read_buffer[2] = {0};
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle->bus_dev_handle, bus_voltage_write_buffer, sizeof(bus_voltage_write_buffer),
                                                    bus_voltage_read_buffer, sizeof(bus_voltage_read_buffer), INA226_I2C_TIMEOUT_MS));
        uint16_t bus_voltage = (bus_voltage_read_buffer[0] << 8) | bus_voltage_read_buffer[1];
        atomic_store(&dev_handle->bus_voltage, bus_voltage);

        /* ----------------- Power read ----------------- */

        uint8_t power_write_buffer[1] = {INA226_REG_POWER};
        uint8_t power_read_buffer[2] = {0};
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle->bus_dev_handle, power_write_buffer, sizeof(power_write_buffer),
                                                    power_read_buffer, sizeof(power_read_buffer), INA226_I2C_TIMEOUT_MS));
        uint16_t power = (power_read_buffer[0] << 8) | power_read_buffer[1];
        atomic_store(&dev_handle->power, power);

        /* ----------------- Current read ----------------- */

        uint8_t current_write_buffer[1] = {INA226_REG_CURRENT};
        uint8_t current_read_buffer[2] = {0};
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle->bus_dev_handle, current_write_buffer, sizeof(current_write_buffer),
                                                    current_read_buffer, sizeof(current_read_buffer), INA226_I2C_TIMEOUT_MS));
        uint16_t current = (current_read_buffer[0] << 8) | current_read_buffer[1];
        atomic_store(&dev_handle->current, current);

        vTaskDelay(pdMS_TO_TICKS(INA226_TELEMETRY_TASK_INTERVAL_MS));
    }
}