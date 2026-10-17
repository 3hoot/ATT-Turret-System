#include <stdint.h>
#include <stdatomic.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "as5600.h"

static const char *TAG = "as5600";

/*
 * Internal telemetry task for the AS5600 sensor.
 * This task continuously reads the angle from the sensor.
 * It runs at a fixed interval defined by AS5600_TELEMETRY_TASK
 * 
 * @param arg Pointer to the AS5600 device handle.
 */
static void as5600_telemetry_task(void *arg);

esp_err_t as5600_init(i2c_master_bus_handle_t bus_handle, const as5600_config_t *config, as5600_handle_t *dev_handle)
{
    ESP_RETURN_ON_FALSE(bus_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid I2C bus handle");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid AS5600 configuration");
    
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDRESS,
        .scl_speed_hz = AS5600_I2C_SPEED_HZ,
    };

    ESP_LOGI(TAG, "Adding device to I2C bus at address 0x%02X", AS5600_I2C_ADDRESS);
    i2c_master_dev_handle_t bus_dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &bus_dev_handle));
    
    dev_handle->bus_handle = bus_handle;
    dev_handle->bus_dev_handle = bus_dev_handle;
    dev_handle->config = *config;
    atomic_store(&dev_handle->angle, 0);
    atomic_store(&dev_handle->zero_angle, 0);

    ESP_LOGI(TAG, "Writing configuration to AS5600 sensor");
    uint8_t conf1_reg_values = (((config->watchdog & AS5600_REG_CONF1_WD_MASK) << AS5600_REG_CONF1_WD_SHIFT) |
                                ((config->fast_filter_threshold & AS5600_REG_CONF1_FTH_MASK) << AS5600_REG_CONF1_FTH_SHIFT) |
                                ((config->slow_filter & AS5600_REG_CONF1_SF_MASK) << AS5600_REG_CONF1_SF_SHIFT));
    uint8_t conf2_reg_values = ((config->hysteresis & AS5600_REG_CONF2_HYST_MASK) << AS5600_REG_CONF2_HYST_SHIFT);
    uint8_t conf_write_buffer[3] = {AS5600_REG_CONF1, conf1_reg_values, conf2_reg_values};
    ESP_ERROR_CHECK(i2c_master_transmit(bus_dev_handle, conf_write_buffer, sizeof(conf_write_buffer), AS5600_I2C_TIMEOUT_MS));

    uint8_t status_write_buffer[1] = {AS5600_REG_STATUS};
    uint8_t status_read_buffer[1] = {0};
    ESP_ERROR_CHECK(i2c_master_transmit_receive(bus_dev_handle, status_write_buffer, sizeof(status_write_buffer), 
                                                                status_read_buffer, sizeof(status_read_buffer), AS5600_I2C_TIMEOUT_MS));
    
    if (status_read_buffer[0] & AS5600_REG_STATUS_MD_MASK)
    {
        ESP_LOGI(TAG, "Magnet detected");
        atomic_store(&dev_handle->status, AS5600_MAGNET_DETECTED);  
    }
    else if (status_read_buffer[0] & AS5600_REG_STATUS_ML_MASK)
    {
        ESP_LOGW(TAG, "Magnet too weak");
        atomic_store(&dev_handle->status, AS5600_MAGNET_TOO_WEAK);  
    }
    else if (status_read_buffer[0] & AS5600_REG_STATUS_MH_MASK)
    {
        ESP_LOGW(TAG, "Magnet too strong");
        atomic_store(&dev_handle->status, AS5600_MAGNET_TOO_STRONG);  
    }
    else
    {
        ESP_LOGW(TAG, "Magnet status unknown, defaulting to MAGNET_DETECTED");
        atomic_store(&dev_handle->status, AS5600_MAGNET_DETECTED);
    }

    ESP_LOGI(TAG, "Creating telemetry task for AS5600 sensor");
    BaseType_t task_creation_result = xTaskCreate(as5600_telemetry_task, "as5600_telemetry_task", AS5600_TELEMETRY_TASK_STACK_SIZE,
                                                  (void *)dev_handle, AS5600_TELEMETRY_TASK_PRIORITY, &dev_handle->telemetry_task_handle);
    ESP_RETURN_ON_FALSE(task_creation_result == pdPASS, ESP_ERR_NO_MEM, TAG, "Failed to create telemetry task for AS5600 sensor");

    ESP_LOGI(TAG, "AS5600 sensor initialized successfully");
    return ESP_OK;
}

esp_err_t as5600_get_status(as5600_handle_t* dev_handle, as5600_status_t *status)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid AS5600 device handle");
    ESP_RETURN_ON_FALSE(status != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid status pointer");

    *status = atomic_load(&dev_handle->status);
    return ESP_OK;
}

esp_err_t as5600_read_angle(as5600_handle_t *dev_handle, float *angle)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid AS5600 device handle");
    ESP_RETURN_ON_FALSE(angle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid angle pointer");

    *angle = atomic_load(&dev_handle->angle) * AS5600_ANGLE_FACTOR;
    return ESP_OK;
}

esp_err_t as5600_set_zero_angle(as5600_handle_t *dev_handle, float zero_angle)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid AS5600 device handle");

    uint16_t zero_angle_raw = (uint16_t)(zero_angle / AS5600_ANGLE_FACTOR);
    atomic_store(&dev_handle->zero_angle, zero_angle_raw);
    return ESP_OK;
}

esp_err_t as5600_get_zero_angle(as5600_handle_t *dev_handle, float *zero_angle)
{
    ESP_RETURN_ON_FALSE(dev_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid AS5600 device handle");
    ESP_RETURN_ON_FALSE(zero_angle != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid zero angle pointer");

    *zero_angle = atomic_load(&dev_handle->zero_angle) * AS5600_ANGLE_FACTOR;
    return ESP_OK;
}

static void as5600_telemetry_task(void *arg)
{
    as5600_handle_t *dev_handle = (as5600_handle_t *)arg;
    if (dev_handle == NULL)
    {
        ESP_LOGE(TAG, "Invalid AS5600 device handle in telemetry task");
        vTaskDelete(NULL);
        return;
    }

    while (true)
    {
        /* ----------------- Angle read ----------------- */
        
        uint8_t angle_write_buffer[1] = {AS5600_REG_ANGLE1};
        uint8_t angle_read_buffer[2] = {0};
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle->bus_dev_handle, angle_write_buffer, sizeof(angle_write_buffer), 
                                                                                angle_read_buffer, sizeof(angle_read_buffer), AS5600_I2C_TIMEOUT_MS));
        uint16_t angle = ((uint16_t)angle_read_buffer[0] & AS5600_REG_ANGLE1_MASK) << AS5600_REG_ANGLE1_SHIFT |
                         ((uint16_t)angle_read_buffer[1] & AS5600_REG_ANGLE2_MASK) << AS5600_REG_ANGLE2_SHIFT;
        atomic_store(&dev_handle->angle, angle);                                                                                

        vTaskDelay(pdMS_TO_TICKS(AS5600_TELEMETRY_TASK_INTERVAL_MS));
    }
}