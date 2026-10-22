#include <stdint.h>
#include <stddef.h>

#include "esp_log.h"
#include "driver/i2c_master.h"

#include "utils.h"

static const char *TAG = "utils";

uint16_t utils_bytes_to_u16_be (const uint8_t *bytes, const size_t start_index)
{
    return (uint16_t)((bytes[start_index] << 8) | bytes[start_index + 1]);
}

uint32_t utils_bytes_to_u32_be (const uint8_t *bytes, const size_t start_index)
{
    return (uint32_t)((bytes[start_index] << 24) | (bytes[start_index + 1] << 16) |
                      (bytes[start_index + 2] << 8) | bytes[start_index + 3]);
}

float utils_bytes_to_float_be(const uint8_t *bytes, const size_t start_index)
{
    uint32_t int_value = utils_bytes_to_u32_be(bytes, start_index);
    float float_value;
    memcpy(&float_value, &int_value, sizeof(float));
    return float_value;
}

void utils_u16_to_bytes_be(const uint16_t value, uint8_t *bytes, const size_t start_index)
{
    bytes[start_index] = (uint8_t)(value >> 8);
    bytes[start_index + 1] = (uint8_t)(value & 0xFF);
}

void utils_u32_to_bytes_be(const uint32_t value, uint8_t *bytes, const size_t start_index)
{
    bytes[start_index] = (uint8_t)(value >> 24);
    bytes[start_index + 1] = (uint8_t)((value >> 16) & 0xFF);
    bytes[start_index + 2] = (uint8_t)((value >> 8) & 0xFF);
    bytes[start_index + 3] = (uint8_t)(value & 0xFF);
}

void utils_float_to_bytes_be(const float value, uint8_t *bytes, const size_t start_index)
{
    uint32_t int_value;
    memcpy(&int_value, &value, sizeof(float));
    utils_u32_to_bytes_be(int_value, bytes, start_index);
}

void utils_scan_i2c_bus(const i2c_master_bus_handle_t bus_handle, uint8_t *found_addresses, size_t *found_count, 
                        const int32_t max_count, const uint32_t timeout_ms)
{
    ESP_LOGI(TAG, "Scanning I2C bus for devices...");
    size_t count = 0;
    for (uint8_t address = 1; address < 127; ++address) 
    {
        if (count >= max_count && max_count != -1) 
        {
            ESP_LOGW(TAG, "Reached maximum count of found devices (%d). Stopping scan.", max_count);
            break;
        }

        esp_err_t result = i2c_master_probe(bus_handle, address, timeout_ms);
        if (result == ESP_OK) 
        {
            ESP_LOGI(TAG, "Found I2C device at address 0x%02X", address);
            found_addresses[count++] = address;
        }
    }
    ESP_LOGI(TAG, "I2C scan complete. Found %zu device(s).", count);
    *found_count = count;
}