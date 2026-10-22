#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stddef.h>

#include "driver/i2c_master.h"

/* Preprocessor directive to avoid C++ name mangling */
#ifdef __cplusplus
extern "C" 
{
#endif

/*
 * Converts a sequence of bytes to a 16-bit unsigned integer in big-endian format.
 *
 * @param bytes A pointer to the array of bytes.
 * @param start_index The index of the first byte to convert.
 *
 * @return The converted 16-bit unsigned integer.
 */
uint16_t utils_bytes_to_u16_be(const uint8_t *bytes, const size_t start_index);

/*
 * Converts a sequence of bytes to a 32-bit unsigned integer in big-endian format.
 *
 * @param bytes A pointer to the array of bytes.
 * @param start_index The index of the first byte to convert.
 *
 * @return The converted 32-bit unsigned integer.
 */
uint32_t utils_bytes_to_u32_be(const uint8_t *bytes, const size_t start_index);

/*
 * Converts a sequence of bytes to a float in big-endian format.
 *
 * @param bytes A pointer to the array of bytes.
 * @param start_index The index of the first byte to convert.
 *
 * @return The converted float.
 */
float utils_bytes_to_float_be(const uint8_t *bytes, const size_t start_index);

/*
 * Converts a 16-bit unsigned integer to a sequence of bytes in big-endian format.
 *
 * @param value The 16-bit unsigned integer to convert.
 * @param bytes A pointer to the array of bytes.
 * @param start_index The index of the first byte to write.
 */
void utils_u16_to_bytes_be(const uint16_t value, uint8_t *bytes, const size_t start_index);

/*
 * Converts a 32-bit unsigned integer to a sequence of bytes in big-endian format.
 *
 * @param value The 32-bit unsigned integer to convert.
 * @param bytes A pointer to the array of bytes.
 * @param start_index The index of the first byte to write.
 */
void utils_u32_to_bytes_be(const uint32_t value, uint8_t *bytes, const size_t start_index);

/*
 * Converts a float to a sequence of bytes in big-endian format.
 *
 * @param value The float to convert.
 * @param bytes A pointer to the array of bytes.
 * @param start_index The index of the first byte to write.
 */
void utils_float_to_bytes_be(const float value, uint8_t *bytes, const size_t start_index);

/*
 * Scans the I2C bus for connected devices and returns their addresses.
 *
 * @param bus_handle The handle of the I2C bus to scan.
 * @param found_addresses A pointer to an array where the found addresses will be stored.
 * @param found_count A pointer to a variable where the number of found addresses will be stored.
 * @param max_count The maximum number of addresses to store in the found_addresses array, if set to -1 all found addresses will be stored.
 * @param timeout_ms The timeout in milliseconds for each I2C transaction.
 */
void utils_scan_i2c_bus(const i2c_master_bus_handle_t bus_handle, uint8_t *found_addresses, size_t *found_count, 
                        const int32_t max_count, const uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */