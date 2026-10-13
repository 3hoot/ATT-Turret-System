#include <stdint.h>
#include <stddef.h>

#include "utils.h"

uint16_t utils_bytes_to_u16_be (const uint8_t *bytes, const size_t start_index)
{
    return (uint16_t)((bytes[start_index] << 8) | bytes[start_index + 1]);
}

uint32_t utils_bytes_to_u32_be (const uint8_t *bytes, const size_t start_index)
{
    return (uint32_t)((bytes[start_index] << 24) | (bytes[start_index + 1] << 16) |
                      (bytes[start_index + 2] << 8) | bytes[start_index + 3]);
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