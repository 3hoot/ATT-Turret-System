#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stddef.h>

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
uint16_t utils_bytes_to_u16_be (const uint8_t *bytes, const size_t start_index);

/*
 * Converts a sequence of bytes to a 32-bit unsigned integer in big-endian format.
 *
 * @param bytes A pointer to the array of bytes.
 * @param start_index The index of the first byte to convert.
 *
 * @return The converted 32-bit unsigned integer.
 */
uint32_t utils_bytes_to_u32_be (const uint8_t *bytes, const size_t start_index);

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

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */