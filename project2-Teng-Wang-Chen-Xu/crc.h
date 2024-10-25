#ifndef CRC_H
#define CRC_H

#include <stdint.h>
#include <stddef.h>

// Initialize the CRC32 table
void init_crc32_table();

// Calculate CRC32
uint32_t crc32(const void *data, size_t length);

#endif // CRC_H
