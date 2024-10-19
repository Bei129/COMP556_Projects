#include "crc.h"

// CRC32 table
static uint32_t crc32_table[256];

// Initialize CRC32 table
void init_crc32_table() {
    uint32_t polynomial = 0xEDB88320;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (uint32_t j = 8; j > 0; j--) {
            if (crc & 1) {
                crc = (crc >> 1) ^ polynomial;
            } else {
                crc >>= 1;
            }
        }
        crc32_table[i] = crc;
    }
}

// Calculate CRC32
uint32_t crc32(const void *data, size_t length) {
    if (crc32_table[0] == 0) {
        init_crc32_table();
    }

    uint32_t crc = 0xFFFFFFFF; 
    const uint8_t *byte_data = (const uint8_t *)data;

    for (size_t i = 0; i < length; i++) {
        uint8_t byte = byte_data[i];
        crc = (crc >> 8) ^ crc32_table[(crc ^ byte) & 0xFF];
    }

    return ~crc; 
}
