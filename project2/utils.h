#ifndef UTILS_H
#define UTILS_H

#include <sys/time.h>

// packet encoding and decoding function declaration
void encode_packet(char *buffer, int seq_num, const char *data, int data_size);
int decode_packet(const char *buffer, int *seq_num, char *data, int buffer_size);

// calculate checksum (used to detect whether the packet is corrupted
unsigned short checksum(const char *data, int len);

// calculates the difference between two times
int timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y);

#endif // UTILS_H
