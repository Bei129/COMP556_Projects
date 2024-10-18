#include "utils.h"
#include <string.h>

void encode_packet(char *buffer, int seq_num, const char *data, int data_size) {
    memcpy(buffer, &seq_num, sizeof(int));
    memcpy(buffer + sizeof(int), data, data_size);
}

int decode_packet(const char *buffer, int *seq_num, char *data, int buffer_size) {
    memcpy(seq_num, buffer, sizeof(int));
    memcpy(data, buffer + sizeof(int), buffer_size - sizeof(int));
    return buffer_size - sizeof(int);
}

unsigned short checksum(const char *data, int len) {
    unsigned int sum = 0;
    for (int i = 0; i < len; i++) {
        sum += (unsigned char)data[i];
    }
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return ~sum;
}

int timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y) {
    if (x->tv_sec < y->tv_sec || (x->tv_sec == y->tv_sec && x->tv_usec < y->tv_usec)) {
        return -1;
    }
    result->tv_sec = x->tv_sec - y->tv_sec;
    result->tv_usec = x->tv_usec - y->tv_usec;
    if (result->tv_usec < 0) {
        result->tv_sec--;
        result->tv_usec += 1000000;
    }
    return 0;
}
