#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "crc.h"

#define PKT_SIZE 1024
#define DATA_SIZE \
  (PKT_SIZE -     \
   sizeof(int32_t) * 3)  // 4(seq_num) + 4(data_length) + 4(crc) = 12 bytes
#define WINDOW_SIZE 5

// RTT calculation parameters for Adaptive Timeout Mechanism
#define ALPHA 0.125
#define BETA 0.25
#define INITIAL_TIMEOUT 5.0

// Update RTT and timeout
double estimated_rtt = INITIAL_TIMEOUT;
double dev_rtt = 0.0;
double timeout_interval = INITIAL_TIMEOUT;

// Update RTT and timeout interval based on sample RTT
void update_timeout(double sample_rtt) {
  estimated_rtt = (1 - ALPHA) * estimated_rtt + ALPHA * sample_rtt;
  dev_rtt = (1 - BETA) * dev_rtt + BETA * fabs(sample_rtt - estimated_rtt);
  timeout_interval = estimated_rtt + 4 * dev_rtt;
}

// Calculate time difference in seconds
double get_time_diff(struct timeval start, struct timeval end) {
  return (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1e6;
}

struct packet
{
  int seq_num;
  int start;
  char data[PKT_SIZE];
  int acked;
  struct timeval send_time;
};

int main(int argc, char **argv) {
  int sock, opt;
  char *recv_host = NULL, *recv_port = NULL, *filename = NULL;
  struct sockaddr_in recv_addr, send_addr;
  struct addrinfo *server_info, hints;
  socklen_t addr_len = sizeof(send_addr);
  FILE *fp;

  // Parse command line arguments
  while ((opt = getopt(argc, argv, "r:f:")) != -1) {
    switch (opt) {
      case 'r':
        recv_host = strtok(optarg, ":");
        recv_port = strtok(NULL, ":");
        break;
      case 'f':
        filename = optarg;
        break;
      default:
        printf("Usage: sendfile -r <recv host>:<recv port> -f <filename>\n");
        return 1;
    }
  }

  if (recv_host == NULL || recv_port == NULL || filename == NULL) {
    printf("Usage: sendfile -r <recv host>:<recv port> -f <filename>\n");
    return 1;
  }

  // Open the file
  if ((fp = fopen(filename, "rb")) == NULL) {
    perror("Error opening file");
    return 1;
  }

  // Resolve server address
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;

  if (getaddrinfo(recv_host, recv_port, &hints, &server_info) != 0) {
    perror("Error resolving server address");
    fclose(fp);
    return 1;
  }

  // Create UDP socket
  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("Error opening socket");
    fclose(fp);
    return 1;
  }

  // Configure receiver address
  memset(&recv_addr, 0, sizeof(recv_addr));
  recv_addr.sin_family = AF_INET;
  recv_addr.sin_port = htons(atoi(recv_port));
  recv_addr.sin_addr = ((struct sockaddr_in *)server_info->ai_addr)->sin_addr;

  // Configure local address
  memset(&send_addr, 0, sizeof(send_addr));
  send_addr.sin_family = AF_INET;
  send_addr.sin_addr.s_addr = INADDR_ANY;
  send_addr.sin_port = htons(0);

  if (bind(sock, (struct sockaddr *)&send_addr, sizeof(send_addr)) < 0) {
    perror("Error binding socket");
    fclose(fp);
    return 1;
  }

  // Get the assigned local port
  if (getsockname(sock, (struct sockaddr *)&send_addr, &addr_len) == -1) {
    perror("getsockname");
  } else {
    printf("Bound to local port: %d\n", ntohs(send_addr.sin_port));
  }

  // Initialize sliding window and related buffers
  char window_data[WINDOW_SIZE][DATA_SIZE];
  int window_data_length[WINDOW_SIZE];
  struct timeval send_times_arr[WINDOW_SIZE];
  int acked[WINDOW_SIZE];
  memset(acked, 0, sizeof(acked));
  memset(window_data_length, 0, sizeof(window_data_length));

  int base = 0, next_seq_num = 0;
  int bytes_read, eof = 0;

  // Send the file name and path
  if (sendto(sock, filename, strlen(filename) + 1, 0,
             (struct sockaddr *)&recv_addr, addr_len) < 0) {
    perror("Error sending file info");
    fclose(fp);
    return 1;
  }
  printf("[send file info] File name: %s\n", filename);

  // Allocate send buffer
  char send_buffer[PKT_SIZE];

  // Send file
  while (!eof || base < next_seq_num) {
    // Send packets within window
    while (next_seq_num < base + WINDOW_SIZE && !eof) {
      bytes_read =
          fread(window_data[next_seq_num % WINDOW_SIZE], 1, DATA_SIZE, fp);
      if (bytes_read <= 0) {
        eof = 1;
        // Send EOF packet
        memset(send_buffer, 0, PKT_SIZE);
        int32_t eof_seq_num = htonl(-1);
        int32_t eof_data_length = htonl(0);
        memcpy(send_buffer, &eof_seq_num, sizeof(int32_t));
        memcpy(send_buffer + sizeof(int32_t), &eof_data_length,
               sizeof(int32_t));

        uint32_t eof_crc =
            htonl(crc32((unsigned char *)send_buffer, sizeof(int32_t) * 2));
        memcpy(send_buffer + sizeof(int32_t) * 2, &eof_crc, sizeof(uint32_t));

        if (sendto(sock, send_buffer, sizeof(int32_t) * 3, 0,
                   (struct sockaddr *)&recv_addr, addr_len) < 0) {
          perror("Error sending EOF marker packet");
          fclose(fp);
          return 1;
        }
        printf("[send EOF marker] Seq_num: -1\n");
        break;
      }

      // Store data_length
      window_data_length[next_seq_num % WINDOW_SIZE] = bytes_read;

      // Prepare the packet
      int32_t net_seq_num = htonl(next_seq_num);
      int32_t net_data_length = htonl(bytes_read);
      memcpy(send_buffer, &net_seq_num, sizeof(int32_t));
      memcpy(send_buffer + sizeof(int32_t), &net_data_length, sizeof(int32_t));
      memcpy(send_buffer + sizeof(int32_t) * 2,
             window_data[next_seq_num % WINDOW_SIZE], bytes_read);

      // Calculate CRC over first 8 + bytes_read bytes
      uint32_t crc_value =
          crc32((unsigned char *)send_buffer, sizeof(int32_t) * 2 + bytes_read);
      uint32_t net_crc = htonl(crc_value);
      memcpy(send_buffer + sizeof(int32_t) * 2 + bytes_read, &net_crc,
             sizeof(uint32_t));

      // Send the packet
      if (sendto(sock, send_buffer,
                 sizeof(int32_t) * 2 + bytes_read + sizeof(uint32_t), 0,
                 (struct sockaddr *)&recv_addr, addr_len) < 0) {
        perror("Error sending packet");
        fclose(fp);
        return 1;
      }

      printf("[send data] Seq_num: %d, Bytes: %d, CRC: %u\n", next_seq_num,
             bytes_read, crc_value);
      gettimeofday(&send_times_arr[next_seq_num % WINDOW_SIZE], NULL);
      acked[next_seq_num % WINDOW_SIZE] = 0;
      next_seq_num++;
    }

    // Set up select for timeout
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock, &read_fds);

    struct timeval timeout;
    timeout.tv_sec = (int)timeout_interval;
    timeout.tv_usec = (timeout_interval - timeout.tv_sec) * 1e6;

    int select_retval = select(sock + 1, &read_fds, NULL, NULL, &timeout);
    if (select_retval == -1) {
      perror("select error");
      fclose(fp);
      return 1;
    }

    // Handle timeout
    if (select_retval == 0) {
      struct timeval current_time;
      gettimeofday(&current_time, NULL);

      for (int i = base; i < next_seq_num; i++) {
        if (!acked[i % WINDOW_SIZE]) {
          double elapsed =
              get_time_diff(send_times_arr[i % WINDOW_SIZE], current_time);
          if (elapsed >= timeout_interval) {
            // Resend the packet
            memset(send_buffer, 0, PKT_SIZE);
            int32_t resend_seq_num = htonl(i);
            int32_t resend_data_length =
                htonl(window_data_length[i % WINDOW_SIZE]);
            memcpy(send_buffer, &resend_seq_num, sizeof(int32_t));
            memcpy(send_buffer + sizeof(int32_t), &resend_data_length,
                   sizeof(int32_t));
            memcpy(send_buffer + sizeof(int32_t) * 2,
                   window_data[i % WINDOW_SIZE],
                   window_data_length[i % WINDOW_SIZE]);

            uint32_t resend_crc = crc32(
                (unsigned char *)send_buffer,
                sizeof(int32_t) * 2 + window_data_length[i % WINDOW_SIZE]);
            uint32_t net_resend_crc = htonl(resend_crc);
            memcpy(send_buffer + sizeof(int32_t) * 2 +
                       window_data_length[i % WINDOW_SIZE],
                   &net_resend_crc, sizeof(uint32_t));

            if (sendto(sock, send_buffer,
                       sizeof(int32_t) * 2 +
                           window_data_length[i % WINDOW_SIZE] +
                           sizeof(uint32_t),
                       0, (struct sockaddr *)&recv_addr, addr_len) < 0) {
              perror("Error resending packet");
              fclose(fp);
              return 1;
            }

            printf("[resend data] Seq_num: %d, CRC: %u\n", i, resend_crc);
            gettimeofday(&send_times_arr[i % WINDOW_SIZE],
                         NULL);  // Update send time
          }
        }
      }
    } else {
      // Receive ACK
      char ack_buffer[32];
      struct sockaddr_in ack_addr;
      socklen_t ack_addr_len = sizeof(ack_addr);
      ssize_t ack_bytes = recvfrom(sock, ack_buffer, sizeof(ack_buffer) - 1, 0,
                                   (struct sockaddr *)&ack_addr, &ack_addr_len);
      if (ack_bytes < 0) {
        perror("Error receiving ACK");
        fclose(fp);
        return 1;
      }
      ack_buffer[ack_bytes] = '\0';

      int ack_num = atoi(ack_buffer);
      printf("[recv ACK] Seq_num: %d from %s:%d\n", ack_num,
             inet_ntoa(ack_addr.sin_addr), ntohs(ack_addr.sin_port));

      if (ack_num >= base && ack_num < next_seq_num) {
        acked[ack_num % WINDOW_SIZE] = 1;

        // Calculate and update RTT
        struct timeval ack_time;
        gettimeofday(&ack_time, NULL);
        double sample_rtt =
            get_time_diff(send_times_arr[ack_num % WINDOW_SIZE], ack_time);
        update_timeout(sample_rtt);

        // Slide the window
        while (acked[base % WINDOW_SIZE] && base < next_seq_num) {
          base++;
        }

        // If EOF and all ACKs received, exit
        if (ack_num == -1 && base == next_seq_num) {
          break;
        }
      }
    }
  }

  fclose(fp);
  close(sock);
  freeaddrinfo(server_info);

  printf("[completed]\n");
  return 0;
}
