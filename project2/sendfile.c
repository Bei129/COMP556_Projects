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
#define DATA_SIZE    \
  (PKT_SIZE -        \
   sizeof(int32_t) * \
       4) // 4(seq_num) + 4(start) + 4(data_length) + 4(crc) = 16 bytes
#define WINDOW_SIZE 20

// RTT calculation parameters for Adaptive Timeout Mechanism
#define ALPHA 0.125
#define BETA 0.25
#define INITIAL_TIMEOUT 1

// Global variables for RTT and timeout
double estimated_rtt = INITIAL_TIMEOUT;
double dev_rtt = 0.0;
double timeout_interval = INITIAL_TIMEOUT;

// Function to update RTT and timeout interval
void update_timeout(double sample_rtt)
{
  estimated_rtt = (1 - ALPHA) * sample_rtt + ALPHA * estimated_rtt;
  dev_rtt = (1 - BETA) * dev_rtt + BETA * fabs(sample_rtt - estimated_rtt);
  timeout_interval = estimated_rtt + 4 * dev_rtt;
}

// record current time
double get_current_time()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec * 1000000.0 + tv.tv_usec;
}

// Function to calculate time difference
double get_time_diff(struct timeval start, struct timeval end)
{
  return (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1e6;
}

int send_file_info_with_timeout(int sock, const char *filename,
                                struct sockaddr_in *recv_addr,
                                socklen_t addr_len)
{
  uint32_t filename_crc = crc32((unsigned char *)filename, strlen(filename));
  char file_info[256];
  int info_size =
      snprintf(file_info, sizeof(file_info), "%s:%u", filename, filename_crc);
  int max_retry = 10, cnt = 0;
  for (cnt = 1; cnt < max_retry; cnt++)
    if (sendto(sock, file_info, info_size + 1, 0, (struct sockaddr *)recv_addr,
               addr_len) < 0)
    {
      perror("Error sending file info");
      return -1;
    }
  printf("[send file info] File info sent %d times\n", max_retry);

  return 0;
}

// Function to prepare and send a data packet
int send_data_packet(int sock, struct sockaddr_in *recv_addr,
                     socklen_t addr_len, char *send_buffer, char *data,
                     int seq_num, int start_offset, int data_length)
{
  int32_t net_seq_num = htonl(seq_num);
  int32_t net_data_length = htonl(data_length);
  int32_t net_start_offset = htonl(start_offset);

  memcpy(send_buffer, &net_seq_num, sizeof(int32_t));
  memcpy(send_buffer + sizeof(int32_t), &net_start_offset, sizeof(int32_t));
  memcpy(send_buffer + sizeof(int32_t) * 2, &net_data_length, sizeof(int32_t));
  memcpy(send_buffer + sizeof(int32_t) * 3, data, data_length);

  uint32_t crc_value =
      crc32((unsigned char *)send_buffer, sizeof(int32_t) * 3 + data_length);
  uint32_t net_crc = htonl(crc_value);
  memcpy(send_buffer + sizeof(int32_t) * 3 + data_length, &net_crc,
         sizeof(uint32_t));

  if (sendto(sock, send_buffer,
             sizeof(int32_t) * 3 + data_length + sizeof(uint32_t), 0,
             (struct sockaddr *)recv_addr, addr_len) < 0)
  {
    perror("Error sending packet");
    return -1;
  }

  printf("[send data] Seq_num: %d, Start:%d, Length: %d, CRC: %u\n", seq_num,
         start_offset, data_length, crc_value);
  return 0;
}

// Main function
int main(int argc, char **argv)
{
  int sock, opt;
  char *recv_host = NULL, *recv_port = NULL, *filename = NULL;
  struct sockaddr_in recv_addr, send_addr;
  struct addrinfo *server_info, hints;
  socklen_t addr_len = sizeof(send_addr);
  FILE *fp;

  // Parse command line arguments
  while ((opt = getopt(argc, argv, "r:f:")) != -1)
  {
    switch (opt)
    {
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

  if (recv_host == NULL || recv_port == NULL || filename == NULL)
  {
    printf("Usage: sendfile -r <recv host>:<recv port> -f <filename>\n");
    return 1;
  }

  // Open the file
  if ((fp = fopen(filename, "rb")) == NULL)
  {
    perror("Error opening file");
    return 1;
  }

  // Resolve server address
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  if (getaddrinfo(recv_host, recv_port, &hints, &server_info) != 0)
  {
    perror("Error resolving server address");
    fclose(fp);
    return 1;
  }

  // Create UDP socket
  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
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
  if (bind(sock, (struct sockaddr *)&send_addr, sizeof(send_addr)) < 0)
  {
    perror("Error binding socket");
    fclose(fp);
    return 1;
  }

  // // Send file info and wait for ACK
  if (send_file_info_with_timeout(sock, filename, &recv_addr, addr_len) < 0)
  {
    printf("Failed to send file info.\n");
    fclose(fp);
    close(sock);
    return 1;
  }

  // Sliding window variables
  int base = 0, next_seq_num = 0;
  int start_offset = 0;
  char window_data[WINDOW_SIZE][DATA_SIZE];
  int window_data_length[WINDOW_SIZE] = {0};
  struct timeval send_times_arr[WINDOW_SIZE];
  int acked[WINDOW_SIZE] = {0}, start[WINDOW_SIZE] = {0};
  int eof = 0;

  // Buffer for sending data
  char send_buffer[PKT_SIZE];

  // start time
  double start_time = get_current_time() / 1000000.0;

  // Main sending loop
  while (!eof || base < next_seq_num)
  {
    // Send packets within the window
    while (next_seq_num < base + WINDOW_SIZE && !eof)
    {
      int bytes_read =
          fread(window_data[next_seq_num % WINDOW_SIZE], 1, DATA_SIZE, fp);
      if (bytes_read <= 0)
      {
        eof = 1;
        break;
      }

      // Send data packet
      if (send_data_packet(sock, &recv_addr, addr_len, send_buffer,
                           window_data[next_seq_num % WINDOW_SIZE],
                           next_seq_num, start_offset, bytes_read) < 0)
      {
        fclose(fp);
        close(sock);
        return 1;
      }

      // Update window variables
      gettimeofday(&send_times_arr[next_seq_num % WINDOW_SIZE], NULL);
      window_data_length[next_seq_num % WINDOW_SIZE] = bytes_read;
      start[next_seq_num % WINDOW_SIZE] = start_offset;
      acked[next_seq_num % WINDOW_SIZE] = 0;
      next_seq_num++;
      start_offset += bytes_read;
    }

    // Handle ACKs and possible timeout
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock, &read_fds);

    struct timeval timeout = {(int)timeout_interval,
                              (timeout_interval - (int)timeout_interval) * 1e6};
    int select_retval = select(sock + 1, &read_fds, NULL, NULL, &timeout);

    if (select_retval == -1)
    {
      perror("select error");
      fclose(fp);
      close(sock);
      return 1;
    }

    // Handle timeout or ACK reception
    if (select_retval == 0)
    {
      // Timeout occurred, handle resending packets
      struct timeval current_time;
      gettimeofday(&current_time, NULL);

      for (int i = base; i < next_seq_num; i++)
      {
        if (!acked[i % WINDOW_SIZE] &&
            get_time_diff(send_times_arr[i % WINDOW_SIZE], current_time) >=
                timeout_interval)
        {
          // Resend packet
          if (send_data_packet(sock, &recv_addr, addr_len, send_buffer,
                               window_data[i % WINDOW_SIZE], i,
                               start[i % WINDOW_SIZE],
                               window_data_length[i % WINDOW_SIZE]) < 0)
          {
            fclose(fp);
            close(sock);
            return 1;
          }
          // printf("[resend data] Seq_num: %d\n", i);
          gettimeofday(&send_times_arr[i % WINDOW_SIZE], NULL);
        }
      }
    }
    else
    {
      // Receive ACK
      char ack_buffer[32];
      struct sockaddr_in ack_addr;
      socklen_t ack_addr_len = sizeof(ack_addr);
      ssize_t ack_bytes = recvfrom(sock, ack_buffer, sizeof(ack_buffer) - 1, 0,
                                   (struct sockaddr *)&ack_addr, &ack_addr_len);
      if (ack_bytes < 0)
      {
        perror("Error receiving ACK");
        fclose(fp);
        close(sock);
        return 1;
      }
      // Ensure the ACK contains both the sequence number and the CRC (at least 4 bytes for seq_num + 4 bytes for CRC)
      if (ack_bytes < sizeof(int32_t) + sizeof(uint32_t))
      {
        printf("Received incomplete ACK. Ignoring...\n");
        continue;
      }

      int32_t net_ack_num;
      uint32_t net_ack_crc;

      memcpy(&net_ack_num, ack_buffer, sizeof(int32_t));
      memcpy(&net_ack_crc, ack_buffer + sizeof(int32_t), sizeof(uint32_t));

      uint32_t received_crc = ntohl(net_ack_crc);
      int ack_num = ntohl(net_ack_num);

      uint32_t calculated_crc =
          crc32((unsigned char *)&net_ack_num, sizeof(int32_t));

      if (calculated_crc != received_crc)
      {
        printf(
            "[recv corrupt ACK] CRC mismatch. Seq_num: %d, recvCRC: %u, "
            "calcCRC: %u\n",
            ack_num, received_crc, calculated_crc);
        continue; // Ignore corrupted ACKs
      }
      //printf("[recv ACK] Seq_num: %d, ACK CRC: %u\n", ack_num, received_crc);

      // Update RTT and slide the window
      if (ack_num >= base && ack_num < next_seq_num)
      {
        acked[ack_num % WINDOW_SIZE] = 1;
        struct timeval ack_time;

        gettimeofday(&ack_time, NULL);
        double sample_rtt =
            get_time_diff(send_times_arr[ack_num % WINDOW_SIZE], ack_time);
        update_timeout(sample_rtt);
        while (acked[base % WINDOW_SIZE] && base < next_seq_num)
        {
          base++;
        }
        // Handle EOF
        if (ack_num == -1 && base == next_seq_num)
        {
          break;
        }
      }
    }
  }

  // After the main loop, send EOF packet and wait for EOF ACK
  if (eof && base == next_seq_num)
  {
    // All data packets have been sent and ACKed
    // Now send EOF and wait for EOF ACK
    int retries = 0, max_retry = 10;
    for (retries = 1; retries <= max_retry; retries++)
    {
      if (send_data_packet(sock, &recv_addr, addr_len, send_buffer, NULL, -1, 0,
                           0) < 0)
      {
        perror("Error sending EOF packet");
        break;
      }
    }
    printf("[send EOF marker] Seq_num: -1 %d times\n", max_retry);
  }

  long file_size = ftell(fp);
  double end_time = get_current_time() / 1000000.0;
  double transmission_time = end_time - start_time;
  double transmission_rate = file_size / transmission_time;

  fclose(fp);
  close(sock);
  freeaddrinfo(server_info);
  printf("[completed]\n");

  printf("File size: %ld bytes\n", file_size);
  printf("Transmission time: %.3f seconds\n", transmission_time);
  printf("Transmission rate: %.3f bytes/second (%.3f Mbps)\n",
         transmission_rate, transmission_rate * 8 / (1024 * 1024));

  return 0;
}
