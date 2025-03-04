#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "crc.h"

// Define packet size
#define PKT_SIZE 1024
#define DATA_SIZE    \
  (PKT_SIZE -        \
   sizeof(int32_t) * \
       4) // 4(seq_num) + 4(start) + 4(data_length) + 4(crc) = 16 bytes
#define WINDOW_SIZE 20

int mkdir_p(const char *path)
{
  if (!path || path[0] == '\0')
    return -1; // Handle null/empty path

  char tmp[512];
  char *p = NULL;
  size_t len;

  if (snprintf(tmp, sizeof(tmp), "%s", path) >= sizeof(tmp))
  {
    return -1; // Prevent buffer overflow
  }

  len = strlen(tmp);
  if (len == 0)
    return -1;
  if (tmp[len - 1] == '/')
    tmp[len - 1] = '\0';

  for (p = tmp + 1; *p; p++)
  {
    if (*p == '/')
    {
      *p = '\0';
      if (mkdir(tmp, S_IRWXU) != 0 && errno != EEXIST)
      {
        return -1;
      }
      *p = '/';
    }
  }

  if (mkdir(tmp, S_IRWXU) != 0 && errno != EEXIST)
  {
    return -1;
  }

  return 0;
}

int send_ack(int seq_num, int sock, struct sockaddr_in *sender_addr, socklen_t addr_len)
{

  char ack_buffer[32];
  int32_t net_seq_num = htonl(seq_num);
  memcpy(ack_buffer, &net_seq_num, sizeof(int32_t));

  uint32_t ack_crc = crc32((unsigned char *)&net_seq_num, sizeof(int32_t));
  ack_crc = htonl(ack_crc);

  memcpy(ack_buffer + sizeof(int32_t), &ack_crc, sizeof(u_int32_t));

  if (sendto(sock, ack_buffer, sizeof(int32_t) + sizeof(u_int32_t), 0,
             (struct sockaddr *)sender_addr, addr_len) < 0)
  {
    perror("Error sending ACK");
    return -1;
  }
  printf("[send ACK] Seq_num: %d\n", seq_num);

  return 0;
}

int main(int argc, char **argv)
{
  int sock, opt;
  char *recv_port = NULL;
  struct sockaddr_in recv_addr, sender_addr;
  socklen_t addr_len = sizeof(sender_addr);

  // Parse command line arguments
  while ((opt = getopt(argc, argv, "p:")) != -1)
  {
    switch (opt)
    {
    case 'p':
      recv_port = optarg;
      break;
    default:
      printf("Usage: recvfile -p <port>\n");
      return 1;
    }
  }

  if (recv_port == NULL)
  {
    printf("Port not specified. Usage: recvfile -p <port>\n");
    return 1;
  }

  // Create UDP socket
  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    perror("Error opening socket");
    return 1;
  }

  // Configure receiver address
  memset(&recv_addr, 0, sizeof(recv_addr));
  recv_addr.sin_family = AF_INET;
  recv_addr.sin_addr.s_addr = INADDR_ANY;
  recv_addr.sin_port = htons(atoi(recv_port));

  // Bind the socket
  if (bind(sock, (struct sockaddr *)&recv_addr, sizeof(recv_addr)) < 0)
  {
    perror("Error binding socket");
    close(sock);
    return 1;
  }

  printf("Listening on port %s...\n", recv_port);
  fflush(stdout);

  // Receive file info and CRC check
  char file_info[256];
  while (1)
  {
    ssize_t file_info_bytes =
        recvfrom(sock, file_info, sizeof(file_info) - 1, 0,
                 (struct sockaddr *)&sender_addr, &addr_len);
    if (file_info_bytes < 0)
    {
      perror("Error receiving file info");
      close(sock);
      return 1;
    }
    file_info[file_info_bytes] = '\0';

    // Extract file name and CRC
    char *recv_filename = strtok(file_info, ":");
    char *recv_crc_str = strtok(NULL, ":");

    if (recv_filename == NULL || recv_crc_str == NULL)
    {
      printf("Error parsing received file info\n");
      continue; // Request resend instead of exiting
    }

    uint32_t recv_crc = (uint32_t)strtoul(recv_crc_str, NULL, 10);
    uint32_t calculated_crc =
        crc32((unsigned char *)recv_filename, strlen(recv_filename));

    // Verify the CRC
    if (calculated_crc == recv_crc)
    {
      // Send ACK if CRC is correct
      char ack_buffer[32] = "ACK";
      if (sendto(sock, ack_buffer, strlen(ack_buffer) + 1, 0,
                 (struct sockaddr *)&sender_addr, addr_len) < 0)
      {
        perror("Error sending ACK for file info");
      }
      printf("[recv file info] File name: %s\n", recv_filename);

      // Proceed with file receiving
      break;
    }
    else
    {
      // Send NACK if CRC is incorrect
      char nack_buffer[32] = "NACK";
      if (sendto(sock, nack_buffer, strlen(nack_buffer) + 1, 0,
                 (struct sockaddr *)&sender_addr, addr_len) < 0)
      {
        perror("Error sending NACK for file info");
      }
      printf("File info CRC mismatch. Requesting resend...\n");
    }
  }

  // Create directories
  char path_copy[256];
  strncpy(path_copy, file_info, sizeof(path_copy) - 1);
  path_copy[sizeof(path_copy) - 1] = '\0';

  // separate directory and file name
  char *last_slash = strrchr(path_copy, '/');
  char *subdir = NULL;
  char *fname = NULL;

  if (last_slash != NULL)
  {
    *last_slash = '\0';
    subdir = path_copy;
    fname = last_slash + 1;

    if (mkdir_p(subdir) != 0)
    {
      perror("Error creating directories");
      close(sock);
      return 1;
    }
  }
  else
  {
    // No subdirectory
    subdir = ".";
    fname = path_copy;
  }

  // Create the received file path
  char output_filename[512];
  snprintf(output_filename, sizeof(output_filename), "%s/%s.recv", subdir,
           fname);

  FILE *fp;
  if ((fp = fopen(output_filename, "wb")) == NULL)
  {
    perror("Error opening file for writing");
    close(sock);
    return 1;
  }

  // Start receiving data packets
  // init vaaribles for window sliding
  char buffer[PKT_SIZE];
  char window[WINDOW_SIZE][PKT_SIZE];
  int acked[WINDOW_SIZE];
  int length[WINDOW_SIZE];
  memset(window, 0, sizeof(window));
  memset(length, 0, sizeof(length));
  memset(acked, 0, sizeof(acked));

  int bytes_received;
  int window_start = 0;
  int expected_seq_num = 0;

  while (1)
  {
    bytes_received = recvfrom(sock, buffer, sizeof(buffer), 0,
                              (struct sockaddr *)&sender_addr, &addr_len);
    if (bytes_received < 0)
    {
      perror("Error receiving packet");
      fclose(fp);
      close(sock);
      return 1;
    }

    // Verify minimum packet size
    if (bytes_received < sizeof(int32_t) * 3 + sizeof(uint32_t))
    {
      printf("[recv corrupt packet] Packet too small.\n");
      continue;
    }

    // Extract sequence number and data length
    int32_t net_seq_num, net_start_offset, net_data_length;
    memcpy(&net_seq_num, buffer, sizeof(int32_t));
    memcpy(&net_start_offset, buffer + sizeof(int32_t), sizeof(int32_t));
    memcpy(&net_data_length, buffer + sizeof(int32_t) * 2, sizeof(int32_t));

    int seq_num = ntohl(net_seq_num);
    int start_offset = ntohl(net_start_offset);
    int data_length = ntohl(net_data_length);

    // Check if data_length is valid
    if (data_length < 0 || data_length > DATA_SIZE)
    {
      printf("[recv corrupt packet] Invalid data_length: %d\n", data_length);
      continue;
    }

    // Check if the total packet size matches
    if (bytes_received !=
        sizeof(int32_t) * 3 + data_length + sizeof(uint32_t))
    {
      printf(
          "[recv corrupt packet] Packet size mismatch. Expected: %zu, "
          "Received: %d\n",
          sizeof(int32_t) * 3 + data_length + sizeof(uint32_t), bytes_received);
      continue;
    }

    // Extract received CRC
    uint32_t received_crc;
    memcpy(&received_crc, buffer + sizeof(int32_t) * 3 + data_length,
           sizeof(uint32_t));
    received_crc = ntohl(received_crc);

    // Calculate CRC over the first 12 + data_length bytes
    uint32_t calculated_crc =
        crc32((unsigned char *)buffer, sizeof(int32_t) * 3 + data_length);

    if (calculated_crc != received_crc)
    {
      printf("[recv corrupt packet] Mismatch recvCRC: %u, calcCRC: %u\n",
             received_crc, calculated_crc);
      continue; // Ignore corrupted packets
    }

    // Check EOF packet
    if (seq_num == -1 && data_length == 0)
    {
      printf(
          "Received EOF marker. Checking if all packets have been "
          "received...\n");

      // Write any buffered data up to expected_seq_num
      while (acked[expected_seq_num % WINDOW_SIZE])
      {
        // size_t write_bytes =
            fwrite(window[expected_seq_num % WINDOW_SIZE] + sizeof(int32_t) * 3,
                   1, length[expected_seq_num % WINDOW_SIZE], fp);
        // printf("wrote %zu bytes to file.\n", write_bytes);
        length[expected_seq_num % WINDOW_SIZE] = 0;
        acked[expected_seq_num % WINDOW_SIZE] = 0;
        expected_seq_num++;
      }
      window_start = expected_seq_num;

      // Send ACK for EOF
      char ack_buffer[32];
      int32_t net_ack_num = htonl(seq_num);
      memcpy(ack_buffer, &net_ack_num, sizeof(int32_t));

      uint32_t ack_crc = crc32((unsigned char *)&net_ack_num, sizeof(int32_t));
      ack_crc = htonl(ack_crc);

      memcpy(ack_buffer + sizeof(int32_t), &ack_crc, sizeof(uint32_t));

      if (sendto(sock, ack_buffer, sizeof(int32_t) + sizeof(uint32_t), 0,
                 (struct sockaddr *)&sender_addr, addr_len) < 0)
      {
        perror("Error sending ACK for EOF");
      }
      printf("[send EOF ACK] Seq_num: %d\n", seq_num);
      printf("EOF ACK sent. Transmission complete.\n");
      break;
    }

    // sliding window part
    if (seq_num >= window_start && seq_num < window_start + WINDOW_SIZE &&
        acked[seq_num % WINDOW_SIZE])
    {
      // duplicate
      printf("[recv data] Seq_num: %d, Start: %d, Length: %d, IGNORED (duplicate)\n",
             seq_num, start_offset, data_length);
      continue;
    }
    if (seq_num < window_start)
    {
      // ignore
      printf("[recv data] Seq_num: %d, Start: %d, Length: %d, IGNORED(ACK-before window)\n",
             seq_num, start_offset, data_length);
      send_ack(seq_num, sock, &sender_addr, addr_len);
      continue;
    }
    if (expected_seq_num == seq_num)
    {
      // in-order
      printf(
          "[recv data] Seq_num: %d, Start: %d, Length: %d ACCEPTED[in-order]\n",
          seq_num, start_offset, data_length);
      // size_t write_bytes =
          fwrite(buffer + sizeof(int32_t) * 3, 1, data_length, fp);
      //printf("wrote %zu bytes to file.\n", write_bytes);

      expected_seq_num++;

      while (acked[expected_seq_num % WINDOW_SIZE])
      {
        // size_t write_bytes =
            fwrite(window[expected_seq_num % WINDOW_SIZE] + sizeof(int32_t) * 3,
                   1, length[expected_seq_num % WINDOW_SIZE], fp);
        //printf("wrote %zu bytes to file.\n", write_bytes);
        length[expected_seq_num % WINDOW_SIZE] = 0;
        acked[expected_seq_num % WINDOW_SIZE] = 0;
        expected_seq_num++;
      }
      window_start = expected_seq_num;
    }
    else if (seq_num < window_start + WINDOW_SIZE &&
             seq_num > expected_seq_num)
    {
      // out-order
      printf(
          "[recv data] Seq_num: %d, Start: %d, Length: %d "
          "ACCEPTED[out-of-order]\n",
          seq_num, start_offset, data_length);
      memcpy(window[seq_num % WINDOW_SIZE], buffer, bytes_received);
      length[seq_num % WINDOW_SIZE] = data_length;
      acked[seq_num % WINDOW_SIZE] = 1;
    }
    else
    {
      // out-of-window ignore
      printf("[recv data] Seq_num: %d, Start: %d, Length: %d, IGNORED(out-of-window)\n",
             seq_num, start_offset, data_length);
    }

    // Send ACK for the received packet
    send_ack(seq_num, sock, &sender_addr, addr_len);
  }

  fclose(fp);
  close(sock);
  printf("[complete] File received and saved as %s.\n", output_filename);
  return 0;
}
