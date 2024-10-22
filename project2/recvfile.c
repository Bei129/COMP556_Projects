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
#define DATA_SIZE \
  (PKT_SIZE -     \
   sizeof(int32_t) * 4)  // 4(seq_num) + 4(start) 4(data_length) + 4(crc) = 16 bytes

int mkdir_p(const char *path) {
  char tmp[512];
  char *p = NULL;
  size_t len;

  snprintf(tmp, sizeof(tmp), "%s", path);
  len = strlen(tmp);
  if (len == 0) return -1;
  if (tmp[len - 1] == '/') tmp[len - 1] = '\0';

  for (p = tmp + 1; *p; p++) {
    if (*p == '/') {
      *p = '\0';
      if (mkdir(tmp, S_IRWXU) != 0) {
        if (errno != EEXIST) {
          return -1;
        }
      }
      *p = '/';
    }
  }
  if (mkdir(tmp, S_IRWXU) != 0) {
    if (errno != EEXIST) {
      return -1;
    }
  }
  return 0;
}

int main(int argc, char **argv) {
  int sock, opt;
  char *recv_port = NULL;
  struct sockaddr_in recv_addr, sender_addr;
  socklen_t addr_len = sizeof(sender_addr);

  // Parse command line arguments
  while ((opt = getopt(argc, argv, "p:")) != -1) {
    switch (opt) {
      case 'p':
        recv_port = optarg;
        break;
      default:
        printf("Usage: recvfile -p <port>\n");
        return 1;
    }
  }
  if (recv_port == NULL) {
    printf("Port not specified. Usage: recvfile -p <port>\n");
    return 1;
  }

  // Create UDP socket
  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("Error opening socket");
    return 1;
  }

  // Configure receiver address
  memset(&recv_addr, 0, sizeof(recv_addr));
  recv_addr.sin_family = AF_INET;
  recv_addr.sin_addr.s_addr = INADDR_ANY;
  recv_addr.sin_port = htons(atoi(recv_port));

  // Bind the socket
  if (bind(sock, (struct sockaddr *)&recv_addr, sizeof(recv_addr)) < 0) {
    perror("Error binding socket");
    close(sock);
    return 1;
  }

  printf("Listening on port %s...\n", recv_port);
  fflush(stdout);

  char buffer[PKT_SIZE];
  int expected_seq_num = 0;
  int bytes_received;

  // Receive file name and path
  char file_info[256];
  ssize_t file_info_bytes =
      recvfrom(sock, file_info, sizeof(file_info) - 1, 0,
               (struct sockaddr *)&sender_addr, &addr_len);
  if (file_info_bytes < 0) {
    perror("Error receiving file info");
    close(sock);
    return 1;
  }
  file_info[file_info_bytes] = '\0';
  printf("[recv file info] File name: %s\n", file_info);
//____________________________________CRC check
  char path_copy[256];
  strncpy(path_copy, file_info, sizeof(path_copy) - 1);
  path_copy[sizeof(path_copy) - 1] = '\0';

  // Find the last '/' to separate directory and file name
  char *last_slash = strrchr(path_copy, '/');
  char *subdir = NULL;
  char *fname = NULL;

  if (last_slash != NULL) {
    *last_slash = '\0';
    subdir = path_copy;
    fname = last_slash + 1;

    if (mkdir_p(subdir) != 0) {
      perror("Error creating directories");
      close(sock);
      return 1;
    }
  } else {
    // No subdirectory
    subdir = ".";
    fname = path_copy;
  }

  // Create the received file path
  char output_filename[512];
  snprintf(output_filename, sizeof(output_filename), "%s/%s.recv", subdir,
           fname);

  FILE *fp;
  if ((fp = fopen(output_filename, "wb")) == NULL) {
    perror("Error opening file for writing");
    close(sock);
    return 1;
  }

  while (1) {
    // Receive data packet
    bytes_received = recvfrom(sock, buffer, sizeof(buffer), 0,
                              (struct sockaddr *)&sender_addr, &addr_len);

    if (bytes_received < 0) {
      perror("Error receiving packet");
      fclose(fp);
      close(sock);
      return 1;
    }

    if (bytes_received <
        sizeof(int32_t) * 3 + sizeof(uint32_t)) {  // Minimum packet size
      printf("[recv corrupt packet] Packet too small.\n");
      continue;
    }

    // Extract seq_num and data_length, convert to host byte order
    int32_t net_seq_num, net_start_offset, net_data_length;
    memcpy(&net_seq_num, buffer, sizeof(int32_t));
    memcpy(&net_start_offset, buffer + sizeof(int32_t), sizeof(int32_t));
    memcpy(&net_data_length, buffer + sizeof(int32_t)*2, sizeof(int32_t));

    int seq_num = ntohl(net_seq_num);
    int start_offset = ntohl(net_start_offset);
    int data_length = ntohl(net_data_length);

    // Check if data_length is valid
    if (data_length < 0 || data_length > DATA_SIZE) {
      printf("[recv corrupt packet] Invalid data_length: %d\n", data_length);
      continue;
    }

    // Check if the total packet size matches
    if (bytes_received !=
        sizeof(int32_t) * 3 + data_length + sizeof(uint32_t)) {
      printf(
          "[recv corrupt packet] Packet size mismatch. Expected: %zu, "
          "Received: %d\n",
          sizeof(int32_t) * 3 + data_length + sizeof(uint32_t), bytes_received);
      continue;
    }

    // Extract received CRC, convert to host byte order
    uint32_t received_crc;
    memcpy(&received_crc, buffer + sizeof(int32_t) * 3 + data_length,
           sizeof(uint32_t));
    received_crc = ntohl(received_crc);

    // Calculate CRC over the first 12 + data_length bytes
    uint32_t calculated_crc =
        crc32((unsigned char *)buffer, sizeof(int32_t) * 3 + data_length);

    if (calculated_crc != received_crc) {
      printf(
          "[recv corrupt packet] CRC mismatch! Received CRC: %u, Calculated "
          "CRC: %u\n",
          received_crc, calculated_crc);
      continue;  // Ignore corrupted packets
    }

    // Check if this is the EOF packet
    if (seq_num == -1) {
      printf("Received EOF marker. File transfer complete.\n");
      // Send ACK for EOF
      char ack_buffer[32];
      snprintf(ack_buffer, sizeof(ack_buffer), "%d", seq_num);
      if (sendto(sock, ack_buffer, strlen(ack_buffer) + 1, 0,
                 (struct sockaddr *)&sender_addr, addr_len) < 0) {
        perror("Error sending ACK for EOF");
      }
      break;
    }

    printf(
        "[recv data] Seq_num: %d, Start: %d, Length: %d\n",
        seq_num, start_offset, data_length);

    fflush(stdout);

    // Check if this is the expected sequence number
    if (seq_num == expected_seq_num) {
      // Write data to file
      size_t write_bytes =
          fwrite(buffer + sizeof(int32_t) * 3, 1, data_length, fp);
      printf("wrote %zu bytes to file.\n", write_bytes);

      expected_seq_num++;

      // Send ACK
      char ack_buffer[32];
      snprintf(ack_buffer, sizeof(ack_buffer), "%d", seq_num);
      if (sendto(sock, ack_buffer, strlen(ack_buffer) + 1, 0,
                 (struct sockaddr *)&sender_addr, addr_len) < 0) {
        perror("Error sending ACK");
      }
      printf("[send ACK] Seq_num: %d\n", seq_num);
    } else {
      // Resend the last ACK
      char ack_buffer[32];
      snprintf(ack_buffer, sizeof(ack_buffer), "%d", expected_seq_num - 1);
      if (sendto(sock, ack_buffer, strlen(ack_buffer) + 1, 0,
                 (struct sockaddr *)&sender_addr, addr_len) < 0) {
        perror("Error resending ACK");
      }
      printf("[resend ACK] Seq_num: %d\n", expected_seq_num - 1);
    }
  }

  fclose(fp);
  close(sock);

  printf("File received and saved as %s.\n", output_filename);
  return 0;
}
