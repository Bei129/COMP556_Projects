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

#include "utils.h"
#include "crc.h"

// define packet size
#define PKT_SIZE 1024

int main(int argc, char **argv) {
  int sock, opt;
  char *recv_port = NULL;
  struct sockaddr_in recv_addr, sender_addr;
  socklen_t addr_len = sizeof(sender_addr);
  FILE *fp;
  char filename[256] = "received_file.txt.recv";  // Default save file name

  // parse command line arguments
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

  // open the receive file
  if ((fp = fopen(filename, "wb")) == NULL) {
    perror("Error opening file for writing");
    return 1;
  }

  // create a UDP socket
  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("Error opening socket");
    return 1;
  }

  // configure the receiver address
  memset(&recv_addr, 0, sizeof(recv_addr));
  recv_addr.sin_family = AF_INET;
  recv_addr.sin_addr.s_addr = INADDR_ANY;
  recv_addr.sin_port = htons(atoi(recv_port));

  // bind the socket
  if (bind(sock, (struct sockaddr *)&recv_addr, sizeof(recv_addr)) < 0) {
    perror("Error binding socket");
    return 1;
  }

  // ensure the program prints to the console immediately
  printf("Listening on port %s...\n", recv_port);
  fflush(stdout);  // Ensure immediate output to console

  // initializes the receive buffer
  char buffer[PKT_SIZE];
  int expected_seq_num = 0;
  int bytes_received;

  // receive file data
  while (1) {
    // receive packet
    bytes_received = recvfrom(sock, buffer, PKT_SIZE, 0,
                              (struct sockaddr *)&sender_addr, &addr_len);
    if (bytes_received < 0) {
      perror("Error receiving packet");
      return 1;
    }

    // Extract the sequence number of the packet (first 4 bytes)
    int seq_num = *(int *)buffer;

    // EOF check
    if (seq_num == -1) {
      printf("Received EOF marker. File transfer complete.\n");
      break;
    }

    printf("[recv data] Seq_num: %d, Bytes: %d from %s:%d\n", seq_num,
           bytes_received, inet_ntoa(sender_addr.sin_addr),
           ntohs(sender_addr.sin_port));

    fflush(stdout);

    // check whether it is the expected serial number
    if (seq_num == expected_seq_num) {
      // Write the data to the file
      size_t write_bytes =
          fwrite(buffer + sizeof(int), 1, bytes_received - sizeof(int), fp);
      printf("Wrote %zu bytes to file.\n", write_bytes);
      expected_seq_num++;

      // Send ACK to confirm receipt of the serial number
      char ack_buffer[32];
      sprintf(ack_buffer, "%d", seq_num);
      if (sendto(sock, ack_buffer, sizeof(ack_buffer), 0,
                 (struct sockaddr *)&sender_addr, addr_len) < 0) {
        perror("Error sending ACK");
      }
      printf("[send ACK] Seq_num: %d\n", seq_num);
    } else {
      // Resend the last ACK for the expected sequence number
      char ack_buffer[32];
      sprintf(ack_buffer, "%d", expected_seq_num - 1);
      if (sendto(sock, ack_buffer, sizeof(ack_buffer), 0,
                 (struct sockaddr *)&sender_addr, addr_len) < 0) {
        perror("Error resending ACK");
      }
      printf("[resend ACK] Seq_num: %d\n", expected_seq_num - 1);
    }
  }

  fclose(fp);
  close(sock);

  printf("File received and saved as %s.\n", filename);
  return 0;
}
