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

#include "utils.h"
#include "crc.h"

// define packet size and window size
#define PKT_SIZE 1024
#define WINDOW_SIZE 5
#define TIME_OUT 5 // need more test

struct packet
{
  int seq_num;
  char data[PKT_SIZE];
  int acked;
  struct timeval send_time;
};

int main(int argc, char **argv)
{
  int sock, opt;
  char *recv_host = NULL, *recv_port = NULL, *filename = NULL;
  struct timeval timeout;
  struct sockaddr_in recv_addr, send_addr;
  struct addrinfo *server_info, hints;
  socklen_t addr_len = sizeof(send_addr);

  FILE *fp;

  // parse command line arguments
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

  // open the file
  if ((fp = fopen(filename, "rb")) == NULL)
  {
    perror("Error opening file");
    return 1;
  }

  // identify the server
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;

  if (getaddrinfo(recv_host, recv_port, &hints, &server_info) != 0)
  {
    perror("Error resolving server address");
    return 1;
  }

  // initialize and connect socket
  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    perror("Error opening socket");
    return 1;
  }

  // config receiver address
  memset(&recv_addr, 0, sizeof(recv_addr));
  recv_addr.sin_family = AF_INET;
  recv_addr.sin_port = htons(atoi(recv_port));
  recv_addr.sin_addr = ((struct sockaddr_in *)server_info->ai_addr)->sin_addr;

  // config local address
  memset(&send_addr, 0, sizeof(send_addr));
  send_addr.sin_family = AF_INET;
  send_addr.sin_addr.s_addr = INADDR_ANY;
  send_addr.sin_port = htons(0);

  if (bind(sock, (struct sockaddr *)&send_addr, sizeof(send_addr)) < 0)
  {
    perror("Error binding socket");
    return 1;
  }

  // After socket is bound, call getsockname to get the assigned port
  if (getsockname(sock, (struct sockaddr *)&send_addr, &addr_len) == -1)
  {
    perror("getsockname");
  }
  else
  {
    printf("Bound to local port: %d\n",
           ntohs(send_addr.sin_port)); // Output the randomly assigned local
                                       // port number
  }

  // create sliding window and buffer
  struct packet window[WINDOW_SIZE];
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
    window[i].seq_num = -1;
    window[i].acked = 0;
  }

  char buffer[PKT_SIZE];
  int base = 0, next_seq_num = 0;
  int bytes_read, total_packets, eof = 0;

  // send file
  while (!eof || base < next_seq_num)
  {
    while (next_seq_num < base + WINDOW_SIZE && !eof)
    {
      bytes_read = fread(window[next_seq_num % WINDOW_SIZE].data + sizeof(int),
                         1, PKT_SIZE - sizeof(int), fp);
      if (bytes_read <= 0)
      {
        eof = 1; // "end of file" marker packet
        int end_marker = -1;
        memcpy(window[next_seq_num % WINDOW_SIZE].data, &end_marker,
               sizeof(int));
        if (sendto(sock, window[next_seq_num % WINDOW_SIZE].data, sizeof(int),
                   0, (struct sockaddr *)&recv_addr, addr_len) < 0)
        {
          perror("Error sending EOF marker packet");
          return 1;
        }
        printf("[send EOF marker] Seq_num: %d\n", end_marker);
        break;
      }

      // Add sequence number to the first 4 bytes
      window[next_seq_num % WINDOW_SIZE].seq_num = next_seq_num;
      memcpy(window[next_seq_num % WINDOW_SIZE].data, &next_seq_num,
             sizeof(int));

      // calculate CRC
      uint32_t crc_value = crc32(window[next_seq_num % WINDOW_SIZE].data + sizeof(int), bytes_read);
      memcpy(window[next_seq_num % WINDOW_SIZE].data + sizeof(int) + bytes_read, &crc_value, sizeof(uint32_t));

      // send packet
      if (sendto(sock, window[next_seq_num % WINDOW_SIZE].data,
                 bytes_read + sizeof(int) + sizeof(uint32_t), 0, (struct sockaddr *)&recv_addr,
                 addr_len) < 0)
      {
        perror("Error sending packet");
        return 1;
      }

      printf("[send data] Seq_num: %d, Bytes: %d\n", next_seq_num, bytes_read);
      gettimeofday(&window[next_seq_num % WINDOW_SIZE].send_time, NULL);
      window[next_seq_num % WINDOW_SIZE].acked = 0;
      next_seq_num++;
    }

    // set timeout
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock, &read_fds);

    timeout.tv_sec = TIME_OUT;
    timeout.tv_usec = 0;

    int select_retval = select(sock + 1, &read_fds, NULL, NULL, &timeout);
    if (select_retval == -1)
    {
      perror("select error");
      return 1;
    }

    // handle time out
    if (select_retval == 0)
    {
      // check which packets time out and retransmit them
      struct timeval current_time;
      gettimeofday(&current_time, NULL);

      for (int i = base; i < next_seq_num; i++)
      {
        struct packet *pkt = &window[i % WINDOW_SIZE];
        if (!pkt->acked)
        {
          struct timeval diff;

          if (diff.tv_sec >= TIME_OUT)
          {
            // resending
            if (sendto(sock, pkt->data, PKT_SIZE, 0,
                       (struct sockaddr *)&recv_addr, addr_len) < 0)
            {
              perror("Error resending packet");
              return 1;
            }
            printf("[resend data] Seq_num: %d\n", pkt->seq_num);
            gettimeofday(&pkt->send_time, NULL); // update send time
          }
        }
      }
    }
    else
    {
      // receive ACK
      char ack_buffer[32];
      if (recvfrom(sock, ack_buffer, sizeof(ack_buffer), 0,
                   (struct sockaddr *)&recv_addr, &addr_len) < 0)
      {
        perror("Error receiving ACK");
        return 1;
      }
      // debug
      printf("[send data] Seq_num: %d, Bytes: %d to %s:%s\n", next_seq_num,
             bytes_read, recv_host, recv_port);

      int ack_num = atoi(ack_buffer);
      // debug
      printf("[recv ACK] Seq_num: %d from %s:%d\n", ack_num,
             inet_ntoa(recv_addr.sin_addr), ntohs(recv_addr.sin_port));

      // update the window, acknowledge the received package
      if (ack_num >= base && ack_num < next_seq_num)
      {
        window[ack_num % WINDOW_SIZE].acked = 1;
        while (window[base % WINDOW_SIZE].acked && base < next_seq_num)
        {
          base++;
        }
      }
    }
  }

  fclose(fp);
  close(sock);
  freeaddrinfo(server_info);

  printf("File transfer completed.\n");
  return 0;
}