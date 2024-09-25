#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <sys/time.h>
#include <stdarg.h>
#include <errno.h>

// uint64_t htobe64(uint64_t val) {
//     return ((val >> 56) & 0x00000000000000FF) |
//            ((val >> 40) & 0x000000000000FF00) |
//            ((val >> 24) & 0x0000000000FF0000) |
//            ((val >> 8)  & 0x00000000FF000000) |
//            ((val << 8)  & 0x000000FF00000000) |
//            ((val << 24) & 0x0000FF0000000000) |
//            ((val << 40) & 0x00FF000000000000) |
//            ((val << 56) & 0xFF00000000000000);
// }

void log_debug(const char* message, ...) {
    va_list args;
    va_start(args, message);
    vprintf(message, args);
    va_end(args);
    printf("\n"); // Add new line for each log entry
}

/* simple client, takes two parameters, the server domain name,
   and the server port number */

int main(int argc, char **argv)
{

  //===========================
  /* our client socket */
  int sock;

  /* variables for identifying the server */
  unsigned int server_addr;
  struct sockaddr_in sin;
  struct addrinfo *getaddrinfo_result, hints;

  struct timeval start;

  /* convert server domain name to IP address */
  memset(&hints, 0, sizeof(struct addrinfo));
  hints.ai_family = AF_INET; /* indicates we want IPv4 */

  /*调试手动参数，启动后需删除*/
  // argv[1]="127.0.0.1";
  // argv[2]="18277";
  // argv[3]="60000";
  // argv[4]="10";


  if (getaddrinfo(argv[1], NULL, &hints, &getaddrinfo_result) == 0)
  {
    server_addr = (unsigned int)((struct sockaddr_in *)(getaddrinfo_result->ai_addr))->sin_addr.s_addr;
    freeaddrinfo(getaddrinfo_result);
  }

  /* server port number */
  unsigned short server_port = atoi(argv[2]);

  char *buffer, *sendbuffer;
  int size = atoi(argv[3]);
  int count = atoi(argv[4]);
  int num;

  if (size < 18 || size > 65535)
  {
    fprintf(stderr, "Size must be between 18 and 65535 bytes\n");
    exit(EXIT_FAILURE);
  }

  if (count < 1 || count > 10000)
  {
    fprintf(stderr, "Count must be between 1 and 10000\n");
    exit(EXIT_FAILURE);
  }

  buffer = (char *)malloc(size+10);
  if (!buffer)
  {
    perror("failed to allocated buffer");
    abort();
  }

  sendbuffer = (char *)malloc(size);
  if (!sendbuffer)
  {
    perror("failed to allocated sendbuffer");
    abort();
  }

  /* create a socket */
  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
  {
    perror("opening TCP socket");
    abort();
  }

  /* fill in the server's address */
  memset(&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;
  sin.sin_addr.s_addr = server_addr;
  sin.sin_port = htons(server_port);

  /* connect to the server */
  if (connect(sock, (struct sockaddr *)&sin, sizeof(sin)) < 0)
  {
    perror("connect to server failed");
    abort();
  }
  //==========================

  for (int i = 0; i < count; i++)
  {
    memset(buffer, 0, size);//TF: randomize buffer
    memset(sendbuffer, 0, 18);
    gettimeofday(&start, NULL);                             // get current time
    *(uint16_t *)(sendbuffer) = htons(size);                // store size
    *(int64_t *)(sendbuffer + 2) = htobe64(start.tv_sec);   // store tvsec
    *(int64_t *)(sendbuffer + 10) = htobe64(start.tv_usec); // store tvusec


    printf("Send %d bytes from %zu bytes sendbuffer.\nData:", size,sizeof(sendbuffer));
    for (int j = 0; j < size; j++)
    {
      printf("%02X ", ((unsigned char *)sendbuffer)[j]);
    }
    printf("\n");

    // Log the attempt to send data
    log_debug("Attempt to send data packet %d", i + 1);

    int totalSent = 0, bytesSent;
    while (totalSent < size)
    {
      bytesSent = send(sock, sendbuffer + totalSent, size - totalSent, 0);
      if (bytesSent < 0)
      {
        perror("send() failed");
        free(sendbuffer);
        free(buffer);
        close(sock);
        exit(EXIT_FAILURE);
        // Log send failure
        log_debug("Failed to send data: %s", strerror(errno));
        break;
      }
      totalSent += bytesSent;
      // Log each chunk sent
        log_debug("Sent %d bytes; Total sent: %d bytes", bytesSent, totalSent);

    }

    int totalReceived = 0, bytesReceived;
    while (totalReceived < size)
    {
      bytesReceived = recv(sock, buffer + totalReceived, size - totalReceived, 0);
      if (bytesReceived < 0)
      {
        perror("recv() failed");
        free(sendbuffer);
        free(buffer);
        close(sock);
        exit(EXIT_FAILURE);
        log_debug("Failed to receive data: %s", strerror(errno));
      }
      else if (bytesReceived == 0)
      {
        fprintf(stderr, "The server closed the connection prematurely.\n");
        free(sendbuffer);
        free(buffer);
        close(sock);
        exit(EXIT_FAILURE);
      }
      totalReceived += bytesReceived;
      // Log each chunk received
    log_debug("Received %d bytes; Total received: %d bytes", bytesReceived, totalReceived);

    }

    struct timeval end;
    gettimeofday(&end, NULL); // Record end time

    // Calculate the round-trip time in milliseconds
    double rtt = (end.tv_sec - start.tv_sec) * 1000.0 +
                 (end.tv_usec - start.tv_usec) / 1000.0;

    // // Print the complete message received
    // printf("Received message: ");
    // for (int j = 0; j < size; j++)
    // {
    //   printf("%02X ", (unsigned char)recvBuffer[j]);
    // }

    printf("\n");
    printf("[%d]Round-trip time: %.3f ms\n\n",i, rtt);
  }

  // Clean up
  free(buffer);
  free(sendbuffer);
  close(sock);

  return 0;
}


