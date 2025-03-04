#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <stdarg.h>
#include <errno.h>
#include <sys/select.h>

void log_debug(const char* message, ...) {
    va_list args;
    va_start(args, message);
    vprintf(message, args);
    va_end(args);
    printf("\n"); // Add new line for each log entry
}

struct node
{
  int socket;
  struct sockaddr_in client_addr;
  int pending_data; 
  char* buf; // new element: buffer zone for each socket
  struct node *next;
};

void dump(struct node *head, int socket)
{
  struct node *current, *temp;

  current = head;

  while (current->next)
  {
    if (current->next->socket == socket)
    {
      /* remove */
      temp = current->next;
      current->next = temp->next;
      free(temp->buf); /* free the buffer zone for each socket */
      free(temp); /* don't forget to free memory */
      return;
    }
    else
    {
      current = current->next;
    }
  }
}

/* create the data structure associated with a connected socket */
void add(struct node *head, int socket, struct sockaddr_in addr, char* data)
{
  struct node *new_node;

  new_node = (struct node *)malloc(sizeof(struct node));
  new_node->socket = socket;
  new_node->client_addr = addr;
  new_node->pending_data = 0;
  new_node->buf = data; // add the buffer zone for each socket
  new_node->next = head->next;
  head->next = new_node;
}

/*****************************************/
/* main program                          */
/*****************************************/

int main(int argc, char **argv)
{


  /* server socket address variables */
  struct sockaddr_in sin, addr;
  unsigned short server_port = atoi(argv[1]);
  
  /* socket and option variables */
  int sock, new_sock, max;
  int optval = 1;

  

  /* socket address variables for a connected client */
  socklen_t addr_len = sizeof(struct sockaddr_in);

  /* maximum number of pending connection requests */
  int BACKLOG = 5;

  /* variables for select */
  fd_set read_set, write_set;
  struct timeval time_out;
  int select_retval;

  /* number of bytes sent/received */
  int count;

  /* numeric value received */
  int num;

  /* linked list for keeping track of connected sockets */
  struct node head;
  struct node *current, *next;

  /* a buffer to read data */
  char *buf;
  int BUF_LEN = 1e7+10;

  buf = (char *)malloc(BUF_LEN);

  /* initialize dummy head node of linked list */
  head.socket = -1;
  head.next = 0;

  /* create a server socket to listen for TCP connection requests */
  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
  {
    perror("opening TCP socket");
    abort();
  }

  /* set option so we can reuse the port number quickly after a restart */
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0)
  {
    perror("setting TCP socket option");
    abort();
  }

  /* fill in the address of the server socket */
  memset(&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;
  sin.sin_addr.s_addr = INADDR_ANY;
  sin.sin_port = htons(server_port);

  /* bind server socket to the address */
  if (bind(sock, (struct sockaddr *)&sin, sizeof(sin)) < 0)
  {
    // perror("binding socket to address");
    abort();
  }

  /* put the server socket in listen mode */
  if (listen(sock, BACKLOG) < 0)
  {
    // perror("listen on socket failed");
    abort();
  }

  //========================== Actual Server Process ========================== 
  while (1)
  {

    /* set up the file descriptor bit map that select should be watching */
    FD_ZERO(&read_set);  /* clear everything */
    FD_ZERO(&write_set); /* clear everything */

    FD_SET(sock, &read_set); /* put the listening socket in */
    max = sock;              /* initialize max */

    /* put connected sockets into the read and write sets to monitor them */
    for (current = head.next; current; current = current->next)
    {
      FD_SET(current->socket, &read_set);

      if (current->pending_data)
      {
        /* there is data pending to be sent, monitor the socket
                 in the write set so we know when it is ready to take more
                 data */
        FD_SET(current->socket, &write_set);
      }

      if (current->socket > max)
      {
        /* update max if necessary */
        max = current->socket;
      }
    }

    time_out.tv_usec = 100000; /* 1-tenth of a second timeout */
    time_out.tv_sec = 0;

    /* invoke select, make sure to pass max+1 !!! */
    select_retval = select(max + 1, &read_set, &write_set, NULL, &time_out);


    if (select_retval < 0)
    {
      // perror("select failed");
      abort();
    }

    if (select_retval == 0)
    {
      /* no descriptor ready, timeout happened */
      continue;
    }

    if (select_retval > 0) /* at least one file descriptor is ready */
    {
      if (FD_ISSET(sock, &read_set)) /* check the server socket */
      {
        /* there is an incoming connection, try to accept it */
        new_sock = accept(sock, (struct sockaddr *)&addr, &addr_len);

        if (new_sock < 0)
        {
          // perror("error accepting connection");
          abort();
        }
        if (fcntl(new_sock, F_SETFL, O_NONBLOCK) < 0)
        {
          // perror("making socket non-blocking");
          abort();
        }
        //printf("Accepted connection. Client IP address is: %s\n",
               //inet_ntoa(addr.sin_addr));
        
        //new buf for concurrency
        char* new_buf=malloc(sizeof(int64_t)*3);
        
        /* remember this client connection in our linked list */
        add(&head, new_sock, addr,new_buf);
      }

      for (current = head.next; current; current = next)
      {
        next = current->next;

        /* see if we can now do some previously unsuccessful writes */
        if (FD_ISSET(current->socket, &write_set))
        {
          count = send(current->socket, buf, BUF_LEN, MSG_DONTWAIT);
          if (count < 0)
          {
            if (errno == EAGAIN)
            {
            }
            else
            {
              // log_debug("Failed to send data on socket %d: %s", current->socket, strerror(errno));
            }
          }
        }

        if (FD_ISSET(current->socket, &read_set))
        {
          count = recv(current->socket, current->buf, 18, 0);
          if (count <= 0)
          {
            /* something is wrong */
            if (count == 0)
            {
              //printf("Client closed connection. Client IP address is: %s\n", inet_ntoa(current->client_addr.sin_addr));
            }
            else
            {
              // perror("error receiving from a client");
            }

            /* connection is closed, clean up */
            close(current->socket);
            dump(&head, current->socket);
          }
          else
          {
            //log_debug("Received %d bytes from socket %d", count, current->socket);

            int expectedSize = ntohs(*(uint16_t *)current->buf); 
            current->buf=(char *)malloc(expectedSize + 1);
            // memset(current->buf,0,expectedSize);
            
            // size of time and size is 18 bytes
            int totalReceived = 18;
            // receive the chunks of full data
            while (totalReceived < expectedSize)
            {
              int received = recv(current->socket, current->buf, expectedSize - totalReceived, 0);
              if (received < 0)
              {
                if (errno != EAGAIN && errno != EWOULDBLOCK)
                {
                  // log_debug("Failed to receive further data from socket %d: %s", current->socket, strerror(errno));
                  // perror("recv failed");
                  close(current->socket);
                  dump(&head, current->socket);
                  break;
                }
                continue;
              }
              else if (received == 0)
              {
                // log_debug("Connection unexpectedly closed by peer, socket %d", current->socket);
                // printf("Connection closed by peer.\n");
                close(current->socket);
                dump(&head, current->socket);
                break;
              }
              totalReceived += received;
            }
            //log_debug("Complete message received: %d bytes from socket %d", totalReceived, current->socket);

            int totalSent = 0;
            while (totalSent < totalReceived) {
                int sent = send(current->socket, buf + totalSent, totalReceived - totalSent, 0);
                if (sent < 0) {
                  // log_debug("Failed to send data on socket %d: %s", current->socket, strerror(errno));
                    // perror("send failed");
                    break;
                }
                totalSent += sent;
                 //log_debug("Sent %d bytes back to socket %d as echo", totalSent, current->socket);
            }
          }
        }
      }
    }
  }
  return 0;
}


