CC	 	= gcc
CFLAGS	 	= -Wall -g

LDFLAGS	 	= 

all: sendfile recvfile

sendfile: sendfile.o utils.o
	$(CC) $(CFLAGS) -o sendfile sendfile.o utils.o $(LDFLAGS)

recvfile: recvfile.o utils.o
	$(CC) $(CFLAGS) -o recvfile recvfile.o utils.o $(LDFLAGS)

sendfile.o: sendfile.c utils.h
	$(CC) $(CFLAGS) -c sendfile.c

recvfile.o: recvfile.c utils.h
	$(CC) $(CFLAGS) -c recvfile.c

utils.o: utils.c utils.h
	$(CC) $(CFLAGS) -c utils.c

clean:
	rm -f *.o
	rm -f sendfile
	rm -f recvfile
