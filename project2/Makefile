CC = gcc
CFLAGS = -Wall -g

LDFLAGS = 

all: sendfile recvfile

sendfile: sendfile.o utils.o crc.o
	$(CC) $(CFLAGS) -o sendfile sendfile.o utils.o crc.o $(LDFLAGS)

recvfile: recvfile.o utils.o crc.o
	$(CC) $(CFLAGS) -o recvfile recvfile.o utils.o crc.o $(LDFLAGS)

sendfile.o: sendfile.c utils.h crc.h
	$(CC) $(CFLAGS) -c sendfile.c

recvfile.o: recvfile.c utils.h crc.h 
	$(CC) $(CFLAGS) -c recvfile.c

utils.o: utils.c utils.h
	$(CC) $(CFLAGS) -c utils.c

crc.o: crc.c crc.h  
	$(CC) $(CFLAGS) -c crc.c

clean:
	rm -f *.o
	rm -f sendfile
	rm -f recvfile
