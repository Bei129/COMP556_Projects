CC = gcc
CFLAGS = -Wall -g

LDFLAGS = 

all: sendfile recvfile

sendfile: sendfile.o crc.o
	$(CC) $(CFLAGS) -o sendfile sendfile.o crc.o $(LDFLAGS)

recvfile: recvfile.o crc.o
	$(CC) $(CFLAGS) -o recvfile recvfile.o crc.o $(LDFLAGS)

sendfile.o: sendfile.c crc.h
	$(CC) $(CFLAGS) -c sendfile.c

recvfile.o: recvfile.c crc.h 
	$(CC) $(CFLAGS) -c recvfile.c

crc.o: crc.c crc.h  
	$(CC) $(CFLAGS) -c crc.c

clean:
	rm -f *.o
	rm -f sendfile
	rm -f recvfile
