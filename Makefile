CC	 	= gcc
LD	 	= gcc
CFLAGS	 	= -Wall -g

LDFLAGS	 	= 
DEFS 	 	=

all:	server_num_test client_num_test

server_num_test: server_num_test.c
	$(CC) $(DEFS) $(CFLAGS) $(LIB) -o server_num_test server_num_test.c

client_num_test: client_num_test.c
	$(CC) $(DEFS) $(CFLAGS) $(LIB) -o client_num_test client_num_test.c



clean:
	rm -f *.o
	rm -f *~
	rm -f core.*
	rm -f server_num_test
	rm -f client_num_test

