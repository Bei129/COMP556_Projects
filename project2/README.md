# COMP556_Project2

## Project Contributors

- Fei Teng (ft28@rice.edu)

- Jiawen Wang (jw223@rice.edu)

- Siyu Chen (sc302@rice.edu)

- Lingyi Xu (lx28@rice.edu)

  

## Project File Structure

- `crc.h`: Declares CRC-related functions and constants for use by other source files.

- `crc.c`: Implements the CRC checksum functionality to ensure data integrity and accuracy during transmission.

- `sendfile.c`: Implements the sender program, which reads a file and transmits it using the sliding window protocol.

- `recvfile.c`: Implements the receiver program, which listens for incoming packets, verifies the data, and writes it to a local file.

- `command.txt`: Contains a series of commands used for testing the `sendfile` and `recvfile` programs. It includes:

- `Makefile`: A build script that compiles both `sendfile.c` and `recvfile.c`.




## File Descriptions

This project implements a reliable file transfer system using a sliding window protocol with CRC for data integrity. The sender splits the file into packets and manages acknowledgments, while the receiver verifies data and writes it to a file. Designed for unreliable networks, it handles issues like delays, loss, and corruption, ensuring accurate and secure transfer.

1. `sendfile.c`: Transmission Program Sender

   - Reliable File Transfer: Files are transmitted via the UDP protocol, and the program ensures reliable transmission by introducing an ACK (Acknowledgment) mechanism and a sliding window protocol.
   
   - CRC32 Checksum: A CRC32 checksum is added to each data packet and file information to ensure no errors occur during transmission. If the CRC checksum of the received packet does not match, the sender will retransmit the packet.
   
   - Adaptive Timeout Mechanism: The timeout duration is dynamically calculated using RTT (Round Trip Time) and DevRTT. By measuring the round-trip time for each packet, the sending timeout is adjusted, ensuring that the retransmission mechanism remains effective even when network latency fluctuates.
   
   How to determine PACKET_SIZE:
   
   - We have defined the maximum size for the entire `PKT_SIZE` as 1024 bytes. The fields for the `Sequence Number`, `start`, `data_length`, and `crc` checksum each take up 4 bytes, since `sizeof(int32_t)` is 4 bytes. Therefore, the total size occupied by these control fields is 16 bytes.
   - To calculate the `DATA_SIZE` (the space available for the actual data), we subtract the size of these control fields from the `PKT_SIZE`: DATA_SIZE = PKT_SIZE - (4 * 4) = 1024 - 16 = 1008 bytes.
   
   How to determine WINDOW_SIZE:
   - We conducted tests on the CLEAR server and look.cs.rice.edu. We calculate the `WINDOW_SIZE` based on the BDP (Bandwidth-Delay Product). The BDP is given by: BDP = Bandwidth × RTT.
   
   - The `WINDOW_SIZE` is computed as: WINDOW_SIZE = BDP / PACKET_SIZE.
   
2. `recvfile.c`: Transmission Program Receiver
   
   - Creating Directory Structure: Before receiving the file, the target directory is created to store the received file.
   - CRC32 Checksum: Each received data packet contains a CRC checksum, and the program uses the CRC32 algorithm to verify the integrity of the packet.
   - Sliding Window Mechanism: A reliable data reception mechanism is implemented based on the sliding window, ensuring the correct order of received data packets.
   - EOF Handling and File Saving: When the data packet with a sequence number of -1 (EOF) is received, the program confirms the end of the file transfer and writes all cached data packets to the file.
   
3. `crc.c`: Implementation of the CRC32 (Cyclic Redundancy Check) algorithm

   - `crc32_table[256]` is a lookup table that stores precomputed CRC values for each possible byte value (0-255).
   - `init_crc32_table()`: This function initializes the CRC32 lookup table using a predefined polynomial (`0xEDB88320`).
   - `crc32(const void *data, size_t length)`: This function calculates the CRC32 checksum for a given block of data.



## Features

### Adaptive RTT

We use a weighted approach to smooth the fluctuations of RTT in order to calculate the RTT. The specific explanation is as follows:

1. Estimated RTT

   $\text{Estimated RTT} =  \alpha \times \text{Old Estimated RTT} + (1 - \alpha) \times \text{Sample RTT}$

   - The estimated round-trip time (RTT) is calculated using a weighted average of the previous estimated RTT and the new sample RTT.

2. Deviation RTT

   $\text{Dev RTT} = (1 - \beta) \times \text{Old Dev RTT} + \beta \times \left| \text{Sample RTT} - \text{Estimated RTT} \right|$

   - The deviation in RTT measures the variation in RTT from its estimated value. It is updated based on the difference between the current sample RTT and the estimated RTT.

3. Timeout Interval

   $\text{Timeout Interval} = \text{Estimated RTT} + 4 \times \text{Dev RTT}$

   - The timeout interval is dynamically adjusted by adding four times the RTT deviation to the estimated RTT. This ensures that the timeout accounts for both the average RTT and potential variations.

### Reliable Transmission

1. Addressing Lost and Delayed Packets

   - Each transmitted packet is assigned a timeout timer. If an acknowledgment is not received before the timer expires, the sender assumes the packet has been lost and retransmits it. This mechanism effectively addresses the issue of lost packets.

2. Addressing Duplicated and Reordered Packets

   - Each packet is assigned a unique sequence number to help the receiver identify duplicate packets and maintain the correct order. The receiver sends an acknowledgment for each received packet. If packets arrive out of order, the receiver acknowledges the last correctly received packet, prompting the sender to retransmit any lost packets. The receiver temporarily stores out-of-order packets until the missing packets arrive, ensuring they can be processed in the correct sequence.

3. Addressing Corrupted Packets

   - Each packet contains a CRC checksum calculated before transmission. When the receiver receives a packet, it recalculates the CRC and compares it with the received checksum. If they do not match, it indicates that the packet has been corrupted. When a packet is identified as corrupted, the sender automatically retransmits the affected packet.

### Efficient Transmission

1. Flow Control

   - By setting the window size, the receiving end can control the number of data packets received simultaneously, preventing buffer overflow. The window size can be dynamically adjusted based on network conditions and the processing capacity of the receiving end.

2. Efficient Use of Network Bandwidth

   - The sliding window protocol we designed allows multiple packets to be received before an acknowledgment (ACK) is received, enabling more efficient use of network bandwidth and reducing idle time caused by waiting for confirmations.

3. Reducing Latency

   - The sliding window protocol we designed allows for the continued reception of data without acknowledgment (ACK), which can significantly reduce network latency, especially in high-latency environments. This enables applications to respond quickly after receiving partial data, without having to wait for the confirmation of all packets.

## Run command

1. Compile the program
   ```bash
   make
   ```

2. Receiving Files
   
   ```bash
   ./recvfile -p 18000 > output.log 2>&1
   ```
   - Starts a file receiving program on port 18000 of the local machine to receive data packets from the network.
   
3. Sending Files
   ```bash
   ./sendfile -r 168.4.100.250:18123 -f test/middle_test.bin > output1.log 2>&1
   ./sendfile -r 128.42.128.85:18000 -f test/large_test.bin > output.log 2>&1
   ./sendfile -r 128.42.128.85:18000 -f test/image.png > output.log 2>&1
   ./sendfile -r 128.42.128.85:18000 -f test/video.mp4 > output.log 2>&1
   ```
   
   - Sends multiple files of various types, including binary files, images, and videos, to specified IP addresses and ports.
   
4. Sending to Different Target Addresses
   
   ```bash
   ./sendfile -r 128.42.124.184:18000 -f test/test.bin > output1.log 2>&1
   ./sendfile -r 128.42.124.184:18000 -f test/middle_test.bin > output1.log 2>&1
   ./sendfile -r 128.42.124.184:18000 -f test/large_test.bin > output.log 2>&1
   ./sendfile -r 128.42.124.184:18000 -f test/image.png > output.log 2>&1
   ./sendfile -r 128.42.124.184:18000 -f test/video.mp4 > output.log 2>&1
   ```
   
   - Sends multiple files to another IP address (128.42.124.184), ensuring that the transmission performance is tested across different target addresses.
   
5. Sending to an Unspecified Address
   ```bash
   ./sendfile -r :18123 -f test/test.bin
   ```
   - Attempts to send a file to port 18123 of the local machine without specifying the target IP address. This may be used to test default behavior or local loopback.

6. Network Delay and Packet Loss Simulation
   ```bash
   /usr/bin/netsim --delay 20 --drop 20
   ```
   - The netsim tool is used to simulate a network environment with a 20 ms delay and a 20% packet drop rate; however, we actually tested a variety of different delay and drop rate configurations.
   
7. Random Data File Generation:
   ```bash
   dd if=/dev/urandom of=test/middle_test.bin bs=1M count=50
   ```
   - Generates a random data file of size 50 MB for testing purposes.
   
8. File Consistency Check

   ```bash
   diff -s test/middle_test.bin test/middle_test.bin.recv
   ```
   - Compares the original file with the received file to check for consistency.

