# COMP556_Project2

## Project Contributors

- Fei Teng (ft28@rice.edu)
- Jiawen Wang (jw223@rice.edu)
- Siyu Chen (sc302@rice.edu)
- Lingyi Xu (lx28@rice.edu)



## Protocols

This project uses the sliding window protocol and Cyclic Redundancy Check (CRC) to implement a simple file transfer function. The sender divides the file into multiple packets and uses the sliding window mechanism for sending and acknowledgment. The receiver is responsible for receiving the packets, verifying data integrity, and writing the data to a file. The overall design includes error handling, file path management, and data integrity verification to ensure the reliability of the transfer.



## Project Overview

1. `sendfile.c`: Transmission Program Sender

   - Reliable File Transfer: Files are transmitted via the UDP protocol, and the program ensures reliable transmission by introducing an ACK (Acknowledgment) mechanism and a sliding window protocol.
   - CRC32 Checksum: A CRC32 checksum is added to each data packet and file information to ensure no errors occur during transmission. If the CRC checksum of the received packet does not match, the sender will retransmit the packet.
   - Adaptive Timeout Mechanism: The timeout duration is dynamically calculated using RTT (Round Trip Time) and DevRTT. By measuring the round-trip time for each packet, the sending timeout is adjusted, ensuring that the retransmission mechanism remains effective even when network latency fluctuates.
2. `recvfile.c`: Transmission Program Receiver
   - Creating Directory Structure: Before receiving the file, the target directory is created to store the received file.
   - CRC32 Checksum: Each received data packet contains a CRC checksum, and the program uses the CRC32 algorithm to verify the integrity of the packet.
   - Sliding Window Mechanism: A reliable data reception mechanism is implemented based on the sliding window, ensuring the correct order of received data packets.
   - EOF Handling and File Saving: When the data packet with a sequence number of -1 (EOF) is received, the program confirms the end of the file transfer and writes all cached data packets to the file.
3. `crc.c`: Implementation of the CRC32 (Cyclic Redundancy Check) algorithm

   - `crc32_table[256]` is a lookup table that stores precomputed CRC values for each possible byte value (0-255).
   - `init_crc32_table()`: This function initializes the CRC32 lookup table using a predefined polynomial (`0xEDB88320`).
   - `crc32(const void *data, size_t length)`: This function calculates the CRC32 checksum for a given block of data.



## Data packet & Window

1. Data Packet Size:

   In our program, we have defined the maximum size for the entire `PKT_SIZE` as 1024 bytes. The fields for the `Sequence Number`, `start`, `data_length`, and `crc` checksum each take up 4 bytes, since `sizeof(int32_t)` is 4 bytes. Therefore, the total size occupied by these control fields is 16 bytes.

   To calculate the `DATA_SIZE` (the space available for the actual data), we subtract the size of these control fields from the `PKT_SIZE`: DATA_SIZE = PKT_SIZE - (4 * 4) = 1024 - 16 = 1008 bytes.

2. Window Size:

   We calculate the `WINDOW_SIZE` based on the BDP (Bandwidth-Delay Product). The BDP is given by: BDP = Bandwidth × RTT.

   And the `WINDOW_SIZE` is computed as: WINDOW_SIZE = BDP / PACKET_SIZE.

   We conducted tests on the CLEAR server and look.cs.rice.edu.

## Features

### RTT

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



## How to run

1. Compile: `make`
2. Run recvfile on CLEAR: `./recvfile -p <recv_port>`
3. Run sendfile on LOOK: `./sendfile -r <recv_host>:<recv_port> -f <subdir>/<filename>`
