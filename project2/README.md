# COMP556_Project2

## Project Contributors

- Fei Teng (ft28@rice.edu)
- Jiawen Wang (jw223@rice.edu)
- Siyu Chen (sc302@rice.edu)
- Lingyi Xu (lx28@rice.edu)



## Protocols

This project uses the sliding window protocol and Cyclic Redundancy Check (CRC) to implement a simple file transfer function. The sender divides the file into multiple packets and uses the sliding window mechanism for sending and acknowledgment. The receiver is responsible for receiving the packets, verifying data integrity, and writing the data to a file. The overall design includes error handling, file path management, and data integrity verification to ensure the reliability of the transfer.



## Data packet

- Sequence Number (4 bytes, `int32_t`): Identifies the order of the packet.
- Data Length (4 bytes, `int32_t`): The actual length of the data on the receiver's side.
- Data (Variable size, up to `DATA_SIZE`): The actual file data to be transmitted.
- CRC32 Checksum (4 bytes, `uint32_t`): The checksum generated using the CRC32 algorithm.



## Features

We use an adaptive timeout mechanism to calculate RTT. 

1. **Estimated RTT **:
   $\text{Estimated RTT} = \alpha \times \text{Old Estimated RTT} + (1 - \alpha) \times \text{Sample RTT}$
   - The estimated round-trip time (RTT) is calculated using a weighted average of the previous estimated RTT and the new sample RTT. 
2. **Deviation RTT **:
   $\text{Dev RTT} = (1 - \beta) \times \text{Old Dev RTT} + \beta \times \left| \text{Sample RTT} - \text{Estimated RTT} \right|$
   - The deviation in RTT measures the variation in RTT from its estimated value. It is updated based on the difference between the current sample RTT and the estimated RTT.
3. **Timeout Interval **:
   $
   \text{Timeout Interval} = \text{Estimated RTT} + 4 \times \text{Dev RTT}
   $
   - The timeout interval is dynamically adjusted by adding four times the RTT deviation to the estimated RTT. This ensures that the timeout accounts for both the average RTT and potential variations.



## How to run

1. Compile: `make`
2. Run recvfile on CLEAR: `./recvfile -p <recv_port>`
3. Run sendfile on LOOK: `./sendfile -r <recv_host>:<recv_port> -f <subdir>/<filename>`