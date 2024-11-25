# COMP556 Project3

## Project Contributors
- Fei Teng (ft28@rice.edu)
- Jiawen Wang (jw223@rice.edu)
- Siyu Chen (sc302@rice.edu)
- Lingyi Xu (lx28@rice.edu)

## Implementation Overview

We have implemented both Distance Vector (DV) and Link State (LS) routing protocols. Our implementation follows all the specifications provided in the project document.

### Project Structure
The specific code implementation is all in **RoutingProtocolImpl.cc**.
In addition to the original test cases, we have added new test cases, with detailed descriptions provided in **test.txt**.

### Key Features

#### Distance Vector Implementation
- Poison reverse variant 
- 30-second periodic updates
- Triggered updates on route changes
- 45-second timeout for stale entries
- Round-trip delay based cost calculation

#### Link State Implementation
- Flooding mechanism with sequence numbers
- 30-second periodic updates
- Triggered updates on neighbor status changes
- 45-second timeout for stale entries
- Dijkstra's algorithm for shortest path computation



### Instructions
1. Run `make` to build the simulator
2. Run tests using: `./Simulator test DV|LS`

