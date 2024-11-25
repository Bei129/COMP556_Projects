# COMP556 Project3

## Project Contributors
- Fei Teng (ft28@rice.edu)
- Lingyi Xu (lx28@rice.edu)
- Jiawen Wang (jw223@rice.edu)
- Siyu Chen (sc302@rice.edu)

## Running
1. Run `make` to build the simulator
2. Run tests using: `./Simulator test DV` or `./Simulator test LS`

## Reminder
During our testing, we observed that the simulator does not guarantee those link change events (e.g., `Event_Link_Come_Up`) are processed before packet transmission events when they share the same timestamp (`time = t`). As a result, in test cases where link change events and packet transmission events occur simultaneously, it is possible for packet transmissions to be attempted before the link change takes effect.

If the link is still considered unavailable at the time of transmission, this behavior may lead to packet loss. For example, when a link recovery event (`Event_Link_Come_Up`) and a PING packet transmission event (`Event_Xmit_Pkt_On_Link`) occur at the same timestamp, the PING packet may be transmitted before the link is restored, resulting in the packet being discarded. Therefore, we cannot guarantee that updates reflecting link changes are completed precisely at `time = t` **all the time**.

What we can guarantee is that the next packet cycle will correctly reflect the link changes, ensuring eventual consistency in the network state.

## Memory Management
We used `Valgrind` to check memory management during the testing and no errors related to manual memory management were found.
