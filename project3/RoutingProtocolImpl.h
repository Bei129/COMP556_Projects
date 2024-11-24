#ifndef ROUTING_PROTOCOL_IMPL_H
#define ROUTING_PROTOCOL_IMPL_H

#include "Node.h"
#include "global.h"
#include "RoutingProtocol.h"
#include <vector>
#include <map>
#include <unordered_map>
#include <queue>

#pragma pack(push, 1) // 设置结构体按1字节对齐，防止填充
struct packet
{
    unsigned char type;
    unsigned char reserved;
    unsigned short size;
    unsigned short src;
    unsigned short dst;
};
#pragma pack(pop) // 恢复默认对齐

struct NeighborInfo {
    unsigned int lastPongTime;
    unsigned short cost;
    bool isAlive;
};


struct PortStatus {
    std::unordered_map<unsigned short, NeighborInfo> neighbors;  // key是neighborId
};

struct RouteEntry
{
    unsigned short destination;
    unsigned short next_hop;
    unsigned short port;
    unsigned int cost;
    unsigned int last_update;
    bool valid;
};

class RoutingProtocolImpl : public RoutingProtocol
{
public:
    RoutingProtocolImpl(Node *n);
    ~RoutingProtocolImpl();

    void init(unsigned short num_ports, unsigned short router_id,
              eProtocolType protocol_type);
    void handle_alarm(void *data);
    void recv(unsigned short port, void *packet, unsigned short size);

private:
    Node *sys;
    unsigned short router_id;
    unsigned short num_ports;
    std::vector<PortStatus> ports; // 数据结构不变，内容变了
    eProtocolType protocol_type; // 保存协议类型


    // 包大小相关常量 这里是12
    static const unsigned short PING_PONG_PACK_SIZE = sizeof(struct packet) + sizeof(unsigned int);

    // 时间相关常量
    static const unsigned int PONG_TIMEOUT = 15000;  // 15s neighbor timeout
    static const unsigned int PING_DURATION = 10000; // 10s ping interval
    static const unsigned int CHECK_DURATION = 1000; // 1s check interval
    static const unsigned int ROUTE_TIMEOUT = 45000; // 45s routing timeout

    // Alarm类型常量
    static const int ALARM_PING = 1;
    static const int ALARM_CHECK = 2;
    static const int ALARM_DV = 3;
    static const int ALARM_DV_TIMEOUT = 4;
    static const int ALARM_LS = 5;
    static const int ALARM_LS_TIMEOUT = 6;

    // LS相关
    std::map<unsigned short, std::map<unsigned short, unsigned int>> link_state_table;
    std::map<unsigned short, unsigned short> ls_sequence_number;
    std::unordered_map<unsigned short, std::unordered_map<unsigned short, unsigned int>> ls_last_update;

    // 无效端口号
    const unsigned short INVALID_PORT = 0xFFFF;


    void send_ping(unsigned short port);
    void handle_ping(unsigned short port, void *packet);
    void handle_pong(unsigned short port, void *packet);
    void check_neighbor_status();
    void print_port_status();
    void *create_packet(unsigned char type, unsigned short size);
    void handle_data(unsigned short port, void *packet,unsigned short size);

    // DV相关方法
    std::map<unsigned short, RouteEntry> routing_table; //first-> router_id
    void send_dv_update(bool triggered = false);
    void handle_dv_packet(unsigned short port, void *packet);
    void check_DV_timeout();
    void print_DV_routing_table();
    unsigned short find_neighbor(unsigned short id);
    void delete_DV_invalid(vector<unsigned short> invalid_ids,bool &udpated);

    void update_route(unsigned short dest, unsigned short next_hop,
                      unsigned short port, unsigned int cost);

    // LS相关方法
    void send_ls_update();
    void handle_ls_packet(unsigned short port, void *packet);
    void calculate_shortest_paths();
    void check_link_state_timeout();
    unsigned short get_port_to_neighbor(unsigned short neighbor_id);
};

#endif