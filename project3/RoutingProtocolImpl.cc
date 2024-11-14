#include "RoutingProtocolImpl.h"
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#else
#include <arpa/inet.h>
#endif

RoutingProtocolImpl::RoutingProtocolImpl(Node *n) : RoutingProtocol(n)
{
    sys = n;
}

RoutingProtocolImpl::~RoutingProtocolImpl()
{
}

void RoutingProtocolImpl::init(unsigned short num_ports, unsigned short router_id,
                               eProtocolType protocol_type)
{
    this->num_ports = num_ports;
    this->router_id = router_id;

    ports.resize(num_ports);
    for (unsigned short i = 0; i < num_ports; i++)
    {
        ports[i].isAlive = false;
        ports[i].neighborId = 0;
        ports[i].lastPongTime = 0;
        ports[i].cost = 0;
    }

    sys->set_alarm(this, 0, (void *)ALARM_PING);
    sys->set_alarm(this, CHECK_DURATION, (void *)ALARM_CHECK);

    // DV
    // 目前写的硬编码3和4，后面应该要改
    sys->set_alarm(this, 0, (void *)3);
    sys->set_alarm(this, 1000, (void *)4);
}

void *RoutingProtocolImpl::create_packet(unsigned short type, unsigned short size)
{
    char *packet = new char[size];
    struct packet *header = (struct packet *)packet;

    header->type = type;
    header->reserved = 0;
    header->size = size;
    header->src = router_id;
    header->dst = 0;

    return packet;
}

void RoutingProtocolImpl::send_ping(unsigned short port)
{
    unsigned short size = sizeof(struct packet) + sizeof(unsigned int);
    void *packet = create_packet(PING, size);

    // 在包体中存入当前时间戳
    unsigned int *timestamp = (unsigned int *)((char *)packet + sizeof(struct packet));
    *timestamp = htonl(sys->time());

    sys->send(port, packet, size);
}

void RoutingProtocolImpl::handle_ping(unsigned short port, void *packet)
{
    struct packet *ping_header = (struct packet *)packet;
    char *pong_packet = new char[ping_header->size];
    memcpy(pong_packet, packet, ping_header->size);

    struct packet *pong_header = (struct packet *)pong_packet;
    pong_header->type = PONG;
    pong_header->src = router_id;
    pong_header->dst = ping_header->src;

    sys->send(port, pong_packet, ping_header->size);
}

// 在链路状态改变时要触发DV更新,还没写这一块
void RoutingProtocolImpl::handle_pong(unsigned short port, void *packet)
{
    struct packet *header = (struct packet *)packet;
    unsigned int *timestamp = (unsigned int *)((char *)packet + sizeof(struct packet));

    // 计算RTT
    unsigned int send_time = ntohl(*timestamp);
    unsigned int rtt = sys->time() - send_time;

    bool was_alive = ports[port].isAlive;
    ports[port].neighborId = header->src;
    ports[port].cost = rtt;
    ports[port].lastPongTime = sys->time();
    ports[port].isAlive = true;

    if (!was_alive)
    {
        printf("Time %d: Router %d Port %d connected to Router %d, RTT=%dms\n",
               sys->time(), router_id, port, ports[port].neighborId, rtt);
    }
}

// 在链路断开时还要更新DV，还没写
void RoutingProtocolImpl::check_neighbor_status()
{
    unsigned int current_time = sys->time();

    for (unsigned short i = 0; i < num_ports; i++)
    {
        if (ports[i].isAlive)
        {
            if (current_time - ports[i].lastPongTime > PONG_TIMEOUT)
            {
                printf("Time %d: Router %d Port %d (neighbor %d) died\n",
                       current_time, router_id, i, ports[i].neighborId);
                ports[i].isAlive = false;
                ports[i].neighborId = 0;
                ports[i].cost = 0;
            }
        }
    }
}

void RoutingProtocolImpl::handle_alarm(void *data)
{
    long alarm_type = (long)data;

    if (alarm_type == ALARM_PING)
    { // PING alarm
        for (unsigned short i = 0; i < num_ports; i++)
        {
            send_ping(i);
        }
        sys->set_alarm(this, 10000, (void *)1);
    }
    else if (alarm_type == ALARM_CHECK)
    { // CHECK alarm
        check_neighbor_status();
        sys->set_alarm(this, 1000, (void *)2);
    }
    // 目前写的硬编码3和4
    else if (alarm_type == 3)
    {
        send_dv_update(false);
        sys->set_alarm(this, 30000, (void *)3); // 30秒后再次更新
    }
    else if (alarm_type == 4)
    {
        check_routes_timeout();
        sys->set_alarm(this, 1000, (void *)4); // 1秒后再次检查
    }
}

void RoutingProtocolImpl::send_dv_update(bool triggered)
{
    // DV包：包头 + 每个路由条目(destination + cost)
    unsigned short num_routes = routing_table.size();
    unsigned short packet_size = sizeof(struct packet) +
                                 num_routes * (sizeof(unsigned short) * 2);

    for (unsigned short port = 0; port < num_ports; port++)
    {
        if (!ports[port].isAlive)
            continue;

        char *dv_packet = new char[packet_size];
        struct packet *pkt = (struct packet *)dv_packet;
        pkt->type = DV;
        pkt->src = router_id;
        pkt->dst = 0;
        pkt->size = packet_size;

        unsigned short *payload = (unsigned short *)(dv_packet + sizeof(struct packet));
        int entry_index = 0;

        for (auto const &route : routing_table)
        {
            if (!route.second.valid)
                continue;

            // 如果下一跳是该端口的邻居，成本无限大
            unsigned int cost = route.second.cost;
            if (route.second.port == port)
            {
                cost = INFINITY_COST;
            }

            payload[entry_index++] = htons(route.first); // destination
            payload[entry_index++] = htons(cost);        // cost
        }

        sys->send(port, dv_packet, packet_size);
    }
}

void RoutingProtocolImpl::update_route(unsigned short dest, unsigned short next_hop,
                                       unsigned short port, unsigned int cost)
{
    auto &route = routing_table[dest];
    route.destination = dest;
    route.next_hop = next_hop;
    route.port = port;
    route.cost = cost;
    route.last_update = sys->time();
    route.valid = true;
}

void RoutingProtocolImpl::handle_dv_packet(unsigned short port, void *packet)
{
    struct packet *pkt = (struct packet *)packet;
    unsigned short src_id = pkt->src;
    unsigned short *payload = (unsigned short *)((char *)packet + sizeof(struct packet));
    int num_entries = (pkt->size - sizeof(struct packet)) / (sizeof(unsigned short) * 2);

    bool route_changed = false;

    // 处理每个路由条目
    for (int i = 0; i < num_entries; i++)
    {
        unsigned short dest = ntohs(payload[i * 2]);
        unsigned int cost = ntohs(payload[i * 2 + 1]);

        if (cost == INFINITY_COST)
            continue;

        // 总成本
        unsigned int total_cost = cost + ports[port].cost;

        auto it = routing_table.find(dest);
        if (it == routing_table.end() || !it->second.valid)
        {
            if (total_cost < INFINITY_COST)
            {
                update_route(dest, src_id, port, total_cost);
                route_changed = true;
            }
        }
        else if (it->second.next_hop == src_id)
        {
            if (total_cost != it->second.cost)
            {
                update_route(dest, src_id, port, total_cost);
                route_changed = true;
            }
        }
        else if (total_cost < it->second.cost)
        {
            update_route(dest, src_id, port, total_cost);
            route_changed = true;
        }
    }

    if (route_changed)
    {
        send_dv_update(true);
    }
}

void RoutingProtocolImpl::check_routes_timeout()
{
    bool route_changed = false;
    unsigned int current_time = sys->time();

    for (auto &route : routing_table)
    {
        if (route.second.valid &&
            current_time - route.second.last_update > 45000)
        { // 45秒超时
            route.second.valid = false;
            route_changed = true;
        }
    }

    if (route_changed)
    {
        send_dv_update(true);
    }
}

void RoutingProtocolImpl::recv(unsigned short port, void *packet, unsigned short size)
{
    // 检查size 是否小于 packet 结构体大小
    if (size < sizeof(struct packet))
    {
        delete[] (char *)packet;
        return;
    }
    // 检查 port 是否有效
    if (port >= num_ports)
    {
        delete[] (char *)packet;
        return;
    }

    struct packet *header = (struct packet *)packet;

    switch (header->type)
    {
    case PING:
        handle_ping(port, packet);
        break;
    case PONG:
        handle_pong(port, packet);
        break;
    case DV:
        handle_dv_packet(port, packet);
        break;
    default:
        printf("Unknown packet type received: %d\n", header->type);
        break;
    }

    delete[] (char *)packet;
}