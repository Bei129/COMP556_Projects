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

    intptr_t alarm_type = (intptr_t)data;

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
    // 后面的 DV 接着这里写
    switch (header->type)
    {
    case PING:
        handle_ping(port, packet);
        break;
    case PONG:
        handle_pong(port, packet);
        break;
    default:
        printf("Unknown packet type received: %d\n", header->type);
        break;
    }

    delete[] (char *)packet;
}