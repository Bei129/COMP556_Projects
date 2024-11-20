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
    printf("Router %d: Initializing with protocol type %d\n", router_id, protocol_type);

    this->num_ports = num_ports;
    this->router_id = router_id;
    this->protocol_type = protocol_type;

    // 初始化端口数组
    ports.resize(num_ports);

    // 初始化链路状态表和序列号
    link_state_table[router_id] = {}; 
    ls_sequence_number[router_id] = 0; 

    sys->set_alarm(this, 0, (void *)ALARM_PING);
    // sys->set_alarm(this, CHECK_DURATION, (void *)ALARM_CHECK);

    if (protocol_type == P_DV) {
        // DV
        // 目前写的硬编码3和4，后面应该要改
        sys->set_alarm(this, 0, (void *)3);
        sys->set_alarm(this, 1000, (void *)4);
    }
    else if (protocol_type == P_LS){ 
        // LS
        sys->set_alarm(this, 30000, (void *)ALARM_LS);
        sys->set_alarm(this, 1000, (void *)ALARM_LS_TIMEOUT); // 每1秒检查一次超时
    }
}

void *RoutingProtocolImpl::create_packet(unsigned char type, unsigned short size)
{
    char *packet = new char[size];
    struct packet *header = (struct packet *)packet;

    header->type = type;
    header->reserved = 0;
    header->size = htons(size);
    header->src = htons(router_id);
    header->dst = htons(0);

    printf("Creating packet: type=%d, size=%d\n", type, size);

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
    unsigned short size = ntohs(ping_header->size);
    char *pong_packet = new char[ping_header->size];

    memcpy(pong_packet, packet, size);
    // memcpy(pong_packet, packet, ntohs(ping_header->size));

    struct packet *pong_header = (struct packet *)pong_packet;
    pong_header->type = PONG;
    // pong_header->type = htons(PONG);
    pong_header->reserved = 0;
    pong_header->src = htons(router_id);
    pong_header->dst = ping_header->src;
    pong_header->size = htons(size);

    // sys->send(port, pong_packet, size);
    sys->send(port, pong_packet, ntohs(ping_header->size));
    // delete[] pong_packet;
}

// 在链路状态改变时要触发DV更新,还没写这一块
void RoutingProtocolImpl::handle_pong(unsigned short port, void *packet)
{
    struct packet *header = (struct packet *)packet;
    unsigned int *timestamp = (unsigned int *)((char *)packet + sizeof(struct packet));
    unsigned short src_id = ntohs(header->src);

    // 计算RTT
    unsigned int send_time = ntohl(*timestamp);
    unsigned int rtt = sys->time() - send_time;

    // 获取或创建邻居信息
    auto &neighbor = ports[port].neighbors[src_id];
    bool was_alive = neighbor.isAlive;

    // 更新邻居信息
    if (rtt > 65535) // 防止溢出
        rtt = 65535;
    neighbor.cost = (unsigned short)rtt;
    neighbor.lastPongTime = sys->time();
    neighbor.isAlive = true;

    // 更新链路状态表
    link_state_table[router_id][src_id] = neighbor.cost;

    if (!was_alive)
    {
        printf("Time %d: Router %d Port %d connected to Router %d, RTT=%dms\n",
               sys->time(), router_id, port, src_id, rtt);
    }

    // 触发LS和DV更新
    if (protocol_type == P_DV)
    {
        printf("Router %d: Topology changed, sending DV update\n", router_id);
        send_dv_update(true);
    } 
    else if (protocol_type == P_LS)
    {
        printf("Router %d: Topology changed, sending LS update\n", router_id);
        send_ls_update();
    }
}

void RoutingProtocolImpl::check_neighbor_status()
{
    unsigned int current_time = sys->time();
    bool topology_changed = false;

    for (unsigned short port = 0; port < num_ports; port++)
    {
        auto &port_status = ports[port];

        // 遍历端口上的所有邻居
        for (auto it = port_status.neighbors.begin(); it != port_status.neighbors.end();)
        {
            auto &neighbor = it->second;
            unsigned short neighbor_id = it->first;

            if (neighbor.isAlive)
            {
                if (current_time - neighbor.lastPongTime > PONG_TIMEOUT)
                {
                    printf("Time %d: Router %d Port %d (neighbor %d) died\n",
                           current_time, router_id, port, neighbor_id);
                    neighbor.isAlive = false;
                    topology_changed = true;

                    // 从链路状态表中移除失效的邻居
                    link_state_table[router_id].erase(neighbor_id);
                }
            }
            it++;
        }
    }

    if (topology_changed)
    {
        if (protocol_type == P_DV)
        {
            printf("Router %d: Topology changed, sending DV update\n", router_id);
            send_dv_update(true);
        } 
        else if (protocol_type == P_LS)
        {
            printf("Router %d: Topology changed, sending LS update\n", router_id);
            send_ls_update();
        }
    }
}

void RoutingProtocolImpl::handle_alarm(void *data)
{
    long alarm_type = (long)data;
    // printf("Router %d: handle_alarm called with type %ld\n", router_id, alarm_type);

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
    else if (alarm_type == ALARM_LS)
    {
        send_ls_update();
        sys->set_alarm(this, 30000, (void *)5); // 30秒后再次更新
    }
    else if (alarm_type == ALARM_LS_TIMEOUT)
    {
        check_link_state_timeout();
        sys->set_alarm(this, 1000, (void *)ALARM_LS_TIMEOUT); // 1秒后再次检查
    }
}

void RoutingProtocolImpl::send_dv_update(bool triggered)
{
    unsigned short num_routes = routing_table.size();
    unsigned short packet_size = sizeof(struct packet) +
                                 num_routes * (sizeof(unsigned short) * 2);

    for (unsigned short port = 0; port < num_ports; port++)
    {
        const auto &port_neighbors = ports[port].neighbors;

        // 如果这个端口没有活跃的邻居，跳过
        bool has_active_neighbors = false;
        for (const std::pair<unsigned short, NeighborInfo> &pair : port_neighbors)
        {
            if (pair.second.isAlive)
            {
                has_active_neighbors = true;
                break;
            }
        }

        if (!has_active_neighbors)
        {
            continue;
        }

        void *packet = create_packet(DV, packet_size);
        unsigned short *payload = (unsigned short *)((char *)packet + sizeof(struct packet));
        int entry_index = 0;

        for (const auto &route : routing_table)
        {
            if (!route.second.valid)
                continue;

            // 改成多个邻居
            unsigned short cost = route.second.cost;
            if (route.second.port == port)
            {
                cost = INFINITY_COST;
            }

            payload[entry_index++] = htons(route.first); // destination
            payload[entry_index++] = htons(cost);        // cost
        }

        for (const auto &neighbor_pair : port_neighbors)
        {
            if (neighbor_pair.second.isAlive)
            {
                char *dv_packet = new char[packet_size];
                memcpy(dv_packet, packet, packet_size);
                sys->send(port, dv_packet, packet_size);
            }
        }

        delete[] (char *)packet;
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
    unsigned short src_id = ntohs(pkt->src);
    unsigned short *payload = (unsigned short *)((char *)packet + sizeof(struct packet));
    // int num_entries = (pkt->size - sizeof(struct packet)) / (sizeof(unsigned short) * 2);
    int num_entries = (ntohs(pkt->size) - sizeof(struct packet)) / (sizeof(unsigned short) * 2);

    auto &port_neighbors = ports[port].neighbors;
    auto neighbor_it = port_neighbors.find(src_id);
    if (neighbor_it == port_neighbors.end() || !neighbor_it->second.isAlive)
    {
        return;
    }

    bool route_changed = false;

    for (int i = 0; i < num_entries; i++)
    {
        unsigned short dest = ntohs(payload[i * 2]);
        unsigned int cost = ntohs(payload[i * 2 + 1]);

        if (cost == INFINITY_COST)
            continue;

        // 使用从邻居信息中获取的cost
        unsigned int total_cost = cost + neighbor_it->second.cost;

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
    // 检查size 是否小于 packet 结构体大小、检查 port 是否有效
    if (size < sizeof(struct packet) || port >= num_ports)
    {
        // delete[] (char *)packet;
        return;
    }

    // printf("Raw packet dump: ");
    // for (int i = 0; i < size; i++) {
    //     printf("%02x ", ((unsigned char *)packet)[i]);
    // }
    // printf("\n");

    struct packet *header = (struct packet *)packet;

    // 转为主机字节序
    unsigned char pkt_type = header->type;
    unsigned short pkt_size = ntohs(header->size);
    unsigned short pkt_src = ntohs(header->src);
    unsigned short pkt_dst = ntohs(header->dst);

    printf("Router %d: Received packet of type %d from %d to %d on port %d\n", router_id, pkt_type, pkt_src, pkt_dst, port);

    // 验证包的大小
    if (pkt_size != size)
    {
        printf("Packet size mismatch: expected %u, got %u\n", pkt_size, size);
        // delete[] (char *)packet;
        return;
    }
    else
    {
        printf("Packet size is correct: expected %u, got %u\n", pkt_size, size);
    }

    // switch (header->type)
    switch (pkt_type)
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
    case LS:
        handle_ls_packet(port, packet);
        break;
    // 缺少DATA
    default:
        printf("Unknown packet type received: %d\n", pkt_type);
        break;
    }

    // delete[] (char *)packet;
}

void RoutingProtocolImpl::send_ls_update()
{
    printf("Router %d: Attempting to send LS update\n", router_id);

    // 统计邻居数量
    unsigned short num_neighbors = 0;
    for (const auto &port : ports)
    {
        for (const auto &neighbor : port.neighbors)
        {
            if (neighbor.second.isAlive)
            {
                num_neighbors++;
            }
        }
    }

    unsigned short packet_size = sizeof(struct packet) + sizeof(unsigned short) * 1 +
                                 num_neighbors * (sizeof(unsigned short) * 2);

    void *packet = create_packet(LS, packet_size);

    unsigned short sequence_num = ++ls_sequence_number[router_id];
    unsigned short *payload = (unsigned short *)((char *)packet + sizeof(struct packet));
    payload[0] = htons(sequence_num);

    int offset = 1;
    for (unsigned short port_num = 0; port_num < num_ports; port_num++)
    {
        const auto &port_neighbors = ports[port_num].neighbors;
        for (const auto &neighbor : port_neighbors)
        {
            if (neighbor.second.isAlive)
            {
                payload[offset++] = htons(neighbor.first);       // Neighbor ID
                payload[offset++] = htons(neighbor.second.cost); // Link cost
            }
        }
    }

    printf("Router %d: Sending LS update with sequence number %d\n", router_id, sequence_num);

    for (unsigned short port = 0; port < num_ports; port++)
    {
        const auto &port_neighbors = ports[port].neighbors;
        bool has_active_neighbors = false;

        for (const auto &neighbor : port_neighbors)
        {
            if (neighbor.second.isAlive)
            {
                has_active_neighbors = true;
                break;
            }
        }
        if (has_active_neighbors)
        {
            char *ls_packet = new char[packet_size];
            memcpy(ls_packet, packet, packet_size);
            sys->send(port, ls_packet, packet_size);
            printf("Router %d: Sent LS update to port %d\n", router_id, port);
        }
    }

    delete[] (char *)packet;
}

void RoutingProtocolImpl::handle_ls_packet(unsigned short port, void *packet)
{
    struct packet *pkt = (struct packet *)packet;
    unsigned short src_id = ntohs(pkt->src);
    unsigned short seq_num = ntohs(*((unsigned short *)((char *)packet + sizeof(struct packet))));

    // auto &port_neighbors = ports[port].neighbors;
    // auto neighbor_it = port_neighbors.find(src_id);
    // if (neighbor_it == port_neighbors.end() || !neighbor_it->second.isAlive)
    // {
    //     printf("Router %d: Received LS packet from non-neighbor or inactive neighbor %d\n",
    //            router_id, src_id);
    // }

    if (ls_sequence_number[src_id] >= seq_num)
    {
        printf("Router %d: Ignoring outdated LS packet from %d with sequence number %d\n",
               router_id, src_id, seq_num);
        return;
    }
    else
    {
        printf("Router %d: Accepted LS packet from %d with sequence number %d (Current seq: %d)\n",
            router_id, src_id, seq_num, ls_sequence_number[src_id]);
    }

    ls_sequence_number[src_id] = seq_num;

    unsigned short *payload = (unsigned short *)((char *)packet + sizeof(struct packet) + sizeof(unsigned short));
    int num_entries = (ntohs(pkt->size) - sizeof(struct packet) - sizeof(unsigned short)) /
                      (sizeof(unsigned short) * 2);

    // Update the link state table for src_id
    link_state_table[src_id].clear();

    for (int i = 0; i < num_entries; i++)
    {
        unsigned short neighbor_id = ntohs(payload[i * 2]);
        unsigned short cost = ntohs(payload[i * 2 + 1]);
        link_state_table[src_id][neighbor_id] = cost;
        ls_last_update[src_id][neighbor_id] = sys->time();
        printf("Router %d: LS packet from %d reports neighbor %d with cost %d\n", router_id, src_id, neighbor_id, cost);
    }

    printf("Router %d: Updating link state table for router %d\n", router_id, src_id);
    for (const auto &entry : link_state_table[src_id])
    {
        printf("  Neighbor %d with cost %d\n", entry.first, entry.second);
    }

    printf("Router %d: Received LS packet from %d with sequence number %d\n", router_id, src_id, seq_num);

    calculate_shortest_paths();

    // Flood the LS packet to all ports except the one it was received on
    for (unsigned short i = 0; i < num_ports; i++)
    {
        if (i == port)
            continue;

        // const auto &fwd_port_neighbors = ports[i].neighbors;
        // bool has_active_neighbors = false;

        // for (const auto &neighbor : fwd_port_neighbors)
        // {
        //     if (neighbor.second.isAlive)
        //     {
        //         has_active_neighbors = true;
        //         break;
        //     }
        // }

        // if (has_active_neighbors)
        // {
            char *ls_packet = new char[ntohs(pkt->size)];
            memcpy(ls_packet, packet, ntohs(pkt->size));
            sys->send(i, ls_packet, ntohs(pkt->size));
            printf("Router %d: Forwarded LS packet to port %d\n", router_id, i);
        // }
    }
}

void RoutingProtocolImpl::calculate_shortest_paths()
{
    printf("Router %d: Calculating shortest paths\n", router_id);

    for (const auto &entry : routing_table)
    {
        if (entry.second.valid)
        {
            printf("  Destination %d via %d (port %d) with cost %d\n",
                entry.first, entry.second.next_hop, entry.second.port, entry.second.cost);
        }
    }

    std::unordered_map<unsigned short, unsigned int> distance;
    std::unordered_map<unsigned short, unsigned short> previous;
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

    // 初始化距离表
    for (const auto &neighbor : link_state_table[router_id])
    {
        distance[neighbor.first] = neighbor.second;
        previous[neighbor.first] = router_id;
        pq.emplace(neighbor.second, neighbor.first);
    }

    while (!pq.empty())
    {
        std::pair<int, int> top = pq.top();
        unsigned int dist = top.first;
        int node = top.second;
        pq.pop();

        if (dist > distance[node])
            continue;

        for (const auto &neighbor : link_state_table[node])
        {
            unsigned int new_cost = distance[node] + neighbor.second;
            if (distance.find(neighbor.first) == distance.end() || new_cost < distance[neighbor.first])
            {
                distance[neighbor.first] = new_cost;
                previous[neighbor.first] = node;
                pq.emplace(new_cost, neighbor.first);
            }
        }
    }

    // 更新转发表
    for (const auto &entry : distance)
    {
        unsigned short dest = entry.first;
        if (dest != router_id)
        {
            unsigned short next_hop = dest;
            while (previous[next_hop] != router_id)
            {
                next_hop = previous[next_hop];
            }

            unsigned short out_port = get_port_to_neighbor(next_hop);

            if (out_port != INVALID_PORT)
            {
                routing_table[dest].destination = dest;
                routing_table[dest].next_hop = next_hop;
                routing_table[dest].port = out_port;
                routing_table[dest].cost = distance[dest];
                routing_table[dest].valid = true;

                printf("Router %d: Path to %d via %d (port %d) with cost %d\n", router_id, dest, next_hop, out_port, distance[dest]);
            }
            else
            {
                printf("Router %d: No valid port found to next_hop %d for destination %d\n", router_id, next_hop, dest);
            }
        }
    }
}

void RoutingProtocolImpl::check_link_state_timeout()
{
    unsigned int current_time = sys->time();
    bool topology_changed = false;

    for (auto &entry : link_state_table)
    {
        unsigned short router_id = entry.first;
        auto &neighbors = entry.second;

        for (auto it = neighbors.begin(); it != neighbors.end();)
        {
            unsigned short neighbor_id = it->first;

            // 检查是否超时
            if (current_time - ls_last_update[router_id][neighbor_id] > 45000) // 45秒超时
            {
                printf("Router %d: Link to %d expired\n", router_id, neighbor_id);
                it = neighbors.erase(it);
                ls_last_update[router_id].erase(neighbor_id);
                topology_changed = true;
            }
            else
            {
                ++it;
            }
        }
    }

    if (topology_changed)
    {
        calculate_shortest_paths();
    }
}

unsigned short RoutingProtocolImpl::get_port_to_neighbor(unsigned short neighbor_id)
{
    for (unsigned short port = 0; port < num_ports; port++)
    {
        const auto &neighbors = ports[port].neighbors;
        if (neighbors.find(neighbor_id) != neighbors.end() && neighbors.at(neighbor_id).isAlive)
        {
            return port;
        }
    }

    return INVALID_PORT;
}

