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
    printf("!!sizeof(struct packet) = %zu\n", sizeof(struct packet));

    printf("Router %d: Initializing with protocol type %d\n", router_id, protocol_type);
    
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

    // LS
    sys->set_alarm(this, 30000, (void *)ALARM_LS);
}

void *RoutingProtocolImpl::create_packet(unsigned char type, unsigned short size)
{
    char *packet = new char[size];
    struct packet *header = (struct packet *)packet;

    header->type = type;
    // header->reserved = 0;
    // header->size = size;
    // header->src = router_id;
    // header->dst = 0;
    // header->type = htons(type);  
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

                // 触发 LS 更新
                send_ls_update();
            }
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
    else if (alarm_type == 5) { // LS 
        send_ls_update();
        sys->set_alarm(this, 30000, (void *)5); // 30秒后再次更新
    }
}

// void RoutingProtocolImpl::send_dv_update(bool triggered)
// {
//     // DV包：包头 + 每个路由条目(destination + cost)
//     unsigned short num_routes = routing_table.size();
//     unsigned short packet_size = sizeof(struct packet) +
//                                  num_routes * (sizeof(unsigned short) * 2);

//     for (unsigned short port = 0; port < num_ports; port++)
//     {
//         if (!ports[port].isAlive)
//             continue;

//         char *dv_packet = new char[packet_size];
//         struct packet *pkt = (struct packet *)dv_packet;
//         // pkt->type = DV;
//         // pkt->src = router_id;
//         // pkt->dst = 0;
//         // pkt->size = packet_size;
//         pkt->type = htons(DV);
//         pkt->src = htons(router_id); 
//         pkt->dst = htons(0); 
//         pkt->size = htons(packet_size); 

//         unsigned short *payload = (unsigned short *)(dv_packet + sizeof(struct packet));
//         int entry_index = 0;

//         for (auto const &route : routing_table)
//         {
//             if (!route.second.valid)
//                 continue;

//             // 如果下一跳是该端口的邻居，成本无限大
//             unsigned int cost = route.second.cost;
//             if (route.second.port == port)
//             {
//                 cost = INFINITY_COST;
//             }

//             payload[entry_index++] = htons(route.first); // destination
//             payload[entry_index++] = htons(cost);        // cost
//         }

//         sys->send(port, dv_packet, packet_size);
//     }
// }
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

        void *packet = create_packet(DV, packet_size);

        unsigned short *payload = (unsigned short *)((char *)packet + sizeof(struct packet));
        int entry_index = 0;

        for (auto const &route : routing_table)
        {
            if (!route.second.valid)
                continue;

            // 如果下一跳是该端口的邻居，成本无限大
            unsigned short cost = route.second.cost;
            if (route.second.port == port)
            {
                cost = INFINITY_COST;
            }

            payload[entry_index++] = htons(route.first); // destination
            payload[entry_index++] = htons(cost);        // cost
        }

        sys->send(port, packet, packet_size);
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
    // unsigned short src_id = pkt->src;
    unsigned short src_id = ntohs(pkt->src); 
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
    // 检查size 是否小于 packet 结构体大小、检查 port 是否有效
    if (size < sizeof(struct packet) || port >= num_ports) {
        delete[] (char *)packet;
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
    // unsigned char pkt_type = ntohs(header->type);
    unsigned short pkt_size = ntohs(header->size);
    unsigned short pkt_src = ntohs(header->src);
    unsigned short pkt_dst = ntohs(header->dst);

    printf("Router %d: Received packet of type %d from %d to %d on port %d\n", router_id, pkt_type, pkt_src, pkt_dst, port);

    // 验证包的大小
    if (pkt_size != size) {
        printf("Packet size mismatch: expected %u, got %u\n", pkt_size, size);
        delete[] (char *)packet;
        return;
    } else {
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
    default:
        printf("Unknown packet type received: %d\n", pkt_type);
        break;
    }

    delete[] (char *)packet;
}

// void RoutingProtocolImpl::send_ls_update() {
//     printf("Router %d: Attempting to send LS update\n", router_id);

//     // Get neighbor count and packet size
//     unsigned short num_neighbors = link_state_table[router_id].size();
//     unsigned short packet_size = sizeof(struct packet) + sizeof(unsigned short) * 1 + num_neighbors * (sizeof(unsigned short) * 2);

//     // Prepare the template packet
//     char *ls_packet_template = new char[packet_size];
//     struct packet *pkt = (struct packet *)ls_packet_template;

//     // pkt->type = LS;
//     // pkt->reserved = 0;
//     // pkt->size = packet_size;
//     // pkt->src = router_id;
//     // pkt->dst = 0;
//     pkt->type = htons(LS);
//     pkt->reserved = 0;
//     pkt->size = htons(packet_size); 
//     pkt->src = htons(router_id); 
//     pkt->dst = htons(0);

//     // Set sequence number
//     unsigned short sequence_num = ++ls_sequence_number[router_id];
//     unsigned short *payload = (unsigned short *)(ls_packet_template + sizeof(struct packet));
//     // payload[0] = sequence_num;
//     payload[0] = htons(sequence_num);

//     // Write neighbor IDs and costs
//     int offset = 1;
//     for (const auto &neighbor : link_state_table[router_id]) {
//         payload[offset++] = neighbor.first;   // Neighbor ID
//         payload[offset++] = neighbor.second;  // Link cost
//     }

//     printf("Router %d: Sending LS update with sequence number %d\n", router_id, sequence_num);

//     // Send the packet to each active port/neighor
//     for (unsigned short port = 0; port < num_ports; port++) {
//         if (ports[port].isAlive) {
//             // Create a copy for each port
//             char *ls_packet = new char[packet_size];
//             memcpy(ls_packet, ls_packet_template, packet_size);
//             sys->send(port, ls_packet, packet_size);
//             printf("Router %d: Sent LS update to port %d\n", router_id, port);
//         }
//     }
//     // Clean up the template packet
//     delete[] ls_packet_template;
// }
void RoutingProtocolImpl::send_ls_update() {
    printf("Router %d: Attempting to send LS update\n", router_id);

    // Get neighbor count and packet size
    unsigned short num_neighbors = link_state_table[router_id].size();
    unsigned short packet_size = sizeof(struct packet) + sizeof(unsigned short) * 1 + num_neighbors * (sizeof(unsigned short) * 2);

    // Create the packet using create_packet
    void *packet = create_packet(LS, packet_size);

    struct packet *pkt = (struct packet *)packet;

    // Set sequence number
    unsigned short sequence_num = ++ls_sequence_number[router_id];
    unsigned short *payload = (unsigned short *)((char *)packet + sizeof(struct packet));
    payload[0] = htons(sequence_num); 

    // Write neighbor IDs and costs
    int offset = 1;
    for (const auto &neighbor : link_state_table[router_id]) {
        payload[offset++] = htons(neighbor.first);   // Neighbor ID
        payload[offset++] = htons(neighbor.second);  // Link cost
    }

    printf("Router %d: Sending LS update with sequence number %d\n", router_id, sequence_num);

    // Send the packet to each active port/neighor
    for (unsigned short port = 0; port < num_ports; port++) {
        if (ports[port].isAlive) {
            // Create a copy for each port
            char *ls_packet = new char[packet_size];
            memcpy(ls_packet, packet, packet_size);
            sys->send(port, ls_packet, packet_size);
            printf("Router %d: Sent LS update to port %d\n", router_id, port);
        }
    }
    // Clean up the original packet
    delete[] (char *)packet;
}


void RoutingProtocolImpl::handle_ls_packet(unsigned short port, void *packet) {
    struct packet *pkt = (struct packet *)packet;

    unsigned short src_id = ntohs(pkt->src);
    unsigned short seq_num = ntohs(*((unsigned short *)((char *)packet + sizeof(struct packet))));

    // 如果此路由器已记录的序列号比当前包的序列号新，则忽略该包
    if (ls_sequence_number[src_id] >= seq_num) {
        printf("Router %d: Ignoring outdated LS packet from %d with sequence number %d\n", router_id, src_id, seq_num);
        return;
    }

    // 更新序列号
    ls_sequence_number[src_id] = seq_num;

    // 更新链路状态表
    unsigned short *payload = (unsigned short *)((char *)packet + sizeof(struct packet) + sizeof(unsigned short));
    int num_entries = (pkt->size - sizeof(struct packet) - sizeof(unsigned short)) / (sizeof(unsigned short) * 2);

    for (int i = 0; i < num_entries; i++) {
        unsigned short neighbor_id = ntohs(payload[i * 2]);
        unsigned short cost = ntohs(payload[i * 2 + 1]);
        link_state_table[src_id][neighbor_id] = cost;
    }

    printf("Router %d: Received LS packet from %d with sequence number %d\n", router_id, src_id, seq_num);

    // 重新计算最短路径
    calculate_shortest_paths();

    // 向其他邻居转发该 LS 包
    for (unsigned short i = 0; i < num_ports; i++) {
        if (i != port && ports[i].isAlive) {
            sys->send(i, packet, pkt->size);
        }
    }
}

void RoutingProtocolImpl::calculate_shortest_paths() {
    printf("Router %d: Calculating shortest paths\n", router_id);

    std::unordered_map<unsigned short, unsigned int> distance;
    std::unordered_map<unsigned short, unsigned short> previous;
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

    // 初始化距离表
    for (const auto &neighbor : link_state_table[router_id]) {
        distance[neighbor.first] = neighbor.second;
        previous[neighbor.first] = router_id;
        pq.emplace(neighbor.second, neighbor.first);
    }

    while (!pq.empty()) {
        std::pair<int, int> top = pq.top();
        unsigned int dist = top.first;
        int node = top.second;
        pq.pop();

        if (dist > distance[node]) continue;

        for (const auto &neighbor : link_state_table[node]) {
            unsigned int new_cost = distance[node] + neighbor.second;
            if (new_cost < distance[neighbor.first]) {
                distance[neighbor.first] = new_cost;
                previous[neighbor.first] = node;
                pq.emplace(new_cost, neighbor.first);
            }
        }
    }

    printf("Router %d: Calculating shortest paths\n", router_id);

    // 更新转发表
    for (const auto &entry : distance) {
        unsigned short dest = entry.first;
        if (dest != router_id) {
            routing_table[dest].next_hop = previous[dest];
            routing_table[dest].cost = distance[dest];
            printf("Router %d: Path to %d via %d with cost %d\n", router_id, dest, previous[dest], distance[dest]);
        }
    }
}
