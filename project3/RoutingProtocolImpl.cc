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

    ports.resize(num_ports);
    link_state_table[router_id] = {};
    ls_sequence_number[router_id] = 0;

    sys->set_alarm(this, 0, (void *)ALARM_PING);
    // sys->set_alarm(this, CHECK_DURATION, (void *)ALARM_CHECK);

    if (protocol_type == P_DV)
    {
        sys->set_alarm(this, 30000, (void *)ALARM_DV);
        sys->set_alarm(this, 1000, (void *)ALARM_DV_TIMEOUT);
    }
    else if (protocol_type == P_LS)
    {
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

    // printf("Creating packet: type=%d, size=%d\n", type, size);

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


void RoutingProtocolImpl::handle_pong(unsigned short port, void *packet)
{
    struct packet *header = (struct packet *)packet;
    unsigned int *timestamp = (unsigned int *)((char *)packet + sizeof(struct packet));
    unsigned short src_id = ntohs(header->src);

    // 计算RTT
    unsigned int send_time = ntohl(*timestamp);
    unsigned int rtt = sys->time() - send_time;
    if (rtt > 65535)
        rtt = 65535;

    // 获取或创建邻居信息
    auto &neighbor = ports[port].neighbors[src_id];
    bool was_alive = neighbor.isAlive;
    // printf("test:was_alive:%d\n", was_alive);
    if (!was_alive)
    {
        printf("Pong: Router %d was_dead before\n", router_id);
    }
    bool need_update = false;
    unsigned int old_rtt = neighbor.cost;

    neighbor.cost = (unsigned short)rtt;
    neighbor.lastPongTime = sys->time();
    neighbor.isAlive = true;

    // 更新链路状态表
    link_state_table[router_id][src_id] = neighbor.cost;
    ls_last_update[router_id][src_id] = sys->time();

    if (!was_alive || old_rtt != rtt)
    {
        need_update = true;
        printf("Time %d: Router %d Port %d connected to Router %d, RTT=%dms\n",
               sys->time(), router_id, port, src_id, rtt);
    }
    // 触发LS和DV更新
    if (need_update)
    {
        if (protocol_type == P_DV)
        {
            printf("Router %d: Topology changed, sending DV update\n", router_id);
            bool updated = false;
            if (routing_table.find(src_id) == routing_table.end())
            {
                update_route(src_id, src_id, port, rtt);
                updated = true;
            }
            else
            {
                auto &route_entry = routing_table[src_id];
                if (route_entry.next_hop == src_id || rtt < route_entry.cost)
                {
                    // 直连，或者新 RTT 更优
                    route_entry.cost = rtt;
                    route_entry.next_hop = src_id;
                    route_entry.last_update = sys->time();
                    updated = true;
                }
            }

            if (updated)
            {
                // path update
                unsigned int current_time = sys->time();
                unsigned short diff = rtt - old_rtt;
                for (auto &it : routing_table)
                {
                    if (it.second.next_hop == src_id)
                    {
                        it.second.cost += diff;
                        it.second.last_update = current_time;
                    }
                    // better cost with direct neighbor:update
                    for (unsigned short tport = 0; tport < num_ports; tport++)
                    {
                        auto &port_status = ports[tport];
                        for (auto &pit : port_status.neighbors)
                            if (pit.first == it.first && pit.second.isAlive && pit.second.cost < it.second.cost)
                            {
                                update_route(it.first, it.first, tport, pit.second.cost);
                            }
                    }
                }
            }
            // trigger
            cout<<"Handling Pong"<<endl;
            print_DV_routing_table();
            send_dv_update(need_update);
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
    {
        // check_neighbor_status();
        // sys->set_alarm(this, 1000, (void *)2);
    }
    else if (alarm_type == ALARM_DV)
    {
        // Round
        send_dv_update(false);
        sys->set_alarm(this, 30000, (void *)ALARM_DV);
    }
    else if (alarm_type == ALARM_DV_TIMEOUT)
    {
        check_DV_timeout();
        sys->set_alarm(this, 1000, (void *)ALARM_DV_TIMEOUT);
    }
    else if (alarm_type == ALARM_LS)
    {
        send_ls_update();
        sys->set_alarm(this, 30000, (void *)5); // 30秒后再次更新
    }
    else if (alarm_type == ALARM_LS_TIMEOUT)
    {
        check_link_state_timeout();
        sys->set_alarm(this, 1000, (void *)6); // 1秒后再次检查
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

        // Send DV Routing table to neighbor with poison reversion

        // bool has_active_neighbors = false;
        for (const std::pair<unsigned short, NeighborInfo> &pair : port_neighbors)
        {
            if (!pair.second.isAlive)
            {
                continue;
            }
            int entry_index = 0;
            void *packet = create_packet(DV, packet_size);
            unsigned short *payload = (unsigned short *)((char *)packet + sizeof(struct packet));

            struct packet *header = (struct packet *)packet;
            header->dst = htons(pair.first);

            for (const auto &route : routing_table)
            {
                if (!route.second.valid)
                    continue;

                unsigned short cost = route.second.cost;

                unsigned short next_nop = route.second.next_hop;
                if (pair.first == next_nop)
                {
                    cost = INFINITY_COST;
                }

                payload[entry_index++] = htons(route.first); // destination
                payload[entry_index++] = htons(cost);        // cost
            }
            sys->send(port, packet, packet_size);
        }
    }
}

void RoutingProtocolImpl::print_DV_routing_table(){
    printf("---------------Router:%d-----------------\n",router_id);
    for(auto it:routing_table){
        printf("dst:%d  nxt:%d cost:%d\n",int(it.second.destination),int(it.second.next_hop),int(it.second.cost));
    }
    printf("---------------------------------------\n");
}


void RoutingProtocolImpl::handle_dv_packet(unsigned short port, void *packet)
{
    struct packet *pkt = (struct packet *)packet;
    unsigned short src_id = ntohs(pkt->src);
    unsigned short dst_id = ntohs(pkt->dst);
    unsigned short *payload = (unsigned short *)((char *)packet + sizeof(struct packet));

    int num_entries = (ntohs(pkt->size) - sizeof(struct packet)) / (sizeof(unsigned short) * 2);

    auto &port_neighbors = ports[port].neighbors;
    auto neighbor_it = port_neighbors.find(src_id);

    if (!num_entries || dst_id != router_id)
    {
        printf("Router %d: Wrong DV packet from %d\n", router_id, src_id);
        delete[] (char *)packet;
        return;
    }

    unordered_map<unsigned short, unsigned short> DV_table;
    for (int i = 0; i < num_entries; i++)
    {
        unsigned short dest = ntohs(payload[i * 2]);
        unsigned int cost = ntohs(payload[i * 2 + 1]);
        DV_table[dest] = cost;
    }

    // check neighbor status?  update : add
    // lost ping packet may cause this
    if (neighbor_it == port_neighbors.end() || !neighbor_it->second.isAlive)
    {
        auto &neighbor = ports[port].neighbors[src_id];
        neighbor.lastPongTime = sys->time();
        neighbor.isAlive = true;
        neighbor.cost = DV_table[router_id];
    }
    else
    {
        auto &neighbor = ports[port].neighbors[src_id];
        neighbor.lastPongTime = sys->time();
        neighbor.isAlive = true;
    }
    bool route_changed = false;
    // update src_id
    update_route(src_id, src_id, port, DV_table[this->router_id]);
    // if (routing_table.find(src_id) != routing_table.end())
    // {
    //     routing_table[src_id].last_update = sys->time();
    //     auto &route_src = routing_table[src_id];
    //     unsigned short new_cost = DV_table[this->router_id];
    //     if (route_src.cost > new_cost && route_src.next_hop != src_id)
    //     {
    //         update_route(src_id, src_id, port, new_cost);
    //         route_changed = true;
    //     }
    // }

    for (int i = 0; i < num_entries; i++)
    {
        unsigned short dest = ntohs(payload[i * 2]);
        unsigned int cost = ntohs(payload[i * 2 + 1]);

        if (dest == this->router_id)
            continue;
        auto it = routing_table.find(dest);

        if (cost == INFINITY_COST)
        {
            // 反向毒化对应处理
            // bool is_neighbor = 0;
            // unsigned int neighbor_cost;
            // unsigned short neighbor_port;

            // for (unsigned short tport = 0; tport < num_ports; tport++)
            // {
            //     auto &port_status = ports[tport];
            //     auto neighbor_it = port_status.neighbors.find(dest);
            //     if (neighbor_it != port_status.neighbors.end() && neighbor_it->second.isAlive)
            //     {
            //         is_neighbor = true;
            //         neighbor_cost = neighbor_it->second.cost;
            //         neighbor_port = tport;
            //         break;
            //     }
            // }

            // if (it == routing_table.end() || it->second.next_hop != src_id)
            //     continue;

            // if (is_neighbor)
            // {
            //     update_route(dest, dest, neighbor_port, neighbor_cost);
            //     route_changed = 1;
            // }
            // else
            // routing_table.erase(dest);
            // route_changed = 1;
            continue;
        }

        unsigned int total_cost = cost + neighbor_it->second.cost;

        if (it == routing_table.end() || !it->second.valid)
        {
            if (total_cost < INFINITY_COST)
            {
                update_route(dest, src_id, port, total_cost);
                route_changed = true;
            }
        }
        else
        {
            if (total_cost < routing_table[dest].cost ||
                (routing_table[dest].next_hop == src_id && total_cost != routing_table[dest].cost))
            {
                update_route(dest, src_id, port, total_cost);
                route_changed = true;
            }
            else if (routing_table[dest].next_hop == src_id && total_cost == routing_table[dest].cost)
            {
                routing_table[dest].last_update = sys->time();
            }
        }
    }

    if (route_changed)
    {
        // trigger
        send_dv_update(true);
        cout<<"Handling DV"<<endl;
        print_DV_routing_table();
    }
    delete[] (char *)packet;
}

// done
void RoutingProtocolImpl::check_DV_timeout()
{
    bool route_changed = false;
    // unsigned int current_time = sys->time();
    vector<unsigned short> invalid_neighbor;

    for (unsigned short port = 0; port < num_ports; port++)
    {
        auto &port_status = ports[port];
        for (auto &it : port_status.neighbors)
        {
            if (it.second.isAlive &&
                sys->time() - it.second.lastPongTime > PONG_TIMEOUT)
            { // 超时移除
                it.second.isAlive = false;
                it.second.cost = 0;
                route_changed = true;
                printf("Router %d 1s Round Check: %d timeout\n", router_id,it.first);
                unsigned short nid=it.first;
                delete_DV_invalid(nid);
            }
        }
    }
    if (route_changed)
    {
        // trigger
        send_dv_update(true);
        cout<<"1s checking"<<endl;
        print_DV_routing_table();
    }
    
}

// return port or INVALID_PORT
unsigned short RoutingProtocolImpl::find_neighbor(unsigned short id)
{
    for (unsigned short port = 0; port < num_ports; port++)
    {
        auto &port_neighbor = ports[port].neighbors;
        auto neighbor_it = port_neighbor.find(id);
        if (neighbor_it != port_neighbor.end())
        {
            if (neighbor_it->second.isAlive)
                return port;
            else
                return INVALID_PORT;
        }
    }
    return INVALID_PORT;
}

void RoutingProtocolImpl::delete_DV_invalid(unsigned short invalid_id)
{

    vector<unsigned short> to_delete;
    for (auto &it : routing_table)
    {
        if (sys->time() - it.second.last_update >= ROUTE_TIMEOUT)
        {
            to_delete.emplace_back(it.first);
            cout<<"Route timeout:"<<it.first<<endl;
            cout<<"Route timeout:"<<it.second.last_update<<endl;
            continue;
        }
        if (it.second.next_hop == invalid_id)
        {
            unsigned short find_port = find_neighbor(it.first);
            if (find_port == INVALID_PORT){
                cout<<"Not Neighbor:"<<it.first<<endl;
                to_delete.emplace_back(it.first);
            }   
            else
            {
                auto &pit = ports[find_port].neighbors;
                auto neighbor_it = pit.find(it.first);
                if (neighbor_it != pit.end() && neighbor_it->second.cost != it.second.cost)
                {
                    update_route(it.first, it.first, find_port, neighbor_it->second.cost);
                }
            }
        }
    }
    for (auto it : to_delete)
    {
        cout<<it<<endl;
        if (routing_table.find(it) != routing_table.end())
            routing_table.erase(it);
    }
}

void RoutingProtocolImpl::recv(unsigned short port, void *packet, unsigned short size)
{
    // 检查size 是否小于 packet 结构体大小、检查 port 是否有效
    // if (size < sizeof(struct packet) || port >= num_ports)
    // {
    //     // delete[] (char *)packet;
    //     printf("Error Packet");
    //     return;
    // }

    struct packet *header = (struct packet *)packet;

    // 转为主机字节序
    ePacketType pkt_type = (ePacketType)(*((unsigned char *)packet));
    unsigned short pkt_size = ntohs(header->size);
    unsigned short pkt_src = ntohs(header->src);
    unsigned short pkt_dst = ntohs(header->dst);

    printf("Router %d:%d: Received %d packet %d -> %d\n", router_id, port, pkt_type, pkt_src, pkt_dst);
    if (pkt_size != size)
    {
        printf("Packet size mismatch: expected %u, got %u\n", pkt_size, size);
        return;
    }

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
    case DATA:
        handle_data(port, packet, size);
        break;
    default:
        printf("Unknown packet type received: %d\n", pkt_type);
        break;
    }

    // delete[] (char *)packet;
}
void RoutingProtocolImpl::handle_data(unsigned short port, void *packet, unsigned short size)
{
    struct packet *header = (struct packet *)packet;
    unsigned short dst = ntohs(header->dst);
    if (dst == router_id)
    {
        // printf("Own Packet:%d",router_id);
        free(packet);
        return;
    }

    auto it = routing_table.find(dst);
    if (it != routing_table.end() && it->second.valid)
    {
        unsigned short next_port = it->second.port;
        sys->send(next_port, packet, size);
        // printf("DATA:%d -> %d\n",router_id,dst);
    }
    else
    {
        printf("Destination %d unreachable from Router %d\n", header->dst, router_id);
    }
}
void RoutingProtocolImpl::send_ls_update()
{
    printf("Router %d: Attempting to send LS update\n", router_id);

    // 统计活动邻居数量
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

    if (num_neighbors == 0)
    {
        printf("Router %d: No active neighbors, not sending LS update\n", router_id);
        return;
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
                payload[offset++] = htons(neighbor.first);
                payload[offset++] = htons(neighbor.second.cost);
            }
        }
    }

    for (unsigned short port = 0; port < num_ports; port++)
    {
        char *ls_packet = new char[packet_size];
        memcpy(ls_packet, packet, packet_size);
        sys->send(port, ls_packet, packet_size);
        printf("Router %d: Broadcasted LS update on port %d\n", router_id, port);
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
        // delete[] (char *)ls_packet;
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

    // Constants for timeouts
    static const unsigned int PONG_TIMEOUT = 15000;
    static const unsigned int LS_ENTRY_TIMEOUT = 45000;

    // Check neighbor timeouts
    for (unsigned short port = 0; port < num_ports; port++)
    {
        auto &port_status = ports[port];

        for (auto it = port_status.neighbors.begin(); it != port_status.neighbors.end();)
        {
            unsigned short neighbor_id = it->first;
            auto &neighbor = it->second;

            if (neighbor.isAlive)
            {
                if (current_time - neighbor.lastPongTime > PONG_TIMEOUT)
                {
                    printf("Time %d: Router %d neighbor %d on port %d timed out\n",
                           current_time, router_id, neighbor_id, port);
                    neighbor.isAlive = false;
                    topology_changed = true;

                    link_state_table[router_id].erase(neighbor_id);
                    ls_last_update[router_id].erase(neighbor_id);
                }
            }
            ++it;
        }
    }

    // Check LS entry timeouts
    for (auto it_router = link_state_table.begin(); it_router != link_state_table.end();)
    {
        unsigned short other_router_id = it_router->first;
        auto &neighbors = it_router->second;

        for (auto it_neighbor = neighbors.begin(); it_neighbor != neighbors.end();)
        {
            unsigned short neighbor_id = it_neighbor->first;

            if (current_time - ls_last_update[other_router_id][neighbor_id] > LS_ENTRY_TIMEOUT)
            {
                printf("Time %d: Router %d: Link from %d to %d expired\n",
                       current_time, router_id, other_router_id, neighbor_id);
                it_neighbor = neighbors.erase(it_neighbor);
                ls_last_update[other_router_id].erase(neighbor_id);

                topology_changed = true;
            }
            else
            {
                ++it_neighbor;
            }
        }

        if (neighbors.empty())
        {
            it_router = link_state_table.erase(it_router);
            ls_last_update.erase(other_router_id);
        }
        else
        {
            ++it_router;
        }
    }

    if (topology_changed)
    {
        calculate_shortest_paths();
        send_ls_update();
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
        if (protocol_type == P_LS)
        {
            printf("Router %d: Topology changed, sending LS update\n", router_id);
            send_ls_update();
        }
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
