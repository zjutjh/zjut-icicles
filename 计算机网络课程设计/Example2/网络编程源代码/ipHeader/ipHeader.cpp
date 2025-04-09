#include <pcap.h>
#include <winsock2.h>
#include <ws2tcpip.h>  // 用于 InetNtopA 函数
#include <iostream>
#include <cstdio>


struct ethernet_header {
    u_char dest[6];
    u_char src[6];
    u_short type;
};

struct ip_header {
    unsigned char ver_ihl;         // 版本 (4 bits) + 首部长度IHL (4 bits)
    unsigned char tos;             // 服务类型
    unsigned short tlen;           // 总长
    unsigned short identification; // 标示
    unsigned short flags_fo;       // 标志和片偏移
    unsigned char ttl;             // 生存时间
    unsigned char proto;           // 协议
    unsigned short crc;            // 首部校验和
    unsigned int saddr;            // 源地址
    unsigned int daddr;            // 目的地址
    
};

// 回调函数：每当有数据包捕获到时调用
void packet_handler(u_char* param, const struct pcap_pkthdr* header, const u_char* pkt_data)
{
    // 以太网头部为14字节
    const ethernet_header* eth = (const ethernet_header*)pkt_data;
    if (ntohs(eth->type) == 0x0800) { // type 0x0800 = IP
        const ip_header* ih = (const ip_header*)(pkt_data + sizeof(ethernet_header));

        // 提取版本与IHL
        unsigned char ver = (ih->ver_ihl & 0xF0) >> 4; // 高四位
        unsigned char ihl = ih->ver_ihl & 0x0F; // 低四位

        // IP首部长度（字节）
        unsigned int ip_header_len = ihl * 4;

        std::cout << "---- 捕获到的 IP 数据包 ----" << std::endl;
        std::cout << "版本: " << static_cast<int>(ver) << std::endl;
        std::cout << "IHL (首部长度): " << static_cast<int>(ihl)
            << " DWORDS = " << ip_header_len << " 字节" << std::endl;
        std::cout << "服务类型 (TOS): " << static_cast<int>(ih->tos) << std::endl;
        std::cout << "总长度: " << ntohs(ih->tlen) << " 字节" << std::endl;
        std::cout << "标识: " << ntohs(ih->identification) << std::endl;

        unsigned short flags = ntohs(ih->flags_fo);
        unsigned short frag_offset = flags & 0x1FFF;  // 低13位为片偏移

        // 使用位掩码提取标志位
        bool reserved = flags & 0x8000;
        bool df = flags & 0x4000;
        bool mf = flags & 0x2000;

        std::cout << "标志/片偏移: 0x" << std::hex << flags << std::dec << std::endl;
        std::cout << "  片偏移: " << frag_offset << " 单位: 8 字节" << std::endl;

        std::cout << "  标志: ";
        if (reserved) {
            std::cout << "保留位 ";
        }
        if (df) {
            std::cout << "不分片 (DF) ";
        }
        if (mf) {
            std::cout << "更多分片 (MF) ";
        }
        if (!reserved && !df && !mf) {
            std::cout << "无标志";
        }
        std::cout << std::endl;

        std::cout << "生存时间 (TTL): " << static_cast<int>(ih->ttl) << std::endl;
        std::cout << "协议: " << static_cast<int>(ih->proto) << " ";

        switch (ih->proto) {
        case 1:
            std::cout << "(ICMP)" << std::endl;
            break;
        case 6:
            std::cout << "(TCP)" << std::endl;
            break;
        case 17:
            std::cout << "(UDP)" << std::endl;
            break;
        default:
            std::cout << "(其他)" << std::endl;
            break;
        }

        std::cout << "首部校验和: 0x" << std::hex << ntohs(ih->crc) << std::dec << std::endl;

        // 转换IP地址为可读格式
        char src_ip[INET_ADDRSTRLEN];
        char dest_ip[INET_ADDRSTRLEN];

        if (InetNtopA(AF_INET, &(ih->saddr), src_ip, INET_ADDRSTRLEN) == NULL) {
            std::cerr << "源IP地址转换失败。" << std::endl;
            return;
        }

        if (InetNtopA(AF_INET, &(ih->daddr), dest_ip, INET_ADDRSTRLEN) == NULL) {
            std::cerr << "目的IP地址转换失败。" << std::endl;
            return;
        }

        std::cout << "源IP地址: " << src_ip << std::endl;
        std::cout << "目的IP地址: " << dest_ip << std::endl;

        std::cout << "-----------------------------" << std::endl;
    }
}

int main()
{
    // 初始化 WinSock
    WSADATA wsaData;
    int wsa_init = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (wsa_init != 0) {
        std::cerr << "WSAStartup 初始化失败，错误代码: " << wsa_init << std::endl;
        return -1;
    }

    pcap_if_t* alldevs;
    pcap_if_t* d;
    pcap_t* adhandle;
    char errbuf[PCAP_ERRBUF_SIZE];

    // 查找所有可用的适配器
    if (pcap_findalldevs(&alldevs, errbuf) == -1) {
        std::cerr << "查找设备时出错: " << errbuf << std::endl;
        WSACleanup();
        return -1;
    }

    // 打印可用设备列表
    int i = 0;
    for (d = alldevs; d; d = d->next)
    {
        std::cout << ++i << ": " << (d->description ? d->description : "无描述") << std::endl;
    }
    if (i == 0) {
        std::cerr << "未找到任何网络接口，请确保已安装Npcap。" << std::endl;
        pcap_freealldevs(alldevs);
        WSACleanup();
        return -1;
    }

    int inum;
    std::cout << "请输入接口编号 (1-" << i << "): ";
    std::cin >> inum;

    if (inum < 1 || inum > i) {
        std::cerr << "接口编号超出范围。" << std::endl;
        pcap_freealldevs(alldevs);
        WSACleanup();
        return -1;
    }

    // 跳转到选定的适配器
    d = alldevs;
    for (int j = 0; j < inum - 1; j++) {
        if (d->next == NULL) {
            std::cerr << "错误: 适配器数量不足。" << std::endl;
            pcap_freealldevs(alldevs);
            WSACleanup();
            return -1;
        }
        d = d->next;
    }

    // 打开选定的适配器
    // 65536 表示捕获数据的最大长度64KB；1 表示混杂模式；1000表示超时(毫秒)
    adhandle = pcap_open_live(d->name, 65536, 1, 1000, errbuf);
    if (adhandle == NULL) {
        std::cerr << "无法打开适配器: " << (d->name ? d->name : "未知") << " 错误信息: " << errbuf << std::endl;
        pcap_freealldevs(alldevs);
        WSACleanup();
        return -1;
    }

    std::cout << "正在监听 " << (d->description ? d->description : d->name) << "..." << std::endl;

    // 开始捕获数据包
    // -1 表示一直捕获，直到出错或用户中断
    if (pcap_loop(adhandle, -1, packet_handler, NULL) < 0) {
        std::cerr << "数据包捕获过程中发生错误: " << pcap_geterr(adhandle) << std::endl;
    }

    pcap_freealldevs(alldevs);
    pcap_close(adhandle);

    // 清理 WinSock
    WSACleanup();
    return 0;
}
