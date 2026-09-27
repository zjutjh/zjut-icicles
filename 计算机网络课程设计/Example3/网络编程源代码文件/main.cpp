// C++ 标准库头文件
#include <bits/stdc++.h>

// raw socket 编程所需头文件，需要链接到动态库WS2_32
#include <winsock2.h>
#include <mstcpip.h>

const char *const filePath = "output.txt";
const int maxHostNameLen = 256; // 8位
const int maxBufferSize = 65536; // 16位

char buffer[maxBufferSize];

struct IP_datagram {
    unsigned char Version_IHL; // 首部长度 4位 Internet Header Length   版本 4位 Version
    unsigned char TOS; // 服务类型 8位 Type Of Service
    unsigned short TL; // 总长度 16位 Total Length
    unsigned short id; // 标识 16位 Identification
    unsigned short flags_fragOffset; // 标志 3位 Flags  片偏移 13位 Fragment Offset
    unsigned char TTL; // 生存时间 8位 Time To Live
    unsigned char protocol; // 协议 8位 Protocol
    unsigned short headerChecksum; // 首部检验和 16位 Header Checksum
    unsigned int srcAdd; // 源地址 32位 Source Address
    unsigned int destAdd; // 目的地址 32位 Destination Address

    std::string getProtocol() const {
        switch (protocol) {
            case 1:
                return "ICMP (1)";
            case 2:
                return "IGMP (2)";
            case 6:
                return "TCP (6)";
            case 17:
                return "UDP (17)";
            default:
                return std::to_string((int)protocol);
        }
    }

    std::string getVersion() const {
        unsigned char version = Version_IHL >> 4;
        switch (version) {
            case 4:
                return "IPv4 (4)";
            case 6:
                return "IPv6 (6)";
            default:
                return std::to_string((int) version);
        }
    }

    std::string getFlag() const{
        unsigned char flags = flags_fragOffset >> 13;
        std::string description;
        if (flags & 0b001) description+="more fragments, ";
        if (flags & 0b010) description+="don't fragment, ";
        if (description.empty()) return "0";
        description.pop_back();description.pop_back();
        description += " (" + std::to_string(flags) + ")";
        return description;
    }

    void invertByte() {
        unsigned short inv[10]; // IP 数据包共 20 字节
        unsigned short *p = (unsigned short *) this;
        for(int i = 0; i < 10; i++){
            memcpy(inv + i, p, sizeof(unsigned short));
            if(i == 1 || i == 2 || i == 3 || i == 5) inv[i] = ntohs(inv[i]);
            p++;
        }
        memcpy(this, inv, sizeof(unsigned short) * 10);
    }
};

static int count = 0; // 用于IP包计数

void analyze(char *buffer, int length) {
    auto *ip = (IP_datagram *) buffer;
    ip->invertByte();
    SOCKADDR_IN address{};

    printf(":: Packet %d:\n", count);
    printf("   Version:                  %s\n", ip->getVersion().c_str());
    printf("   Internet Header Length:   %d bytes\n", ((ip->Version_IHL) & 0b00001111) * 4);
    printf("   Type of Service:          %d\n", ip->TOS);
    printf("   Flags:                    %s\n", ip->getFlag().c_str());
    printf("   Fragment Offset:          %d\n", (ip->flags_fragOffset) & 0b0001111111111111);
    printf("   Total Length:             %d bytes\n", ip->TL);
    printf("   Identification:           %d\n", ip->id);
    printf("   Time To Live:             %d\n", ip->TTL);
    printf("   Protocol:                 %s\n", ip->getProtocol().c_str());

    address.sin_addr.s_addr = ip->srcAdd;
    printf("   Source Address:           %s\n", inet_ntoa(address.sin_addr)); // 将网络地址转换成“.”点隔的字符串格式
    address.sin_addr.s_addr = ip->destAdd;
    printf("   Destination Address:      %s\n", inet_ntoa(address.sin_addr));

    puts("");
}

int captureData() {
    WSAData wsa_data{};
    char hostname[maxHostNameLen];

    DWORD dwInBuff = 1;
    DWORD dwOutBuff;
    DWORD dwBytesRet;

    // 初始化winsock库，版本号为 0x0202
    if (WSAStartup(0x0202, &wsa_data)) {
        fprintf(stderr, "WSA startup failed.\n");
        return -1;
    }

    // 获取主机名
    if (gethostname(hostname, maxHostNameLen)) {
        fprintf(stderr, "Failed to get hostname.\n");
        WSACleanup();
        return -1;
    }

    fprintf(stderr, "Hostname: %s\n", hostname);

    // 根据主机名获取 ip 地址
    HOSTENT *host = gethostbyname(hostname);
    fprintf(stderr, "IP: %d.%d.%d.%d\n", host->h_addr_list[0][0] & 0xff, host->h_addr_list[0][1] & 0xff,
            host->h_addr_list[0][2] & 0xff, host->h_addr_list[0][3] & 0xff);

    SOCKADDR_IN address{};
    address.sin_addr.s_addr = *(unsigned long *) host->h_addr; // 绑定主机 ip
    address.sin_family = AF_INET;
    address.sin_port = htons(0); // 绑定端口

    // 创建 raw socket 对象
    SOCKET sockServer = socket(AF_INET, SOCK_RAW, IPPROTO_IP);
    if (sockServer < 0) {
        fprintf(stderr, "Failed to open a raw socket entity.");
        WSACleanup();
        return -1;
    }

    // 绑定本机网卡
    if (bind(sockServer, (SOCKADDR *) &address, sizeof(SOCKADDR))) {
        fprintf(stderr, "Failed to bind to the ip.");
        WSACleanup();
        closesocket(sockServer);
        return -1;
    }

    // 设置接受所有数据包
    int ret = WSAIoctl(sockServer, SIO_RCVALL, &dwInBuff, sizeof(dwInBuff),
                       &dwOutBuff, sizeof(dwOutBuff), &dwBytesRet, nullptr, nullptr);
    if (ret) {
        fprintf(stderr, "Failed to set ioctl configuration.");
        WSACleanup();
        closesocket(sockServer);
        return -1;
    }

    // 开始抓包
    while (true) {
        memset(buffer, 0x00, sizeof(buffer));
        int bufferSize = recv(sockServer, buffer, maxBufferSize, 0);

        if (bufferSize < 0) {
            fprintf(stderr, "Capture error.\n");
            break;
        }

        fputc('.', stderr);
        ++count;
        analyze(buffer, bufferSize);
    }

    closesocket(sockServer);
    WSACleanup();
    return 0;
}

int main(int argc, char *argv[]) {
    if (argc == 1) {
        fprintf(stderr, "The results will be output to the default file: %s \n" , filePath);
        freopen(filePath, "w", stdout);
    } else if (argc == 2) {
        fprintf(stderr, "The results will be output to: %s \n" , argv[1]);
        freopen(argv[1], "w", stdout);
    } else {
        fprintf(stderr, "usage: %s <output-file>\n", argv[0]);
        return -1;
    }
    return captureData();
}