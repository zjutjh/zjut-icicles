#include "TerminalInterface.hpp"

#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#else
#include <cstdio>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

// 获取原始按键
int waitRawKey() {
#ifdef _WIN32
    return _getch();
#else
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#endif
}

// 解析复合按键
TUIEventType waitKey() {
    int ch = waitRawKey();
#ifdef _WIN32
    if (ch == 0 || ch == 224) {
        ch = _getch();
        switch (ch) {
        case 72:
            return TUIEventType::KeyUp;
        case 80:
            return TUIEventType::KeyDown;
        case 75:
            return TUIEventType::KeyLeft;
        case 77:
            return TUIEventType::KeyRight;
        default:
            return TUIEventType::Unknown;
        }
    }
#else
    if (ch == 27) { // ESC 序列
        // 使用非阻塞检测更佳，这里简化处理逻辑
        int n2 = waitRawKey();
        if (n2 == '[') {
            int n3 = waitRawKey();
            switch (n3) {
            case 'A':
                return TUIEventType::KeyUp;
            case 'B':
                return TUIEventType::KeyDown;
            case 'C':
                return TUIEventType::KeyRight;
            case 'D':
                return TUIEventType::KeyLeft;
            default:
                return TUIEventType::Unknown;
            }
        }
        return TUIEventType::Unknown;
    }
#endif
    return static_cast<TUIEventType>(ch);
}

// 缓冲区清理
void flushTerminalBuffer() {
#ifdef _WIN32
    while (_kbhit())
        _getch();
#else
    tcflush(STDIN_FILENO, TCIFLUSH);
#endif
}

int getTerminalWidth() {
#if defined(_WIN32)
    // 尝试直接打开控制台输出设备 "CONOUT$"
    // 这样即使 stdout 被重定向也能获取到控制台大小
    HANDLE hConsole = CreateFileW(L"CONOUT$", GENERIC_READ | GENERIC_WRITE,
                                  FILE_SHARE_READ | FILE_SHARE_WRITE,
                                  NULL, OPEN_EXISTING, 0, NULL);
    if (hConsole == INVALID_HANDLE_VALUE) {
        // 如果打开 CONOUT$ 失败，降级尝试获取标准输出句柄
        hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    }
    if (hConsole != INVALID_HANDLE_VALUE) {
        CONSOLE_SCREEN_BUFFER_INFO csbi;
        if (GetConsoleScreenBufferInfo(hConsole, &csbi)) {
            // 如果是通过 CreateFile 打开的句柄，记得关闭
            // 注意：不要关闭从 GetStdHandle 获取的全局句柄
            if (hConsole != GetStdHandle(STD_OUTPUT_HANDLE)) {
                CloseHandle(hConsole);
            }
            return csbi.srWindow.Right - csbi.srWindow.Left + 1;
        }
        if (hConsole != GetStdHandle(STD_OUTPUT_HANDLE)) {
            CloseHandle(hConsole);
        }
    }
    return 80; // 彻底失败时的默认值
#else
    // Linux / macOS 保持原样
    struct winsize w;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) == 0) {
        return w.ws_col;
    }
    // 如果标准输出被重定向，尝试从控制台设备直接读取
    int fd = open("/dev/tty", O_RDONLY);
    if (fd >= 0) {
        if (ioctl(fd, TIOCGWINSZ, &w) == 0) {
            close(fd);
            return w.ws_col;
        }
        close(fd);
    }
    return 80;
#endif
}