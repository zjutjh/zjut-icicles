#include "TUIBlock.hpp"

// 静态成员定义
TUIStyleCode TUIBlock::styleCode = defaultStyleCode;

std::ostream &operator<<(std::ostream &os, const TUIBlock &pixel) {
    std::string output;
    output += TUIStyleCode::prefix;
    output += TUIStyleCode::reset;
    output += TUIStyleCode::separator;
    output += pixel.convertColorCode(pixel.bColor, true);
    output += TUIStyleCode::separator;
    output += pixel.convertColorCode(pixel.fColor, false);
    output += pixel.convertStyleSet();
    output += TUIStyleCode::suffix;
    output += pixel.str;
    output += TUIStyleCode::prefix;
    output += TUIStyleCode::reset;
    output += TUIStyleCode::suffix;

    return os << output;
}

int TUIBlock::getCharBytes(unsigned char first) {
    if (first < 0x80)
        return 1;
    if ((first & 0xE0) == 0xC0)
        return 2;
    if ((first & 0xF0) == 0xE0)
        return 3;
    if ((first & 0xF8) == 0xF0)
        return 4;
    return 1; // Default/fallback
}

void TUIBlock::delLastChar(std::string &str) {
    if (str.empty()) {
        return;
    }
    std::vector<int> byteIdx = charByteIdx(str);
    str.resize(str.size() - byteIdx.back() - 1);
    // resize(result[i].size() - byteIdx[byteIdx.size() - 1] - 1);
}

int TUIBlock::width(std::string str) {
    if (str.empty())
        return 0;

    int totalWidth = 0;
    size_t i = 0;

    // 遍历字符串中的每个 UTF-8 字符
    while (i < str.size()) {
        unsigned char first = static_cast<unsigned char>(str[i]);
        char32_t cp = 0;
        int charBytes = getCharBytes(first);

        // 1. 解码 UTF-8 字符为 Unicode Codepoint
        if (charBytes == 1) {
            cp = first;
        } else if (charBytes == 2) {
            if (i + 1 < str.size()) {
                cp = ((first & 0x1F) << 6) | (static_cast<unsigned char>(str[i + 1]) & 0x3F);
            }
        } else if (charBytes == 3) {
            if (i + 2 < str.size()) {
                cp = ((first & 0x0F) << 12) | ((static_cast<unsigned char>(str[i + 1]) & 0x3F) << 6) | (static_cast<unsigned char>(str[i + 2]) & 0x3F);
            }
        } else if (charBytes == 4) {
            if (i + 3 < str.size()) {
                cp = ((first & 0x07) << 18) | ((static_cast<unsigned char>(str[i + 1]) & 0x3F) << 12) | ((static_cast<unsigned char>(str[i + 2]) & 0x3F) << 6) | (static_cast<unsigned char>(str[i + 3]) & 0x3F);
            }
        }

        // 2. 获取该字符的宽度并累加
        totalWidth += getCharWidth(cp);
        i += charBytes;
    }

    return totalWidth;
}

int TUIBlock::width(std::vector<TUIBlock> blocks) {
    int totalWidth = 0;
    for (const TUIBlock &block : blocks) {
        totalWidth += block.width();
    }
    return totalWidth;
}

std::vector<int> TUIBlock::charByteIdx(std::string str) {
    std::vector<int> res;
    res.reserve(str.size());
    size_t i = 0;
    while (i < str.size()) {
        int charBytes = getCharBytes(static_cast<unsigned char>(str[i]));
        if (i + charBytes > str.size()) {
            charBytes = str.size() - i;
        }
        for (int j = 0; j < charBytes; ++j) {
            res.push_back(j);
        }
        i += charBytes;
    }
    return res;
}
