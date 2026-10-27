#pragma once
#include "TUIDefinitions.hpp"
#include "TUIStyleCode.hpp"
#include <stdint.h>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

class TUIBlock {
    friend std::ostream &operator<<(std::ostream &os, const TUIBlock &pixel);

public:
    virtual ~TUIBlock() = default;
    enum class Color {
        Default,
        Black,
        Blue,
        Green,
        Aqua,
        Red,
        Purple,
        Yellow,
        White,
        Gray
    };
    enum Flag : int16_t {
        Bold = 1 << 0,
        Underline = 1 << 1,
        Invert = 1 << 2
    };
    static void initStyleCode() {
        styleCode = defaultStyleCode;
    }
    explicit TUIBlock(bool isInput,
                      std::string str = "",
                      Color fColor = Color::Default,
                      Color bColor = Color::Default,
                      int16_t styleFlags = 0)
        : isInput(isInput), str(std::move(str)), fColor(fColor), bColor(bColor), styleFlags(styleFlags) {}
    explicit TUIBlock(std::string ch,
                      Color fColor = Color::Default,
                      Color bColor = Color::Default,
                      int16_t styleFlags = 0)
        : str(std::move(ch)), fColor(fColor), bColor(bColor), styleFlags(styleFlags) {}
    explicit TUIBlock(std::string_view ch,
                      Color fColor = Color::Default,
                      Color bColor = Color::Default,
                      int16_t styleFlags = 0)
        : TUIBlock(std::string(ch), fColor, bColor, styleFlags) {}
    explicit TUIBlock(const char *ch,
                      Color fColor = Color::Default,
                      Color bColor = Color::Default,
                      int16_t styleFlags = 0)
        : TUIBlock(std::string_view(ch), fColor, bColor, styleFlags) {}
    explicit TUIBlock(std::string_view ch, int count,
                      Color fColor = Color::Default,
                      Color bColor = Color::Default,
                      int16_t styleFlags = 0)
        : fColor(fColor), bColor(bColor), styleFlags(styleFlags) {
        std::string result;
        for (int i = 0; i < count; i++) {
            result += ch;
        }
        str = result;
    }
    bool operator==(const TUIBlock &other) const;
    bool operator!=(const TUIBlock &other) const { return !(*this == other); }
    std::vector<TUIBlock> operator+(const TUIBlock &other) const;
    std::vector<TUIBlock> operator+(std::vector<TUIBlock> other) const;
    std::vector<TUIBlock> operator*(int count) const;
    std::string_view convertColorCode(Color color, bool isBackground = false) const;
    std::string convertStyleSet() const;
    int width() const;
    static int width(std::string str);
    static int width(std::vector<TUIBlock> blocks);
    static std::vector<int> charByteIdx(std::string str);
    static int getCharWidth(char32_t cp);
    static int getCharBytes(unsigned char first);
    static void delLastChar(std::string &str);
    void setStyleFlag(Flag flag) { styleFlags |= flag; }
    bool isInputBlock() const { return isInput; }

private:
    bool isInput = false;
    std::string str;
    int16_t styleFlags = 0;
    Color fColor = Color::Default;
    Color bColor = Color::Default;
    static TUIStyleCode styleCode;
};

inline bool TUIBlock::operator==(const TUIBlock &other) const {
    return str == other.str && fColor == other.fColor &&
           bColor == other.bColor && styleFlags == other.styleFlags;
}

inline std::vector<TUIBlock> TUIBlock::operator+(const TUIBlock &other) const {
    std::vector<TUIBlock> result;
    result.push_back(*this);
    result.push_back(other);
    return result;
}

inline std::vector<TUIBlock> TUIBlock::operator+(std::vector<TUIBlock> other) const {
    other.insert(other.begin(), *this);
    return other;
}

inline std::vector<TUIBlock> operator+(std::vector<TUIBlock> lhs, const TUIBlock &rhs) {
    lhs.push_back(rhs);
    return lhs;
}

inline std::vector<TUIBlock> operator+(std::vector<TUIBlock> lhs, const std::vector<TUIBlock> &rhs) {
    lhs.insert(lhs.end(), rhs.begin(), rhs.end());
    return lhs;
}

inline std::vector<TUIBlock> TUIBlock::operator*(int count) const {
    std::vector<TUIBlock> result(count, *this);
    return result;
}

inline std::string_view TUIBlock::convertColorCode(Color color, bool isBackground) const {
    switch (color) {
    case Color::Default:
        return "";
    case Color::Black:
        return isBackground ? styleCode.bgBlack : styleCode.fgBlack;
    case Color::Blue:
        return isBackground ? styleCode.bgBlue : styleCode.fgBlue;
    case Color::Green:
        return isBackground ? styleCode.bgGreen : styleCode.fgGreen;
    case Color::Aqua:
        return isBackground ? styleCode.bgAqua : styleCode.fgAqua;
    case Color::Red:
        return isBackground ? styleCode.bgRed : styleCode.fgRed;
    case Color::Purple:
        return isBackground ? styleCode.bgPurple : styleCode.fgPurple;
    case Color::Yellow:
        return isBackground ? styleCode.bgYellow : styleCode.fgYellow;
    case Color::White:
        return isBackground ? styleCode.bgWhite : styleCode.fgWhite;
    case Color::Gray:
        return isBackground ? styleCode.bgGray : styleCode.fgGray;
    default:
        return "";
    }
}

inline std::string TUIBlock::convertStyleSet() const {
    std::string result;
    if (styleFlags & Flag::Bold) {
        result += (std::string(TUIStyleCode::separator) + std::string(styleCode.bold));
    }
    if (styleFlags & Flag::Underline) {
        result += (std::string(TUIStyleCode::separator) + std::string(styleCode.underline));
    }
    if (styleFlags & Flag::Invert) {
        result += (std::string(TUIStyleCode::separator) + std::string(styleCode.invert));
    }
    return result;
}

namespace TUIPredefinedBlocks {
    static const TUIBlock space = TUIBlock(" ");
    static TUIBlock hLine(int width) {
        return TUIBlock(TUISpecialChar::h, width);
    }
    static TUIBlock hDoubleLine(int width) {
        return TUIBlock(TUISpecialChar::dh, width);
    }
    static const TUIBlock clearLineRemain = TUIBlock("\033[0K");
}

inline int TUIBlock::width() const {
    return width(str);
}

// Generated
// 计算单个 UTF-8 字符的显示宽度
inline int TUIBlock::getCharWidth(char32_t cp) {
    // 处理不可见字符/控制字符
    if (cp < 0x20 || (cp >= 0x7F && cp < 0xA0))
        return 0;

    // 东亚宽字符 (Wide) 与 全角字符 (Fullwidth) 判定
    if ((cp >= 0x1100 && (cp <= 0x115F ||
                          cp == 0x2329 || cp == 0x232A ||
                          (cp >= 0x2E80 && cp <= 0xA4CF && cp != 0x303F) ||
                          (cp >= 0xAC00 && cp <= 0xD7A3) ||
                          (cp >= 0xF900 && cp <= 0xFAFF) ||
                          (cp >= 0xFE10 && cp <= 0xFE19) ||
                          (cp >= 0xFE30 && cp <= 0xFE6F) ||
                          (cp >= 0xFF01 && cp <= 0xFF60) ||
                          (cp >= 0xFFE0 && cp <= 0xFFE6) ||
                          (cp >= 0x20000 && cp <= 0x3FFFF)))) {
        return 2;
    }
    return 1;
}