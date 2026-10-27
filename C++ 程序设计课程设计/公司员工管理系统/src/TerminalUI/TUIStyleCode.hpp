#pragma once
#include <string_view>
struct TUIStyleCode {
    static constexpr std::string_view bold = "1";
    static constexpr std::string_view underline = "4";
    static constexpr std::string_view invert = "7";
    std::string_view bgBlack;
    std::string_view bgBlue;
    std::string_view bgGreen;
    std::string_view bgAqua;
    std::string_view bgRed;
    std::string_view bgPurple;
    std::string_view bgYellow;
    std::string_view bgWhite;
    std::string_view bgGray;
    std::string_view fgBlack;
    std::string_view fgBlue;
    std::string_view fgGreen;
    std::string_view fgAqua;
    std::string_view fgRed;
    std::string_view fgPurple;
    std::string_view fgYellow;
    std::string_view fgWhite;
    std::string_view fgGray;
    static constexpr std::string_view reset = "0";
    static constexpr std::string_view prefix = "\033[";
    static constexpr std::string_view separator = ";";
    static constexpr std::string_view suffix = "m";
};

static constexpr TUIStyleCode defaultStyleCode = {
    "40",  // bgBlack
    "44",  // bgBlue
    "42",  // bgGreen
    "46",  // bgAqua
    "41",  // bgRed
    "45",  // bgPurple
    "43",  // bgYellow
    "47",  // bgWhite
    "100", // bgGray
    "30",  // fgBlack
    "34",  // fgBlue
    "32",  // fgGreen
    "36",  // fgAqua
    "31",  // fgRed
    "35",  // fgPurple
    "33",  // fgYellow
    "37",  // fgWhite
    "90"   // fgGray
};
