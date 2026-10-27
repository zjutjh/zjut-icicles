#pragma once
#include "TUIEvent.hpp"
#include <functional>
#include <string_view>
#include <vector>

namespace TUISpecialChar {
    static constexpr std::string_view h = "─";
    static constexpr std::string_view dh = "═";
    static constexpr std::string_view v = "│";
    static constexpr std::string_view ellipsis = "…";
}

struct TUIPageOperation {
    std::string_view name;
    int keyHintIdx;
    TUIEventType triggerEvent;
    std::function<void()> action;
};
