#pragma once
#include "TUIBlock/TUIBlock.hpp"
#include "TUIComponents/TUICBase.hpp"
#include "TerminalInterface.hpp"
#include <functional>
#include <iostream>
#include <sstream>
#include <string_view>
#include <vector>

class TUIRenderer {
public:
    TUIRenderer(std::function<void(TUIEventType)> onEvent = nullptr,
                std::function<std::vector<std::reference_wrapper<TUICBase>>()> onUpdate = nullptr)
        : onEvent(onEvent), onUpdate(onUpdate) {};
    void updatePage();
    int renderLoop();

private:
    void renderBlocks(const std::vector<std::vector<TUIBlock>> &blocks);
    const std::string_view cursorResetCode = "\033[0;0H";
    const std::string_view cursorMoveCode = "\033[%d;%dH";
    const std::string_view clearScreenCode = "\033[2J\033[H";
    const std::string_view clearScreenAfterCode = "\033[0J";
    const std::string_view clearLineAfterCode = "\033[0K";
    const std::string_view hideCursorCode = "\033[?25l";
    const std::string_view showCursorCode = "\033[?25h";
    int lastTerminalWidth = 0;
    std::vector<std::vector<TUIBlock>> lastFrame{};
    std::function<void(TUIEventType)> onEvent;
    std::function<std::vector<std::reference_wrapper<TUICBase>>()> onUpdate;
};
