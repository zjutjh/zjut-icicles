#include "TUICBase.hpp"
class TUICButton : public TUICBase {
public:
    TUICButton(std::string label, std::function<void()> onClick)
        : label(std::move(label)), onClick(std::move(onClick)) {}
    void Focus() { focused = true; }
    void Unfocus() { focused = false; }
    std::vector<std::vector<TUIBlock>> Render(int width) override {
        return {
            {(focused ? (TUIBlock("     ", TUIBlock::Color::White, TUIBlock::Color::Green) +
                         TUIBlock(label, TUIBlock::Color::White, TUIBlock::Color::Green) +
                         TUIBlock("     ", TUIBlock::Color::White, TUIBlock::Color::Green))
                      : (TUIBlock("     ", TUIBlock::Color::Black, TUIBlock::Color::White) +
                         TUIBlock(label, TUIBlock::Color::Black, TUIBlock::Color::White) +
                         TUIBlock("     ", TUIBlock::Color::Black, TUIBlock::Color::White)))}};
    }
    bool OnEvent(TUIEventType event) override {
        if (event == TUIEventType::KeyEnter && focused) {
            if (onClick) {
                onClick();
            }
            return true;
        }
        return false;
    }

private:
    bool focused = false;
    std::string label;
    std::function<void()> onClick;
};