#pragma once
#include "TUIBlock/TUIBlock.hpp"
#include "TUIComponents/TUICBase.hpp"
#include "TUIEvent.hpp"
#include <vector>
class TUICKVInfo : public TUICBase {
public:
    TUICKVInfo(const std::string &key, const std::string &value, int keyWidth = 0)
        : key(key), value(value), keyWidth(keyWidth) {};
    std::vector<std::vector<TUIBlock>> Render(int width) override {
        TUIBlock keyBlock(key, TUIBlock::Color::Green, TUIBlock::Color::Default, TUIBlock::Flag::Bold);
        int keyPadding = std::max(0, keyWidth - keyBlock.width());
        TUIBlock paddingBlock(std::string(keyPadding, ' '));
        TUIBlock valueBlock(value);
        return {{TUIPredefinedBlocks::space, keyBlock, paddingBlock, valueBlock}};
    };

private:
    std::string key;
    std::string value;
    int keyWidth;
};