#include "TUICTitle.hpp"

std::vector<std::vector<TUIBlock>> TUICTitle::Render(int width) {
    // 标题使用高亮样式，作为页面顶部主视觉
    TUIBlock titleBlock(titleText, TUIBlock::Color::Aqua, TUIBlock::Color::Default, TUIBlock::Flag::Bold);
    int usedWidth = titleBlock.width();
    std::vector<TUIBlock> opZone;

    // 将每个操作名按是否包含快捷键提示拆分渲染，方便把关键字单独高亮
    for (int i = 0; i < operations.size(); i++) {
        if (operations[i].keyHintIdx >= 0) {
            // 快捷键提示位于操作名中间时，将左右文本与按键分开处理
            TUIBlock opHintLeft(operations[i].name.substr(0, operations[i].keyHintIdx), TUIBlock::Color::Default, TUIBlock::Color::Default);
            TUIBlock opHintKey(operations[i].name.substr(operations[i].keyHintIdx, 1), TUIBlock::Color::Yellow, TUIBlock::Color::Default, TUIBlock::Flag::Underline);
            TUIBlock opHintRight(operations[i].name.substr(operations[i].keyHintIdx + 1), TUIBlock::Color::Default, TUIBlock::Color::Default);
            usedWidth += opHintLeft.width() + opHintKey.width() + opHintRight.width();
            opZone.push_back(opHintLeft);
            opZone.push_back(opHintKey);
            opZone.push_back(opHintRight);
        } else {
            TUIBlock opHint(operations[i].name, TUIBlock::Color::Yellow, TUIBlock::Color::Default, TUIBlock::Flag::Underline);
            usedWidth += opHint.width();
            opZone.push_back(opHint);
        }
        // 每个操作后追加空格，保持提示之间有固定间距
        opZone.push_back(TUIPredefinedBlocks::space);
        usedWidth += 1;
    }
    // 根据剩余宽度填充标题与操作区之间的空白，保证右侧提示尽量贴边
    int padding = width - usedWidth - 1;
    TUIBlock paddingBlock = TUIBlock(std::string(padding, ' '), TUIBlock::Color::Default, TUIBlock::Color::Default);
    std::vector<TUIBlock> line = {TUIPredefinedBlocks::space, titleBlock, paddingBlock};
    return std::vector<std::vector<TUIBlock>>({line + opZone, {TUIPredefinedBlocks::hLine(width)}});
}