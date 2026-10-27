#pragma once
#include "TUICBase.hpp"
class TUICHeader : public TUICBase {
public:
    enum PageIndex {
        PageQuery = 0,
        PageStats = 1
    };
    TUICHeader(int &pageIndex) : pageIndex(pageIndex) {};
    std::vector<std::vector<TUIBlock>> Render(int width) override;

private:
    TUIBlock progTitle = TUIBlock("CPCPEMS", TUIBlock::Color::Purple, TUIBlock::Color::Default, TUIBlock::Flag::Bold);
    TUIBlock verTitle = TUIBlock("v1.2", TUIBlock::Color::Aqua, TUIBlock::Color::Default, TUIBlock::Flag::Bold);
    std::vector<std::vector<TUIBlock>> pageNameBlocks{
        {TUIBlock("Q", TUIBlock::Color::Default, TUIBlock::Color::Default, TUIBlock::Flag::Underline), TUIBlock("UERY ")},
        {TUIBlock("S", TUIBlock::Color::Default, TUIBlock::Color::Default, TUIBlock::Flag::Underline), TUIBlock("TATS ")},
        {TUIBlock("E", TUIBlock::Color::Red, TUIBlock::Color::Default, TUIBlock::Flag::Underline | TUIBlock::Flag::Bold),
         TUIBlock("xit ", TUIBlock::Color::Red, TUIBlock::Color::Default, TUIBlock::Flag::Bold)}};

    std::vector<std::vector<TUIBlock>> pageSelectedBlocks{
        {TUIBlock("QUERY ", TUIBlock::Color::Yellow, TUIBlock::Color::Default, TUIBlock::Flag::Bold)},
        {TUIBlock("STATS ", TUIBlock::Color::Yellow, TUIBlock::Color::Default, TUIBlock::Flag::Bold)}};
    int &pageIndex;
};