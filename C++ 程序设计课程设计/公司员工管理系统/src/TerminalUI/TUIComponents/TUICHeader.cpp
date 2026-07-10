#include "TUICHeader.hpp"
#include "TUIBlock/TUIBlock.hpp"
#include "TUIDefinitions.hpp"

std::vector<std::vector<TUIBlock>> TUICHeader::Render(int width) {
    std::vector<TUIBlock> pageNameZone{};
    for (int i = 0; i < pageNameBlocks.size(); i++) {
        if (i == pageIndex) {
            pageNameZone = pageNameZone + pageSelectedBlocks[i];
        } else {
            pageNameZone = pageNameZone + pageNameBlocks[i];
        }
        pageNameZone.push_back(TUIBlock(" "));
    }
    int padding = width -
                  TUIBlock::width(pageNameZone) -
                  progTitle.width() -
                  verTitle.width() - 5;
    std::vector<TUIBlock> line2 =
        TUIBlock(" ") +
        progTitle +
        TUIBlock("  ") +
        verTitle +
        TUIBlock(" ", padding) +
        pageNameZone +
        TUIBlock(" ");
    ;
    return std::vector<std::vector<TUIBlock>>{
        {TUIPredefinedBlocks::hDoubleLine(width)},
        line2,
        {TUIPredefinedBlocks::hLine(width)}};
}
