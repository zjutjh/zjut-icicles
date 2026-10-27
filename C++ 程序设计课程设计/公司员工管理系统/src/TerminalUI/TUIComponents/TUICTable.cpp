#include "TUICTable.hpp"

std::vector<std::vector<TUIBlock>> TUICTable::Render(int width) {
    // 每次渲染前都刷新数据源，保证表格内容与外部状态同步
    currentData = dataSource();
    std::vector<std::vector<TUIBlock>> result;
    // 先根据总宽度分配各列宽度，再逐行拼装渲染块
    std::vector<int> columnWidths = decideColumnWidths(tableHeader, width - tableHeader.size() - 1);
    std::vector<TUIBlock> headerLine;
    for (int h = 0; h < tableHeader.size(); h++) {
        // 表头使用竖线分隔，标题文字使用高亮颜色
        headerLine.push_back(TUIBlock(TUISpecialChar::v));
        TUIBlock textBlock = TUIBlock(tableHeader[h].name, TUIBlock::Color::Yellow);
        headerLine.push_back(textBlock);
        headerLine.push_back(TUIBlock(std::string(columnWidths[h] - textBlock.width(), ' ')));
    }
    headerLine.push_back(TUIBlock(TUISpecialChar::v));
    result.push_back(headerLine);

    // 仅渲染当前页范围内的数据，并按选中行做高亮
    for (int l = pageOffset; l < std::min(pageOffset + pageSize, static_cast<int>(currentData.size())); l++) {
        std::vector<std::string> fittedLine = fitDataline(currentData[l], columnWidths);
        std::vector<TUIBlock> line;
        bool needHighlight = (l - pageOffset == selectedRow);
        int lineCol = fittedLine.size();
        for (int i = 0; i < lineCol; i++) {
            line.push_back(withHighlight(TUIBlock(TUISpecialChar::v), needHighlight));
            TUIBlock textBlock = TUIBlock(fittedLine[i], TUIBlock::Color::Default);
            line.push_back(withHighlight(textBlock, needHighlight));
            line.push_back(withHighlight(TUIBlock(std::string(std::max(columnWidths[i] - textBlock.width(), 0), ' ')), needHighlight));
        }
        line.push_back(withHighlight(TUIBlock(TUISpecialChar::v), needHighlight));
        result.push_back(line);
    }

    // 底部补一条横线，形成完整表格边界
    result.push_back(std::vector<TUIBlock>{TUIPredefinedBlocks::hLine(width)});
    return result;
}

/// @brief 根据每列的最小宽度和期望宽度，为表格列分配实际宽度
/// @param tableHeader 表头定义
/// @param totalWidth 可分配的总宽度
/// @return 每列的最终宽度
std::vector<int> TUICTable::decideColumnWidths(std::vector<TUICTableHeader> &tableHeader, int totalWidth) {
    int usedWidth = 0;
    std::vector<int> result(tableHeader.size(), 0);
    // 先满足每列最小宽度，避免内容被压缩到不可读
    for (int i = 0; i < tableHeader.size(); i++) {
        usedWidth += tableHeader[i].minWidth;
        result[i] = tableHeader[i].minWidth;
    }

    // 如果最小宽度之和已经超出可用空间，则直接返回最小宽度方案
    if (usedWidth >= totalWidth) {
        return result;
    }

    // 剩余宽度优先分配给最需要扩展的列，尽量逼近期望宽度
    for (; usedWidth < totalWidth; usedWidth++) {
        int mostNeedIdx = 0;
        int mostNeedVal = INT_MIN;
        for (int i = 0; i < tableHeader.size(); i++) {
            int need = tableHeader[i].preferredWidth - result[i];
            if (need > mostNeedVal) {
                mostNeedVal = need;
                mostNeedIdx = i;
            }
        }
        result[mostNeedIdx]++;
    }
    return result;
}

/// @brief 将一行数据按列宽截断到可显示范围，并在被截断时追加省略号
/// @param line 原始数据行
/// @param columnWidths 每列宽度
/// @return 与列宽匹配的可显示字符串行
std::vector<std::string> TUICTable::fitDataline(const std::vector<std::string> &line, const std::vector<int> &columnWidths) {
    std::vector<std::string> result = line;
    for (int i = 0; i < line.size(); i++) {
        bool needFit = false;
        // 以宽度为准截断，保证多字节字符不会被切坏
        while (TUIBlock::width(result[i]) + 1 > columnWidths[i]) {
            std::vector<int> byteIdx = TUIBlock::charByteIdx(result[i]);
            result[i].resize(result[i].size() - byteIdx[byteIdx.size() - 1] - 1);
            needFit = true;
        }
        if (needFit) {
            // 被截断的单元格统一追加省略号提示用户内容未完全显示
            result[i] += std::string(TUISpecialChar::ellipsis);
        }
    }
    return result;
}
