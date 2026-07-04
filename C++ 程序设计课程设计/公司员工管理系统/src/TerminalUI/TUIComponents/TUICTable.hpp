#include "TUICBase.hpp"
#include "TUIDefinitions.hpp"
#include <climits>
#include <functional>

struct TUICTableHeader {
    std::string name;
    int minWidth;
    int preferredWidth;
    bool operator==(const TUICTableHeader &other) const {
        return name == other.name && minWidth == other.minWidth && preferredWidth == other.preferredWidth;
    }
    bool operator==(std::string other) const { return name == other; }
};

class TUICTable : public TUICBase {
public:
    TUICTable(std::vector<TUICTableHeader> &columnHeaders,
              std::function<std::vector<std::vector<std::string>>()> dataSource)
        : tableHeader(columnHeaders), dataSource(dataSource) {};
    std::vector<std::vector<TUIBlock>> Render(int width) override;
    bool OnEvent(TUIEventType event) override;
    int getSelectedIdx() { return pageOffset + selectedRow; }
    int getCurrentPage() const { return pageOffset / pageSize; }
    int getPageSize() const { return pageSize; }
    int getItemCount() const { return currentData.size(); }

private:
    int pageOffset = 0;
    int pageSize = 10;
    int selectedRow = 0;
    std::vector<TUICTableHeader> &tableHeader;
    std::vector<std::vector<std::string>> currentData;
    std::function<std::vector<std::vector<std::string>>()> dataSource;
    TUIBlock withHighlight(TUIBlock block, bool highlight);
    std::vector<int> decideColumnWidths(std::vector<TUICTableHeader> &tableHeader, int totalWidth);
    std::vector<std::string> fitDataline(const std::vector<std::string> &line, const std::vector<int> &columnWidths);
};

inline bool TUICTable::OnEvent(TUIEventType event) {
    currentData = dataSource();
    int totalRows = static_cast<int>(currentData.size());

    if (event == TUIEventType::KeyUp) {
        if (selectedRow > 0) {
            selectedRow--;
        } else if (pageOffset > 0) { // 翻到上一页
            pageOffset -= pageSize;
            selectedRow = pageSize - 1;
        }
        return true;
    } else if (event == TUIEventType::KeyDown) {
        if (selectedRow < std::min(pageSize - 1, totalRows - pageOffset - 1)) {
            selectedRow++;
        } else if (pageOffset + pageSize < totalRows) { // 翻到下一页
            pageOffset += pageSize;
            selectedRow = 0;
        }
        return true;
    } else if (event == TUIEventType::KeyLeft) { // 切换到上一页
        if (pageOffset > 0) {
            pageOffset -= pageSize;
            selectedRow = 0;
        }
        return true;
    } else if (event == TUIEventType::KeyRight) { // 切换到下一页
        if (pageOffset + pageSize < totalRows) {
            pageOffset += pageSize;
            selectedRow = 0;
        }
        return true;
    } else {
        return false;
    }
}

inline TUIBlock TUICTable::withHighlight(TUIBlock block, bool highlight) {
    if (highlight) {
        block.setStyleFlag(TUIBlock::Flag::Invert);
    }
    return block;
}