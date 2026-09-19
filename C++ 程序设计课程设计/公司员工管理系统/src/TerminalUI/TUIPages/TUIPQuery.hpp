#pragma once
#include "TUIComponents/TUICInput.hpp"
#include "TUIComponents/TUICTable.hpp"
#include "TUIComponents/TUICTitle.hpp"
#include "TUIPBase.hpp"
#include <functional>
class TUIPQuery : public TUIPBase {
public:
    TUIPQuery(std::vector<TUICTableHeader> &headers,
              std::function<std::vector<std::vector<std::string>>()> source,
              const std::vector<TUIPageOperation> &operations);
    std::vector<std::reference_wrapper<TUICBase>> getComponents() override;
    bool OnEvent(TUIEventType event) override;
    int getSelectedIdx();
    void setNormalMode();
    void setSearchMode();
    void setSortMode();

private:
    enum class Mode { Normal,
                      Search,
                      Sort } mode = Mode::Normal;

    TUICTitle *titleComponent;
    TUICTable *tableComponent;
    TUICTable *tempTable;
    TUICInput *searchInput;
    TUICInput *sortSelect;

    std::vector<TUICTableHeader> &dataHeaders;
    std::function<std::vector<std::vector<std::string>>()> dataSource;

    std::string title = "Query";
    const std::vector<TUIPageOperation> operations;
    bool inputting = false;
    Mode currentMode = Mode::Normal;
    std::string searchKeyword;
    std::string sortKey;
    std::vector<int> tempDataMapping;
    std::vector<std::vector<std::string>> getFilteredData();
    std::vector<std::vector<std::string>> getSortedData();
    std::vector<std::string> getHeaderNames();
    std::string onInputSubmit();
    void refreshTitle();
};

inline int TUIPQuery::getSelectedIdx() {
    int selected = currentMode == Mode::Normal ? tableComponent->getSelectedIdx() : tempTable->getSelectedIdx();
    if (currentMode != Mode::Normal) {
        return tempDataMapping[selected];
    } else {
        return selected;
    }
}

inline void TUIPQuery::setNormalMode() {
    inputting = false;
    currentMode = Mode::Normal;
    searchInput->Unfocus();
}

inline void TUIPQuery::setSearchMode() {
    inputting = true;
    currentMode = Mode::Search;
    searchInput->Focus();
    searchInput->OnEvent(TUIEventType::KeyEnter); // 触发一次输入事件以显示输入框
}

inline void TUIPQuery::setSortMode() {
    inputting = true;
    currentMode = Mode::Sort;
    sortSelect->Focus();
}

inline std::string TUIPQuery::onInputSubmit() {
    searchInput->Unfocus(false);
    sortSelect->Unfocus(false);
    inputting = false;
    return "";
}

inline std::vector<std::string> TUIPQuery::getHeaderNames() {
    std::vector<std::string> names;
    for (const auto &header : dataHeaders) {
        names.push_back(header.name);
    }
    return names;
}

inline void TUIPQuery::refreshTitle() {
    int totalItems = currentMode == Mode::Normal ? tableComponent->getItemCount() : tempTable->getItemCount();
    int currentPage = currentMode == Mode::Normal ? tableComponent->getCurrentPage() + 1 : tempTable->getCurrentPage() + 1;
    int pageSize = currentMode == Mode::Normal ? tableComponent->getPageSize() : tempTable->getPageSize();
    int totalPages = (totalItems + pageSize - 1) / pageSize;
    title = "Page " + std::to_string(currentPage) + "/" + std::to_string(totalPages) + " - Total = " + std::to_string(totalItems);
}