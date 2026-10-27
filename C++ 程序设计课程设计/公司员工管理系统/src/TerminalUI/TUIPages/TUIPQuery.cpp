#include "TUIPQuery.hpp"
#include <algorithm>
#include <cctype>
#include <sstream>

/// @brief 查询页构造函数，初始化标题、原始表格、临时表格以及检索/排序输入控件。
/// @param headers 表头定义，作为列名与排序候选项的来源。
/// @param source 页面数据源，返回当前时刻应展示的全部数据。
/// @param showOperations 标题栏需要展示的页面操作。
TUIPQuery::TUIPQuery(std::vector<TUICTableHeader> &headers,
                     std::function<std::vector<std::vector<std::string>>()> source,
                     const std::vector<TUIPageOperation> &showOperations)
    : dataHeaders(headers), dataSource(source), operations(showOperations) {
    // 先构造静态组件，再把临时表格的数据源绑定到当前模式对应的派生数据。
    titleComponent = new TUICTitle(title, operations);
    tableComponent = new TUICTable(dataHeaders, dataSource);
    tempTable = new TUICTable(dataHeaders, [this]() {
        if (currentMode == Mode::Search)
            return this->getFilteredData();
        if (currentMode == Mode::Sort)
            return this->getSortedData();
        return this->dataSource();
    });
    searchInput = new TUICInput("检索关键词: ", searchKeyword, 0, [this](std::string input) { return this->onInputSubmit(); });
    sortSelect = new TUICInput("排序依据: ", sortKey, 0, [this](std::string input) { return this->onInputSubmit(); }, getHeaderNames(), false);
    refreshTitle();
}

/// @brief 根据当前模式返回本页需要渲染的组件集合。
/// @return 按渲染顺序排列的组件引用列表。
std::vector<std::reference_wrapper<TUICBase>> TUIPQuery::getComponents() {
    // 每次取组件前都刷新标题，确保分页信息与当前数据视图同步。
    refreshTitle();
    if (currentMode == Mode::Search) {
        return std::vector<std::reference_wrapper<TUICBase>>{*searchInput, *titleComponent, *tempTable};
    } else if (currentMode == Mode ::Sort) {
        return std::vector<std::reference_wrapper<TUICBase>>{*sortSelect, *titleComponent, *tempTable};
    } else {
        return std::vector<std::reference_wrapper<TUICBase>>{*titleComponent, *tableComponent};
    }
}

bool TUIPQuery::OnEvent(TUIEventType event) {
    // 输入状态下优先处理输入框，避免导航事件误落到表格上。
    if (inputting) {
        if (currentMode == Mode::Search && searchInput->OnEvent(event))
            return true;
        if (currentMode == Mode::Sort && sortSelect->OnEvent(event))
            return true;
    }

    // 非普通模式时，表格展示的是派生结果，因此事件应作用到临时表格。
    if (currentMode != Mode::Normal) {
        if (tempTable->OnEvent(event))
            return true;
    } else {
        if (tableComponent->OnEvent(event))
            return true;
    }

    // 最后再检查页面级操作，保证表格操作的优先级高于全局触发键。
    for (int i = 0; i < operations.size(); i++) {
        if (event == operations[i].triggerEvent) {
            if (operations[i].action) {
                operations[i].action();
                return true;
            }
        }
    }
    return false;
}

/// @brief 生成搜索模式下的可视数据，并维护原始行号到结果行号的映射。
/// @return 按关键词匹配与权重排序后的结果集；无关键词时返回原始数据。
std::vector<std::vector<std::string>> TUIPQuery::getFilteredData() {
    std::vector<std::vector<std::string>> allData = dataSource();

    // 每次重建过滤结果时都清空映射，确保后续选中行能回写到原始数据索引。
    tempDataMapping.clear();

    // 没有关键字时直接返回原始数据，并构建一一对应的索引映射。
    if (searchKeyword.empty()) {
        tempDataMapping.reserve(allData.size());
        for (int i = 0; i < static_cast<int>(allData.size()); ++i) {
            tempDataMapping.push_back(i);
        }
        return allData;
    }

    // 按空白拆分关键词，统一转为小写后做包含匹配。
    std::istringstream iss(searchKeyword);
    std::vector<std::string> tokens;
    std::string tok;
    while (iss >> tok) {
        std::transform(tok.begin(), tok.end(), tok.begin(), [](unsigned char c) { return std::tolower(c); });
        if (!tok.empty())
            tokens.push_back(tok);
    }

    if (tokens.empty()) {
        tempDataMapping.reserve(allData.size());
        for (int i = 0; i < static_cast<int>(allData.size()); ++i)
            tempDataMapping.push_back(i);
        return allData;
    }

    // 统计每行与所有 token 的命中次数，作为搜索排序权重。
    struct Entry {
        int idx;
        int weight;
        const std::vector<std::string> *row;
    };
    std::vector<Entry> matches;
    matches.reserve(allData.size());

    for (int i = 0; i < static_cast<int>(allData.size()); ++i) {
        int weight = 0;
        for (const auto &cell : allData[i]) {
            std::string cellLower = cell;
            std::transform(cellLower.begin(), cellLower.end(), cellLower.begin(), [](unsigned char c) { return std::tolower(c); });
            for (const auto &t : tokens) {
                if (!t.empty() && cellLower.find(t) != std::string::npos) {
                    ++weight;
                }
            }
        }
        if (weight > 0) {
            matches.push_back({i, weight, &allData[i]});
        }
    }

    // 没有任何匹配时返回空结果，映射也保持为空。
    if (matches.empty())
        return std::vector<std::vector<std::string>>();

    // 按命中权重降序排序，权重相同时保留原始顺序。
    std::stable_sort(matches.begin(), matches.end(), [](const Entry &a, const Entry &b) {
        return a.weight > b.weight;
    });

    // 同步输出结果集与原始索引映射。
    std::vector<std::vector<std::string>> result;
    result.reserve(matches.size());
    tempDataMapping.reserve(matches.size());
    for (const auto &e : matches) {
        result.push_back(*e.row);
        tempDataMapping.push_back(e.idx);
    }

    return result;
}

/// @brief 生成排序模式下的可视数据，并维护原始行号到结果行号的映射。
/// @return 按指定列排序后的结果集；无有效列名时返回原始数据。
std::vector<std::vector<std::string>> TUIPQuery::getSortedData() {
    std::vector<std::vector<std::string>> allData = dataSource();

    // 重新生成排序结果时，先清空上一轮映射。
    tempDataMapping.clear();

    // 根据列名查找排序字段在表头中的位置。
    int sortIdx = -1;
    for (int i = 0; i < static_cast<int>(dataHeaders.size()); ++i) {
        if (dataHeaders[i].name == sortKey) {
            sortIdx = i;
            break;
        }
    }

    // 没有合法的排序列时，维持原始顺序并建立身份映射。
    if (sortIdx < 0) {
        tempDataMapping.reserve(allData.size());
        for (int i = 0; i < static_cast<int>(allData.size()); ++i) {
            tempDataMapping.push_back(i);
        }
        return allData;
    }

    // 把行号和行指针拆开，方便稳定排序后回填原始索引。
    struct Entry {
        int idx;
        const std::vector<std::string> *row;
    };

    std::vector<Entry> entries;
    entries.reserve(allData.size());
    for (int i = 0; i < static_cast<int>(allData.size()); ++i) {
        entries.push_back({i, &allData[i]});
    }

    // 按目标列字典序排序，稳定排序确保相同值的相对顺序不变。
    std::stable_sort(entries.begin(), entries.end(), [sortIdx](const Entry &a, const Entry &b) {
        return (*a.row)[sortIdx] < (*b.row)[sortIdx];
    });

    // 生成排序后的结果与原始索引映射，供选中行回溯使用。
    std::vector<std::vector<std::string>> result;
    result.reserve(entries.size());
    tempDataMapping.reserve(entries.size());
    for (const auto &entry : entries) {
        result.push_back(*entry.row);
        tempDataMapping.push_back(entry.idx);
    }

    return result;
}
