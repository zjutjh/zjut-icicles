#include "TUIController.hpp"
#include <unordered_map>

/// @brief 初始化 TUI，各页面组件与渲染器在此构建并启动渲染循环
/// @details 会注册 query 页面特有的操作，构建表头并创建各页面实例
void TUIController::setupTUI() {
    pageQueryOperations.push_back({"search", 0, static_cast<TUIEventType>('s'), [this]() { pageQuery->setSearchMode(); }});
    pageQueryOperations.push_back({"sort", 1, static_cast<TUIEventType>('o'), [this]() { pageQuery->setSortMode(); }});
    pageQueryOperations.push_back({"", -1, TUIEventType::KeyEsc, [this]() { pageQuery->setNormalMode(); }});

    buildQueryTableHeader();
    cHeader = new TUICHeader(pageIndex);
    pageQuery = new TUIPQuery(
        tableHeader,
        [this]() -> std::vector<std::vector<std::string>> { return this->convEmpDataToTable(); },
        pageQueryOperations);
    pageStats = new TUIPStats([this]() -> std::vector<NamedPairs> { return this->genStatsData(); });
    uiRenderer = new TUIRenderer(
        [this](TUIEventType key) { this->onEvent(key); },
        [this]() { return this->onUpdate(); });
    switchPage(pageQuery);
    uiRenderer->renderLoop();
}

/// @brief 弹出员工输入页面并将焦点切换到该页面
/// @param title 页面显示的标题
/// @param emp 要编辑或创建的员工实体引用
void TUIController::callEmpInput(std::string title, Employee &emp) {
    pageIndex = 0;
    if (pageInputEmp)
        delete pageInputEmp;
    pageInputEmp = new TUIPInputEmp(emp, [this]() { this->onInputSubmit(); }, title);
    switchPage(pageInputEmp);
}

void TUIController::onEvent(TUIEventType key) {
    switch (static_cast<int>(key)) {
    case 'S':
        pageIndex = 1;
        switchPage(pageStats);
        break;
    case 'Q':
        pageIndex = 0;
        switchPage(pageQuery);
        break;
    case 'R':
        cbRandomEmpInfo();
        break;
    case 'E':
        if (cbTrySaveData && cbTrySaveData()) {
            exit(0);
        }
        break;
    default:
        if (!currentPage->OnEvent(key) && key == TUIEventType::KeyEsc) {
            pageIndex = 0;
            switchPage(pageQuery);
        }
        break;
    }
}

/// @brief onUpdate 回调，组合页头与当前页面的组件供渲染器使用
/// @return 当前渲染应展示的组件列表，第一项为 header
std::vector<std::reference_wrapper<TUICBase>> TUIController::onUpdate() {
    std::vector<std::reference_wrapper<TUICBase>> pageComps = currentPage->getComponents();
    std::vector<std::reference_wrapper<TUICBase>> components = {std::ref(*cHeader)};
    components.insert(components.end(), pageComps.begin(), pageComps.end());
    return components;
}

/// @brief 处理员工输入页面的提交动作，释放页面资源并回到查询页
void TUIController::onInputSubmit() {
    cbTrySaveData();
    delete pageInputEmp;
    pageInputEmp = nullptr;
    pageIndex = 0;
    switchPage(pageQuery);
}

/// @brief 根据当前员工数据源生成用于统计页的多个 `NamedPairs` 分组
/// @details 教育水平使用预定义顺序统计，其余字段使用通用分组构建器
std::vector<NamedPairs> TUIController::genStatsData() {
    std::vector<NamedPairs> stats;
    const std::vector<Employee> employees = empDatasource ? empDatasource() : std::vector<Employee>();
    // 通用分组构建器：根据传入的字符串向量统计每个不同值的出现次数
    auto buildCountGroupFromValues = [](const std::string &groupName, const std::vector<std::string> &values) {
        NamedPairs group;
        group.name = groupName;
        std::unordered_map<std::string, std::size_t> indexMap;
        group.keys.reserve(values.size());
        group.values.reserve(values.size());
        for (const auto &v : values) {
            auto it = indexMap.find(v);
            if (it == indexMap.end()) {
                indexMap.emplace(v, group.keys.size());
                group.keys.push_back(v);
                group.values.push_back("1");
            } else {
                std::size_t idx = it->second;
                group.values[idx] = std::to_string(std::stoi(group.values[idx]) + 1);
            }
        }
        return group;
    };

    // 辅助函数：从 employees 中收集某个字段到字符串向量
    auto collectField = [&](auto extractor) {
        std::vector<std::string> vals;
        vals.reserve(employees.size());
        for (const auto &e : employees)
            vals.push_back(extractor(e));
        return vals;
    };

    // 教育水平使用预定义映射，先创建键并初始化计数为 0，然后遍历累加
    NamedPairs educationGroup;
    educationGroup.name = "受教育水平";
    educationGroup.keys = EducationalLevelStrCN;
    educationGroup.values.assign(educationGroup.keys.size(), "0");
    for (const auto &employee : employees) {
        const std::size_t idx = static_cast<std::size_t>(employee.eduLevel);
        if (idx < educationGroup.values.size()) {
            educationGroup.values[idx] = std::to_string(std::stoi(educationGroup.values[idx]) + 1);
        }
    }

    stats.push_back(std::move(educationGroup));

    // 其余字段通过 collectField + buildCountGroupFromValues 生成分组
    stats.push_back(buildCountGroupFromValues("民族", collectField([](const Employee &e) { return e.ethnicity; })));
    stats.push_back(buildCountGroupFromValues("专业", collectField([](const Employee &e) { return e.major; })));
    stats.push_back(buildCountGroupFromValues("职称", collectField([](const Employee &e) { return e.jobTitle; })));
    stats.push_back(buildCountGroupFromValues("部门", collectField([](const Employee &e) { return e.department; })));
    stats.push_back(buildCountGroupFromValues("职务", collectField([](const Employee &e) { return e.positionTitle; })));

    return stats;
}
