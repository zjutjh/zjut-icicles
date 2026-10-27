#include "TUIPages/TUIPStats.hpp"
#include "TUIPStats.hpp"

/// @brief 统计页构造函数，绑定统计数据源并初始化标题、统计项选择框与占位组件
/// @param dataSource 接收当前统计数据集合的回调
TUIPStats::TUIPStats(std::function<std::vector<NamedPairs>()> dataSource)
    : statsDataSource(dataSource) {
    titleComponents = new TUICTitle(title, operations);
    selectComponents = new TUICInput("请选择统计项: ", selectedStat, 0, nullptr, getStatChoices(), false);
    blankComponent = new TUICBase();
}

std::vector<std::reference_wrapper<TUICBase>> TUIPStats::getComponents() {
    // 每次重建组件前，先清理上一轮动态生成的统计详情控件，避免重复渲染和内存泄漏
    for (auto *component : statInfoComponents) {
        delete component;
    }
    statInfoComponents.clear();

    // 顶部固定区域始终包含标题、选择框与占位行
    std::vector<std::reference_wrapper<TUICBase>> components;
    components.emplace_back(*titleComponents);
    components.emplace_back(*selectComponents);
    components.emplace_back(*blankComponent);

    // 读取当前统计数据，仅当选中的统计项命中时才展开明细
    const std::vector<NamedPairs> statsData = statsDataSource();
    for (const auto &stat : statsData) {
        if (stat.name != selectedStat) {
            continue;
        }
        // 键值对按最短长度配对，避免 keys 与 values 长度不一致时越界
        const std::size_t pairCount = std::min(stat.keys.size(), stat.values.size());
        // 为每个键值对创建一个显示组件，并加入到页面组件列表中
        for (std::size_t i = 0; i < pairCount; ++i) {
            auto *infoComponent = new TUICKVInfo(stat.keys[i], stat.values[i], 24);
            statInfoComponents.push_back(infoComponent);
            components.emplace_back(*infoComponent);
        }
        break;
    }
    return components;
}

/// @brief 生成统计项选择框的候选列表
/// @return 当前统计数据源中所有统计项名称
std::vector<std::string> TUIPStats::getStatChoices() {
    // 从数据源提取所有统计项名称
    std::vector<NamedPairs> statsData = statsDataSource();
    std::vector<std::string> choices;
    for (const auto &stat : statsData) {
        choices.push_back(stat.name);
    }
    return choices;
}
