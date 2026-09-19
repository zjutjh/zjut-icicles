#pragma once
#include "../Model/NamedPairs.hpp"
#include "TUIComponents/TUICInput.hpp"
#include "TUIComponents/TUICKVInfo.hpp"
#include "TUIComponents/TUICTitle.hpp"
#include "TUIPBase.hpp"

class TUIPStats : public TUIPBase {
public:
    TUIPStats(std::function<std::vector<NamedPairs>()> dataSource);
    std::vector<std::reference_wrapper<TUICBase>> getComponents() override;
    bool OnEvent(TUIEventType event) override { return selectComponents->OnEvent(event); }

private:
    std::string title = "Statistics";
    const std::vector<TUIPageOperation> operations;

    TUICTitle *titleComponents;
    TUICInput *selectComponents;
    TUICBase *blankComponent;

    std::string selectedStat;
    std::vector<TUICKVInfo *> statInfoComponents;
    std::function<std::vector<NamedPairs>()> statsDataSource;
    std::vector<std::string> getStatChoices();
};