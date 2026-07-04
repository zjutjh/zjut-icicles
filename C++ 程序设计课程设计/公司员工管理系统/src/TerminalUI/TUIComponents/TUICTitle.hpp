#pragma once
#include "TUICBase.hpp"
#include "TUIDefinitions.hpp"
#include <vector>
class TUICTitle : public TUICBase {
public:
    TUICTitle(const std::string &titleText, const std::vector<TUIPageOperation> &operations)
        : titleText(titleText), operations(operations) {}
    std::vector<std::vector<TUIBlock>> Render(int width) override;

private:
    const std::string &titleText;
    const std::vector<TUIPageOperation> operations;
};