#pragma once
#include "TUIComponents/TUICBase.hpp"
#include "TUIDefinitions.hpp"
#include <functional>
#include <vector>
class TUIPBase {
public:
    virtual std::vector<std::reference_wrapper<TUICBase>> getComponents() = 0;
    virtual bool OnEvent(TUIEventType event) { return false; };
    virtual ~TUIPBase() = default;
};