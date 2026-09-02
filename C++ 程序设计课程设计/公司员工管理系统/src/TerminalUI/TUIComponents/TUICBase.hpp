#pragma once
#include "TUIBlock/TUIBlock.hpp"
#include "TUIEvent.hpp"
#include <vector>
class TUICBase {
public:
    virtual std::vector<std::vector<TUIBlock>> Render(int width) { return {{}}; };
    virtual bool OnEvent(TUIEventType event) { return false; };
    virtual ~TUICBase() = default;

protected:
};