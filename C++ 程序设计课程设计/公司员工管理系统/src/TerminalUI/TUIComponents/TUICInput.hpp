#pragma once
#include "TUICBase.hpp"
#include <functional>
#include <vector>
class TUICInput : public TUICBase {
public:
    TUICInput(std::string prompt,
              std::string &inputBuffer,
              int promptWidth = 0,
              std::function<std::string(std::string)> validator = nullptr,
              std::vector<std ::string> choices = {},
              bool enableCustomInput = true);
    TUICInput(std::string prompt,
              std::string &inputBuffer,
              int defaultChoiceIdx,
              int promptWidth = 0,
              std::function<std::string(std::string)> validator = nullptr,
              std::vector<std ::string> choices = {},
              bool enableCustomInput = true);
    std::vector<std::vector<TUIBlock>> Render(int width) override;
    void Focus() { focused = true; }
    bool isInputting() const { return inputting; }
    void Unfocus(bool needValidate = true);
    bool OnEvent(TUIEventType event) override;

private:
    bool enableCustomInput;
    std::string prompt;
    std::string errPrompt;
    int promptWidth = 0;
    std::function<std::string(std::string)> validator{};
    std::vector<std::string> choices{{}};
    int currentChoiceIdx = 0;
    bool inputting = false;
    bool focused = false;
    int charRemainBytes = 0;
    std::string &inputBuffer;
    bool rollChoice(bool forward);
    void validate();
};

inline void TUICInput::Unfocus(bool needValidate) {
    focused = false;
    inputting = false;
    if (needValidate) {
        validate();
    }
}