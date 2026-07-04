#include "TUICInput.hpp"

TUICInput::TUICInput(
    std::string prompt,
    std::string &inputBuffer,
    int promptWidth,
    std::function<std::string(std::string)> validator,
    std::vector<std::string> choices,
    bool enableCustomInput)
    : prompt(prompt),
      inputBuffer(inputBuffer),
      promptWidth(promptWidth),
      validator(validator),
      choices(choices),
      enableCustomInput(enableCustomInput) {
    if (!choices.empty()) {
        currentChoiceIdx = 0;
        inputBuffer = choices[0];
    }
}

TUICInput::TUICInput(
    std::string prompt,
    std::string &inputBuffer,
    int defaultChoiceIdx,
    int promptWidth,
    std::function<std::string(std::string)> validator,
    std::vector<std::string> choices,
    bool enableCustomInput)
    : prompt(prompt),
      inputBuffer(inputBuffer),
      promptWidth(promptWidth),
      validator(validator),
      choices(choices),
      enableCustomInput(enableCustomInput) {
    if (!choices.empty()) {
        currentChoiceIdx = std::max(0, std::min(defaultChoiceIdx, static_cast<int>(choices.size() - 1)));
        inputBuffer = choices[currentChoiceIdx];
    }
}

std::vector<std::vector<TUIBlock>> TUICInput::Render(int width) {
    TUIBlock::Color promptColor = focused ? TUIBlock::Color::Green : TUIBlock::Color::Default;
    TUIBlock promptBlock(prompt, promptColor, TUIBlock::Color::Default, TUIBlock::Flag::Bold);
    int promptPadding = std::max(0, promptWidth - promptBlock.width());
    TUIBlock paddingBlock(std::string(promptPadding, ' '));

    std::vector<TUIBlock> inputZone{};

    if (!choices.empty()) {
        inputZone.push_back(TUIBlock("◄", TUIBlock::Color::Default, TUIBlock::Color::Default, TUIBlock::Flag::Invert));
        inputZone.push_back(TUIBlock(" "));
    }
    inputZone.push_back(TUIBlock(inputBuffer, TUIBlock::Color::Yellow, TUIBlock::Color::Default, TUIBlock::Flag::Underline));
    if (inputting) {
        inputZone.push_back(TUIBlock(true));
    }
    if (!choices.empty()) {
        inputZone.push_back(TUIBlock(" "));
        inputZone.push_back(TUIBlock("►", TUIBlock::Color::Default, TUIBlock::Color::Default, TUIBlock::Flag::Invert));
    }

    if (!errPrompt.empty()) {
        inputZone.clear();
        inputZone.push_back(TUIBlock(errPrompt, TUIBlock::Color::Red, TUIBlock::Color::Default));
    }

    return {{TUIPredefinedBlocks::space + promptBlock + paddingBlock + inputZone}};
}

bool TUICInput::OnEvent(TUIEventType event) {
    if (charRemainBytes <= 0) {
        if (event == TUIEventType::KeyEnter) {
            if (enableCustomInput)
                inputting = !inputting;
            if (inputting && enableCustomInput) {
                charRemainBytes = 0;
                errPrompt = "";
            } else
                validate();
            return true;
        }
        if (event == TUIEventType::KeyEsc) {
            if (inputting) {
                inputting = false;
                inputBuffer = "";
                charRemainBytes = 0;
                return true;
            }
        }
        if (event == TUIEventType::KeyDelete) {
            if (inputBuffer.empty()) {
                return false;
            }
            TUIBlock::delLastChar(inputBuffer);
            return true;
        }
    }
    if (inputting) {
        if (!charRemainBytes) {
            if (event == TUIEventType::KeyRight) {
                return rollChoice(true);
            } else if (event == TUIEventType::KeyLeft) {
                return rollChoice(false);
            } else if (event == TUIEventType::KeyUp || event == TUIEventType::KeyDown) {
                return false; // reserved
            }
        }
        if (charRemainBytes <= 0) {
            charRemainBytes = TUIBlock::getCharBytes(static_cast<unsigned char>(event));
        }
        inputBuffer += static_cast<char>(event);
        charRemainBytes--;
        return true;
    } else {
        switch (event) {
        case TUIEventType::KeyRight:
            return rollChoice(true);
        case TUIEventType::KeyLeft:
            return rollChoice(false);
        default:
            return false;
        }
    }
    return false;
}

bool TUICInput::rollChoice(bool forward) {
    if (choices.empty()) {
        return false;
    }
    if (forward)
        currentChoiceIdx = (currentChoiceIdx + 1) % choices.size();
    else
        currentChoiceIdx = (currentChoiceIdx - 1 + choices.size()) % choices.size();
    inputBuffer = choices[currentChoiceIdx];
    return true;
}

void TUICInput::validate() {
    if (validator) {
        errPrompt = validator(inputBuffer);
        if (!errPrompt.empty()) {
            inputBuffer = "";
        }
    }
}
