#pragma once
#include "../Model/Employee.hpp"
#include "TUIComponents/TUICButton.hpp"
#include "TUIComponents/TUICInput.hpp"
#include "TUIComponents/TUICTitle.hpp"
#include "TUIPBase.hpp"
#include <functional>
class TUIPInputEmp : public TUIPBase {
public:
    TUIPInputEmp(Employee &emp, std::function<void()> submitCallback, std::string title = "Insert Employee");
    std::vector<std::reference_wrapper<TUICBase>> getComponents() override;
    bool OnEvent(TUIEventType event);

private:
    Employee &emp;
    std::string title;
    TUICTitle *titleComponents;
    TUICButton *submitButton;
    TUICBase *spacer = new TUICBase();
    std::vector<std::reference_wrapper<TUICInput>> inputComponents;
    std::function<void()> cbInputSubmit;
    int currentFocusIdx = 0;
    bool isComplete = false;
    bool rollInputFocus(bool forward);
    std::string validateLength(std::string input, int maxLength);
    const std::vector<std::string> eduLevelChoices = {"无", "小学", "初中", "高中", "中专", "大专", "本科", "硕士", "博士", "博士后"};
    std::string eduLvlBuf{};
    std::string birthdayBuf{};
    std::string validateEduLevel(std::string input);
    std::string validateBirthday(std::string input);
    void completionCheck();
};