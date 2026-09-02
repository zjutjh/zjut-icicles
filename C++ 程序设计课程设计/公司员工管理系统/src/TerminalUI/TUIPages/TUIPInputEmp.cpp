#include "TUIPInputEmp.hpp"
/// @brief 构造输入页面，绑定目标员工引用并初始化所有字段输入控件
/// @param emp 目标员工对象的引用
/// @param submitCallback 提交成功时的回调
/// @param title 页面标题
TUIPInputEmp::TUIPInputEmp(Employee &emp, std::function<void()> submitCallback, std::string title)
    : emp(emp), cbInputSubmit(submitCallback), title(title) {
    // 定义标题栏的快捷操作
    std::vector<TUIPageOperation> operations;
    operations.push_back({"▲ UP", -1, TUIEventType::KeyUp, nullptr});
    operations.push_back({"▼ DOWN", -1, TUIEventType::KeyDown, nullptr});
    operations.push_back({"ESC", -1, TUIEventType::KeyEsc, nullptr});
    titleComponents = new TUICTitle(this->title, operations);
    // 提交按钮，触发外部回调
    submitButton = new TUICButton("确认", [this]() {if (cbInputSubmit) cbInputSubmit(); });
    int pw = 24; // prompt width
    TUICInput *input;
    // 一系列字段输入控件，验证器用于即时校验输入合法性
    input = new TUICInput("请输入员工编号: ", emp.id, pw, [this](std::string input) { return validateLength(input, 24); });
    inputComponents.push_back(std::ref(*input));
    input = new TUICInput("请输入员工姓名: ", emp.name, pw, [this](std::string input) { return validateLength(input, 24); });
    inputComponents.push_back(std::ref(*input));
    input = new TUICInput("请输入员工受教育程度: ", eduLvlBuf, static_cast<int>(emp.eduLevel) - 1, pw, [this](std::string input) { return validateEduLevel(input); }, eduLevelChoices, false);
    inputComponents.push_back(std::ref(*input));
    // 生日字段使用字符串缓冲区进行解析与校验
    birthdayBuf = emp.birthday.toString("y-m-d");
    input = new TUICInput("请输入员工生日 (y-m-d): ", birthdayBuf, pw, [this](std::string input) { return validateBirthday(input); });
    inputComponents.push_back(std::ref(*input));
    input = new TUICInput("请输入员工民族: ", emp.ethnicity, pw, [this](std::string input) { return validateLength(input, 24); });
    inputComponents.push_back(std::ref(*input));
    input = new TUICInput("请输入员工专业: ", emp.major, pw, [this](std::string input) { return validateLength(input, 24); });
    inputComponents.push_back(std::ref(*input));
    input = new TUICInput("请输入员工职位: ", emp.jobTitle, pw, [this](std::string input) { return validateLength(input, 24); });
    inputComponents.push_back(std::ref(*input));
    input = new TUICInput("请输入员工部门: ", emp.department, pw, [this](std::string input) { return validateLength(input, 24); });
    inputComponents.push_back(std::ref(*input));
    input = new TUICInput("请输入员工岗位: ", emp.positionTitle, pw, [this](std::string input) { return validateLength(input, 24); });
    inputComponents.push_back(std::ref(*input));
    // 初始化完成状态并将焦点置于当前索引的输入控件
    completionCheck();
    inputComponents[currentFocusIdx].get().Focus();
}

std::vector<std::reference_wrapper<TUICBase>> TUIPInputEmp::getComponents() {
    std::vector<std::reference_wrapper<TUICBase>> components;
    components.push_back(std::ref(*titleComponents));
    for (auto &comp : inputComponents) {
        components.push_back(std::ref(static_cast<TUICBase &>(comp.get())));
    }
    components.push_back(std::ref(*spacer));
    if (isComplete)
        components.push_back(std::ref(*submitButton));
    return components;
}

bool TUIPInputEmp::OnEvent(TUIEventType event) {
    switch (event) {
    case TUIEventType::KeyDown:
        return rollInputFocus(true);
    case TUIEventType::KeyUp:
        return rollInputFocus(false);
    case TUIEventType::KeyEsc:
        if (inputComponents[std::min(currentFocusIdx, static_cast<int>(inputComponents.size() - 1))].get().OnEvent(event)) {
            return true;
        } else if (completionCheck(), isComplete) {
            cbInputSubmit();
            return true;
        }
        return false;
    case TUIEventType::KeyEnter:
        completionCheck();
        if (submitButton->OnEvent(event)) {
            return true;
        }
        if (inputComponents[std::min(currentFocusIdx, static_cast<int>(inputComponents.size() - 1))].get().isInputting()) {
            rollInputFocus(true);
        }
        return inputComponents[std::min(currentFocusIdx, static_cast<int>(inputComponents.size() - 1))].get().OnEvent(event);
    default:
        return inputComponents[std::min(currentFocusIdx, static_cast<int>(inputComponents.size() - 1))].get().OnEvent(event);
    }
    return false;
}

/// @brief 在输入控件之间循环切换焦点
/// @param forward 为 true 则向下切换，否则向上切换
/// @return 切换是否成功
bool TUIPInputEmp::rollInputFocus(bool forward) {
    if (inputComponents.empty()) {
        return false;
    }
    inputComponents[std::min(currentFocusIdx, static_cast<int>(inputComponents.size() - 1))].get().Unfocus();
    int maxItemCount = (inputComponents.size() + isComplete);
    if (forward)
        currentFocusIdx = (currentFocusIdx + 1) % maxItemCount;
    else
        currentFocusIdx = (currentFocusIdx - 1 + maxItemCount) % maxItemCount;
    if (currentFocusIdx >= inputComponents.size()) {
        submitButton->Focus();
    } else {
        submitButton->Unfocus();
        inputComponents[currentFocusIdx].get().Focus();
    }
    return true;
}

/// @brief 验证文本长度与空值要求
/// @param input 用户输入字符串
/// @param maxLength 最大显示宽度限制
/// @return 空字符串表示校验通过，否则返回错误提示
std::string TUIPInputEmp::validateLength(std::string input, int maxLength) {
    if (input.empty()) {
        return "请输入内容";
    }
    if (TUIBlock::width(input) > maxLength) {
        return "输入长度不能超过" + std::to_string(maxLength) + "个字符";
    }
    return "";
}

/// @brief 校验并解析受教育程度，匹配成功则写回 emp.eduLevel
/// @param input 教育程度候选项字符串
/// @return 空字符串表示校验通过，否则返回错误提示
std::string TUIPInputEmp::validateEduLevel(std::string input) {
    for (int i = 0; i < eduLevelChoices.size(); i++) {
        if (input == eduLevelChoices[i]) {
            emp.eduLevel = static_cast<EducationalLevel>(i + 1);
            return "";
        }
    }
    eduLvlBuf = "";
    return "请输入有效的受教育程度";
}

/// @brief 验证生日字符串格式并解析到 emp.birthday
/// @param input 日期字符串，期望格式 y-m-d
/// @return 空字符串表示校验通过，否则返回错误提示
std::string TUIPInputEmp::validateBirthday(std::string input) {
    Date date;
    if (!date.fromString(input, "y-m-d")) {
        birthdayBuf = "";
        return "格式不正确，输入例：1990-01-01";
    }
    emp.birthday = date;
    return "";
}

/// @brief 检查所有必填字段是否已填写完整，并更新 isComplete 标志
void TUIPInputEmp::completionCheck() {
    isComplete = !(birthdayBuf.empty() ||
                   eduLvlBuf.empty() ||
                   emp.id.empty() ||
                   emp.name.empty() ||
                   emp.ethnicity.empty() ||
                   emp.major.empty() ||
                   emp.jobTitle.empty() ||
                   emp.department.empty() ||
                   emp.positionTitle.empty());
}
