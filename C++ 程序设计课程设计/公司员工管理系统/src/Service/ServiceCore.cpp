#include "ServiceCore.hpp"

ServiceCore::ServiceCore() {
}

void ServiceCore::initServiceCore() {
    empData = db.loadEmployees();
    uiCtrl.bindEmployeeDatasource([this]() { return empData; });
    uiCtrl.bindFuncCallback([this]() { this->useRandomEmployeeInfo(); }, [this]() { return this->trySaveData(); });
    uiCtrl.bindQueryPageOperation({{"↵", -1, TUIEventType::KeyEnter, [this]() { this->editSelectEmp(); }},
                                   {"insert", 0, static_cast<TUIEventType>('i'), [this]() { this->insertBeforeSelectEmp(); }},
                                   {"delete", 0, static_cast<TUIEventType>('d'), [this]() { this->deleteSelectEmp(); }},
                                   {"edit", 0, static_cast<TUIEventType>('e'), [this]() { this->editSelectEmp(); }}});
    uiCtrl.bindStatsPageOperation({{"view", 0, static_cast<TUIEventType>('v'), nullptr}});
    uiCtrl.setupTUI();
}

void ServiceCore::useRandomEmployeeInfo() {
    RandomDataGenerator randGen;
    empData = randGen.generateRandomEmployees(200);
}

void ServiceCore::deleteSelectEmp() {
    if (empData.empty()) {
        return;
    }
    int idx = std::max(0, std::min(uiCtrl.getQuerySelectIdx(), static_cast<int>(empData.size() - 1)));
    empData.erase(empData.begin() + idx);
    trySaveData();
}

void ServiceCore::insertBeforeSelectEmp() {
    Employee newEmp{};
    int idx = std::max(0, std::min(uiCtrl.getQuerySelectIdx(), static_cast<int>(empData.size())));
    empData.insert(empData.begin() + idx, newEmp);
    uiCtrl.callEmpInput("Insert Employee", empData[idx]);
}

void ServiceCore::editSelectEmp() {
    if (empData.empty()) {
        return;
    }
    int idx = std::max(0, std::min(uiCtrl.getQuerySelectIdx(), static_cast<int>(empData.size() - 1)));
    uiCtrl.callEmpInput("Edit Employee", empData[idx]);
}

bool ServiceCore::trySaveData() {
    if (db.saveData(empData)) {
        std::cout << TUIBlock("Data saved successfully!", TUIBlock::Color::Green) << std::endl;
        return true;
    }
    std::cout << TUIBlock("Failed to save data!", TUIBlock::Color::Red) << std::endl;
    return false;
}
