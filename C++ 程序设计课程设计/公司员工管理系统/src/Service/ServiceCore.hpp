#pragma once
#include "../Model/Employee.hpp"
#include "../Model/LinkedList.hpp"
#include "../Repository/Database.hpp"
#include "../TerminalUI/TUIBlock/TUIBlock.hpp"
#include "RandDataGen.hpp"
#include "TUIController.hpp"
class ServiceCore {
public:
    ServiceCore();
    void initServiceCore();

private:
    LinkedList<Employee> empData{};
    Employee newEmp{};
    int empCreatingState = 0;

    Database db{};
    TUIController uiCtrl{};
    void useRandomEmployeeInfo();
    void deleteSelectEmp();
    void insertBeforeSelectEmp();
    void editSelectEmp();
    bool trySaveData();
};
