#include "../Model/Employee.hpp"
#include "TUIComponents/TUICHeader.hpp"
#include "TUIPages/TUIPBase.hpp"
#include "TUIPages/TUIPInputEmp.hpp"
#include "TUIPages/TUIPQuery.hpp"
#include "TUIPages/TUIPStats.hpp"
#include "TUIRenderer/TUIRenderer.hpp"
#include <functional>

class TUIController {
public:
    void setupTUI();
    void bindEmployeeDatasource(std::function<std::vector<Employee>()> dataSource) { empDatasource = dataSource; };
    void bindFuncCallback(std::function<void()> cbRandomEmpInfo, std::function<bool()> cbTrySaveData) {
        this->cbRandomEmpInfo = cbRandomEmpInfo;
        this->cbTrySaveData = cbTrySaveData;
    };
    void bindQueryPageOperation(const std::vector<TUIPageOperation> &operation) { pageQueryOperations = operation; };
    void bindStatsPageOperation(const std::vector<TUIPageOperation> &operation) { pageStatsOperations = operation; };
    int getQuerySelectIdx() { return pageQuery->getSelectedIdx(); };
    void callEmpInput(std::string title, Employee &emp);

private:
    const std::vector<std::string> EducationalLevelStrCN = {
        "未知", "无", "小学", "初中", "高中", "中专", "大专", "本科", "硕士", "博士", "博士后"};

    TUIRenderer *uiRenderer;
    TUICHeader *cHeader;
    TUIPQuery *pageQuery;
    TUIPStats *pageStats;
    TUIPInputEmp *pageInputEmp;
    TUIPBase *currentPage;

    std::vector<TUICTableHeader> tableHeader;
    std::function<std::vector<Employee>()> empDatasource;

    std::vector<TUIPageOperation> pageQueryOperations{};
    std::vector<TUIPageOperation> pageStatsOperations{};
    std::function<void()> cbRandomEmpInfo;
    std::function<void()> cbInputSubmit;
    std::function<bool()> cbTrySaveData;

    int pageIndex = 0;
    void onInputSubmit();
    void buildQueryTableHeader();
    void onEvent(TUIEventType key);
    std::vector<NamedPairs> genStatsData();
    std::vector<std::reference_wrapper<TUICBase>> onUpdate();
    std::vector<std::vector<std::string>> convEmpDataToTable();
    void switchPage(TUIPBase *tarPage) { currentPage = tarPage; }
    std::string convEduLevelToCNStr(EducationalLevel level) { return EducationalLevelStrCN[static_cast<int>(level)]; };
};

inline void TUIController::buildQueryTableHeader() {
    tableHeader = {
        {"编号", 4, 4},
        {"姓名", 4, 8},
        {"学历", 4, 8},
        {"生日", 4, 10},
        {"民族", 4, 4},
        {"专业", 4, 12},
        {"职位", 4, 12},
        {"部门", 4, 6},
        {"岗位", 4, 10}};
}

inline std::vector<std::vector<std::string>> TUIController::convEmpDataToTable() {
    std::vector<std::vector<std::string>> data;
    for (const Employee &e : empDatasource()) {
        data.push_back({e.id, e.name, convEduLevelToCNStr(e.eduLevel),
                        e.birthday.toString("y-m-d"), e.ethnicity, e.major, e.jobTitle, e.department, e.positionTitle});
    }
    return data;
}