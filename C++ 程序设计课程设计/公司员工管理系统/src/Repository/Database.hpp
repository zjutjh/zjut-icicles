#include "../Model/Employee.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
class Database {
public:
    Database(std::string dbDir = "data/") : dbDir(std::move(dbDir)) {}
    std::vector<Employee> loadEmployees();
    bool saveData(const std::vector<Employee> &employees);

private:
    std::string dbDir;
};