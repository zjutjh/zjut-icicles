#include "RandDataGen.hpp"

std::vector<Employee> RandomDataGenerator::generateRandomEmployees(int count) {
    std::vector<Employee> employees;
    srand(static_cast<unsigned int>(time(nullptr)));
    for (int i = 0; i < count; i++) {
        Employee e;
        e.id = std::to_string(i + 1);
        e.name = firstNames[rand() % firstNames.size()] + lastNames[rand() % lastNames.size()];
        e.eduLevel = static_cast<EducationalLevel>(rand() % static_cast<int>(EducationalLevel::Postdoctorate) + 1);
        e.birthday = Date(1950 + rand() % 50, 1 + rand() % 12, 1 + rand() % 28);
        e.ethnicity = ethnicities[rand() % ethnicities.size()];
        e.major = majors[rand() % majors.size()];
        e.jobTitle = jobTitles[rand() % jobTitles.size()];
        e.department = departments[rand() % departments.size()];
        e.positionTitle = positionTitles[rand() % positionTitles.size()];
        employees.push_back(e);
    }
    return employees;
}