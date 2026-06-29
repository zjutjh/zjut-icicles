cmake_minimum_required(VERSION 3.15)
project(CPCPEMS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# TerminalUI 库
# ============================================================================
add_library(TerminalUI
    src/TerminalUI/TUIBlock/TUIBlock.cpp
    src/TerminalUI/TUIBlock/TUIBlock.hpp
    src/TerminalUI/TUIStyleCode.hpp
    src/TerminalUI/TUIRenderer/TUIRenderer.cpp
    src/TerminalUI/TUIRenderer/TUIRenderer.hpp
    src/TerminalUI/TUIRenderer/TerminalInterface.cpp
    src/TerminalUI/TUIRenderer/TerminalInterface.hpp
    src/TerminalUI/TUIComponents/TUICHeader.cpp
    src/TerminalUI/TUIComponents/TUICHeader.hpp
    src/TerminalUI/TUIComponents/TUICBase.hpp
    src/TerminalUI/TUIEvent.hpp
    src/TerminalUI/TUIDefinitions.hpp
    src/TerminalUI/TUIComponents/TUICTitle.cpp
    src/TerminalUI/TUIComponents/TUICTitle.hpp
    src/TerminalUI/TUIPages/TUIPBase.hpp
    src/TerminalUI/TUIPages/TUIPQuery.hpp
    src/TerminalUI/TUIPages/TUIPQuery.cpp
    src/TerminalUI/TUIPages/TUIPStats.hpp
    src/TerminalUI/TUIPages/TUIPStats.cpp
    src/TerminalUI/TUIComponents/TUICTable.hpp
    src/TerminalUI/TUIComponents/TUICTable.cpp
    src/TerminalUI/TUIComponents/TUICInput.hpp
    src/TerminalUI/TUIComponents/TUICInput.cpp
    src/TerminalUI/TUIPages/TUIPInputEmp.hpp
    src/TerminalUI/TUIPages/TUIPInputEmp.cpp
    src/TerminalUI/TUIComponents/TUICButton.hpp
    src/TerminalUI/TUIComponents/TUICKVInfo.hpp
)
# ============================================================================
# Repository 库
# ============================================================================
add_library(Repository
    src/Repository/Database.cpp
    src/Repository/Database.hpp
)

# 为 TerminalUI 库指定 include 目录
target_include_directories(TerminalUI PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/TerminalUI
)
target_include_directories(Repository PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Repository
)

# ============================================================================
# Service 库
# ============================================================================
add_library(Service
    src/Service/ServiceCore.cpp
    src/Service/ServiceCore.hpp
    src/Service/RandDataGen.cpp
    src/Service/RandDataGen.hpp
    src/Service/TUIController.cpp
    src/Service/TUIController.hpp
    src/Model/Employee.hpp
    src/Model/NamedPairs.hpp
    src/Model/LinkedList.hpp
)
target_include_directories(Service PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Service
)
target_link_libraries(Service PUBLIC Repository TerminalUI)

# ============================================================================
# 主程序
# ============================================================================
add_executable(CPCPEMS src/main.cpp)
target_link_libraries(CPCPEMS PRIVATE TerminalUI Service Repository)

# ============================================================================
# 输出信息
# ============================================================================
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Generator: ${CMAKE_GENERATOR}")