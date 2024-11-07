cmake_minimum_required(VERSION 3.5)
project(astar_algorithm)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Source files
set(PROJECT_SOURCES
    src/main.cpp
    src/main_window.cpp
    src/AStarPathfinder.cpp
)

# Header files
set(PROJECT_HEADERS
    include/astar_algorithm/main_window.hpp
    include/astar_algorithm/AStarPathfinder.h
)

# UI files
set(PROJECT_UIS
    ui/mainwindow.ui
)

# Create executable first without resources
add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCES}
    ${PROJECT_HEADERS}
)

# UI and MOC generation
qt5_wrap_ui(QT_UI_HPP ${PROJECT_UIS})
qt5_wrap_cpp(QT_MOC_HPP ${PROJECT_HEADERS})

# Add UI and MOC files to the target
target_sources(${PROJECT_NAME} PRIVATE
    ${QT_UI_HPP}
    ${QT_MOC_HPP}
)

# Set include directories for the target
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME} 
    rclcpp
)

target_link_libraries(${PROJECT_NAME}
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui
    Qt5::Network
    ${Boost_LIBRARIES}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY resources
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()