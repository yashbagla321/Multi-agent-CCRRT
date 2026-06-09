cmake_minimum_required(VERSION 3.16)
project(multi_agent_ccrrt VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CCRRT_ENABLE_VISUALIZATION "Build with SFML visualization" ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FetchContent)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(eigen)

add_library(ccrrt_core
    src/geometry.cpp
    src/kalman_filter.cpp
    src/collision_checker.cpp
    src/ccrrt_planner.cpp
    src/multi_agent_planner.cpp
    src/trajectory_exporter.cpp
)

target_include_directories(ccrrt_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ccrrt_core PUBLIC Eigen3::Eigen)

if(CCRRT_ENABLE_VISUALIZATION)
    include(FindBundledSFML)
    if(SFML_FOUND)
        target_sources(ccrrt_core PRIVATE src/sfml_renderer.cpp)
        target_compile_definitions(ccrrt_core PUBLIC CCRRT_HAS_SFML=1)
        target_link_libraries(ccrrt_core PUBLIC sfml-graphics sfml-window sfml-system)
    else()
        message(WARNING "SFML not found. Run: powershell -File scripts/fetch_sfml.ps1")
        message(WARNING "Building without visualization.")
        target_compile_definitions(ccrrt_core PUBLIC CCRRT_HAS_SFML=0)
    endif()
else()
    target_compile_definitions(ccrrt_core PUBLIC CCRRT_HAS_SFML=0)
endif()

add_library(ccrrt_scenarios
    scenarios/paper_figures.cpp
    scenarios/performance_scenarios.cpp
    scenarios/scenarios.cpp
)
target_include_directories(ccrrt_scenarios PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ccrrt_scenarios PUBLIC ccrrt_core)

add_executable(multi_agent_ccrrt main.cpp)
target_include_directories(multi_agent_ccrrt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(multi_agent_ccrrt PRIVATE ccrrt_scenarios ccrrt_core)

# Copy bundled SFML DLLs next to the executable on Windows.
if(CCRRT_SFML_DLL_DIR)
    add_custom_command(
        TARGET multi_agent_ccrrt POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CCRRT_SFML_DLL_DIR}"
            "$<TARGET_FILE_DIR:multi_agent_ccrrt>"
        COMMENT "Copying bundled SFML DLLs to output directory"
    )
endif()

if(MSVC)
    target_compile_options(ccrrt_core PRIVATE /W4)
else()
    target_compile_options(ccrrt_core PRIVATE -Wall -Wextra -Wpedantic)
endif()
