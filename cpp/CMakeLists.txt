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

FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(json)

add_library(ccrrt_core
    src/geometry.cpp
    src/kalman_filter.cpp
    src/collision_checker.cpp
    src/legacy_collision_checker.cpp
    src/ccrrt_planner.cpp
    src/multi_agent_planner.cpp
    src/trajectory_exporter.cpp
    src/runtime_config.cpp
)

target_include_directories(ccrrt_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ccrrt_core PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json)

if(CCRRT_ENABLE_VISUALIZATION)
    include(FindBundledSFML)
    if(SFML_FOUND)
        target_sources(ccrrt_core PRIVATE src/sfml_renderer.cpp src/sfml_live_visualizer.cpp)
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

add_executable(multi_agent_ccrrt main.cpp)
target_include_directories(multi_agent_ccrrt PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(multi_agent_ccrrt PRIVATE ccrrt_core)

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

add_custom_command(
    TARGET multi_agent_ccrrt POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:multi_agent_ccrrt>/config"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/config/ccrrt.json"
        "$<TARGET_FILE_DIR:multi_agent_ccrrt>/ccrrt.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/config/ccrrt.json"
        "$<TARGET_FILE_DIR:multi_agent_ccrrt>/config/ccrrt.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/config/python_compat.json"
        "$<TARGET_FILE_DIR:multi_agent_ccrrt>/config/python_compat.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/config/scenarios.json"
        "$<TARGET_FILE_DIR:multi_agent_ccrrt>/config/scenarios.json"
    COMMENT "Copying runtime config files next to executable"
)

if(MSVC)
    target_compile_options(ccrrt_core PRIVATE /W4)
else()
    target_compile_options(ccrrt_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Unit tests (Google Test)
# ---------------------------------------------------------------------------
option(CCRRT_BUILD_TESTS "Build unit tests" ON)

if(CCRRT_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
        GIT_SHALLOW TRUE
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()

    add_executable(ccrrt_tests
        tests/test_geometry.cpp
        tests/test_kalman_filter.cpp
        tests/test_types.cpp
        tests/test_collision_checker.cpp
        tests/test_legacy_collision_checker.cpp
        tests/test_ccrrt_planner.cpp
        tests/test_multi_agent_planner.cpp
        tests/test_runtime_config.cpp
        tests/test_trajectory_exporter.cpp
    )

    target_include_directories(ccrrt_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_compile_definitions(ccrrt_tests PRIVATE
        CCRRT_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
    )

    target_link_libraries(ccrrt_tests PRIVATE ccrrt_core GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(ccrrt_tests)
endif()
