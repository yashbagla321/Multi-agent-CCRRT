# Prefer SFML from cpp/third_party/sfml/ when present.

set(CCRRT_SFML_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/sfml")

if(EXISTS "${CCRRT_SFML_ROOT}/lib/cmake/SFML/SFMLConfig.cmake")
    set(SFML_DIR "${CCRRT_SFML_ROOT}/lib/cmake/SFML")
    list(PREPEND CMAKE_PREFIX_PATH "${CCRRT_SFML_ROOT}")
    message(STATUS "Using bundled SFML from ${CCRRT_SFML_ROOT}")
endif()

if(MINGW)
    set(SFML_STATIC_LIBRARIES TRUE)
    message(STATUS "MinGW detected; linking bundled SFML statically")
endif()

find_package(SFML 2.6 COMPONENTS graphics window system QUIET)

if(SFML_FOUND AND NOT SFML_STATIC_LIBRARIES AND EXISTS "${CCRRT_SFML_ROOT}/bin")
    set(CCRRT_SFML_DLL_DIR "${CCRRT_SFML_ROOT}/bin" CACHE PATH "Bundled SFML DLL directory")
endif()
