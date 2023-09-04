cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(HUMS)

add_executable(HUMS
    HUMS.cpp
)


find_package(MAVSDK REQUIRED)

target_link_libraries(HUMS
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(testes_logs PRIVATE -Wall -Wextra)
else()
    add_compile_options(testes_logs PRIVATE -WX -W2)
endif()
