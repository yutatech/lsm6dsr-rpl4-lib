cmake_minimum_required(VERSION 3.22)
project(lsm6dsr_rpl4_lib)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
    include
)

file(GLOB SRCS src/*.c*)
target_sources(${PROJECT_NAME} INTERFACE ${SRCS})