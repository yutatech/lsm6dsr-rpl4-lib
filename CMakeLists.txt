cmake_minimum_required(VERSION 3.22)
project(lsm6dsr_rpl4_lib)

add_library(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
)

file(GLOB SRCS src/*.c*)
target_sources(${PROJECT_NAME} PRIVATE ${SRCS})

target_link_libraries(${PROJECT_NAME}
    rpl4
)