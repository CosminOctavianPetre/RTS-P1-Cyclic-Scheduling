cmake_minimum_required(VERSION 3.13)
project(RTS-P1-Cyclic-Scheduling
        VERSION 0.1.0
        DESCRIPTION "Real time systems Project 1: Cyclic Scheduling"
        LANGUAGES C
)

# make sure that RTEMS installation path is provided
if(NOT DEFINED RTEMS_INSTALL_PATH)
    message(FATAL_ERROR "RTEMS installation path missing. Please provide it through \"-DRTEMS_INSTALL_PATH=<RTEMS5-INSTALLATION-PATH>\"")
endif()

# set compilation target names
set(TARGET_A controller_A)
set(TARGET_B controller_B)
set(TARGET_C controller_C)
set(TARGET_D controller_D)

# set language standards
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    set(CMAKE_C_STANDARD 11)
    #set(CMAKE_CXX_STANDARD 20)
endif()

# add all section subdirectories
list(APPEND Sections A B C D)
foreach(section IN LISTS Sections)
    add_subdirectory(RTEMS_${section})
endforeach()
