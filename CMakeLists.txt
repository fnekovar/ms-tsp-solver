cmake_minimum_required(VERSION 2.8.4)
project(msctsp)

set(CMAKE_MODULE_PATH               "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS                 "${CMAKE_CXX_FLAGS} -std=c++17 -DIL_STD -m64 -Werror")
set(CMAKE_CXX_FLAGS_DEBUG           "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE         "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -O3 -flto")
set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX   "-isystem ")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "${CMAKE_CURRENT_SOURCE_DIR}/build")
set(LINKER_OPTIONS                  -flto -Wl,--no-as-needed)
set(CPLEX_ROOT_DIR                 "/opt/ibm/ILOG/CPLEX_Studio1210")

if(CMAKE_SYSTEM_NAME STREQUAL Darwin)
    set(CMAKE_CXX_FLAGS_DEBUG       "${CMAKE_CXX_FLAGS_DEBUG} -save-temps=obj")
endif()

link_libraries(stdc++fs)

find_package(Cplex)
find_package(X11)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(SYSTEM ${CPLEX_INCLUDE_DIRS})

include_directories(PUBLIC include)

set(SOURCE_FILES
        src/target.h
        src/target_shift.h
        src/solution.h
        src/solver_grasp.cpp
        src/solver_grasp.h
        src/solver_ilp.cpp
        src/solver_ilp.h
        src/insertion.h
        src/main.cpp
        src/uav_agent.h
        src/mstsp.cpp
        src/mstsp.h)

add_executable(         msctsp ${SOURCE_FILES})
target_link_libraries(  msctsp PRIVATE ${CPLEX_LIBRARIES})
target_link_libraries(  msctsp PRIVATE dl)
target_link_libraries(  msctsp PRIVATE config++)
target_link_libraries(  msctsp PRIVATE stdc++fs)
target_link_libraries ( msctsp PRIVATE Eigen3::Eigen)

add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_SOURCE_DIR}/config.cfg $<TARGET_FILE_DIR:${PROJECT_NAME}>)
