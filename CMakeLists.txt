cmake_minimum_required(VERSION 3.0.0)
project(LearningCG VERSION 0.1.0)

include(CTest)
enable_testing()
find_package(Eigen3 CONFIG REQUIRED)

include_directories(include)


aux_source_directory(src SRCS)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_SOURCE_DIR}/bin/)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/bin/)
add_executable(LearningCG main.cpp ${SRCS})
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
