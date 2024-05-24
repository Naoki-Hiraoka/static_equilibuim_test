cmake_minimum_required(VERSION 3.0.2)
project(wrench_distribution_existence)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  clpeigen)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wrench_distribution_existence
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/wrench_distribution_existence.cpp
  )

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
  )

add_executable(sample
  sample/sample.cpp
  )

target_link_libraries(sample PUBLIC
  ${PROJECT_NAME}
  )

add_executable(sample2
  sample/sample2.cpp
  )

target_link_libraries(sample2 PUBLIC
  ${PROJECT_NAME}
  )

add_executable(sample3
  sample/sample3.cpp
  )

target_link_libraries(sample3 PUBLIC
  ${PROJECT_NAME}
  )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )
