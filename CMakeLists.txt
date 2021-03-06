cmake_minimum_required(VERSION 2.8.3)
project(problib)

#set(CMAKE_BUILD_TYPE "Debug")

#set (CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS -std=c++11)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-std=c++11 HAS_STD_CPP11_FLAG)
if(HAS_STD_CPP11_FLAG)
  add_compile_options(-std=c++11)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  genmsg
)

#find_package(Armadillo REQUIRED)
#include_directories(${ARMADILLO_INCLUDE_DIRS})

#find_package(liblapack)

#find_package(BLAS)
#find_package(LAPACK)
#if(LAPACK_FOUND AND BLAS_FOUND)
#   set(lapackblas_libraries ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
#endif()




# Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES DimensionDescription.msg PDFImpl.msg PDF.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES problib
  CATKIN_DEPENDS message_runtime #armadillo
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${ARMADILLO_INCLUDE_DIRS}
        include
)

########## feature extraction ###########

add_library(problib
        src/pdfs/PDF.cpp
        src/pdfs/Gaussian.cpp
        src/pdfs/Uniform.cpp
        src/pdfs/Mixture.cpp
        src/pdfs/Hybrid.cpp
        src/pdfs/PMF.cpp
        src/conversions.cpp
        include/${PROJECT_NAME}/conversions.h
        include/${PROJECT_NAME}/datatypes.h
        include/${PROJECT_NAME}/globals.h
        include/${PROJECT_NAME}/pdfs/PDF.h
        include/${PROJECT_NAME}/pdfs/Gaussian.h
        include/${PROJECT_NAME}/pdfs/Uniform.h
        include/${PROJECT_NAME}/pdfs/Mixture.h
        include/${PROJECT_NAME}/pdfs/Hybrid.h
        include/${PROJECT_NAME}/pdfs/PMF.h)
target_link_libraries(problib ${lapackblas_libraries} ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
add_dependencies(problib ${${PROJECT_NAME}_EXPORTED_TARGETS})


# add test executable
add_executable(problib_test src/test.cpp )
target_link_libraries(problib_test problib ${ARMADILLO_LIBRARIES})
