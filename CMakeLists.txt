cmake_minimum_required(VERSION 3.0.0)
project(QRAGV)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

SET(CMAKE_CXX_FLAGS "-pthread -I/usr/local/include -L/usr/local/lib")

file(GLOB_RECURSE sources CONFIGURE_DEPENDS
      # ${CMAKE_CURRENT_SOURCE_DIR}/modules/pgv100/src/*.cpp
      # ${CMAKE_CURRENT_SOURCE_DIR}/modules/pgv100/src/*.c

      # ${CMAKE_CURRENT_SOURCE_DIR}/modules/isv2motor/src/*.cpp
      # ${CMAKE_CURRENT_SOURCE_DIR}/modules/isv2motor/src/*.c

      # ${CMAKE_CURRENT_SOURCE_DIR}/modules/driving_controller/src/*.cpp
      # ${CMAKE_CURRENT_SOURCE_DIR}/modules/driving_controller/src/*.c


      ${CMAKE_CURRENT_SOURCE_DIR}/modules/qragv/src/*.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/modules/qragv/src/*.c

)

#Exclude test files
list(REMOVE_ITEM sources 
                    "${CMAKE_CURRENT_SOURCE_DIR}/modules/pgv100/src/pgv100_test.cpp"
                    "${CMAKE_CURRENT_SOURCE_DIR}/modules/isv2motor/src/isv2motor_test.cpp"
                    "${CMAKE_CURRENT_SOURCE_DIR}/modules/driving_controller/src/driving_controller_test.cpp"  
                    )


# make excutable
add_executable(${PROJECT_NAME} 
                ${CMAKE_CURRENT_SOURCE_DIR}/main/src/main.cpp
                ${sources}
                    )

target_include_directories(${PROJECT_NAME} PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/main/include 
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/hw_interface/include
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/pgv100/include
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/isv2motor/include 
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/driving_controller/include 

  ${CMAKE_CURRENT_SOURCE_DIR}/modules/qragv/include 

)

# target_link_libraries(${PROJECT_NAME} 
 
#                         )


# make library
# add_library(${PROJECT_NAME} 
#                     ${sources}
#                     )


# target_include_directories(${PROJECT_NAME} PUBLIC 
#   ${CMAKE_CURRENT_SOURCE_DIR}/include 
#   ${HARDWARE_INTERFACE_DIR}/includes
# )




