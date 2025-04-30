# project_starter folder
set(PROJECT_STARTER "${CMAKE_CURRENT_SOURCE_DIR}")
set(EXAMPLE_NAME 03-cartesian_motion_control)

add_definitions(-DPROJECT_STARTER="${PROJECT_STARTER}")

set(APP_NAME basketbot)
# create an executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASKETBOT_BINARY_DIR}/${APP_NAME})
add_executable(${APP_NAME} ${APP_NAME}.cpp)
add_executable(${APP_NAME}-simviz ${APP_NAME}-simviz.cpp)
add_executable(${EXAMPLE_NAME} ${EXAMPLE_NAME}.cpp)


# and link the library against the executable
target_link_libraries(${APP_NAME} ${BASKETBOT_COMMON_LIBRARIES})
target_link_libraries(${APP_NAME}-simviz ${BASKETBOT_COMMON_LIBRARIES})
target_link_libraries(${EXAMPLE_NAME} ${BASKETBOT_COMMON_LIBRARIES})

# add_subdirectory(hw0)
# add_subdirectory(hw1)
# add_subdirectory(hw2)
# add_subdirectory(hw3)

