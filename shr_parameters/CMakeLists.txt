cmake_minimum_required(VERSION 3.8)
project(shr_parameters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Generate the library from the YAML file
generate_parameter_library(shr_parameters
        params/shr_parameters.yaml
)

# Install the parameter YAML file
install(DIRECTORY params
        DESTINATION share/${PROJECT_NAME}
)

# Install the generated header
install(
        FILES ${shr_parameters_CONFIG}
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/
        DESTINATION include
)

# Install and export the generated target
install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_package()