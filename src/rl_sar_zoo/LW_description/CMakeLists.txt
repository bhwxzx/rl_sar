cmake_minimum_required(VERSION 3.8)
project(lw_description VERSION 2.0.0)

find_package(ament_cmake REQUIRED)

foreach(resource_directory mjcf urdf)
    install(
        DIRECTORY "${resource_directory}/"
        DESTINATION "share/${PROJECT_NAME}/${resource_directory}"
    )
endforeach()

ament_package()
