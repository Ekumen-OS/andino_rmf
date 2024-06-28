cmake_minimum_required(VERSION 3.8)
project(rmf_maps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

message(NOTICE "Starting Compilation...")

file(GLOB_RECURSE traffic_editor_paths "maps/*.building.yaml")

foreach(path ${traffic_editor_paths})
  
  message(NOTICE "path: ${path}")
  # Get the output world name
  string(REPLACE "." ";" list1 ${path})
  list(GET list1 0 name)
  string(REPLACE "/" ";" list2 ${name})
  list(GET list2 -1 world_name)

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})
  set(output_world_path ${output_dir}/${output_world_name}.world)
  set(output_model_dir ${output_dir}/models)

  message(NOTICE "map_path: ${map_path}")
  message(NOTICE "output_world_name: ${output_world_name}")
  message(NOTICE "output_dir: ${output_dir}")
  message(NOTICE "output_world_path: ${output_world_path}")
  message(NOTICE "output_model_dir: ${output_model_dir}")

  ##############################################################################
  ## Generate worlds and download models
  ##############################################################################
  message(NOTICE "BUILDING WORLDFILE WITH COMMAND: ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}")
  message(NOTICE "....Downloading Models and Building to ${output_model_dir}")
  # first, generate the world
  add_custom_command(
    OUTPUT ${output_world_path}
    COMMAND ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}
    COMMAND ros2 run rmf_building_map_tools building_map_model_downloader ${map_path} -f -e ${output_model_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name} ALL
  DEPENDS ${output_world_path}
  )

  message(NOTICE "....Downloading and Building Models to ${output_model_dir} >>>>> DONE")


  message(NOTICE "....BUILDING WORLDFILE...DONE")
  ##############################################################################
  # Generate the nav graphs
  ##############################################################################

  # now, generate the nav graphs
  set(output_nav_graphs_dir ${output_dir}/nav_graphs/)
  set(output_nav_graphs_phony ${output_nav_graphs_dir}/phony)
  add_custom_command(
    OUTPUT ${output_nav_graphs_phony}
    COMMAND ros2 run rmf_building_map_tools building_map_generator nav ${map_path} ${output_nav_graphs_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name}_nav_graphs ALL
    DEPENDS ${output_nav_graphs_phony}
  )

  install(
    DIRECTORY ${output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )

  message(NOTICE "....Generating Lane Graphs in  ${output_nav_graphs_dir}...DONE")

endforeach()

install(DIRECTORY
  maps
  rviz_config
  DESTINATION share/${PROJECT_NAME}
)
