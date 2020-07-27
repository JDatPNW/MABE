cmake_minimum_required(VERSION 3.5.1)

option(enable_World_Physics "Enable Module PhysicsWorld" ON)
if (enable_World_Physics)
  register_module(World Physics)

  set(required_components utils-urdf gui-osg)
  set(required_libraries dart dart-utils-urdf dart-gui-osg)

  find_package(DART 6.6.0 REQUIRED COMPONENTS ${required_components} CONFIG)

  file(GLOB srcs "*.cpp" "*.hpp")

  target_link_libraries(${EXE} PUBLIC ${required_libraries})
  target_sources(${EXE} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/PhysicsWorld.cpp)
  target_sources(${EXE} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/PhysicsWorld.h)

endif()


