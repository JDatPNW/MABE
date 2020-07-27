option(enable_World_Physics "Enable Module PhysicsWorld" OFF) # set OFF to on to make it work - Needs new incldues though!
if (enable_World_Physics)
  register_module(World Physics)
  target_sources(${EXE} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/PhysicsWorld.cpp)
  target_sources(${EXE} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/PhysicsWorld.h)
endif()
