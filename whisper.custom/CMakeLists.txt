if (WHISPER_SDL2)

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    # stream
    set(TARGET stream)
    add_executable(${TARGET} stream.cpp)

    include(DefaultTargetOptions)

    target_link_libraries(${TARGET} PRIVATE common common-sdl whisper ${CMAKE_THREAD_LIBS_INIT})

    ament_target_dependencies(${TARGET} rclcpp std_msgs)

    install(TARGETS
      ${TARGET}
      DESTINATION lib/${TARGET})
    
    ament_package()

endif ()

add_executable(talker src/publisher_member_function.cpp)
