set(CMAKE_AUTORCC ON)

add_executable(horizon WIN32
    main.cpp
    resources/resources.qrc
)

target_link_libraries(horizon
    PRIVATE
        Horizon::UI
        spdlog::spdlog
)

install(TARGETS horizon
    RUNTIME DESTINATION bin
)
