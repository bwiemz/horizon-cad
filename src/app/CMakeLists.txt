add_executable(horizon WIN32
    main.cpp
)

target_link_libraries(horizon
    PRIVATE
        Horizon::UI
        spdlog::spdlog
)

install(TARGETS horizon
    RUNTIME DESTINATION bin
)
