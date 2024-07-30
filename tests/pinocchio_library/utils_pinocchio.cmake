function(build_target TARGET_NAME)
    add_executable(${TARGET_NAME}
        src/${TARGET_NAME}.cpp
    )

    target_link_libraries(${TARGET_NAME}
    PRIVATE
    ${pinocchio_LIBRARIES}
    # boost_system
    # boost_filesystem
    )

    # target_link_directories(${TARGET_NAME}
    # 	PRIVATE
    # 		/usr/local/lib
    # )

    # target_include_directories(${TARGET_NAME}
    #     PRIVATE
    #         # /usr/include/eigen3
    # 		/usr/local/include
    # )
endfunction()