zephyr_library()
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
zephyr_include_directories(include)

# Hook and replace standard behavior_backlight.c with our one
if (CONFIG_ZMK_BACKLIGHT)
    set_source_files_properties(
            ${APPLICATION_SOURCE_DIR}/src/behaviors/behavior_backlight.c
            TARGET_DIRECTORY app
            PROPERTIES HEADER_FILE_ONLY ON)
        zephyr_library_sources(src/behaviors/behavior_backlight.c)
endif()


add_subdirectory(drivers)