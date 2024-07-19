zephyr_library()
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/module)

# Hook and replace standard behavior_backlight.c with our one
if (CONFIG_ZMK_BACKLIGHT)
    set_source_files_properties(
            ${APPLICATION_SOURCE_DIR}/src/behaviors/behavior_backlight.c
            TARGET_DIRECTORY app
            PROPERTIES HEADER_FILE_ONLY ON)
        zephyr_library_sources(src/behaviors/behavior_backlight.c)
endif()

if (CONFIG_ZMK_BATTERY_REPORTING)
    set_source_files_properties(
            ${APPLICATION_SOURCE_DIR}/module/drivers/sensor/battery/battery_voltage_divider.c
            TARGET_DIRECTORY app
            PROPERTIES HEADER_FILE_ONLY ON)
        zephyr_library_sources(drivers/sensor/battery/battery_voltage_divider.c)
endif()
