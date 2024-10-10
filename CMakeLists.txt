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

# Hook and replace standard ble.c with our one
# if (CONFIG_ZMK_BLE)
#     set_source_files_properties(
#             ${APPLICATION_SOURCE_DIR}/src/ble.c
#             TARGET_DIRECTORY app
#             PROPERTIES HEADER_FILE_ONLY ON)
#         zephyr_library_sources(src/ble.c)
#     zephyr_library_sources(src/behaviors/behavior_advertising_mode.c)
# endif()

if (CONFIG_ZMK_STP_INDICATORS)
    if(CONFIG_ZMK_STP_INDICATORS_KEYPAD)
        zephyr_library_sources(src/keypad_indicators.c)
    else()
        zephyr_library_sources(src/mwave_indicators.c)
    endif()

endif()

add_subdirectory(drivers)