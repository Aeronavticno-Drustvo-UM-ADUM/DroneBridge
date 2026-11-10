cmake_minimum_required(VERSION 3.16)

project(db_esp32_unit_tests C)

include(CTest)

add_executable(db_espnow_mavlink_parser_test
        db_espnow_mavlink_parser_test.c
        ../main/db_espnow_mavlink_parser.c)

target_compile_features(db_espnow_mavlink_parser_test PRIVATE c_std_11)
target_include_directories(db_espnow_mavlink_parser_test PRIVATE
        ../main
        ../components/fmav_c_lib_v2)

add_test(NAME db_espnow_mavlink_parser_test
        COMMAND db_espnow_mavlink_parser_test)

add_executable(db_rtcm_test
        db_rtcm_test.c
        ../main/db_rtcm.c)

target_compile_features(db_rtcm_test PRIVATE c_std_11)
target_compile_options(db_rtcm_test PRIVATE -Wall -Wextra -Wconversion -Werror)
target_include_directories(db_rtcm_test PRIVATE
        ../main)

add_test(NAME db_rtcm_test
        COMMAND db_rtcm_test)
