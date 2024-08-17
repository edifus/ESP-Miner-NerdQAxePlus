idf_component_register(
SRCS
    "serial.c"
    "bm13xx.cpp"
    "bm1397.cpp"
    "bm1366.cpp"
    "bm1368.cpp"
    "crc.c"

INCLUDE_DIRS
    "include"

REQUIRES
    "freertos"
    "driver"
    "stratum"
)


# Include the header files from "main" directory
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../main")

# Include the header files from "main/tasks" directory
target_include_directories(${COMPONENT_LIB} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../main/tasks")