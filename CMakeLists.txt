#https://github.com/truschival/TivaWare-CMake/blob/master/CMakeLists.txt
cmake_minimum_required(VERSION 3.13)

#Toolchain file
include(helper.cmake)
include(tm4c1294ncpdt_gcc.cmake)

#Project name
project(shield)

#Cmake properties
SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(CMAKE_COLOR_MAKEFILE ON)
SET(CMAKE_ERROR_DEPRECATED ON)
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#Executable
add_executable(${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}
PROPERTIES
SUFFIX ".elf"
)

#Binary outputs
set(BINARY_NAME "${PROJECT_NAME}.elf")
set(BINARY_DIR "../../../../binary/ti/${PROJECT_NAME}/${CMAKE_BUILD_TYPE}")

#Binpack folder and executable
set(BINPACK_APP "binpack")
set(BINPACK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/binpack")

#Custom commands
#bin output
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
COMMAND ${CMAKE_OBJCOPY} ${BINARY_NAME} -O binary ${PROJECT_NAME}.bin
COMMENT "Invoking: ${CMAKE_OBJCOPY} ${BINARY_NAME} -O binary ${PROJECT_NAME}.bin"
)

#Symbol file
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
COMMAND ${CMAKE_NM} -n ${BINARY_NAME} > ${PROJECT_NAME}.sym
COMMENT "Invoking: ${CMAKE_NM} -n ${BINARY_NAME} > ${PROJECT_NAME}.sym"
)

#Disassembly
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
COMMAND ${CMAKE_OBJDUMP} -S ${BINARY_NAME} > ${PROJECT_NAME}.disasm
COMMENT "Invoking: ${CMAKE_OBJDUMP} -S ${BINARY_NAME} > ${PROJECT_NAME}.disasm"
)

#print size
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} ${BINARY_NAME}
    COMMAND ${CMAKE_SIZE} ${BINARY_NAME} > size.txt
    COMMENT "Invoking: ${CMAKE_SIZE} ${BINARY_NAME}"
)

#CRC
if(${CMAKE_BUILD_TYPE} STREQUAL release)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        WORKING_DIRECTORY /$<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy ${BINPACK_DIR}/${BINPACK_APP} ${BINPACK_APP}
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE}/${BINPACK_APP} -xv -i ${PROJECT_NAME}.bin -o ${PROJECT_NAME}_signed.bin > crc.txt
        COMMENT "Signing image with CRC"
    )
endif()

#copy to output folder
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
COMMAND ${MAKE_DIR_CMD} ${BINARY_DIR}
COMMAND ${CMAKE_COMMAND} -E copy ${BINARY_NAME} ${BINARY_DIR}
COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_NAME}.bin ${BINARY_DIR}
COMMENT "Copy: ${BINARY_NAME} ${BINARY_DIR}\nCopy: ${PROJECT_NAME}.bin ${BINARY_DIR}"
)

#target specific defenitions
if(${CMAKE_BUILD_TYPE} STREQUAL debug)
    add_definitions(-DDEBUG)
    add_definitions(-DLOG_LEVEL_3)
    add_definitions(-DPROFILE)

else()
endif()

#include dirs
target_include_directories(${PROJECT_NAME} PRIVATE
 "./"
 "../"
 "driverlib"
 "../driverlib/inc"
)


#libraty path
target_link_directories(${PROJECT_NAME} PRIVATE
    "driverlib/build/"
)

#target libraries
target_link_libraries(${PROJECT_NAME}
    m #libm
    c #libc
    gcc #libgcc
    stdc++
    driver #libdriverlib
    rdimon #librdimon
 )

#target linker script
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/tm4c1294ncpdt_gcc_${CMAKE_BUILD_TYPE}.ld")
set(CMAKE_EXE_LINKER_FLAGS "-T ${LINKER_SCRIPT} -Wl,-Map=${PROJECT_NAME}.map -Wl,--entry=reset_isr ${CMAKE_EXE_LINKER_FLAGS}")

#target sources
target_sources (${PROJECT_NAME}
	PRIVATE
		config.cpp
		control.cpp
		main.cpp
		manufacturer.cpp
		mcp9600.cpp
		motion.cpp
		optical.cpp
		startup_gcc.cpp
		utils/uartstdio.c
		utils/ustdlib.c
	PUBLIC
)