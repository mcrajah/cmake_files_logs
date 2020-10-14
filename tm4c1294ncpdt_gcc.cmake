#Set cross compilation information
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)

# GCC toolchain prefix
set(TOOLCHAIN_PREFIX "arm-none-eabi")

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-as)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}-ar)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump)
set(CMAKE_NM ${TOOLCHAIN_PREFIX}-nm)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}-size)

enable_language(ASM)

set(CPU "-mcpu=cortex-m4")
set(FPU "-mfloat-abi=hard -mfpu=fpv4-sp-d16")
set(ARCH "-mthumb -mabi=aapcs")
set(CHIP "-DPART_TM4C1294NCPDT -DTARGET_IS_SNOWFLAKE_RA1")
#set(SPECS "--specs=nano.specs --specs=nosys.specs --specs=rdimon.specs -nostdlib")
#set(SPECS "--specs=nosys.specs --specs=rdimon.specs -nostdlib")
set(SPECS "--specs=nosys.specs")
set(WARN "-Wall -Wextra -Wundef -Wshadow -Wimplicit-function-declaration -Wredundant-decls -Wstrict-prototypes -Wmissing-prototypes -Wconversion -Wdouble-promotion -Wfloat-conversion -pedantic" )
set(FLAGS "-fno-common -ffunction-sections -fdata-sections -fmessage-length=0 -fno-builtin -fno-strict-aliasing -fno-exceptions")
#set(ERROR "-Werror")
set(PREP "-MD -MP -MF\"$(@:%.o=%.d)\" -MT\"$(@:%.o=%.d)\"")
set(STD "-std=gnu++98")

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 11)

set(CMAKE_C_FLAGS "${CPU} ${FPU} ${ARCH} ${CHIP} ${WARN} ${ERROR} ${FLAGS} ${PREP} ${STD}")

set(CMAKE_CXX_FLAGS "${CPU} ${FPU} ${ARCH} ${CHIP} ${WARN} ${ERROR} ${FLAGS} -fno-rtti ${PREP} ${STD}")

set(CMAKE_C_FLAGS_DEBUG_INIT "-O0 -ggdb -g3")

set(CMAKE_C_FLAGS_RELEASE_INIT "-Os")

set(CMAKE_CXX_FLAGS_DEBUG_INIT "-O0 -ggdb -g3")

set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Os")

set(CMAKE_EXE_LINKER_FLAGS "-Wl,--build-id -Wl,--gc-sections ${SPECS}")

add_definitions(-Dgcc)

# set(FLASH_EXECUTABLE "lm4flash")
# ADD_CUSTOM_TARGET("flash" DEPENDS ${CMAKE_PROJECT_NAME}.axf
# 	  COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.axf ${CMAKE_PROJECT_NAME}.bin
# 	    COMMAND ${FLASH_EXECUTABLE} ${CMAKE_PROJECT_NAME}.bin
# 	    )
