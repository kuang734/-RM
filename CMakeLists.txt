cmake_minimum_required(VERSION 3.16)
# 指定编译平台/架构与语言标准, 推荐指定Ninja为构建工具,可以加快编译速度(相比make)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
# 指定工具链
set(CMAKE_C_COMPILER_FORCED TRUE) # skip compiler test
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size) 
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

# 指定工程名称和语言类型
project(basic_framework C ASM) 

# 选择构建类型
set(CMAKE_BUILD_TYPE Debug) # Debug Release RelWithDebInfo MinSizeRel

# board specific settings, arch/fpu/instruction
set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mthumb-interwork -mfloat-abi=hard -mfpu=fpv4-sp-d16)
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32F407IGHx_FLASH.ld") # 指定链接脚本
set(DSP_NAME "libCMSISDSP.a") # 指定DSP库名称
# 动态计算 DSP 库相对路径（兼容任意构建目录层级，避免中文绝对路径导致 ld 无法搜索库）
file(RELATIVE_PATH DSP_LIB_REL ${PROJECT_BINARY_DIR} ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib)
add_link_options(-Wl,-L${DSP_LIB_REL})

# Generic compiler settings for optimization and basic link lib
add_compile_options(-pipe ${MCU_FLAGS} -Wall -fmessage-length=0 # basic options
                    -ffunction-sections -fdata-sections -fno-common # optimize options 
                    )
add_link_options(-pipe ${MCU_FLAGS} -T${LINKER_SCRIPT} -Wl,--no-warn-rwx-segments # close RWX warning
                 -lm -lc -lnosys # lib options
                 -Wl,--gc-sections -flto -specs=nano.specs -specs=nosys.specs # optimize options
                 -Wl,-Map,${PROJECT_NAME}.map -Wl,--cref -Wl,--print-memory-usage # map options
                 ) # if your executable is too large , try option '-s' to strip symbols

# add_compile_definitions() works for compile stage
# while add_definitions() works for both compile and link stage
add_definitions(
    -DUSE_HAL_DRIVER 
    -DSTM32F407xx 
    -DARM_MATH_CM4 
    ) # need -D<macro> to define macro

# add inc
# 递归包含头文件的函数
function(include_sub_directories_recursively root_dir)
    if (IS_DIRECTORY ${root_dir})               # 当前路径是一个目录吗，是的话就加入到包含目录
        message("include dir: " ${root_dir})
        include_directories(${root_dir})
    endif()

    file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*) # 获得当前目录下的所有文件，让如ALL_SUB列表中
    foreach(sub ${ALL_SUB})
        if (IS_DIRECTORY ${root_dir}/${sub})
            include_sub_directories_recursively(${root_dir}/${sub}) # 对子目录递归调用，包含
        endif()
    endforeach()
endfunction()
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Drivers)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Middlewares)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/bsp)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/modules)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/application)
include_sub_directories_recursively(${CMAKE_SOURCE_DIR}/Inc)

# add source, only surfix .c
file(GLOB_RECURSE SOURCES
    "Drivers/*.c"
    "Src/*.c"
    "Middlewares/*.c"
    "bsp/*.c"
    "modules/*.c"
    "application/*.c"
)
# 排除 syscalls.c/sysmem.c，避免与 -lnosys/--specs=nosys.specs 的符号冲突
list(FILTER SOURCES EXCLUDE REGEX ".*/(syscalls|sysmem)\\.c$")

# 汇编文件路径
# ENABLE_LANGUAGE(ASM)
set(ASM_SOURCES
    startup_stm32f407xx.s
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s
    )
set_source_files_properties(${ASM_SOURCES} PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

# Build types
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g -gdwarf-2)
    add_definitions(-DESC_DEBUG) # ESC Debug
endif ()

# build binary and hex file
add_executable(${PROJECT_NAME}.elf  ${SOURCES} ${ASM_SOURCES} ${LINKER_SCRIPT})
target_link_libraries(${PROJECT_NAME}.elf ${DSP_NAME} m) # link DSP lib and math lib

add_custom_command(
    TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE_NAME:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE_NAME:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    COMMENT "Building hex & bin file..."
    COMMENT "EXCUTABLE SIZE:"
    COMMAND ${SIZE} ${PROJECT_NAME}.elf
  )