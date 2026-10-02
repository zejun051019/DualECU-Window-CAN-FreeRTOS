set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.22)
# 设置目标处理器架构,芯片型号,和目标板简称
set(CMAKE_SYSTEM_PROCESSOR cortex-m0plus)
set(Target_Board MSPM0xx)
set(Board_Name mspm0g3507)

if(NOT DEFINED ARM_GCC_TOOLCHAIN_ROOT_DIR)
    if(DEFINED ENV{ARM_GCC_ROOT} AND NOT "$ENV{ARM_GCC_ROOT}" STREQUAL "")
        file(TO_CMAKE_PATH "$ENV{ARM_GCC_ROOT}" ARM_GCC_TOOLCHAIN_ROOT_DIR)
    else()
        message(FATAL_ERROR
            "Set ARM_GCC_ROOT or ARM_GCC_TOOLCHAIN_ROOT_DIR to the Arm GNU Toolchain directory.")
    endif()
endif()
if("${ARM_GCC_TOOLCHAIN_ROOT_DIR}" STREQUAL "")
    message(FATAL_ERROR
        "ARM_GCC_TOOLCHAIN_ROOT_DIR is empty; set ARM_GCC_ROOT or the CMake cache variable.")
endif()

# 指定交叉编译工具链arm-none-eabi-gcc
set(CMAKE_C_COMPILER "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-gcc.exe")
set(CMAKE_ASM_COMPILER "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-gcc.exe")
set(CMAKE_AR "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-ar.exe")
set(CMAKE_LINKER "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-ld.exe")
set(CMAKE_OBJCOPY "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-objcopy.exe")
set(CMAKE_OBJDUMP "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-objdump.exe")
set(CMAKE_SIZE "${ARM_GCC_TOOLCHAIN_ROOT_DIR}/bin/arm-none-eabi-size.exe")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 项目名称
project(MSPM0 C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
option(DUALECU_ENABLE_TEST_FAULT_INJECTION
    "Enable debug-only software injection for DualECU acceptance tests" OFF)
option(DUALECU_ENABLE_MOTOR_STEP_TEST
    "Enable a debugger-triggered bounded motor STEP bench test" OFF)
option(DUALECU_ENABLE_ZDT_CAN_FEEDBACK
    "Build opt-in read-only ZDT CAN position polling" OFF)
option(DUALECU_ENABLE_ZDT_UART_BENCH_TEST
    "Build an opt-in debugger-triggered ZDT UART motion and feedback bench test" OFF)
option(DUALECU_ENABLE_ZDT_UART_CONTROL
    "Build the CAN-side ZDT UART safety gate; uncalibrated motion remains locked" OFF)
# 添加关于硬件浮点的支持的相关编译选项
message(STATUS "${Target_Board}")
add_compile_options(-mfloat-abi=soft)
add_link_options(-mfloat-abi=soft)

# 指定芯片型号的相关编译选项
add_compile_options(-mcpu=${CMAKE_SYSTEM_PROCESSOR} -mthumb)
add_link_options(-mcpu=${CMAKE_SYSTEM_PROCESSOR} -mthumb)

add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)
add_link_options(-specs=nosys.specs -lc -lm)
add_link_options(-u _printf_float)


# 进行汇编文件的预编译
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)
# 提供编译选项,根据不同的编译类型进行不同的优化,常见的编译类型有Debug,Release,RelWithDebInfo,MinSizeRel
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
    add_compile_options(-Og -g)
endif ()


# 添加链接脚本
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/linker/mspm0g3507.ld)

# 添加链接选项,生成map文件
add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-T ${LINKER_SCRIPT})


set(LINK_HEAP_SIZE 0x800)
set(LINK_STACK_SIZE 0x800)

add_link_options(
    -Wl,--defsym=__heap_size__=${LINK_HEAP_SIZE}
    -Wl,--defsym=__stack_size__=${LINK_STACK_SIZE}
)

# 递归收集内核、驱动和用户代码
file(
    GLOB_RECURSE PROJECT_SOURCE_FILES CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/source/core/startup_mspm0g350x_gcc.c
    ${CMAKE_SOURCE_DIR}/source/core/ti/driverlib/*.c
    ${CMAKE_SOURCE_DIR}/user/*.c)

list(SORT PROJECT_SOURCE_FILES)

set(DUALECU_SHARED_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../shared"
    CACHE PATH "Path to the canonical DualECU protocol and state modules")
if(NOT EXISTS "${DUALECU_SHARED_DIR}/can_protocol.c" OR
   NOT EXISTS "${DUALECU_SHARED_DIR}/window_state.c" OR
   NOT EXISTS "${DUALECU_SHARED_DIR}/mcan_timestamp.c")
    message(FATAL_ERROR "DUALECU_SHARED_DIR must point to the DualECU shared source directory.")
endif()

list(APPEND PROJECT_SOURCE_FILES
    "${DUALECU_SHARED_DIR}/can_protocol.c"
    "${DUALECU_SHARED_DIR}/window_state.c"
    "${DUALECU_SHARED_DIR}/mcan_timestamp.c"
    "${DUALECU_SHARED_DIR}/zdt_motor_can.c"
    "${DUALECU_SHARED_DIR}/zdt_uart_protocol.c")
list(APPEND PROJECT_SOURCE_FILES "${DUALECU_SHARED_DIR}/motor_range.c")
list(APPEND PROJECT_SOURCE_FILES "${DUALECU_SHARED_DIR}/window_demo.c")

# 生成可执行文件
add_executable(${PROJECT_NAME}.elf ti_msp_dl_config.c ${PROJECT_SOURCE_FILES} ${LINKER_SCRIPT})

set(COMMON_INCLUDE_DIRECTORIES
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/core
        ${CMAKE_SOURCE_DIR}/include/core/ti/driverlib
        ${CMAKE_SOURCE_DIR}/include/core/third_party/CMSIS/Core/Include
        ${CMAKE_SOURCE_DIR}/user
        ${DUALECU_SHARED_DIR}
)


include_directories(${CMAKE_SOURCE_DIR})

function(apply_common_target_settings target_name)
    target_include_directories(${target_name} PUBLIC ${COMMON_INCLUDE_DIRECTORIES})
endfunction()

apply_common_target_settings(${PROJECT_NAME}.elf)
target_link_libraries(${PROJECT_NAME}.elf PRIVATE m)
if(DUALECU_ENABLE_TEST_FAULT_INJECTION)
    target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
        DUALECU_ENABLE_TEST_FAULT_INJECTION=1)
endif()
if(DUALECU_ENABLE_MOTOR_STEP_TEST)
    target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
        DUALECU_ENABLE_MOTOR_STEP_TEST=1)
endif()
if(DUALECU_ENABLE_ZDT_CAN_FEEDBACK)
    target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
        DUALECU_ENABLE_ZDT_CAN_FEEDBACK=1)
endif()
if(DUALECU_ENABLE_ZDT_UART_BENCH_TEST)
    target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
        DUALECU_ENABLE_ZDT_UART_BENCH_TEST=1)
endif()
if(DUALECU_ENABLE_ZDT_UART_CONTROL)
    target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
        DUALECU_ENABLE_ZDT_UART_CONTROL=1)
endif()

# 编译宏定义
add_definitions(
        -D__MSPM0G3507__  # 定义芯片型号
)

# 定义其他类型的可执行文件
set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
# 生成hex和bin文件
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
