# Prefer tools on PATH; also support STM32Cube bundles on Windows.
# ARM_GCC_PATH may point to a toolchain root or its bin directory.
set(_stm32_bundle_roots "$ENV{CUBE_BUNDLE_PATH}" "$ENV{LOCALAPPDATA}/stm32cube/bundles")
set(_stm32_gcc_hints "$ENV{ARM_GCC_PATH}" "$ENV{ARM_GCC_PATH}/bin")
set(_stm32_ninja_hints)
foreach(_root IN LISTS _stm32_bundle_roots)
    if(_root)
        file(GLOB _gcc_bins LIST_DIRECTORIES true "${_root}/gnu-tools-for-stm32/*/bin")
        file(GLOB _ninja_bins LIST_DIRECTORIES true "${_root}/ninja/*/bin")
        list(SORT _gcc_bins COMPARE NATURAL ORDER DESCENDING)
        list(SORT _ninja_bins COMPARE NATURAL ORDER DESCENDING)
        list(APPEND _stm32_gcc_hints ${_gcc_bins})
        list(APPEND _stm32_ninja_hints ${_ninja_bins})
    endif()
endforeach()
find_program(ARM_GCC_EXECUTABLE NAMES arm-none-eabi-gcc
    HINTS ${_stm32_gcc_hints} NO_CMAKE_FIND_ROOT_PATH REQUIRED)
get_filename_component(ARM_GCC_BIN_DIR "${ARM_GCC_EXECUTABLE}" DIRECTORY)
set(TOOLCHAIN_PREFIX "${ARM_GCC_BIN_DIR}/arm-none-eabi-")
if(CMAKE_GENERATOR MATCHES "Ninja")
    find_program(CMAKE_MAKE_PROGRAM NAMES ninja
        HINTS ${_stm32_ninja_hints} NO_CMAKE_FIND_ROOT_PATH REQUIRED)
endif()

# Windows requires the executable extension when an absolute path is used.
set(TOOLCHAIN_SUFFIX "")
if(ARM_GCC_EXECUTABLE MATCHES "\\.exe$")
    set(TOOLCHAIN_SUFFIX ".exe")
endif()
