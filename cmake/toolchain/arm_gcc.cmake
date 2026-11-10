set(CMAKE_SYSTEM_NAME Generic)

if (NOT DEFINED CROSS_COMPILE)
  set(CROSS_COMPILE arm-none-eabi-)
endif ()
set(CMAKE_C_COMPILER "${CROSS_COMPILE}gcc")
set(CMAKE_CXX_COMPILER "${CROSS_COMPILE}g++")
set(CMAKE_ASM_COMPILER "${CROSS_COMPILE}gcc")
# CMake derives a binutils prefix only from a dashed compiler name; a cache entry, as CMP0125 is OLD at 3.17.
if (NOT DEFINED CMAKE_OBJCOPY)
  find_program(CMAKE_OBJCOPY "${CROSS_COMPILE}objcopy")
  if (NOT CMAKE_OBJCOPY)
    set(CMAKE_OBJCOPY "${CROSS_COMPILE}objcopy" CACHE FILEPATH "" FORCE)
  endif ()
endif ()
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set_property(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS FALSE)

# Look for includes and libraries only in the target system prefix.
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

include(${CMAKE_CURRENT_LIST_DIR}/../cpu/${CMAKE_SYSTEM_PROCESSOR}.cmake)

# enable all possible warnings for building examples
list(APPEND TOOLCHAIN_COMMON_FLAGS
  -fdata-sections
  -ffunction-sections
  -fno-strict-aliasing
  -mabi=aapcs
  -ggdb
  )

set(TOOLCHAIN_EXE_LINKER_FLAGS
  -Wl,--print-memory-usage
  -Wl,--gc-sections
  #-Wl,--cref
  )

# join the toolchain flags into a single string
list(JOIN TOOLCHAIN_COMMON_FLAGS " " TOOLCHAIN_COMMON_FLAGS)
list(JOIN TOOLCHAIN_EXE_LINKER_FLAGS " " CMAKE_EXE_LINKER_FLAGS_INIT)

set(CMAKE_C_FLAGS_INIT "${TOOLCHAIN_COMMON_FLAGS} ${TOOLCHAIN_C_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${TOOLCHAIN_COMMON_FLAGS} ${TOOLCHAIN_ASM_FLAGS}")
