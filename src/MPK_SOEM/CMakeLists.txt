cmake_minimum_required(VERSION 3.16)
project(mpk_soem_four_motor LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MPK_SOEM_BUILD_TESTS "Build MPK SOEM unit tests" ON)
option(MPK_SOEM_BUILD_GUI "Build the Qt motor control GUI" ON)
set(SOEM_ROOT "" CACHE PATH "Optional SOEM install/source prefix")

find_package(Threads REQUIRED)

function(add_mpk_vendored_soem soem_dir)
  set(SOEM_SOURCE_DIR "${soem_dir}")
  set(SOEM_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_soem_include")

  set(EC_BUFSIZE "EC_MAXECATFRAME")
  set(EC_MAXBUF 16)
  set(EC_MAXEEPBITMAP 128)
  set(EC_MAXEEPBUF "EC_MAXEEPBITMAP << 5")
  set(EC_LOGGROUPOFFSET 16)
  set(EC_MAXELIST 64)
  set(EC_MAXNAME 40)
  set(EC_MAXSLAVE 200)
  set(EC_MAXGROUP 2)
  set(EC_MAXIOSEGMENTS 64)
  set(EC_MAXMBX 1486)
  set(EC_MBXPOOLSIZE 32)
  set(EC_MAXEEPDO "0x200")
  set(EC_MAXSM 8)
  set(EC_MAXFMMU 4)
  set(EC_MAXLEN_ADAPTERNAME 128)
  set(EC_MAX_MAPT 1)
  set(EC_MAXODLIST 1024)
  set(EC_MAXOELIST 256)
  set(EC_SOE_MAXNAME 60)
  set(EC_SOE_MAXMAPPING 64)
  set(EC_TIMEOUTRET 2000)
  set(EC_TIMEOUTRET3 "EC_TIMEOUTRET * 3")
  set(EC_TIMEOUTSAFE 20000)
  set(EC_TIMEOUTEEP 20000)
  set(EC_TIMEOUTTXM 20000)
  set(EC_TIMEOUTRXM 700000)
  set(EC_TIMEOUTSTATE 2000000)
  set(EC_DEFAULTRETRIES 3)
  set(EC_PRIMARY_MAC_ARRAY "{0x0101, 0x0101, 0x0101}")
  set(EC_SECONDARY_MAC_ARRAY "{0x0404, 0x0404, 0x0404}")

  configure_file(
    "${soem_dir}/include/soem/ec_options.h.in"
    "${SOEM_GENERATED_INCLUDE_DIR}/soem/ec_options.h"
    @ONLY
  )

  add_library(soem STATIC
    "${soem_dir}/src/ec_base.c"
    "${soem_dir}/src/ec_coe.c"
    "${soem_dir}/src/ec_config.c"
    "${soem_dir}/src/ec_dc.c"
    "${soem_dir}/src/ec_eoe.c"
    "${soem_dir}/src/ec_foe.c"
    "${soem_dir}/src/ec_main.c"
    "${soem_dir}/src/ec_print.c"
    "${soem_dir}/src/ec_soe.c"
    "${soem_dir}/osal/linux/osal.c"
    "${soem_dir}/oshw/linux/oshw.c"
    "${soem_dir}/oshw/linux/nicdrv.c"
  )
  target_include_directories(soem SYSTEM PUBLIC
    "${soem_dir}/include"
    "${SOEM_GENERATED_INCLUDE_DIR}"
    "${soem_dir}/osal"
    "${soem_dir}/osal/linux"
    "${soem_dir}/oshw/linux"
  )
  target_link_libraries(soem PUBLIC Threads::Threads rt)
  target_compile_options(soem PRIVATE -Wall -Wextra)
endfunction()

add_library(mpk_soem_core
  src/cia402.cpp
  src/four_motor_master.cpp
)
target_include_directories(mpk_soem_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(mpk_soem_core PRIVATE -Wall -Wextra -Wpedantic)

set(_bundled_soem_dir ${CMAKE_CURRENT_SOURCE_DIR}/third_party/SOEM)
if(EXISTS "${_bundled_soem_dir}/CMakeLists.txt")
  add_mpk_vendored_soem("${_bundled_soem_dir}")
  target_link_libraries(mpk_soem_core PUBLIC soem)
  target_compile_definitions(mpk_soem_core PUBLIC MPK_SOEM_HAS_SOEM=1)
else()
  find_path(SOEM_INCLUDE_DIR
    NAMES soem/soem.h ethercat.h
    HINTS
      ${SOEM_ROOT}
      ${SOEM_ROOT}/include
      ${SOEM_ROOT}/include/soem
    PATH_SUFFIXES include include/soem soem
  )
  find_library(SOEM_LIBRARY
    NAMES soem
    HINTS
      ${SOEM_ROOT}
      ${SOEM_ROOT}/lib
      ${SOEM_ROOT}/lib64
      ${SOEM_ROOT}/build
  )

  if(NOT SOEM_INCLUDE_DIR OR NOT SOEM_LIBRARY)
    message(FATAL_ERROR
      "SOEM was not found. Install SOEM, pass -DSOEM_ROOT=/path/to/soem/install, "
      "or place the SOEM source tree at MPK_SOEM/third_party/SOEM.")
  endif()

  target_include_directories(mpk_soem_core PUBLIC ${SOEM_INCLUDE_DIR})
  target_link_libraries(mpk_soem_core PUBLIC ${SOEM_LIBRARY})
  target_compile_definitions(mpk_soem_core PUBLIC MPK_SOEM_HAS_SOEM=1)
endif()

add_executable(mpk_soem_four_motor src/main.cpp)
target_link_libraries(mpk_soem_four_motor PRIVATE mpk_soem_core Threads::Threads)
target_compile_options(mpk_soem_four_motor PRIVATE -Wall -Wextra -Wpedantic)

if(MPK_SOEM_BUILD_GUI)
  find_package(Qt5 COMPONENTS Widgets QUIET)
  if(Qt5Widgets_FOUND)
    add_executable(mpk_soem_gui src/gui_main.cpp)
    target_link_libraries(mpk_soem_gui PRIVATE mpk_soem_core Qt5::Widgets Threads::Threads)
    target_compile_options(mpk_soem_gui PRIVATE -Wall -Wextra -Wpedantic)
  else()
    message(WARNING "Qt5 Widgets was not found; mpk_soem_gui will not be built.")
  endif()
endif()

if(EXISTS "${_bundled_soem_dir}/samples/slaveinfo/slaveinfo.c")
  add_executable(mpk_soem_slaveinfo "${_bundled_soem_dir}/samples/slaveinfo/slaveinfo.c")
  target_link_libraries(mpk_soem_slaveinfo PRIVATE soem)
endif()

if(MPK_SOEM_BUILD_TESTS)
  enable_testing()
  add_executable(test_cia402 tests/test_cia402.cpp src/cia402.cpp)
  target_include_directories(test_cia402 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_options(test_cia402 PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME test_cia402 COMMAND test_cia402)
endif()

install(TARGETS mpk_soem_four_motor RUNTIME DESTINATION bin)
if(TARGET mpk_soem_gui)
  install(TARGETS mpk_soem_gui RUNTIME DESTINATION bin)
endif()
if(TARGET mpk_soem_slaveinfo)
  install(TARGETS mpk_soem_slaveinfo RUNTIME DESTINATION bin)
endif()
