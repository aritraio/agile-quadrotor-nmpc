cmake_minimum_required(VERSION 3.20)
project(quadrotor_nmpc VERSION 1.0.0 LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# Toolchain / standards
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Homebrew prefix for macOS (Eigen, GTest, yaml-cpp, MuJoCo).
if(APPLE)
  list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
endif()

# ---------------------------------------------------------------------------
# Options: optional solver + visual backends (default OFF = zero new deps)
# ---------------------------------------------------------------------------
option(USE_ACADOS "Build with acados solver backend" OFF)
option(QUADROTOR_USE_ACADOS "Build with acados/CasADi generated backend" OFF)
option(QUADROTOR_ENABLE_MUJOCO "Build native MuJoCo visual bridge when found" ON)

# Back-compat: either spelling enables the CasADi generated backend.
if(USE_ACADOS AND NOT QUADROTOR_USE_ACADOS)
  set(QUADROTOR_USE_ACADOS ON)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(GTest REQUIRED)
find_package(yaml-cpp QUIET)

# Optional codegen backends (quiet: never fatal when absent).
find_package(casadi QUIET)
find_package(acados QUIET)

# Optional MuJoCo: CMake config, then Homebrew/system path fallback.
set(QUADROTOR_HAS_MUJOCO OFF)
set(QUADROTOR_MUJOCO_LIB "")
if(QUADROTOR_ENABLE_MUJOCO)
  find_package(mujoco QUIET)
  if(mujoco_FOUND)
    set(QUADROTOR_HAS_MUJOCO ON)
  elseif(EXISTS "/opt/homebrew/include/mujoco/mujoco.h")
    set(QUADROTOR_HAS_MUJOCO ON)
    find_library(QUADROTOR_MUJOCO_LIB mujoco
      PATHS /opt/homebrew/lib /usr/local/lib NO_DEFAULT_PATH)
    if(QUADROTOR_MUJOCO_LIB STREQUAL "QUADROTOR_MUJOCO_LIB-NOTFOUND")
      set(QUADROTOR_MUJOCO_LIB "")
    endif()
  elseif(EXISTS "/usr/local/include/mujoco/mujoco.h")
    set(QUADROTOR_HAS_MUJOCO ON)
  endif()
endif()

include(CTest)
enable_testing()

# ---------------------------------------------------------------------------
# Common compile options (SIMD + warnings)
# ---------------------------------------------------------------------------
add_library(quadrotor_options INTERFACE)
target_compile_features(quadrotor_options INTERFACE cxx_std_20)
target_compile_options(quadrotor_options INTERFACE
  $<$<CXX_COMPILER_ID:Clang,AppleClang,GNU>:-O3;-march=native;-Wall;-Wextra;-Wpedantic;-Wshadow;-Wconversion;-Wsign-conversion>
  $<$<CXX_COMPILER_ID:Clang,AppleClang>:-Wimplicit-int-float-conversion>
  $<$<CXX_COMPILER_ID:MSVC>:/O2;/W4>
)
# NOTE: -mavx2 from the README is x86-only; -march=native already enables the
# best SIMD (AVX2 on x86_64, NEON on arm64) portably.
target_link_libraries(quadrotor_options INTERFACE Eigen3::Eigen)

# Strict C warnings for the CasADi-exported pure C99 sources.
add_library(quadrotor_c_options INTERFACE)
target_compile_options(quadrotor_c_options INTERFACE
  $<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Wpedantic;-Wshadow;-Wconversion;-Wsign-conversion>
  $<$<AND:$<COMPILE_LANGUAGE:C>,$<C_COMPILER_ID:Clang,AppleClang>>:-Wimplicit-int-float-conversion>
)

# ---------------------------------------------------------------------------
# Libraries
# ---------------------------------------------------------------------------
add_library(quadrotor_dynamics
  src/dynamics/quadrotor_model.cpp
)
target_include_directories(quadrotor_dynamics PUBLIC include)
target_link_libraries(quadrotor_dynamics PUBLIC quadrotor_options)

add_library(quadrotor_estimation
  src/estimation/es_ekf.cpp
)
target_include_directories(quadrotor_estimation PUBLIC include)
target_link_libraries(quadrotor_estimation PUBLIC quadrotor_dynamics quadrotor_options)

add_library(quadrotor_controller
  src/controller/nmpc_solver.cpp
  src/controller/trajectory_generator.cpp
)
target_include_directories(quadrotor_controller PUBLIC include)
target_link_libraries(quadrotor_controller PUBLIC quadrotor_dynamics quadrotor_options)

add_library(quadrotor_sim
  simulation/mujoco_quadrotor_env/quadrotor_env.cpp
  simulation/mujoco_quadrotor_env/mujoco_native_bridge.cpp
)
target_include_directories(quadrotor_sim PUBLIC
  include
  simulation/mujoco_quadrotor_env
)
target_link_libraries(quadrotor_sim PUBLIC quadrotor_dynamics quadrotor_options)
if(QUADROTOR_HAS_MUJOCO)
  target_compile_definitions(quadrotor_sim PUBLIC QUADROTOR_HAS_MUJOCO=1)
  if(TARGET mujoco::mujoco)
    target_link_libraries(quadrotor_sim PUBLIC mujoco::mujoco)
  else()
    if(EXISTS "/opt/homebrew/include/mujoco/mujoco.h")
      target_include_directories(quadrotor_sim PUBLIC /opt/homebrew/include)
    endif()
    if(NOT QUADROTOR_MUJOCO_LIB STREQUAL "")
      target_link_libraries(quadrotor_sim PUBLIC ${QUADROTOR_MUJOCO_LIB})
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# CasADi/acados exported C backend (optional, default OFF).
# scripts/export_ocp_casadi.py --outdir generated emits these sources.
# ---------------------------------------------------------------------------
if(QUADROTOR_USE_ACADOS OR USE_ACADOS)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/generated/quadrotor_dynamics_rk4.c")
    add_library(quadrotor_casadi_generated
      generated/quadrotor_dynamics_rk4.c
      generated/quadrotor_cost.c
    )
    target_include_directories(quadrotor_casadi_generated PUBLIC generated)
    target_link_libraries(quadrotor_casadi_generated PUBLIC quadrotor_c_options)
    target_compile_definitions(quadrotor_controller PUBLIC QUADROTOR_USE_CASADI_BACKEND=1)
    message(STATUS "acados/CasADi backend enabled: generated C sources linked")
  else()
    message(WARNING "USE_ACADOS requested but generated/*.c missing; run scripts/export_ocp_casadi.py first")
  endif()
  if(casadi_FOUND)
    target_link_libraries(quadrotor_controller PUBLIC casadi)
  endif()
  if(acados_FOUND)
    target_link_libraries(quadrotor_controller PUBLIC acados)
  endif()
endif()

# The CasADi wrapper header is always available (inline reference fallback);
# expose generated/ includes so QUADROTOR_USE_CASADI_BACKEND builds resolve.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/generated/quadrotor_dynamics_rk4.h")
  target_include_directories(quadrotor_controller PUBLIC generated)
endif()

# ---------------------------------------------------------------------------
# Main closed-loop node
# ---------------------------------------------------------------------------
add_executable(main_node src/main_node.cpp)
# NOTE: quadrotor_dynamics is PUBLIC-transitive via estimation/controller/sim;
# link only the aggregating libs to avoid duplicate static-lib warnings on ld.
target_link_libraries(main_node PRIVATE
  quadrotor_estimation quadrotor_controller quadrotor_sim
  quadrotor_options
)
if(yaml-cpp_FOUND)
  target_link_libraries(main_node PRIVATE yaml-cpp::yaml-cpp)
  target_compile_definitions(main_node PRIVATE QUADROTOR_HAS_YAML_CPP=1)
endif()

# ---------------------------------------------------------------------------
# Visual simulator target (native MuJoCo when found, else graceful fallback)
# ---------------------------------------------------------------------------
add_executable(mujoco_visual_node simulation/mujoco_quadrotor_env/mujoco_visual_node.cpp)
target_link_libraries(mujoco_visual_node PRIVATE
  quadrotor_estimation quadrotor_controller quadrotor_sim
  quadrotor_options
)
if(QUADROTOR_HAS_MUJOCO)
  message(STATUS "MuJoCo backend enabled: mujoco_visual_node uses native MJCF bridge")
else()
  message(STATUS "MuJoCo not found: mujoco_visual_node builds with standalone fallback")
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
function(quadrotor_add_test name source)
  add_executable(${name} ${source})
  # GTest::gtest_main already pulls GTest::gtest; listing both duplicates
  # libgtest.a on the link line (ld "ignoring duplicate libraries" warning).
  target_link_libraries(${name} PRIVATE
    quadrotor_estimation quadrotor_controller quadrotor_sim
    quadrotor_options GTest::gtest_main
  )
  if(TARGET quadrotor_casadi_generated)
    target_link_libraries(${name} PRIVATE quadrotor_casadi_generated)
  endif()
  add_test(NAME ${name} COMMAND ${name})
  set_tests_properties(${name} PROPERTIES TIMEOUT 120)
endfunction()

quadrotor_add_test(test_quaternion tests/test_quaternion.cpp)
quadrotor_add_test(test_integrator tests/test_integrator.cpp)
quadrotor_add_test(test_es_ekf tests/test_es_ekf.cpp)
quadrotor_add_test(test_nmpc tests/test_nmpc.cpp)
quadrotor_add_test(test_casadi_backend tests/test_casadi_backend.cpp)
