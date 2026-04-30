cmake_minimum_required(VERSION 3.14)
project(anti_swing_control VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# ============================================================================
# Find dependencies
# ============================================================================
find_package(ruckig CONFIG REQUIRED)
find_package(fmt REQUIRED)

# ============================================================================
# Core library (new modular code)
# ============================================================================
add_library(pendulum STATIC
    src/observer/pend_observer.cpp
    src/observer/pendulum_observer.cpp
    src/utils/math_utils.cpp
    src/utils/csv_logger.cpp
    src/dynamics/slung_load_dynamics.cpp
    src/sensor/imu_sensor_model.cpp
    src/controller/lqr_controller.cpp
    src/simulation/closed_loop_simulation.cpp
)

target_link_libraries(pendulum PRIVATE m)

# ============================================================================
# Executables
# ============================================================================

# Legacy open-loop simulation (migrated from original main.cpp)
add_executable(run_open_loop_sim src/apps/run_open_loop_sim.cpp)
target_link_libraries(run_open_loop_sim PRIVATE pendulum m)

# Legacy CSV replay (migrated from original replay_csv.cpp)
add_executable(run_csv_replay src/apps/run_csv_replay.cpp)
target_link_libraries(run_csv_replay PRIVATE pendulum m)

# Create symlinks in build directory so executables can find data/ and results/
# regardless of whether they are run from project root or build directory.
add_custom_command(TARGET run_open_loop_sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_SOURCE_DIR}/data ${CMAKE_BINARY_DIR}/data
    COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_SOURCE_DIR}/results ${CMAKE_BINARY_DIR}/results
    COMMENT "Linking data/ and results/ into build/"
)

# New closed-loop LQR simulation
add_executable(run_closed_loop_lqr src/apps/run_closed_loop_lqr.cpp)
target_link_libraries(run_closed_loop_lqr PRIVATE pendulum)

# ============================================================================
# Ruckig S-curve trajectory planner (standalone demo)
# ============================================================================
add_executable(run_ruckig_trajectory
    src/apps/run_ruckig_trajectory.cpp
    src/trajectory/ruckig_planner.cpp
)
target_link_libraries(run_ruckig_trajectory PRIVATE
    pendulum
    ruckig::ruckig
)

# ============================================================================
# Install targets (optional)
# ============================================================================
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
install(TARGETS run_open_loop_sim run_csv_replay run_closed_loop_lqr
        RUNTIME DESTINATION bin)
