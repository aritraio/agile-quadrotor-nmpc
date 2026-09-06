# Development & Operational Workflow

This document outlines the standard engineering workflow for building, testing, benchmarking, and maintaining the Quadrotor NMPC & State Estimation flight stack.

---

## 1. Quick Development Commands

### 1.1. Clean Build (Default Native Backend)
```bash
# Configure build with Release optimizations and native SIMD
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release

# Compile all targets with maximum parallel jobs
cmake --build build -j$(nproc 2>/dev/null || sysctl -n hw.ncpu)
```

### 1.2. Building with CasADi / acados C-Generated Backend
```bash
# 1. Generate pure C99 sources from symbolic Python model
python3 scripts/export_ocp_casadi.py --outdir generated

# 2. Configure CMake with acados/CasADi C backend enabled
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DUSE_ACADOS=ON

# 3. Compile
cmake --build build -j
```

---

## 2. Test Execution & Verification

### 2.1. Running the Automated CTest Suite
```bash
# Run all 5 unit test suites with detailed failure logs
ctest --test-dir build --output-on-failure
```

### 2.2. Running Individual Unit Test Binaries
```bash
./build/test_quaternion       # SO(3) kinematics & Lie algebra tests
./build/test_integrator       # 6-DoF dynamics & RK4 energy conservation
./build/test_es_ekf           # 15-state ES-EKF covariance & bias estimation
./build/test_nmpc             # NMPC real-time benchmark & constraint validation
./build/test_casadi_backend   # CasADi C99 vs native iLQR cross-validation
```

---

## 3. Simulation & Benchmarking Workflows

### 3.1. Main Closed-Loop Flight Simulation (`main_node`)
Executes the closed-loop flight stack: **Estimator (500 Hz IMU / 30 Hz VO) $\rightarrow$ Trajectory Generator $\rightarrow$ NMPC (100 Hz) $\rightarrow$ Quadrotor Plant**:

```bash
# Syntax: ./build/main_node [trajectory] [duration_sec] [log_csv_path]

# 1. Agile 3D Lemniscate (Figure-8)
./build/main_node lemniscate 10.0 logs/flight_lemniscate.csv

# 2. High-Speed Horizontal Circle
./build/main_node circle 5.0 logs/flight_circle.csv

# 3. Aggressive Step Translation
./build/main_node step 5.0 logs/flight_step.csv

# 4. Stationary Hover Precision
./build/main_node hover 5.0 logs/flight_hover.csv
```

### 3.2. Visual Simulator Bridge (`mujoco_visual_node`)
Executes the flight stack against the native MuJoCo MJCF model or standalone fallback:

```bash
./build/mujoco_visual_node lemniscate 8.0 logs/visual_log.csv simulation/mujoco_quadrotor_env/quadrotor.xml
```

---

## 4. Code Quality & Standards Enforcement

### Zero-Warning Standard
The project mandates zero compiler warnings under strict warning flags:
```bash
cmake -B build -S . -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow"
cmake --build build --clean-first
```

### Formatting
Format all C++ source files using `clang-format` (Google style):
```bash
find include src simulation tests -name "*.hpp" -o -name "*.cpp" | xargs clang-format -i
```

---

## 5. Continuous Integration (CI) Architecture

The project includes an automated GitHub Actions pipeline (`.github/workflows/ci.yml`) that executes on every push and pull request:
1. **Multi-OS Matrix:** Builds and tests on `ubuntu-24.04` and `macos-latest` (Apple Silicon).
2. **Dependency Provisioning:** Installs Eigen3, GoogleTest, and yaml-cpp via package managers (`apt` / `brew`).
3. **Symbolic Code Generation:** Runs `scripts/export_ocp_casadi.py` to generate C99 dynamics and cost functions.
4. **Compilation:** Builds with `-O3 -march=native` under strict warning-as-error checks.
5. **Testing & Latency Budget Check:** Executes `ctest` and asserts that the NMPC solve time benchmark strictly passes under $10\,\text{ms}$.
