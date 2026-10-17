# Test files for Robotat Linalg libraries in ESP32, STM32 Nucleo and Arduino Mega

## About this branch:

This branch contains all the test files used for evaluation and validation of the Robotat Libraries (which are contained in the `main` branch), including: PlatformIO projects for ESP32, Arduino Mega 2560 and STM32 Nucleo F446RE development boards, and MATLAB scripts to generate baseline results. These test files have been developed starting from July 2025, with the most recent changes/adjustments ocurring in October 2026.

## Summary of testing pipeline: 

1. Run MATLAB scripts to generate baseline results: numerical input data and results are saved in a `.h` header file file, while execution time results are saved in a `.mat` file (check that all printing and exporting commands like `fprintf` and `save` are properly uncommented).

2. To compare MATLAB numerical results with those obtained in each embedded device, the resulting `.h` files should go in the `include/` folder inside each PlatformIO project (check the `.h` file has the correct name as it's imported inside the `main.cpp`).
    2.1 Inside each `main.cpp`, there's the corresponding variables to contain the MATLAB results as read from the `.h`, make sure to uncomment all relevant code sections.
    2.2 Comment or uncomment the test and printing code as needed. However, take into account that trying all individual time measurements in the larger files (such as those for matf32 that contain many routines being tested) might lead to crashing the device due to the large amount of data being printed out. It's recommended to test the routines in smaller batches (especially in the Arduino Mega), the code structure allows to comment/uncomment with ease each routine's test section.
    2.3 Time measurements and numerical error results, from PlatformIO projects, can be copypasted to a `.txt` to be processed in MATLAB afterwards.

## File organization: 

Folders are organized first by Robotat Linalg library, followed by platform. As such, there's many subfolders, so here's a brief overview:

```
robotat-linalg/
├── control_tests/
│   ├── control_matlab/
│   │   ├── kalman_filter_test_matlab/
│   │   ├── lti_sys_test_matlab/
│   │   ├── mpc_test_matlab/
│   │   ├── nonlin_sys_test_matlab/
│   │   └── pid_test_matlab/
│   ├── robotat_control_arduino_mega/
│   │   ├── kalman_filter_test_arduino_mega/
│   │   ├── lti_sys_test_arduino_mega/
│   │   ├── mpc_test_arduino_mega/
│   │   ├── nonlin_sys_test_arduino_mega/
│   │   └── pid_test_arduino_mega/
│   ├── robotat_control_esp32/
│   │   ├── kalman_filter_test_esp32/
│   │   ├── lti_sys_test_esp32/
│   │   ├── mpc_test_esp32/
│   │   ├── nonlin_sys_test_esp32/
│   │   └── pid_test_esp32/
│   └── robotat_control_stm32_nucleo/
│       ├── kalman_filter_test_stm32_nucleo/
│       ├── lti_sys_test_stm32_nucleo/
│       ├── mpc_test_stm32_nucleo/
│       ├── nonlin_sys_test_stm32_nucleo/
│       └── pid_test_stm32_nucleo/
├── linsolve_tests/
│   ├── linsolve_test_arduino_mega/
│   ├── linsolve_test_esp32/
│   ├── linsolve_test_matlab/
│   └── linsolve_test_stm32_nucleo/
├── matf32_tests/
│   ├── matf32_test_arduino_mega/
│   ├── matf32_test_esp32/
│   ├── matf32_test_stm32_nucleo/
│   └── math_test_matlab/
├── quadprog_tests/
│   ├── quadprog_arduino_mega/
│   │   ├── quadprog_qp_test_arduino_mega/
│   │   └── quadprog_sqp_test_arduino_mega/
│   ├── quadprog_esp32/
│   │   ├── quadprog_qp_test_esp32/
│   │   └── quadprog_sqp_test_esp32/
│   ├── quadprog_matlab/
│   │   ├── quadprog_qp_test_matlab/
│   │   └── quadprog_sqp_test_matlab/
│   └── quadprog_stm32_nucleo/
│       ├── quadprog_qp_test_stm32_nucleo/
│       └── quadprog_sqp_test_stm32_nucleo/
├── robotics_tests/
│   ├── peter_corke_robotics_toolbox_test_matlab/
│   ├── robotat_robotics_test_arduino_mega/
│   ├── robotat_robotics_test_esp32/
│   └── robotat_robotics_test_stm32_nucleo/
└── README.md
```