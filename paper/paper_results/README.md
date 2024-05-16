# Reproducing Paper Results

This directory contains scripts and tools necessary to reproduce the results presented in our paper. By following the instructions below, you will be able to run benchmarks on both CPU and GPU, and generate plots illustrating the performance characteristics discussed in the paper.

## Prerequisites

Before you begin, ensure that you have the following installed on your system:

- CMake (version 3.10 or later)
- A modern C++ compiler that supports C++11 (e.g., GCC, Clang)
- Python3 (with matplotlib library for plotting)
- GPU drivers and CUDA Toolkit (if you wish to run GPU benchmarks)

Please refer to the official documentation of each software for installation instructions.

## Structure

- `build/` - Directory where the project is built and benchmarks are compiled.
- `data/` - Directory where output from benchmarks is saved.
- `plotting/` - Contains Python scripts for generating plots from benchmark data.

## Running Benchmarks

1. **Run the script**

    Execute the script `bench.sh` to start the process:

    ```bash
    bash bench.sh
    ```
    The script will first ask if you want to build and run GPU benchmarks. Type y for yes or n for no, then press Enter. Based on your input, the script will either build with or without GPU benchmark support and then proceed to run the CPU and GPU benchmarks.

    Note: Running GPU benchmarks requires a CUDA-compatible GPU and appropriate drivers and toolkit installed.
2. **Results**  

    Benchmark results will be saved in the `data/` directory. Plots that you see in the paper will be generated and saved in the `plotting/images/` directory. 