#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Define the version and file name
SUNDIALS_VERSION="6.7.0"
SUNDIALS_TAR="sundials-${SUNDIALS_VERSION}.tar.gz"
SUNDIALS_URL="https://github.com/LLNL/sundials/releases/download/v${SUNDIALS_VERSION}/${SUNDIALS_TAR}"

# Define the directory names
SUNDIALS_DIR="sundials-${SUNDIALS_VERSION}"
BUILD_DIR="sundials-build"

# Download Sundials
echo "Downloading Sundials version ${SUNDIALS_VERSION}..."
wget ${SUNDIALS_URL} -O ${SUNDIALS_TAR}

# Extract the downloaded tarball
echo "Extracting ${SUNDIALS_TAR}..."
tar -xzvf ${SUNDIALS_TAR}

# Create the build directory
echo "Creating build directory ${BUILD_DIR}..."
mkdir -p ${BUILD_DIR}

# Change to the build directory
cd ${BUILD_DIR}

# Configure the build with CMake
echo "Configuring the build with CMake..."
cmake ../${SUNDIALS_DIR} -DBUILD_SHARED_LIBS=ON -DEXAMPLES_INSTALL_PATH=${PWD}/sundials-examples

# Build Sundials
echo "Building Sundials..."
make

# Echo completion message
echo "Sundials has been built successfully. Install manually if needed."
