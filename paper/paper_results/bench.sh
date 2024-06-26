# Create and navigate to build directory
mkdir -p build
cd build

# Ask the user if they want to build GPU tests
read -p "Do you want to build GPU Benchmarks? (y/n): " answer

# Convert the answer to lowercase
answer=$(echo "$answer" | tr '[:upper:]' '[:lower:]')

# Check the user's answer
if [ "$answer" == "y" ] || [ "$answer" == "yes" ]; then
    echo "Building with GPU Benchmarks..."
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_GPU_TESTS=ON ..
    make -j4

    # Run the GPU benchmarks
    echo "Running GPU Benchmarks..."
    echo "11 DOF"
    ./11dof_gpu_bench 1024 16 > ../data/LFDM/11dof.out
    ./11dof_gpu_bench 2048 16 >> ../data/LFDM/11dof.out
    ./11dof_gpu_bench 4096 16 >> ../data/LFDM/11dof.out
    ./11dof_gpu_bench 8192 16 >> ../data/LFDM/11dof.out
    ./11dof_gpu_bench 16384 16 >> ../data/LFDM/11dof.out
    ./11dof_gpu_bench 32768 16 >> ../data/LFDM/11dof.out
    ./11dof_gpu_bench 65536 16 >> ../data/LFDM/11dof.out
    ./11dof_gpu_bench 131072 16 >> ../data/LFDM/11dof.out
    ./11dof_gpu_bench 262144 16 >> ../data/LFDM/11dof.out
    ./11dof_gpu_bench 393216 16 >> ../data/LFDM/11dof.out
    ./11dof_gpu_bench 524288 16 >> ../data/LFDM/11dof.out
    echo "18 DOF"
    ./18dof_gpu_bench 1024 16 > ../data/LFDM/18dof.out
    ./18dof_gpu_bench 2048 16 >> ../data/LFDM/18dof.out
    ./18dof_gpu_bench 4096 16 >> ../data/LFDM/18dof.out
    ./18dof_gpu_bench 8192 16 >> ../data/LFDM/18dof.out
    ./18dof_gpu_bench 16384 16 >> ../data/LFDM/18dof.out
    ./18dof_gpu_bench 32768 16 >> ../data/LFDM/18dof.out
    ./18dof_gpu_bench 65536 16 >> ../data/LFDM/18dof.out
    ./18dof_gpu_bench 131072 16 >> ../data/LFDM/18dof.out
    ./18dof_gpu_bench 262144 16 >> ../data/LFDM/18dof.out
    ./18dof_gpu_bench 393216 16 >> ../data/LFDM/18dof.out
    ./18dof_gpu_bench 524288 16 >> ../data/LFDM/18dof.out
    echo "24 DOF"
    ./24dof_gpu_bench 1024 16 > ../data/LFDM/24dof.out
    ./24dof_gpu_bench 2048 16 >> ../data/LFDM/24dof.out
    ./24dof_gpu_bench 4096 16 >> ../data/LFDM/24dof.out
    ./24dof_gpu_bench 8192 16 >> ../data/LFDM/24dof.out
    ./24dof_gpu_bench 16384 16 >> ../data/LFDM/24dof.out
    ./24dof_gpu_bench 32768 16 >> ../data/LFDM/24dof.out
    ./24dof_gpu_bench 65536 16 >> ../data/LFDM/24dof.out
    ./24dof_gpu_bench 131072 16 >> ../data/LFDM/24dof.out
    ./24dof_gpu_bench 262144 16 >> ../data/LFDM/24dof.out
    ./24dof_gpu_bench 393216 16 >> ../data/LFDM/24dof.out
    ./24dof_gpu_bench 524288 16 >> ../data/LFDM/24dof.out
else
    echo "Building without GPU Benchmarks..."
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_GPU_TESTS=OFF ..
    make -j4
fi

# Run the CPU benchmarks
echo "Running CPU Benchmarks..."
./accTest
./dlTest
./timingTest

# Rest of the code...
