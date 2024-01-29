cd build

echo "Starting GPU benchmark..."

echo "11 DOF"
./11dof_gpu_bench 131072 16
./11dof_gpu_bench 131072 32
./11dof_gpu_bench 131072 64
./11dof_gpu_bench 131072 128
./11dof_gpu_bench 131072 254
./11dof_gpu_bench 131072 512
./11dof_gpu_bench 131072 1024
echo "18 DOF"
./18dof_gpu_bench 131072 16
./18dof_gpu_bench 131072 32
./18dof_gpu_bench 131072 64
./18dof_gpu_bench 131072 128
./18dof_gpu_bench 131072 254
./18dof_gpu_bench 131072 512
./18dof_gpu_bench 131072 1024
echo "24 DOF"
./24dof_gpu_bench 131072 16
./24dof_gpu_bench 131072 32
./24dof_gpu_bench 131072 64
./24dof_gpu_bench 131072 128
./24dof_gpu_bench 131072 254
./24dof_gpu_bench 131072 512
./24dof_gpu_bench 131072 1024


