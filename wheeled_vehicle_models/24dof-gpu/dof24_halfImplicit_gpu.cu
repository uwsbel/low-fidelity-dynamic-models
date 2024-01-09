#include <algorithm>
#include <cassert>
#include <random>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <vector>

#include "dof24_gpu.cuh"
#include "dof24_halfImplicit_gpu.cuh"

using namespace d24;
// ======================================================================================================================
// Constructor
d24SolverHalfImplicitGPU::d24SolverHalfImplicitGPU(unsigned int total_num_vehicles)
    : m_step(0.001),
      m_output(false),
      m_vehicle_count_tracker_params(0),
      m_vehicle_count_tracker_states(0),
      m_kernel_sim_time(2.),
      m_host_dump_time(10.),
      m_threads_per_block(32),
      m_tend(0.) {
    m_total_num_vehicles = total_num_vehicles;

    // Allocate memory for the simData and simStates
    CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_data, sizeof(d24::SimData) * m_total_num_vehicles));
    CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_data_nr, sizeof(d24::SimDataNr) * m_total_num_vehicles));
    CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_states, sizeof(d24::SimState) * m_total_num_vehicles));
    CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_states_nr, sizeof(d24::SimStateNr) * m_total_num_vehicles));

    // Set device and host arrays to nullptrs in case SetOutput is not called by the user
    m_device_response = nullptr;
    m_host_response = nullptr;

    int deviceId = 0;         // Assume we are using GPU 0
    cudaSetDevice(deviceId);  // Set the device
}
__host__ d24SolverHalfImplicitGPU::~d24SolverHalfImplicitGPU() {
    // Only need to delete the memory of the simData and simStates of the respective tire as the rest of the memory is
    // freed as soon as we have information of what tire the user is using
    if (m_tire_type == TireType::TMeasy) {
        cudaFree(m_sim_data);
        cudaFree(m_sim_states);
    } else {
        cudaFree(m_sim_data_nr);
        cudaFree(m_sim_states_nr);
    }
    cudaFree(m_device_response);
    delete[] m_host_response;
}
// ======================================================================================================================
__host__ void d24SolverHalfImplicitGPU::Construct(const std::string& vehicle_params_file,
                                                  const std::string& tire_params_file,
                                                  const std::string& sus_params_file,
                                                  unsigned int num_vehicles,
                                                  const std::string& driver_inputs_file) {
    // Check if num_vehicles added is less than the total number of vehicles
    assert((num_vehicles + m_vehicle_count_tracker_params <= m_total_num_vehicles) &&
           "Number of vehicles added makes the vehicle count greater than the total number of vehicles");
    // If there is no tire type specified, then use TMeasy
    m_tire_type = TireType::TMeasy;
    // Because of this, we free the memory of the TMeasyNR tire
    cudaFree(m_sim_data_nr);
    cudaFree(m_sim_states_nr);
    // Set these to nullptr so that we don't try to free them again in the destructor
    m_sim_data_nr = nullptr;
    m_sim_states_nr = nullptr;

    // Since cudaMallocManaged does not call the constructor for non-POD types, we create cpu structs and fill them up
    // and then copy them over to the simData structs
    d24::VehicleParam veh_param;
    d24::TMeasyParam tire_param;
    d24::SuspensionParam sus_param;

    setVehParamsJSON(veh_param, vehicle_params_file.c_str());
    setTireParamsJSON(tire_param, tire_params_file.c_str());
    setSuspensionParamsJSON(sus_param, sus_params_file.c_str());
    // Initialize tire parameters that depend on other parameters
    tireInit(&tire_param);

    DriverData driver_data;
    LoadDriverData(driver_data, driver_inputs_file);
    unsigned int driver_data_len = driver_data.size();
    size_t old_vehicle_count = m_vehicle_count_tracker_params;
    m_vehicle_count_tracker_params += num_vehicles;
    for (size_t i = old_vehicle_count; i < m_vehicle_count_tracker_params; i++) {
        m_sim_data[i]._driver_data_len = driver_data_len;
        // Allocate memory for the driver data
        CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_data[i]._driver_data,
                                           sizeof(DriverInput) * m_sim_data[i]._driver_data_len));
        // Copy the driver data from cpu to managed memory
        std::copy(driver_data.begin(), driver_data.end(), m_sim_data[i]._driver_data);
        // Fill up simulation data from the cpu structs
        m_sim_data[i]._veh_params = veh_param;
        m_sim_data[i]._tireTM_params = tire_param;
        m_sim_data[i]._sus_params = sus_param;
    }
    CHECK_CUDA_ERROR(cudaMemPrefetchAsync(m_sim_data, sizeof(m_sim_data[0]) * m_vehicle_count_tracker_params,
                                          0));  // move the simData onto the GPU
}

__host__ void d24SolverHalfImplicitGPU::Construct(const std::string& vehicle_params_file,
                                                  const std::string& tire_params_file,
                                                  const std::string& sus_params_file,
                                                  unsigned int num_vehicles,
                                                  const std::string& driver_inputs_file,
                                                  TireType type) {
    // Check if num_vehicles added is less than the total number of vehicles
    assert((num_vehicles + m_vehicle_count_tracker_params <= m_total_num_vehicles) &&
           "Number of vehicles added makes the vehicle count greater than the total number of vehicles");
    // If there is no tire type specified, then use TMeasy
    m_tire_type = type;

    if (m_tire_type == TireType::TMeasy) {
        // Because of this, we free the memory of the TMeasyNR tire
        cudaFree(m_sim_data_nr);
        cudaFree(m_sim_states_nr);
        // Set these to nullptr so that we don't try to free them again in the destructor
        m_sim_data_nr = nullptr;
        m_sim_states_nr = nullptr;

        // Since cudaMallocManaged does not call the constructor for non-POD types, we create cpu structs and fill them
        // up and then copy them over to the simData structs
        d24::VehicleParam veh_param;
        d24::TMeasyParam tire_param;
        d24::SuspensionParam sus_param;

        setVehParamsJSON(veh_param, vehicle_params_file.c_str());
        setTireParamsJSON(tire_param, tire_params_file.c_str());
        setSuspensionParamsJSON(sus_param, sus_params_file.c_str());
        // Initialize tire parameters that depend on other parameters
        tireInit(&tire_param);

        DriverData driver_data;
        LoadDriverData(driver_data, driver_inputs_file);
        unsigned int driver_data_len = driver_data.size();
        size_t old_vehicle_count = m_vehicle_count_tracker_params;
        m_vehicle_count_tracker_params += num_vehicles;
        for (size_t i = old_vehicle_count; i < m_vehicle_count_tracker_params; i++) {
            m_sim_data[i]._driver_data_len = driver_data_len;
            // Allocate memory for the driver data
            CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_data[i]._driver_data,
                                               sizeof(DriverInput) * m_sim_data[i]._driver_data_len));
            // Copy the driver data from cpu to managed memory
            std::copy(driver_data.begin(), driver_data.end(), m_sim_data[i]._driver_data);
            // Fill up simulation data from the cpu structs
            m_sim_data[i]._veh_params = veh_param;
            m_sim_data[i]._tireTM_params = tire_param;
            m_sim_data[i]._sus_params = sus_param;
        }
        CHECK_CUDA_ERROR(cudaMemPrefetchAsync(m_sim_data, sizeof(m_sim_data[0]) * m_vehicle_count_tracker_params,
                                              0));  // move the simData onto the GPU
    } else {
        // Because of this, we free the memory of the TMeasyNR tire
        cudaFree(m_sim_data);
        cudaFree(m_sim_states);
        // Set these to nullptr so that we don't try to free them again in the destructor
        m_sim_data = nullptr;
        m_sim_states = nullptr;

        // Since cudaMallocManaged does not call the constructor for non-POD types, we create cpu structs and fill them
        // up and then copy them over to the simData structs
        d24::VehicleParam veh_param;
        d24::TMeasyNrParam tire_param;
        d24::SuspensionParam sus_param;

        setVehParamsJSON(veh_param, vehicle_params_file.c_str());
        setTireParamsJSON(tire_param, tire_params_file.c_str());
        setSuspensionParamsJSON(sus_param, sus_params_file.c_str());
        // Initialize tire parameters that depend on other parameters
        tireInit(&tire_param);

        DriverData driver_data;
        LoadDriverData(driver_data, driver_inputs_file);
        unsigned int driver_data_len = driver_data.size();
        size_t old_vehicle_count = m_vehicle_count_tracker_params;
        m_vehicle_count_tracker_params += num_vehicles;
        for (size_t i = old_vehicle_count; i < m_vehicle_count_tracker_params; i++) {
            m_sim_data_nr[i]._driver_data_len = driver_data_len;
            // Allocate memory for the driver data
            CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_data_nr[i]._driver_data,
                                               sizeof(DriverInput) * m_sim_data_nr[i]._driver_data_len));
            // Copy the driver data from cpu to managed memory
            std::copy(driver_data.begin(), driver_data.end(), m_sim_data_nr[i]._driver_data);
            // Fill up simulation data from the cpu structs
            m_sim_data_nr[i]._veh_params = veh_param;
            m_sim_data_nr[i]._tireTMNr_params = tire_param;
            m_sim_data_nr[i]._sus_params = sus_param;
        }
        CHECK_CUDA_ERROR(cudaMemPrefetchAsync(m_sim_data_nr, sizeof(m_sim_data_nr[0]) * m_vehicle_count_tracker_params,
                                              0));  // move the simData onto the GPU
    }
}

// Overload for situations when a controller is used and we don't have a driver data file
__host__ void d24SolverHalfImplicitGPU::Construct(const std::string& vehicle_params_file,
                                                  const std::string& tire_params_file,
                                                  const std::string& sus_params_file,
                                                  unsigned int num_vehicles) {
    // Check if num_vehicles added is less than the total number of vehicles
    assert((num_vehicles + m_vehicle_count_tracker_params <= m_total_num_vehicles) &&
           "Number of vehicles added makes the vehicle count greater than the total number of vehicles");
    // If there is no tire type specified, then use TMeasy
    m_tire_type = TireType::TMeasy;
    // Because of this, we free the memory of the TMeasyNR tire
    cudaFree(m_sim_data_nr);
    cudaFree(m_sim_states_nr);
    // Set these to nullptr so that we don't try to free them again in the destructor
    m_sim_data_nr = nullptr;
    m_sim_states_nr = nullptr;

    // Since cudaMallocManaged does not call the constructor for non-POD types, we create cpu structs and fill them up
    // and then copy them over to the simData structs
    d24::VehicleParam veh_param;
    d24::TMeasyParam tire_param;
    d24::SuspensionParam sus_param;

    setVehParamsJSON(veh_param, vehicle_params_file.c_str());
    setTireParamsJSON(tire_param, tire_params_file.c_str());
    setSuspensionParamsJSON(sus_param, sus_params_file.c_str());
    // Initialize tire parameters that depend on other parameters
    tireInit(&tire_param);

    size_t old_vehicle_count = m_vehicle_count_tracker_params;
    m_vehicle_count_tracker_params += num_vehicles;
    for (size_t i = old_vehicle_count; i < m_vehicle_count_tracker_params; i++) {
        // Fill up simulation data from the cpu structs
        m_sim_data[i]._veh_params = veh_param;
        m_sim_data[i]._tireTM_params = tire_param;
        m_sim_data[i]._sus_params = sus_param;
    }
    CHECK_CUDA_ERROR(cudaMemPrefetchAsync(m_sim_data, sizeof(m_sim_data[0]) * m_vehicle_count_tracker_params,
                                          0));  // move the simData onto the GPU
}

__host__ void d24SolverHalfImplicitGPU::Construct(const std::string& vehicle_params_file,
                                                  const std::string& tire_params_file,
                                                  const std::string& sus_params_file,
                                                  unsigned int num_vehicles,
                                                  TireType type) {
    // Check if num_vehicles added is less than the total number of vehicles
    assert((num_vehicles + m_vehicle_count_tracker_params <= m_total_num_vehicles) &&
           "Number of vehicles added makes the vehicle count greater than the total number of vehicles");
    // If there is no tire type specified, then use TMeasy
    m_tire_type = type;

    if (m_tire_type == TireType::TMeasy) {
        // Because of this, we free the memory of the TMeasyNR tire
        cudaFree(m_sim_data_nr);
        cudaFree(m_sim_states_nr);
        // Set these to nullptr so that we don't try to free them again in the destructor
        m_sim_data_nr = nullptr;
        m_sim_states_nr = nullptr;

        // Since cudaMallocManaged does not call the constructor for non-POD types, we create cpu structs and fill them
        // up and then copy them over to the simData structs
        d24::VehicleParam veh_param;
        d24::TMeasyParam tire_param;
        d24::SuspensionParam sus_param;

        setVehParamsJSON(veh_param, vehicle_params_file.c_str());
        setTireParamsJSON(tire_param, tire_params_file.c_str());
        setSuspensionParamsJSON(sus_param, sus_params_file.c_str());
        // Initialize tire parameters that depend on other parameters
        tireInit(&tire_param);

        size_t old_vehicle_count = m_vehicle_count_tracker_params;
        m_vehicle_count_tracker_params += num_vehicles;
        for (size_t i = old_vehicle_count; i < m_vehicle_count_tracker_params; i++) {
            // Fill up simulation data from the cpu structs
            m_sim_data[i]._veh_params = veh_param;
            m_sim_data[i]._tireTM_params = tire_param;
            m_sim_data[i]._sus_params = sus_param;
        }
        CHECK_CUDA_ERROR(cudaMemPrefetchAsync(m_sim_data, sizeof(m_sim_data[0]) * m_vehicle_count_tracker_params,
                                              0));  // move the simData onto the GPU
    } else {
        // Because of this, we free the memory of the TMeasyNR tire
        cudaFree(m_sim_data);
        cudaFree(m_sim_states);
        // Set these to nullptr so that we don't try to free them again in the destructor
        m_sim_data = nullptr;
        m_sim_states = nullptr;

        // Since cudaMallocManaged does not call the constructor for non-POD types, we create cpu structs and fill them
        // up and then copy them over to the simData structs
        d24::VehicleParam veh_param;
        d24::TMeasyNrParam tire_param;
        d24::SuspensionParam sus_param;

        setVehParamsJSON(veh_param, vehicle_params_file.c_str());
        setTireParamsJSON(tire_param, tire_params_file.c_str());
        setSuspensionParamsJSON(sus_param, sus_params_file.c_str());
        // Initialize tire parameters that depend on other parameters
        tireInit(&tire_param);

        size_t old_vehicle_count = m_vehicle_count_tracker_params;
        m_vehicle_count_tracker_params += num_vehicles;
        for (size_t i = old_vehicle_count; i < m_vehicle_count_tracker_params; i++) {
            // Fill up simulation data from the cpu structs
            m_sim_data_nr[i]._veh_params = veh_param;
            m_sim_data_nr[i]._tireTMNr_params = tire_param;
            m_sim_data_nr[i]._sus_params = sus_param;
        }
        CHECK_CUDA_ERROR(cudaMemPrefetchAsync(m_sim_data_nr, sizeof(m_sim_data_nr[0]) * m_vehicle_count_tracker_params,
                                              0));  // move the simData onto the GPU
    }
}
// ======================================================================================================================
__host__ void d24SolverHalfImplicitGPU::Initialize(d24::VehicleState& vehicle_states,
                                                   d24::TMeasyState& tire_states_LF,
                                                   d24::TMeasyState& tire_states_RF,
                                                   d24::TMeasyState& tire_states_LR,
                                                   d24::TMeasyState& tire_states_RR,
                                                   d24::SuspensionState& sus_states_LF,
                                                   d24::SuspensionState& sus_states_RF,
                                                   d24::SuspensionState& sus_states_LR,
                                                   d24::SuspensionState& sus_states_RR,
                                                   unsigned int num_vehicles) {
    // Esnure that construct was called with TMeasy tire type
    assert((m_tire_type == TireType::TMeasy) &&
           "Construct function called with TMeasyNr tire type, but Initialize called with TMeasy tire type");
    assert((num_vehicles + m_vehicle_count_tracker_states <= m_total_num_vehicles) &&
           "Number of vehicles added makes the vehicle count greater than the total number of vehicles");
    size_t old_vehicle_count = m_vehicle_count_tracker_states;
    m_vehicle_count_tracker_states += num_vehicles;
    for (size_t i = old_vehicle_count; i < m_vehicle_count_tracker_states; i++) {
        initializeTireSus(&vehicle_states, &tire_states_LF, &tire_states_RF, &tire_states_LR, &tire_states_RR,
                          &sus_states_LF, &sus_states_RF, &sus_states_LR, &sus_states_RR, &m_sim_data[i]._veh_params,
                          &m_sim_data[i]._tireTM_params, &m_sim_data[i]._sus_params);
        // Fill up simulation data from the cpu structs
        m_sim_states[i]._v_states = vehicle_states;
        m_sim_states[i]._tirelf_st = tire_states_LF;
        m_sim_states[i]._tirerf_st = tire_states_RF;
        m_sim_states[i]._tirelr_st = tire_states_LR;
        m_sim_states[i]._tirerr_st = tire_states_RR;
        m_sim_states[i]._suslf_st = sus_states_LF;
        m_sim_states[i]._susrf_st = sus_states_RF;
        m_sim_states[i]._suslr_st = sus_states_LR;
        m_sim_states[i]._susrr_st = sus_states_RR;
    }
    CHECK_CUDA_ERROR(cudaMemPrefetchAsync(m_sim_states, sizeof(SimState) * m_vehicle_count_tracker_states,
                                          0));  // move the simState onto the GPU
}

__host__ void d24SolverHalfImplicitGPU::Initialize(d24::VehicleState& vehicle_states,
                                                   d24::TMeasyNrState& tire_states_LF,
                                                   d24::TMeasyNrState& tire_states_RF,
                                                   d24::TMeasyNrState& tire_states_LR,
                                                   d24::TMeasyNrState& tire_states_RR,
                                                   d24::SuspensionState& sus_states_LF,
                                                   d24::SuspensionState& sus_states_RF,
                                                   d24::SuspensionState& sus_states_LR,
                                                   d24::SuspensionState& sus_states_RR,
                                                   unsigned int num_vehicles) {
    // Esnure that construct was called with TMeasy tire type
    assert((m_tire_type == TireType::TMeasy) &&
           "Construct function called with TMeasyNr tire type, but Initialize called with TMeasy tire type");
    assert((num_vehicles + m_vehicle_count_tracker_states <= m_total_num_vehicles) &&
           "Number of vehicles added makes the vehicle count greater than the total number of vehicles");
    size_t old_vehicle_count = m_vehicle_count_tracker_states;
    m_vehicle_count_tracker_states += num_vehicles;
    for (size_t i = old_vehicle_count; i < m_vehicle_count_tracker_states; i++) {
        initializeTireSus(&vehicle_states, &tire_states_LF, &tire_states_RF, &tire_states_LR, &tire_states_RR,
                          &sus_states_LF, &sus_states_RF, &sus_states_LR, &sus_states_RR, &m_sim_data_nr[i]._veh_params,
                          &m_sim_data_nr[i]._tireTMNr_params, &m_sim_data_nr[i]._sus_params);
        // Fill up simulation data from the cpu structs
        m_sim_states_nr[i]._v_states = vehicle_states;
        m_sim_states_nr[i]._tirelf_st = tire_states_LF;
        m_sim_states_nr[i]._tirerf_st = tire_states_RF;
        m_sim_states_nr[i]._tirelr_st = tire_states_LR;
        m_sim_states_nr[i]._tirerr_st = tire_states_RR;
        m_sim_states_nr[i]._suslf_st = sus_states_LF;
        m_sim_states_nr[i]._susrf_st = sus_states_RF;
        m_sim_states_nr[i]._suslr_st = sus_states_LR;
        m_sim_states_nr[i]._susrr_st = sus_states_RR;
    }
    CHECK_CUDA_ERROR(cudaMemPrefetchAsync(m_sim_states, sizeof(SimState) * m_vehicle_count_tracker_states,
                                          0));  // move the simState onto the GPU
}

// ======================================================================================================================

__host__ void d24SolverHalfImplicitGPU::SetOutput(const std::string& output_file,
                                                  double output_freq,
                                                  bool store_all,
                                                  unsigned int no_outs) {
    m_output = true;
    m_store_all = store_all;
    if (!m_store_all) {
        // Check if number of outputs asked is greater than the total number of vehicles, if this is the case, raise
        // awarning and set to m_total_num_vehicles
        if (no_outs > m_total_num_vehicles) {
            std::cout << "Number of outputs asked is greater than the total number of vehicles, setting number of "
                         "outputs to total number of vehicles"
                      << std::endl;
            no_outs = m_total_num_vehicles;
        }
        m_num_outs = no_outs;
        // If store_all is false, randomly assign which vehicles need to be dumped into csv
        float some_seed = 68;
        std::mt19937 generator(some_seed);

        // Generate a range of numbers and shuffle them
        std::vector<int> numbers(m_total_num_vehicles);
        std::iota(numbers.begin(), numbers.end(), 0);  // Fill with values from 0 to m_total_num_vehicles - 1
        std::shuffle(numbers.begin(), numbers.end(), generator);

        // Resize m_which_outs and assign the first 'no_outs' numbers from the shuffled range
        m_which_outs.resize(no_outs);
        std::copy(numbers.begin(), numbers.begin() + no_outs, m_which_outs.begin());
    } else {
        m_num_outs = m_total_num_vehicles;
    }
    // Allocate memory for the csv_writers
    m_csv_writers_ptr = std::make_unique<CSV_writer[]>(m_num_outs);
    m_output_file = output_file;
    m_dtout = 1.0 / output_freq;

    // Here we also initialize the device and host arrays that store the response across vehicles and states -> If
    // output is not required, then nothing is stored, however the user has access to the states at the last time step
    // through the simState and simData structs

    // Number of time steps to be collected on the device
    m_device_collection_timeSteps = ceil(m_kernel_sim_time / m_dtout);

    // Number of states to store -> For now we only allow storage of the major states which are common to both tire
    // tire models [time,x,y,u,v,phi,psi,theta,wx,wz,wy,lf_omega,rf_omega,lr_omega,rr_omega]
    m_collection_states = 15;

    // Thus device array size becomes
    m_device_size = sizeof(double) * m_total_num_vehicles * m_collection_states * (m_device_collection_timeSteps);

    CHECK_CUDA_ERROR(cudaMalloc((void**)&m_device_response, m_device_size));

    // Now the host response
    m_host_collection_timeSteps = ceil(m_host_dump_time / m_dtout);

    // Thus the host size becomes -> Usually much larger than the device size
    m_host_response = new double[m_total_num_vehicles * m_collection_states * (m_host_collection_timeSteps)]();
}

// ======================================================================================================================
__host__ void d24SolverHalfImplicitGPU::Solve() {
    assert(m_tend != 0. && "Final time not set, please use SetEndTime function");
    // Calculate the number of blocks required
    m_num_blocks = (m_total_num_vehicles + m_threads_per_block - 1) / m_threads_per_block;
#ifdef DEBUG
    std::cout << "Number of blocks: " << m_num_blocks << std::endl;
    std::cout << "Number of threads per block: " << m_threads_per_block << std::endl;
    std::cout << "Total number of vehicles: " << m_total_num_vehicles << std::endl;
#endif
    // If m_output is false, then we still need to initialize device array -> We don't need the host array as its
    // purpose is just to store the final output
    if (!m_output) {
        // Number of time steps to be collected on the device
        m_device_collection_timeSteps = ceil(m_kernel_sim_time / m_dtout);

        // Number of states to store -> For now we only allow storage of the major states which are common to both tire
        // tire models [time,x,y,u,v,phi,psi,theta,wx,wz,wy,lf_omega,rf_omega,lr_omega,rr_omega]
        m_collection_states = 15;

        // Thus device array size becomes
        m_device_size = sizeof(double) * m_total_num_vehicles * m_collection_states * (m_device_collection_timeSteps);

        CHECK_CUDA_ERROR(cudaMalloc((void**)&m_device_response, m_device_size));
    }

    double current_time = 0.;
    unsigned int kernel_launches_since_last_dump = 0;  // Track the number of kernel launches since the last dump of the
                                                       // host response
    double time_since_last_dump = 0.;                  // Track the time since the last dump of the host response
    // Write the initial conditions
    if (m_output) {
        Write(current_time);
    }

    while (current_time < m_tend) {
        // Calculate when this kernel is supposed to end

        double kernel_end_time = current_time + m_kernel_sim_time;

        // Launch the kernel
        if (m_tire_type == TireType::TMeasy) {
            Integrate<<<m_num_blocks, m_threads_per_block>>>(current_time, m_kernel_sim_time, m_step, m_output,
                                                             m_total_num_vehicles, m_collection_states, m_dtout,
                                                             m_device_response, m_sim_data, m_sim_states);
            cudaError_t err = cudaGetLastError();
            if (err != cudaSuccess) {
                fprintf(stderr, "Kernel launch failed: %s\n", cudaGetErrorString(err));
            }
        } else {
            Integrate<<<m_num_blocks, m_threads_per_block>>>(current_time, m_kernel_sim_time, m_step, m_output,
                                                             m_total_num_vehicles, m_collection_states, m_dtout,
                                                             m_device_response, m_sim_data_nr, m_sim_states_nr);
            cudaError_t err = cudaGetLastError();
            if (err != cudaSuccess) {
                fprintf(stderr, "Kernel launch failed: %s\n", cudaGetErrorString(err));
            }
        }

        // Get the new time the simulation has reached
        current_time = kernel_end_time;
        time_since_last_dump += m_kernel_sim_time;

        // If we have to save output, copy over the device into the response
        if (m_output) {
            // Amount of respoonse already filled
            unsigned int filled_response = m_total_num_vehicles * m_collection_states * m_device_collection_timeSteps *
                                           kernel_launches_since_last_dump;

            // Copy the device to the right part of the host response
            CHECK_CUDA_ERROR(cudaMemcpy(m_host_response + filled_response, m_device_response, m_device_size,
                                        cudaMemcpyDeviceToHost));

            kernel_launches_since_last_dump++;
            // Check if host is full and dump that into a csv writer
            if (abs(time_since_last_dump - m_host_dump_time) < 1e-6) {
                Write(current_time);
                time_since_last_dump = 0.;
                kernel_launches_since_last_dump = 0;
            }
        }
    }
    // If the simulation ended at a non multiple of m_host_dump_time, we dump the remaining data
    if (m_output && (kernel_launches_since_last_dump != 0)) {
        unsigned int time_steps_to_write = kernel_launches_since_last_dump * m_device_collection_timeSteps;
        Write(current_time, time_steps_to_write);
    }
    // End of simulation, write to the csv file
    if (m_output) {
        WriteToFile();
    }
}
// ======================================================================================================================
__host__ double d24SolverHalfImplicitGPU::SolveStep(double t,
                                                    double steering,
                                                    double throttle,
                                                    double braking) {  // Calculate the number of blocks required
    // if m_output is true, then raise assertion
    if (m_output) {
        // Handle the error: log, return an error code, etc.
        std::cerr << "Cannot get csv file output if SolveStep is called, please access sim_states through GetSimSate"
                  << std::endl;
        exit(EXIT_FAILURE);
    }
    m_num_blocks = (m_total_num_vehicles + m_threads_per_block - 1) / m_threads_per_block;
    // If m_output is false and its the first time step then we still need to initialize device array -> We don't need
    // the host array as its purpose is just to store the final output
    if (t == 0.) {
        // Number of time steps to be collected on the device
        m_device_collection_timeSteps = ceil(m_kernel_sim_time / m_dtout);

        // Number of states to store -> For now we only allow storage of the major states which are common to both tire
        // tire models [time,x,y,u,v,phi,psi,theta,wx,wz,wy,lf_omega,rf_omega,lr_omega,rr_omega]
        m_collection_states = 15;

        // Thus device array size becomes
        m_device_size = sizeof(double) * m_total_num_vehicles * m_collection_states * (m_device_collection_timeSteps);

        CHECK_CUDA_ERROR(cudaMalloc((void**)&m_device_response, m_device_size));
    }
    m_current_time = t;

    // Launch the kernel
    double kernel_end_time = m_current_time + m_kernel_sim_time;

    if (m_tire_type == TireType::TMeasy) {
        Integrate<<<m_num_blocks, m_threads_per_block>>>(m_current_time, steering, throttle, braking, m_kernel_sim_time,
                                                         m_step, m_output, m_total_num_vehicles, m_collection_states,
                                                         m_dtout, m_device_response, m_sim_data, m_sim_states);
        cudaError_t err = cudaGetLastError();
        if (err != cudaSuccess) {
            fprintf(stderr, "Kernel launch failed: %s\n", cudaGetErrorString(err));
        }
    } else {
        Integrate<<<m_num_blocks, m_threads_per_block>>>(m_current_time, steering, throttle, braking, m_kernel_sim_time,
                                                         m_step, m_output, m_total_num_vehicles, m_collection_states,
                                                         m_dtout, m_device_response, m_sim_data_nr, m_sim_states_nr);
        cudaError_t err = cudaGetLastError();
        if (err != cudaSuccess) {
            fprintf(stderr, "Kernel launch failed: %s\n", cudaGetErrorString(err));
        }
    }

    m_current_time = kernel_end_time;
    m_time_since_last_dump += m_kernel_sim_time;
    return m_current_time;
}
// ======================================================================================================================
__host__ void d24SolverHalfImplicitGPU::Write(double t, unsigned int time_steps_to_write) {
    unsigned int loop_limit = 0;
    if (m_store_all) {
        loop_limit = m_total_num_vehicles;
    } else {
        loop_limit = m_num_outs;
    }

    // If time_steps_to_write is not specified, we write all the data
    if (time_steps_to_write == 0) {
        time_steps_to_write = m_host_collection_timeSteps;
    }

    if (t < m_step) {
        for (unsigned int sim_no = 0; sim_no < loop_limit; sim_no++) {
            CSV_writer& csv = m_csv_writers_ptr[sim_no];
            csv << "time";
            csv << "x";
            csv << "y";
            csv << "vx";
            csv << "vy";
            csv << "roll";
            csv << "yaw";
            csv << "pitch";
            csv << "roll_rate";
            csv << "yaw_rate";
            csv << "pitch_rate";
            csv << "wlf";
            csv << "wrf";
            csv << "wlr";
            csv << "wrr";
            csv << std::endl;

            csv << 0;
            csv << 0;
            csv << 0;
            csv << 0;
            csv << 0;
            csv << 0;
            csv << 0;
            csv << 0;
            csv << 0;
            csv << 0;
            csv << 0;
            csv << 0;
            csv << 0;
            csv << 0;
            csv << 0;
            csv << std::endl;
        }
        return;
    } else {
        for (unsigned int sim_no = 0; sim_no < loop_limit; sim_no++) {
            unsigned int index_by = 0;
            // If we are no storing all, we will have to index by random numbers
            if (m_store_all) {
                index_by = sim_no;
            } else {
                index_by = m_which_outs[sim_no];
            }
            CSV_writer& csv = m_csv_writers_ptr[sim_no];
            unsigned int steps_written = 0;
            while (steps_written < time_steps_to_write) {
                unsigned int time_offset = steps_written * m_total_num_vehicles * m_collection_states;
                csv << m_host_response[time_offset + (m_total_num_vehicles * 0) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 1) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 2) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 3) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 4) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 5) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 6) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 7) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 8) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 9) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 10) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 11) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 12) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 13) + index_by];
                csv << m_host_response[time_offset + (m_total_num_vehicles * 14) + index_by];
                csv << std::endl;
                steps_written++;
            }
        }
    }
}

__host__ void d24SolverHalfImplicitGPU::WriteToFile() {
    if (!m_output) {
        std::cout << "No output file specified. Call SetOutput() before calling WriteToFile()" << std::endl;
        return;
    }
    unsigned int loop_limit = 0;
    if (m_store_all) {
        loop_limit = m_total_num_vehicles;
    } else {
        loop_limit = m_num_outs;
    }
    for (unsigned int sim_no = 0; sim_no < loop_limit; sim_no++) {
        unsigned int index_by = 0;
        // If we are no storing all, we will have to index by random numbers
        if (m_store_all) {
            index_by = sim_no;
        } else {
            index_by = m_which_outs[sim_no];
        }
        if (m_tire_type == TireType::TMeasy) {
            CSV_writer& csv = m_csv_writers_ptr[sim_no];
            csv.write_to_file(m_output_file + "_" + std::to_string(index_by) + ".csv");
            csv.clearData();
        } else {
            CSV_writer& csv = m_csv_writers_ptr[sim_no];
            csv.write_to_file(m_output_file + "_" + std::to_string(index_by) + ".csv");
            csv.clearData();
        }
    }
}
//======================================================================================================================

__host__ SimState d24SolverHalfImplicitGPU::GetSimState(unsigned int vehicle_index) {
    assert((vehicle_index < m_total_num_vehicles) && "Vehicle index out of bounds");

    // Allocate space for a single SimState on the host
    SimState host_state;

    if (m_tire_type == TireType::TMeasy) {
        // Copy the specific SimState from the GPU to the host
        cudaMemcpy(&host_state, &m_sim_states[vehicle_index], sizeof(SimState), cudaMemcpyDeviceToHost);
    } else {
        // Similarly for m_sim_states_nr
        cudaMemcpy(&host_state, &m_sim_states_nr[vehicle_index], sizeof(SimState), cudaMemcpyDeviceToHost);
    }

    return host_state;
}

// ======================================================================================================================
__device__ void rhsFun(double t, unsigned int total_num_vehicles, SimData* sim_data, SimState* sim_states) {
    // Get the vehicle index
    unsigned int vehicle_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (vehicle_index < total_num_vehicles) {
        // All vehicles have one or the other tire type and thus no thread divergence
        VehicleParam& veh_params = sim_data[vehicle_index]._veh_params;
        VehicleState& v_states = sim_states[vehicle_index]._v_states;
        TMeasyParam& tireTM_params = sim_data[vehicle_index]._tireTM_params;
        TMeasyState& tireTMlf_st = sim_states[vehicle_index]._tirelf_st;
        TMeasyState& tireTMrf_st = sim_states[vehicle_index]._tirerf_st;
        TMeasyState& tireTMlr_st = sim_states[vehicle_index]._tirelr_st;
        TMeasyState& tireTMrr_st = sim_states[vehicle_index]._tirerr_st;
        SuspensionParam& sus_params = sim_data[vehicle_index]._sus_params;
        SuspensionState& suslf_st = sim_states[vehicle_index]._suslf_st;
        SuspensionState& susrf_st = sim_states[vehicle_index]._susrf_st;
        SuspensionState& suslr_st = sim_states[vehicle_index]._suslr_st;
        SuspensionState& susrr_st = sim_states[vehicle_index]._susrr_st;

        DriverInput* driver_data = sim_data[vehicle_index]._driver_data;
        unsigned int len = sim_data[vehicle_index]._driver_data_len;
        // Get controls at the current timeStep
        auto controls = GetDriverInput(t, driver_data, len);

        vehToSusTransform(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                          &suslr_st, &susrr_st, &veh_params, controls.m_steering);
        vehToTireTransform(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                           &suslr_st, &susrr_st, &veh_params, controls.m_steering);
        // Tire velocities
        computeTireRHS(&v_states, &tireTMlf_st, &veh_params, &tireTM_params, controls.m_steering);
        computeTireRHS(&v_states, &tireTMrf_st, &veh_params, &tireTM_params, controls.m_steering);
        computeTireRHS(&v_states, &tireTMlr_st, &veh_params, &tireTM_params, 0);  // No rear steering
        computeTireRHS(&v_states, &tireTMrr_st, &veh_params, &tireTM_params, 0);  // No rear steering
        // compute the tire compression velocity which is to be integrated
        computeTireCompressionVelocity(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st,
                                       &susrf_st, &suslr_st, &susrr_st);
        // Compute the powertrain RHS which provides omegas for tires
        computePowertrainRHS(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &veh_params,
                             &tireTM_params, &controls);

        // Tire to vehicle transform to get the tire forces in right frame
        // for suspension RHS
        tireToVehTransform(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &veh_params,
                           controls.m_steering);
        // Suspension velocities
        computeSusRHS(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                      &suslr_st, &susrr_st, &veh_params, &sus_params);

        // Transform the forces from the tire along with suspension velocities to forces on
        // the chassis
        computeForcesThroughSus(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                                &suslr_st, &susrr_st, &veh_params, &tireTM_params, &sus_params);

        // Compute the chassis accelerations
        computeVehicleRHS(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                          &suslr_st, &susrr_st, &veh_params, &tireTM_params, &sus_params);
    }
}

__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d24::SimData* sim_data,
                       d24::SimState* sim_states,
                       double steering,
                       double throttle,
                       double braking) {  // Get the vehicle index
    unsigned int vehicle_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (vehicle_index < total_num_vehicles) {
        // All vehicles have one or the other tire type and thus no thread divergence
        VehicleParam& veh_params = sim_data[vehicle_index]._veh_params;
        VehicleState& v_states = sim_states[vehicle_index]._v_states;
        TMeasyParam& tireTM_params = sim_data[vehicle_index]._tireTM_params;
        TMeasyState& tireTMlf_st = sim_states[vehicle_index]._tirelf_st;
        TMeasyState& tireTMrf_st = sim_states[vehicle_index]._tirerf_st;
        TMeasyState& tireTMlr_st = sim_states[vehicle_index]._tirelr_st;
        TMeasyState& tireTMrr_st = sim_states[vehicle_index]._tirerr_st;
        SuspensionParam& sus_params = sim_data[vehicle_index]._sus_params;
        SuspensionState& suslf_st = sim_states[vehicle_index]._suslf_st;
        SuspensionState& susrf_st = sim_states[vehicle_index]._susrf_st;
        SuspensionState& suslr_st = sim_states[vehicle_index]._suslr_st;
        SuspensionState& susrr_st = sim_states[vehicle_index]._susrr_st;

        // Get controls at the current timeStep
        DriverInput controls;
        controls.m_steering = steering;
        controls.m_throttle = throttle;
        controls.m_braking = braking;

        vehToSusTransform(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                          &suslr_st, &susrr_st, &veh_params, controls.m_steering);
        vehToTireTransform(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                           &suslr_st, &susrr_st, &veh_params, controls.m_steering);
        // Tire velocities
        computeTireRHS(&v_states, &tireTMlf_st, &veh_params, &tireTM_params, controls.m_steering);
        computeTireRHS(&v_states, &tireTMrf_st, &veh_params, &tireTM_params, controls.m_steering);
        computeTireRHS(&v_states, &tireTMlr_st, &veh_params, &tireTM_params, 0);  // No rear steering
        computeTireRHS(&v_states, &tireTMrr_st, &veh_params, &tireTM_params, 0);  // No rear steering
        // compute the tire compression velocity which is to be integrated
        computeTireCompressionVelocity(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st,
                                       &susrf_st, &suslr_st, &susrr_st);
        // Compute the powertrain RHS which provides omegas for tires
        computePowertrainRHS(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &veh_params,
                             &tireTM_params, &controls);

        // Tire to vehicle transform to get the tire forces in right frame
        // for suspension RHS
        tireToVehTransform(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &veh_params,
                           controls.m_steering);
        // Suspension velocities
        computeSusRHS(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                      &suslr_st, &susrr_st, &veh_params, &sus_params);

        // Transform the forces from the tire along with suspension velocities to forces on
        // the chassis
        computeForcesThroughSus(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                                &suslr_st, &susrr_st, &veh_params, &tireTM_params, &sus_params);

        // Compute the chassis accelerations
        computeVehicleRHS(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                          &suslr_st, &susrr_st, &veh_params, &tireTM_params, &sus_params);
    }
}

//======================================================================================================================

__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d24::SimDataNr* sim_data_nr,
                       d24::SimStateNr* sim_states_nr) {  // Get the vehicle index
    unsigned int vehicle_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (vehicle_index < total_num_vehicles) {
        // All vehicles have one or the other tire type and thus no thread divergence
        VehicleParam& veh_params = sim_data_nr[vehicle_index]._veh_params;
        VehicleState& v_states = sim_states_nr[vehicle_index]._v_states;
        TMeasyNrParam& tireTM_params = sim_data_nr[vehicle_index]._tireTMNr_params;
        TMeasyNrState& tireTMlf_st = sim_states_nr[vehicle_index]._tirelf_st;
        TMeasyNrState& tireTMrf_st = sim_states_nr[vehicle_index]._tirerf_st;
        TMeasyNrState& tireTMlr_st = sim_states_nr[vehicle_index]._tirelr_st;
        TMeasyNrState& tireTMrr_st = sim_states_nr[vehicle_index]._tirerr_st;
        SuspensionParam& sus_params = sim_data_nr[vehicle_index]._sus_params;
        SuspensionState& suslf_st = sim_states_nr[vehicle_index]._suslf_st;
        SuspensionState& susrf_st = sim_states_nr[vehicle_index]._susrf_st;
        SuspensionState& suslr_st = sim_states_nr[vehicle_index]._suslr_st;
        SuspensionState& susrr_st = sim_states_nr[vehicle_index]._susrr_st;

        DriverInput* driver_data = sim_data_nr[vehicle_index]._driver_data;
        unsigned int len = sim_data_nr[vehicle_index]._driver_data_len;
        // Get controls at the current timeStep
        auto controls = GetDriverInput(t, driver_data, len);

        vehToSusTransform(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                          &suslr_st, &susrr_st, &veh_params, controls.m_steering);
        vehToTireTransform(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                           &suslr_st, &susrr_st, &veh_params, controls.m_steering);
        // Tire velocities
        computeTireRHS(&v_states, &tireTMlf_st, &veh_params, &tireTM_params, controls.m_steering);
        computeTireRHS(&v_states, &tireTMrf_st, &veh_params, &tireTM_params, controls.m_steering);
        computeTireRHS(&v_states, &tireTMlr_st, &veh_params, &tireTM_params, 0);  // No rear steering
        computeTireRHS(&v_states, &tireTMrr_st, &veh_params, &tireTM_params, 0);  // No rear steering
        // compute the tire compression velocity which is to be integrated
        computeTireCompressionVelocity(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st,
                                       &susrf_st, &suslr_st, &susrr_st);
        // Compute the powertrain RHS which provides omegas for tires
        computePowertrainRHS(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &veh_params,
                             &tireTM_params, &controls);

        // Tire to vehicle transform to get the tire forces in right frame
        // for suspension RHS
        tireToVehTransform(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &veh_params,
                           controls.m_steering);
        // Suspension velocities
        computeSusRHS(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                      &suslr_st, &susrr_st, &veh_params, &sus_params);

        // Transform the forces from the tire along with suspension velocities to forces on
        // the chassis
        computeForcesThroughSus(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                                &suslr_st, &susrr_st, &veh_params, &tireTM_params, &sus_params);

        // Compute the chassis accelerations
        computeVehicleRHS(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                          &suslr_st, &susrr_st, &veh_params, &tireTM_params, &sus_params);
    }
}

__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d24::SimDataNr* sim_data_nr,
                       d24::SimStateNr* sim_states_nr,
                       double steering,
                       double throttle,
                       double braking) {
    unsigned int vehicle_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (vehicle_index < total_num_vehicles) {
        // All vehicles have one or the other tire type and thus no thread divergence
        VehicleParam& veh_params = sim_data_nr[vehicle_index]._veh_params;
        VehicleState& v_states = sim_states_nr[vehicle_index]._v_states;
        TMeasyNrParam& tireTM_params = sim_data_nr[vehicle_index]._tireTMNr_params;
        TMeasyNrState& tireTMlf_st = sim_states_nr[vehicle_index]._tirelf_st;
        TMeasyNrState& tireTMrf_st = sim_states_nr[vehicle_index]._tirerf_st;
        TMeasyNrState& tireTMlr_st = sim_states_nr[vehicle_index]._tirelr_st;
        TMeasyNrState& tireTMrr_st = sim_states_nr[vehicle_index]._tirerr_st;
        SuspensionParam& sus_params = sim_data_nr[vehicle_index]._sus_params;
        SuspensionState& suslf_st = sim_states_nr[vehicle_index]._suslf_st;
        SuspensionState& susrf_st = sim_states_nr[vehicle_index]._susrf_st;
        SuspensionState& suslr_st = sim_states_nr[vehicle_index]._suslr_st;
        SuspensionState& susrr_st = sim_states_nr[vehicle_index]._susrr_st;

        // Get controls at the current timeStep
        DriverInput controls;
        controls.m_steering = steering;
        controls.m_throttle = throttle;
        controls.m_braking = braking;

        vehToSusTransform(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                          &suslr_st, &susrr_st, &veh_params, controls.m_steering);
        vehToTireTransform(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                           &suslr_st, &susrr_st, &veh_params, controls.m_steering);
        // Tire velocities
        computeTireRHS(&v_states, &tireTMlf_st, &veh_params, &tireTM_params, controls.m_steering);
        computeTireRHS(&v_states, &tireTMrf_st, &veh_params, &tireTM_params, controls.m_steering);
        computeTireRHS(&v_states, &tireTMlr_st, &veh_params, &tireTM_params, 0);  // No rear steering
        computeTireRHS(&v_states, &tireTMrr_st, &veh_params, &tireTM_params, 0);  // No rear steering
        // compute the tire compression velocity which is to be integrated
        computeTireCompressionVelocity(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st,
                                       &susrf_st, &suslr_st, &susrr_st);
        // Compute the powertrain RHS which provides omegas for tires
        computePowertrainRHS(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &veh_params,
                             &tireTM_params, &controls);

        // Tire to vehicle transform to get the tire forces in right frame
        // for suspension RHS
        tireToVehTransform(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &veh_params,
                           controls.m_steering);
        // Suspension velocities
        computeSusRHS(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                      &suslr_st, &susrr_st, &veh_params, &sus_params);

        // Transform the forces from the tire along with suspension velocities to forces on
        // the chassis
        computeForcesThroughSus(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                                &suslr_st, &susrr_st, &veh_params, &tireTM_params, &sus_params);

        // Compute the chassis accelerations
        computeVehicleRHS(&v_states, &tireTMlf_st, &tireTMrf_st, &tireTMlr_st, &tireTMrr_st, &suslf_st, &susrf_st,
                          &suslr_st, &susrr_st, &veh_params, &tireTM_params, &sus_params);
    }
}

//======================================================================================================================

__global__ void Integrate(double current_time,
                          double kernel_sim_time,
                          double step,
                          bool output,
                          unsigned int total_num_vehicles,
                          unsigned int collection_states,
                          double dtout,
                          double* device_response,
                          SimData* sim_data,
                          SimState* sim_states) {
    double t = current_time;           // Set the current time
    double kernel_time = 0;            // Time since kernel was launched
    unsigned int timeStep_stored = 0;  // Number of time steps already stored in the device response
    double end_time = (t + kernel_sim_time) - step / 10.;
    unsigned int vehicle_id = blockIdx.x * blockDim.x + threadIdx.x;  // Get the vehicle id
    if (vehicle_id < total_num_vehicles) {
        while (t < end_time) {
            // Call the RHS to get accelerations for all the vehicles
            rhsFun(t, total_num_vehicles, sim_data, sim_states);

            // Integrate according to half implicit method for second order states
            // Integrate according to explicit method for first order states

            // Extract the states of the vehicle and the tires
            VehicleState& v_states = sim_states[vehicle_id]._v_states;
            VehicleParam& veh_param = sim_data[vehicle_id]._veh_params;
            TMeasyState& tirelf_st = sim_states[vehicle_id]._tirelf_st;
            TMeasyState& tirerf_st = sim_states[vehicle_id]._tirerf_st;
            TMeasyState& tirelr_st = sim_states[vehicle_id]._tirelr_st;
            TMeasyState& tirerr_st = sim_states[vehicle_id]._tirerr_st;
            SuspensionState& suslf_st = sim_states[vehicle_id]._suslf_st;
            SuspensionState& susrf_st = sim_states[vehicle_id]._susrf_st;
            SuspensionState& suslr_st = sim_states[vehicle_id]._suslr_st;
            SuspensionState& susrr_st = sim_states[vehicle_id]._susrr_st;

            // First the tire states
            // LF
            tirelf_st._xe += tirelf_st._xedot * step;
            tirelf_st._ye += tirelf_st._yedot * step;
            tirelf_st._xt += tirelf_st._dxt * step;
            tirelf_st._omega += tirelf_st._dOmega * step;
            // RF
            tirerf_st._xe += tirerf_st._xedot * step;
            tirerf_st._ye += tirerf_st._yedot * step;
            tirerf_st._xt += tirerf_st._dxt * step;
            tirerf_st._omega += tirerf_st._dOmega * step;
            // LR
            tirelr_st._xe += tirelr_st._xedot * step;
            tirelr_st._ye += tirelr_st._yedot * step;
            tirelr_st._xt += tirelr_st._dxt * step;
            tirelr_st._omega += tirelr_st._dOmega * step;
            // RR
            tirerr_st._xe += tirerr_st._xedot * step;
            tirerr_st._ye += tirerr_st._yedot * step;
            tirerr_st._xt += tirerr_st._dxt * step;
            tirerr_st._omega += tirerr_st._dOmega * step;

            // Integrate the suspension states
            suslf_st._wu += suslf_st._dwu * step;
            suslf_st._xs += suslf_st._dxs * step;

            susrf_st._wu += susrf_st._dwu * step;
            susrf_st._xs += susrf_st._dxs * step;

            suslr_st._wu += suslr_st._dwu * step;
            suslr_st._xs += suslr_st._dxs * step;

            susrr_st._wu += susrr_st._dwu * step;
            susrr_st._xs += susrr_st._dxs * step;

            // Now the vehicle states
            if (veh_param._tcbool) {
                v_states._crankOmega += v_states._dOmega_crank * step;
            }

            v_states._u += v_states._udot * step;
            v_states._v += v_states._vdot * step;
            v_states._w += v_states._wdot * step;
            v_states._wx += v_states._wxdot * step;
            v_states._wy += v_states._wydot * step;
            v_states._wz += v_states._wzdot * step;

            // We have to recompute this here so that we are doing the half implicit
            // Now to integrate the cardan angles, we first write equations for them
            v_states._dtheta = v_states._wy * std::cos(v_states._phi) - v_states._wz * std::sin(v_states._phi);
            v_states._dpsi =
                (v_states._wy * std::sin(v_states._phi) / std::cos(v_states._theta)) +
                (v_states._wz * std::cos(v_states._phi) / std::cos(v_states._theta));  // Trouble when theta is 90?
            v_states._dphi = v_states._wx + v_states._wy * std::sin(v_states._phi) * std::tan(v_states._theta) +
                             v_states._wz * std::cos(v_states._phi) * std::tan(v_states._theta);

            v_states._theta += v_states._dtheta * step;
            v_states._psi += v_states._dpsi * step;
            v_states._phi += v_states._dphi * step;

            // This computation is also repeated because of half implicit
            // Update global position
            v_states._x += (v_states._u * std::cos(v_states._psi) - v_states._v * std::sin(v_states._psi)) * step;
            v_states._y += (v_states._u * std::sin(v_states._psi) + v_states._v * std::cos(v_states._psi)) * step;

            // Update time
            t += step;
            kernel_time += step;

            // Write to response if required -> regardless of no_outs or store_all we write all the vehicles to the
            // response
            if (output) {
                // The +1 here is because state at time 0 is not stored in device response
                if (abs(kernel_time - (timeStep_stored + 1) * dtout) < 1e-7) {
                    unsigned int time_offset = timeStep_stored * total_num_vehicles * collection_states;

                    device_response[time_offset + (total_num_vehicles * 0) + vehicle_id] = t;
                    device_response[time_offset + (total_num_vehicles * 1) + vehicle_id] = v_states._x;
                    device_response[time_offset + (total_num_vehicles * 2) + vehicle_id] = v_states._y;
                    device_response[time_offset + (total_num_vehicles * 3) + vehicle_id] = v_states._u;
                    device_response[time_offset + (total_num_vehicles * 4) + vehicle_id] = v_states._v;
                    device_response[time_offset + (total_num_vehicles * 5) + vehicle_id] = v_states._phi;
                    device_response[time_offset + (total_num_vehicles * 6) + vehicle_id] = v_states._psi;
                    device_response[time_offset + (total_num_vehicles * 7) + vehicle_id] = v_states._theta;
                    device_response[time_offset + (total_num_vehicles * 8) + vehicle_id] = v_states._wx;
                    device_response[time_offset + (total_num_vehicles * 9) + vehicle_id] = v_states._wz;
                    device_response[time_offset + (total_num_vehicles * 10) + vehicle_id] = v_states._wy;
                    device_response[time_offset + (total_num_vehicles * 11) + vehicle_id] = tirelf_st._omega;
                    device_response[time_offset + (total_num_vehicles * 12) + vehicle_id] = tirerf_st._omega;
                    device_response[time_offset + (total_num_vehicles * 13) + vehicle_id] = tirelr_st._omega;
                    device_response[time_offset + (total_num_vehicles * 14) + vehicle_id] = tirerr_st._omega;
                    timeStep_stored++;
                }
            }
        }
    }
}

__global__ void Integrate(double current_time,
                          double steering,
                          double throttle,
                          double braking,
                          double kernel_sim_time,
                          double step,
                          bool output,
                          unsigned int total_num_vehicles,
                          unsigned int collection_states,
                          double dtout,
                          double* device_response,
                          d24::SimData* sim_data,
                          d24::SimState* sim_states) {
    double t = current_time;           // Set the current time
    double kernel_time = 0;            // Time since kernel was launched
    unsigned int timeStep_stored = 0;  // Number of time steps already stored in the device response
    double end_time = (t + kernel_sim_time) - step / 10.;
    unsigned int vehicle_id = blockIdx.x * blockDim.x + threadIdx.x;  // Get the vehicle id
    if (vehicle_id < total_num_vehicles) {
        while (t < end_time) {
            // Call the RHS to get accelerations for all the vehicles
            rhsFun(t, total_num_vehicles, sim_data, sim_states, steering, throttle, braking);

            // Integrate according to half implicit method for second order states
            // Integrate according to explicit method for first order states

            // Extract the states of the vehicle and the tires
            VehicleState& v_states = sim_states[vehicle_id]._v_states;
            VehicleParam& veh_param = sim_data[vehicle_id]._veh_params;
            TMeasyState& tirelf_st = sim_states[vehicle_id]._tirelf_st;
            TMeasyState& tirerf_st = sim_states[vehicle_id]._tirerf_st;
            TMeasyState& tirelr_st = sim_states[vehicle_id]._tirelr_st;
            TMeasyState& tirerr_st = sim_states[vehicle_id]._tirerr_st;
            SuspensionState& suslf_st = sim_states[vehicle_id]._suslf_st;
            SuspensionState& susrf_st = sim_states[vehicle_id]._susrf_st;
            SuspensionState& suslr_st = sim_states[vehicle_id]._suslr_st;
            SuspensionState& susrr_st = sim_states[vehicle_id]._susrr_st;

            // First the tire states
            // LF
            tirelf_st._xe += tirelf_st._xedot * step;
            tirelf_st._ye += tirelf_st._yedot * step;
            tirelf_st._xt += tirelf_st._dxt * step;
            tirelf_st._omega += tirelf_st._dOmega * step;
            // RF
            tirerf_st._xe += tirerf_st._xedot * step;
            tirerf_st._ye += tirerf_st._yedot * step;
            tirerf_st._xt += tirerf_st._dxt * step;
            tirerf_st._omega += tirerf_st._dOmega * step;
            // LR
            tirelr_st._xe += tirelr_st._xedot * step;
            tirelr_st._ye += tirelr_st._yedot * step;
            tirelr_st._xt += tirelr_st._dxt * step;
            tirelr_st._omega += tirelr_st._dOmega * step;
            // RR
            tirerr_st._xe += tirerr_st._xedot * step;
            tirerr_st._ye += tirerr_st._yedot * step;
            tirerr_st._xt += tirerr_st._dxt * step;
            tirerr_st._omega += tirerr_st._dOmega * step;

            // Integrate the suspension states
            suslf_st._wu += suslf_st._dwu * step;
            suslf_st._xs += suslf_st._dxs * step;

            susrf_st._wu += susrf_st._dwu * step;
            susrf_st._xs += susrf_st._dxs * step;

            suslr_st._wu += suslr_st._dwu * step;
            suslr_st._xs += suslr_st._dxs * step;

            susrr_st._wu += susrr_st._dwu * step;
            susrr_st._xs += susrr_st._dxs * step;

            // Now the vehicle states
            if (veh_param._tcbool) {
                v_states._crankOmega += v_states._dOmega_crank * step;
            }

            v_states._u += v_states._udot * step;
            v_states._v += v_states._vdot * step;
            v_states._w += v_states._wdot * step;
            v_states._wx += v_states._wxdot * step;
            v_states._wy += v_states._wydot * step;
            v_states._wz += v_states._wzdot * step;

            // We have to recompute this here so that we are doing the half implicit
            // Now to integrate the cardan angles, we first write equations for them
            v_states._dtheta = v_states._wy * std::cos(v_states._phi) - v_states._wz * std::sin(v_states._phi);
            v_states._dpsi =
                (v_states._wy * std::sin(v_states._phi) / std::cos(v_states._theta)) +
                (v_states._wz * std::cos(v_states._phi) / std::cos(v_states._theta));  // Trouble when theta is 90?
            v_states._dphi = v_states._wx + v_states._wy * std::sin(v_states._phi) * std::tan(v_states._theta) +
                             v_states._wz * std::cos(v_states._phi) * std::tan(v_states._theta);

            v_states._theta += v_states._dtheta * step;
            v_states._psi += v_states._dpsi * step;
            v_states._phi += v_states._dphi * step;

            // This computation is also repeated because of half implicit
            // Update global position
            v_states._x += (v_states._u * std::cos(v_states._psi) - v_states._v * std::sin(v_states._psi)) * step;
            v_states._y += (v_states._u * std::sin(v_states._psi) + v_states._v * std::cos(v_states._psi)) * step;

            // Update time
            t += step;
            kernel_time += step;

            // Write to response if required -> regardless of no_outs or store_all we write all the vehicles to the
            // response
            if (output) {
                // The +1 here is because state at time 0 is not stored in device response
                if (abs(kernel_time - (timeStep_stored + 1) * dtout) < 1e-7) {
                    unsigned int time_offset = timeStep_stored * total_num_vehicles * collection_states;

                    device_response[time_offset + (total_num_vehicles * 0) + vehicle_id] = t;
                    device_response[time_offset + (total_num_vehicles * 1) + vehicle_id] = v_states._x;
                    device_response[time_offset + (total_num_vehicles * 2) + vehicle_id] = v_states._y;
                    device_response[time_offset + (total_num_vehicles * 3) + vehicle_id] = v_states._u;
                    device_response[time_offset + (total_num_vehicles * 4) + vehicle_id] = v_states._v;
                    device_response[time_offset + (total_num_vehicles * 5) + vehicle_id] = v_states._phi;
                    device_response[time_offset + (total_num_vehicles * 6) + vehicle_id] = v_states._psi;
                    device_response[time_offset + (total_num_vehicles * 7) + vehicle_id] = v_states._theta;
                    device_response[time_offset + (total_num_vehicles * 8) + vehicle_id] = v_states._wx;
                    device_response[time_offset + (total_num_vehicles * 9) + vehicle_id] = v_states._wz;
                    device_response[time_offset + (total_num_vehicles * 10) + vehicle_id] = v_states._wy;
                    device_response[time_offset + (total_num_vehicles * 11) + vehicle_id] = tirelf_st._omega;
                    device_response[time_offset + (total_num_vehicles * 12) + vehicle_id] = tirerf_st._omega;
                    device_response[time_offset + (total_num_vehicles * 13) + vehicle_id] = tirelr_st._omega;
                    device_response[time_offset + (total_num_vehicles * 14) + vehicle_id] = tirerr_st._omega;
                    timeStep_stored++;
                }
            }
        }
    }
}

//======================================================================================================================
__global__ void Integrate(double current_time,
                          double kernel_sim_time,
                          double step,
                          bool output,
                          unsigned int total_num_vehicles,
                          unsigned int collection_states,
                          double dtout,
                          double* device_response,
                          d24::SimDataNr* sim_data_nr,
                          d24::SimStateNr* sim_states_nr) {
    double t = current_time;           // Set the current time
    double kernel_time = 0;            // Time since kernel was launched
    unsigned int timeStep_stored = 0;  // Number of time steps already stored in the device response
    double end_time = (t + kernel_sim_time) - step / 10.;
    unsigned int vehicle_id = blockIdx.x * blockDim.x + threadIdx.x;  // Get the vehicle id
    if (vehicle_id < total_num_vehicles) {
        while (t < end_time) {
            // Call the RHS to get accelerations for all the vehicles
            rhsFun(t, total_num_vehicles, sim_data_nr, sim_states_nr);

            // Integrate according to half implicit method for second order states
            // Integrate according to explicit method for first order states

            // Extract the states of the vehicle and the tires
            VehicleState& v_states = sim_states_nr[vehicle_id]._v_states;
            VehicleParam& veh_param = sim_data_nr[vehicle_id]._veh_params;
            TMeasyNrState& tirelf_st = sim_states_nr[vehicle_id]._tirelf_st;
            TMeasyNrState& tirerf_st = sim_states_nr[vehicle_id]._tirerf_st;
            TMeasyNrState& tirelr_st = sim_states_nr[vehicle_id]._tirelr_st;
            TMeasyNrState& tirerr_st = sim_states_nr[vehicle_id]._tirerr_st;
            SuspensionState& suslf_st = sim_states_nr[vehicle_id]._suslf_st;
            SuspensionState& susrf_st = sim_states_nr[vehicle_id]._susrf_st;
            SuspensionState& suslr_st = sim_states_nr[vehicle_id]._suslr_st;
            SuspensionState& susrr_st = sim_states_nr[vehicle_id]._susrr_st;

            // First the tire states
            // LF
            tirelf_st._xt += tirelf_st._dxt * step;
            tirelf_st._omega += tirelf_st._dOmega * step;
            // RF
            tirerf_st._xt += tirerf_st._dxt * step;
            tirerf_st._omega += tirerf_st._dOmega * step;
            // LR
            tirelr_st._xt += tirelr_st._dxt * step;
            tirelr_st._omega += tirelr_st._dOmega * step;
            // RR
            tirerr_st._xt += tirerr_st._dxt * step;
            tirerr_st._omega += tirerr_st._dOmega * step;

            // Integrate the suspension states
            suslf_st._wu += suslf_st._dwu * step;
            suslf_st._xs += suslf_st._dxs * step;

            susrf_st._wu += susrf_st._dwu * step;
            susrf_st._xs += susrf_st._dxs * step;

            suslr_st._wu += suslr_st._dwu * step;
            suslr_st._xs += suslr_st._dxs * step;

            susrr_st._wu += susrr_st._dwu * step;
            susrr_st._xs += susrr_st._dxs * step;

            // Now the vehicle states
            if (veh_param._tcbool) {
                v_states._crankOmega += v_states._dOmega_crank * step;
            }

            v_states._u += v_states._udot * step;
            v_states._v += v_states._vdot * step;
            v_states._w += v_states._wdot * step;
            v_states._wx += v_states._wxdot * step;
            v_states._wy += v_states._wydot * step;
            v_states._wz += v_states._wzdot * step;

            // We have to recompute this here so that we are doing the half implicit
            // Now to integrate the cardan angles, we first write equations for them
            v_states._dtheta = v_states._wy * std::cos(v_states._phi) - v_states._wz * std::sin(v_states._phi);
            v_states._dpsi =
                (v_states._wy * std::sin(v_states._phi) / std::cos(v_states._theta)) +
                (v_states._wz * std::cos(v_states._phi) / std::cos(v_states._theta));  // Trouble when theta is 90?
            v_states._dphi = v_states._wx + v_states._wy * std::sin(v_states._phi) * std::tan(v_states._theta) +
                             v_states._wz * std::cos(v_states._phi) * std::tan(v_states._theta);

            v_states._theta += v_states._dtheta * step;
            v_states._psi += v_states._dpsi * step;
            v_states._phi += v_states._dphi * step;

            // This computation is also repeated because of half implicit
            // Update global position
            v_states._x += (v_states._u * std::cos(v_states._psi) - v_states._v * std::sin(v_states._psi)) * step;
            v_states._y += (v_states._u * std::sin(v_states._psi) + v_states._v * std::cos(v_states._psi)) * step;

            // Update time
            t += step;
            kernel_time += step;

            // Write to response if required -> regardless of no_outs or store_all we write all the vehicles to the
            // response
            if (output) {
                // The +1 here is because state at time 0 is not stored in device response
                if (abs(kernel_time - (timeStep_stored + 1) * dtout) < 1e-7) {
                    unsigned int time_offset = timeStep_stored * total_num_vehicles * collection_states;

                    device_response[time_offset + (total_num_vehicles * 0) + vehicle_id] = t;
                    device_response[time_offset + (total_num_vehicles * 1) + vehicle_id] = v_states._x;
                    device_response[time_offset + (total_num_vehicles * 2) + vehicle_id] = v_states._y;
                    device_response[time_offset + (total_num_vehicles * 3) + vehicle_id] = v_states._u;
                    device_response[time_offset + (total_num_vehicles * 4) + vehicle_id] = v_states._v;
                    device_response[time_offset + (total_num_vehicles * 5) + vehicle_id] = v_states._phi;
                    device_response[time_offset + (total_num_vehicles * 6) + vehicle_id] = v_states._psi;
                    device_response[time_offset + (total_num_vehicles * 7) + vehicle_id] = v_states._theta;
                    device_response[time_offset + (total_num_vehicles * 8) + vehicle_id] = v_states._wx;
                    device_response[time_offset + (total_num_vehicles * 9) + vehicle_id] = v_states._wz;
                    device_response[time_offset + (total_num_vehicles * 10) + vehicle_id] = v_states._wy;
                    device_response[time_offset + (total_num_vehicles * 11) + vehicle_id] = tirelf_st._omega;
                    device_response[time_offset + (total_num_vehicles * 12) + vehicle_id] = tirerf_st._omega;
                    device_response[time_offset + (total_num_vehicles * 13) + vehicle_id] = tirelr_st._omega;
                    device_response[time_offset + (total_num_vehicles * 14) + vehicle_id] = tirerr_st._omega;
                    timeStep_stored++;
                }
            }
        }
    }
}

__global__ void Integrate(double current_time,
                          double steering,
                          double throttle,
                          double braking,
                          double kernel_sim_time,
                          double step,
                          bool output,
                          unsigned int total_num_vehicles,
                          unsigned int collection_states,
                          double dtout,
                          double* device_response,
                          d24::SimDataNr* sim_data_nr,
                          d24::SimStateNr* sim_states_nr) {
    double t = current_time;           // Set the current time
    double kernel_time = 0;            // Time since kernel was launched
    unsigned int timeStep_stored = 0;  // Number of time steps already stored in the device response
    double end_time = (t + kernel_sim_time) - step / 10.;
    unsigned int vehicle_id = blockIdx.x * blockDim.x + threadIdx.x;  // Get the vehicle id
    if (vehicle_id < total_num_vehicles) {
        while (t < end_time) {
            // Call the RHS to get accelerations for all the vehicles
            rhsFun(t, total_num_vehicles, sim_data_nr, sim_states_nr, steering, throttle, braking);

            // Integrate according to half implicit method for second order states
            // Integrate according to explicit method for first order states

            // Extract the states of the vehicle and the tires
            VehicleState& v_states = sim_states_nr[vehicle_id]._v_states;
            VehicleParam& veh_param = sim_data_nr[vehicle_id]._veh_params;
            TMeasyNrState& tirelf_st = sim_states_nr[vehicle_id]._tirelf_st;
            TMeasyNrState& tirerf_st = sim_states_nr[vehicle_id]._tirerf_st;
            TMeasyNrState& tirelr_st = sim_states_nr[vehicle_id]._tirelr_st;
            TMeasyNrState& tirerr_st = sim_states_nr[vehicle_id]._tirerr_st;
            SuspensionState& suslf_st = sim_states_nr[vehicle_id]._suslf_st;
            SuspensionState& susrf_st = sim_states_nr[vehicle_id]._susrf_st;
            SuspensionState& suslr_st = sim_states_nr[vehicle_id]._suslr_st;
            SuspensionState& susrr_st = sim_states_nr[vehicle_id]._susrr_st;

            // First the tire states
            // LF
            tirelf_st._xt += tirelf_st._dxt * step;
            tirelf_st._omega += tirelf_st._dOmega * step;
            // RF
            tirerf_st._xt += tirerf_st._dxt * step;
            tirerf_st._omega += tirerf_st._dOmega * step;
            // LR
            tirelr_st._xt += tirelr_st._dxt * step;
            tirelr_st._omega += tirelr_st._dOmega * step;
            // RR
            tirerr_st._xt += tirerr_st._dxt * step;
            tirerr_st._omega += tirerr_st._dOmega * step;

            // Integrate the suspension states
            suslf_st._wu += suslf_st._dwu * step;
            suslf_st._xs += suslf_st._dxs * step;

            susrf_st._wu += susrf_st._dwu * step;
            susrf_st._xs += susrf_st._dxs * step;

            suslr_st._wu += suslr_st._dwu * step;
            suslr_st._xs += suslr_st._dxs * step;

            susrr_st._wu += susrr_st._dwu * step;
            susrr_st._xs += susrr_st._dxs * step;

            // Now the vehicle states
            if (veh_param._tcbool) {
                v_states._crankOmega += v_states._dOmega_crank * step;
            }

            v_states._u += v_states._udot * step;
            v_states._v += v_states._vdot * step;
            v_states._w += v_states._wdot * step;
            v_states._wx += v_states._wxdot * step;
            v_states._wy += v_states._wydot * step;
            v_states._wz += v_states._wzdot * step;

            // We have to recompute this here so that we are doing the half implicit
            // Now to integrate the cardan angles, we first write equations for them
            v_states._dtheta = v_states._wy * std::cos(v_states._phi) - v_states._wz * std::sin(v_states._phi);
            v_states._dpsi =
                (v_states._wy * std::sin(v_states._phi) / std::cos(v_states._theta)) +
                (v_states._wz * std::cos(v_states._phi) / std::cos(v_states._theta));  // Trouble when theta is 90?
            v_states._dphi = v_states._wx + v_states._wy * std::sin(v_states._phi) * std::tan(v_states._theta) +
                             v_states._wz * std::cos(v_states._phi) * std::tan(v_states._theta);

            v_states._theta += v_states._dtheta * step;
            v_states._psi += v_states._dpsi * step;
            v_states._phi += v_states._dphi * step;

            // This computation is also repeated because of half implicit
            // Update global position
            v_states._x += (v_states._u * std::cos(v_states._psi) - v_states._v * std::sin(v_states._psi)) * step;
            v_states._y += (v_states._u * std::sin(v_states._psi) + v_states._v * std::cos(v_states._psi)) * step;

            // Update time
            t += step;
            kernel_time += step;

            // Write to response if required -> regardless of no_outs or store_all we write all the vehicles to the
            // response
            if (output) {
                // The +1 here is because state at time 0 is not stored in device response
                if (abs(kernel_time - (timeStep_stored + 1) * dtout) < 1e-7) {
                    unsigned int time_offset = timeStep_stored * total_num_vehicles * collection_states;

                    device_response[time_offset + (total_num_vehicles * 0) + vehicle_id] = t;
                    device_response[time_offset + (total_num_vehicles * 1) + vehicle_id] = v_states._x;
                    device_response[time_offset + (total_num_vehicles * 2) + vehicle_id] = v_states._y;
                    device_response[time_offset + (total_num_vehicles * 3) + vehicle_id] = v_states._u;
                    device_response[time_offset + (total_num_vehicles * 4) + vehicle_id] = v_states._v;
                    device_response[time_offset + (total_num_vehicles * 5) + vehicle_id] = v_states._phi;
                    device_response[time_offset + (total_num_vehicles * 6) + vehicle_id] = v_states._psi;
                    device_response[time_offset + (total_num_vehicles * 7) + vehicle_id] = v_states._theta;
                    device_response[time_offset + (total_num_vehicles * 8) + vehicle_id] = v_states._wx;
                    device_response[time_offset + (total_num_vehicles * 9) + vehicle_id] = v_states._wz;
                    device_response[time_offset + (total_num_vehicles * 10) + vehicle_id] = v_states._wy;
                    device_response[time_offset + (total_num_vehicles * 11) + vehicle_id] = tirelf_st._omega;
                    device_response[time_offset + (total_num_vehicles * 12) + vehicle_id] = tirerf_st._omega;
                    device_response[time_offset + (total_num_vehicles * 13) + vehicle_id] = tirelr_st._omega;
                    device_response[time_offset + (total_num_vehicles * 14) + vehicle_id] = tirerr_st._omega;
                    timeStep_stored++;
                }
            }
        }
    }
}