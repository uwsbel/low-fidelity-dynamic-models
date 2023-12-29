#include <algorithm>
#include <cassert>
#include <random>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <vector>

#include "dof18_gpu.cuh"
#include "dof18_halfImplicit_gpu.cuh"

using namespace d18;
// ======================================================================================================================
d18SolverHalfImplicitGPU::d18SolverHalfImplicitGPU(unsigned int total_num_vehicles)
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
    CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_data, sizeof(d18::SimData) * m_total_num_vehicles));
    CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_data_nr, sizeof(d18::SimDataNr) * m_total_num_vehicles));
    CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_states, sizeof(d18::SimState) * m_total_num_vehicles));
    CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_states_nr, sizeof(d18::SimStateNr) * m_total_num_vehicles));

    // Set device and host arrays to nullptrs in case SetOutput is not called by the user
    m_device_response = nullptr;
    m_host_response = nullptr;
}
d18SolverHalfImplicitGPU::~d18SolverHalfImplicitGPU() {
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
// Construct the solver using path to vehicle parameters, tire parameters, number of vehicles and driver
__host__ void d18SolverHalfImplicitGPU::Construct(const std::string& vehicle_params_file,
                                                  const std::string& tire_params_file,
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
    d18::VehicleParam veh_param;
    d18::TMeasyParam tire_param;

    setVehParamsJSON(veh_param, vehicle_params_file.c_str());
    setTireParamsJSON(tire_param, tire_params_file.c_str());
    // Initialize tire parameters that depend on other parameters
    tireInit(&tire_param);

    DriverData driver_data;
    LoadDriverData(driver_data, driver_inputs_file);
    unsigned int driver_data_len = driver_data.size();
    size_t old_vehicle_count = m_vehicle_count_tracker_params;
    m_vehicle_count_tracker_params += num_vehicles;
    for (size_t i = old_vehicle_count; i <= m_vehicle_count_tracker_params; i++) {
        m_sim_data[i]._driver_data_len = driver_data_len;
        // Allocate memory for the driver data
        CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_data[i]._driver_data,
                                           sizeof(DriverInput) * m_sim_data[i]._driver_data_len));
        // Copy the driver data from cpu to managed memory
        std::copy(driver_data.begin(), driver_data.end(), m_sim_data[i]._driver_data);
        // Fill up simulation data from the cpu structs
        m_sim_data[i]._veh_param = veh_param;
        m_sim_data[i]._tireTM_param = tire_param;
    }
    cudaMemPrefetchAsync(&m_sim_data, sizeof(SimData) * m_vehicle_count_tracker_params,
                         0);  // move the simData onto the GPU
}

__host__ void d18SolverHalfImplicitGPU::Construct(const std::string& vehicle_params_file,
                                                  const std::string& tire_params_file,
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
        d18::VehicleParam veh_param;
        d18::TMeasyParam tire_param;

        setVehParamsJSON(veh_param, vehicle_params_file.c_str());
        setTireParamsJSON(tire_param, tire_params_file.c_str());
        // Initialize tire parameters that depend on other parameters
        tireInit(&tire_param);

        DriverData driver_data;
        LoadDriverData(driver_data, driver_inputs_file);
        unsigned int driver_data_len = driver_data.size();
        size_t old_vehicle_count = m_vehicle_count_tracker_params;
        m_vehicle_count_tracker_params += num_vehicles;
        for (size_t i = old_vehicle_count; i <= m_vehicle_count_tracker_params; i++) {
            m_sim_data[i]._driver_data_len = driver_data_len;
            // Allocate memory for the driver data
            CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_data[i]._driver_data,
                                               sizeof(DriverInput) * m_sim_data[i]._driver_data_len));
            // Copy the driver data from cpu to managed memory
            std::copy(driver_data.begin(), driver_data.end(), m_sim_data[i]._driver_data);
            // Fill up simulation data from the cpu structs
            m_sim_data[i]._veh_param = veh_param;
            m_sim_data[i]._tireTM_param = tire_param;
        }
        cudaMemPrefetchAsync(&m_sim_data, sizeof(SimData) * m_vehicle_count_tracker_params,
                             0);  // move the simData onto the GPU
    } else {
        // Because of this, we free the memory of the TMeasyNR tire
        cudaFree(m_sim_data);
        cudaFree(m_sim_states);
        // Set these to nullptr so that we don't try to free them again in the destructor
        m_sim_data = nullptr;
        m_sim_states = nullptr;

        // Since cudaMallocManaged does not call the constructor for non-POD types, we create cpu structs and fill them
        // up and then copy them over to the simData structs
        d18::VehicleParam veh_param;
        d18::TMeasyNrParam tire_param;

        setVehParamsJSON(veh_param, vehicle_params_file.c_str());
        setTireParamsJSON(tire_param, tire_params_file.c_str());
        // Initialize tire parameters that depend on other parameters
        tireInit(&tire_param);

        DriverData driver_data;
        LoadDriverData(driver_data, driver_inputs_file);
        unsigned int driver_data_len = driver_data.size();
        size_t old_vehicle_count = m_vehicle_count_tracker_params;
        m_vehicle_count_tracker_params += num_vehicles;
        for (size_t i = old_vehicle_count; i <= m_vehicle_count_tracker_params; i++) {
            m_sim_data_nr[i]._driver_data_len = driver_data_len;
            // Allocate memory for the driver data
            CHECK_CUDA_ERROR(cudaMallocManaged((void**)&m_sim_data_nr[i]._driver_data,
                                               sizeof(DriverInput) * m_sim_data_nr[i]._driver_data_len));
            // Copy the driver data from cpu to managed memory
            std::copy(driver_data.begin(), driver_data.end(), m_sim_data_nr[i]._driver_data);
            // Fill up simulation data from the cpu structs
            m_sim_data_nr[i]._veh_param = veh_param;
            m_sim_data_nr[i]._tireTMNr_param = tire_param;
            // Set the final integration time for each of the vehicles
            m_sim_data_nr[i]._t_end = driver_data.back().m_time;
        }
        cudaMemPrefetchAsync(&m_sim_data_nr, sizeof(SimData) * m_vehicle_count_tracker_params,
                             0);  // move the simData onto the GPU
    }
}

// Overload for situations when a controller is used and we don't have a driver data file
__host__ void d18SolverHalfImplicitGPU::Construct(const std::string& vehicle_params_file,
                                                  const std::string& tire_params_file,
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
    d18::VehicleParam veh_param;
    d18::TMeasyParam tire_param;

    setVehParamsJSON(veh_param, vehicle_params_file.c_str());
    setTireParamsJSON(tire_param, tire_params_file.c_str());
    // Initialize tire parameters that depend on other parameters
    tireInit(&tire_param);

    size_t old_vehicle_count = m_vehicle_count_tracker_params;
    m_vehicle_count_tracker_params += num_vehicles;
    for (size_t i = old_vehicle_count; i <= m_vehicle_count_tracker_params; i++) {
        // Fill up simulation data from the cpu structs
        m_sim_data[i]._veh_param = veh_param;
        m_sim_data[i]._tireTM_param = tire_param;
    }

    cudaMemPrefetchAsync(&m_sim_data, sizeof(SimData) * m_vehicle_count_tracker_params,
                         0);  // move the simData onto the GPU
}

__host__ void d18SolverHalfImplicitGPU::Construct(const std::string& vehicle_params_file,
                                                  const std::string& tire_params_file,
                                                  unsigned int num_vehicles,
                                                  TireType type) {
    // Check if num_vehicles added is less than the total number of vehicles
    assert((num_vehicles + m_vehicle_count_tracker_params <= m_total_num_vehicles) &&
           "Number of vehicles added makes the vehicle count greater than the total number of vehicles");
    m_tire_type = type;
    // If there is no tire type specified, then use TMeasy
    if (m_tire_type == TireType::TMeasy) {
        // Because of this, we free the memory of the TMeasyNR tire
        cudaFree(m_sim_data_nr);
        cudaFree(m_sim_states_nr);
        // Set these to nullptr so that we don't try to free them again in the destructor
        m_sim_data_nr = nullptr;
        m_sim_states_nr = nullptr;

        // Since cudaMallocManaged does not call the constructor for non-POD types, we create cpu structs and fill them
        // up and then copy them over to the simData structs
        d18::VehicleParam veh_param;
        d18::TMeasyParam tire_param;

        setVehParamsJSON(veh_param, vehicle_params_file.c_str());
        setTireParamsJSON(tire_param, tire_params_file.c_str());
        // Initialize tire parameters that depend on other parameters
        tireInit(&tire_param);

        size_t old_vehicle_count = m_vehicle_count_tracker_params;
        m_vehicle_count_tracker_params += num_vehicles;
        for (size_t i = old_vehicle_count; i <= m_vehicle_count_tracker_params; i++) {
            // Fill up simulation data from the cpu structs
            m_sim_data[i]._veh_param = veh_param;
            m_sim_data[i]._tireTM_param = tire_param;
        }
        cudaMemPrefetchAsync(&m_sim_data, sizeof(SimData) * m_vehicle_count_tracker_params,
                             0);  // move the simData onto the GPU
    } else {
        // Because of this, we free the memory of the TMeasyNR tire
        cudaFree(m_sim_data);
        cudaFree(m_sim_states);
        // Set these to nullptr so that we don't try to free them again in the destructor
        m_sim_data = nullptr;
        m_sim_states = nullptr;

        // Since cudaMallocManaged does not call the constructor for non-POD types, we create cpu structs and fill them
        // up and then copy them over to the simData structs
        d18::VehicleParam veh_param;
        d18::TMeasyNrParam tire_param;

        setVehParamsJSON(veh_param, vehicle_params_file.c_str());
        setTireParamsJSON(tire_param, tire_params_file.c_str());
        // Initialize tire parameters that depend on other parameters
        tireInit(&tire_param);

        size_t old_vehicle_count = m_vehicle_count_tracker_params;
        m_vehicle_count_tracker_params += num_vehicles;
        for (size_t i = old_vehicle_count; i <= m_vehicle_count_tracker_params; i++) {
            // Fill up simulation data from the cpu structs
            m_sim_data_nr[i]._veh_param = veh_param;
            m_sim_data_nr[i]._tireTMNr_param = tire_param;
        }
        cudaMemPrefetchAsync(&m_sim_data_nr, sizeof(SimData) * m_vehicle_count_tracker_params,
                             0);  // move the simData onto the GPU
    }
}
// ======================================================================================================================

__host__ void d18SolverHalfImplicitGPU::Initialize(d18::VehicleState& vehicle_states,
                                                   d18::TMeasyState& tire_states_LF,
                                                   d18::TMeasyState& tire_states_RF,
                                                   d18::TMeasyState& tire_states_LR,
                                                   d18::TMeasyState& tire_states_RR,
                                                   unsigned int num_vehicles) {
    // Esnure that construct was called with TMeasy tire type
    assert((m_tire_type == TireType::TMeasy) &&
           "Construct function called with TMeasyNr tire type, but Initialize called with TMeasy tire type");

    size_t old_vehicle_count = m_vehicle_count_tracker_states;
    m_vehicle_count_tracker_states += num_vehicles;
    for (size_t i = old_vehicle_count; i <= m_vehicle_count_tracker_states; i++) {
        // Fill up simulation data from the cpu structs
        m_sim_states[i]._veh_state = vehicle_states;
        m_sim_states[i]._tirelf_state = tire_states_LF;
        m_sim_states[i]._tirerf_state = tire_states_RF;
        m_sim_states[i]._tirelr_state = tire_states_LR;
        m_sim_states[i]._tirerr_state = tire_states_RR;
    }
    cudaMemPrefetchAsync(&m_sim_states, sizeof(SimState) * m_vehicle_count_tracker_states,
                         0);  // move the simState onto the GPU

    // TODO: Add Jacobian support
    // // Size the jacobian matrices - size relies on the torque converter bool
    // m_num_controls = 2;
    // if (m_veh_param._tcbool) {
    //     m_num_states = 21;
    //     m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
    //     m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    // } else {
    //     m_num_states = 20;
    //     m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
    //     m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    // }
}

// TMeasy without relaxation does not have tire states and so the jacobian size reduces by 8
__host__ void d18SolverHalfImplicitGPU::Initialize(d18::VehicleState& vehicle_states,
                                                   d18::TMeasyNrState& tire_states_LF,
                                                   d18::TMeasyNrState& tire_states_RF,
                                                   d18::TMeasyNrState& tire_states_LR,
                                                   d18::TMeasyNrState& tire_states_RR,
                                                   unsigned int num_vehicles) {
    // Esnure that construct was called with TMeasyNr tire type
    assert((m_tire_type == TireType::TMeasyNr) &&
           "Construct function called with TMeasy tire type, but Initialize called with TMeasyNR tire type");
    size_t old_vehicle_count = m_vehicle_count_tracker_states;
    m_vehicle_count_tracker_states += num_vehicles;
    for (size_t i = old_vehicle_count; i <= m_vehicle_count_tracker_states; i++) {
        // Fill up simulation data from the cpu structs
        m_sim_states_nr[i]._veh_state = vehicle_states;
        m_sim_states_nr[i]._tirelf_state = tire_states_LF;
        m_sim_states_nr[i]._tirerf_state = tire_states_RF;
        m_sim_states_nr[i]._tirelr_state = tire_states_LR;
        m_sim_states_nr[i]._tirerr_state = tire_states_RR;
    }
    cudaMemPrefetchAsync(&m_sim_states_nr, sizeof(SimState) * m_vehicle_count_tracker_states,
                         0);  // move the simState onto the GPU

    // TODO: Add Jacobian support
    // Size the jacobian matrices - size relies on the torque converter bool
    // m_num_controls = 2;
    // if (m_veh_param._tcbool) {
    //     m_num_states = 13;
    //     m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
    //     m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    // } else {
    //     m_num_states = 12;
    //     m_jacobian_state.resize(m_num_states, std::vector<double>(m_num_states, 0));
    //     m_jacobian_controls.resize(m_num_states, std::vector<double>(m_num_controls, 0));
    // }
}

// ======================================================================================================================
__host__ void d18SolverHalfImplicitGPU::SetOutput(const std::string& output_file,
                                                  double output_freq,
                                                  bool store_all = false,
                                                  unsigned int no_outs = 50) {
    m_output = true;
    m_store_all = store_all;
    if (!m_store_all) {
        m_num_outs = no_outs;
        // If store_all is false, randomly assign which vehicles need to be dumped into csv
        float some_seed = 68;
        std::mt19937 generator(some_seed);
        const int minval_input = 0., maxval_input = m_total_num_vehicles - 1;
        std::uniform_int_distribution<int> dist_input(minval_input, maxval_input);

        m_which_outs.resize(no_outs);
        // fill in our numbers
        for (std::size_t i = 0; i < no_outs; i++) {
            m_which_outs[i] = dist_input(generator);
        }
    }
    m_output_file = output_file;
    m_dtout = 1.0 / output_freq;

    // Here we also initialize the device and host arrays that store the response across vehicles and states -> If
    // output is not required, then nothing is stored, however the user has access to the states at the last time step
    // through the simState and simData structs

    // Number of time steps to be collected on the device
    m_device_collection_timeSteps = ceil(m_kernel_sim_time / m_dtout);

    // Number of states to store -> For now we only allow storage of the major states which are common to both tire
    // models [time,x,y,u,v,phi,psi,wx,wz,lf_omega,rf_omega,lr_omega,rr_omega]
    m_collection_states = 13;

    // Thus device array size becomes
    m_device_size = sizeof(double) * m_total_num_vehicles * m_collection_states * (m_device_collection_timeSteps);

    CHECK_CUDA_ERROR(cudaMalloc((void**)&m_device_response, m_device_size));

    // Now the host response
    m_host_collection_timeSteps = ceil(m_host_dump_time / m_dtout);

    // Thus the host size becomes -> Usually much larger than the device size
    m_host_response = new double[m_total_num_vehicles * m_collection_states * (m_host_collection_timeSteps)];
}

// ======================================================================================================================

/// @brief Solve the system of equations by calling the integrate function
__host__ void d18SolverHalfImplicitGPU::Solve() {
    assert(!m_driver_data.empty() && "No controls provided, please use construct to pass path to driver inputs");
    assert(m_tend != 0. && "Final time not set, please use SetEndTime function");
    // Calculate the number of blocks required
    m_num_blocks = (m_total_num_vehicles + m_threads_per_block - 1) / m_threads_per_block;

    double current_time = 0.;
    unsigned int kernel_launches_since_last_dump = 0;  // Track the number of kernel launches since the last dump of the
                                                       // host response
    double time_since_last_dump = 0.;                  // Track the time since the last dump of the host response

    while (current_time < m_tend) {
        // Calculate when this kernel is supposed to end

        double kernel_end_time = current_time + m_kernel_sim_time;

        // Launch the kernel
        Integrate<<<m_num_blocks, m_threads_per_block>>>(current_time);

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
            if (time_since_last_dump > m_host_dump_time) {
                Write(current_time);
                time_since_last_dump = 0.;
                kernel_launches_since_last_dump = 0;
            }
        }
    }

    // End of simulation, write to the csv file
    if (m_output) {
        WriteToFile();
    }
}

// ======================================================================================================================

/// @brief Integrate the system of equations using the half implicit method - Calls the RHS function at each time step
__device__ void d18SolverHalfImplicitGPU::Integrate(double current_time) {
    double t = current_time;           // Set the current time
    double kernel_time = 0;            // Time since kernel was launched
    unsigned int timeStep_stored = 0;  // Number of time steps already stored in the device response

    unsigned int vehicle_id = blockIdx.x * blockDim.x + threadIdx.x;  // Get the vehicle id
    while (t < ((t + m_kernel_sim_time) - m_step / 10.)) {
        // Call the RHS to get accelerations for all the vehicles
        rhsFun(t);

        // Integrate according to half implicit method for second order states
        // Integrate according to explicit method for first order states

        if (m_tire_type == TireType::TMeasy) {
            // Extract the states of the vehicle and the tires
            VehicleState& v_states = m_sim_states[vehicle_id]._veh_state;
            VehicleParam& veh_param = m_sim_data[vehicle_id]._veh_param;
            TMeasyState& tirelf_st = m_sim_states[vehicle_id]._tirelf_state;
            TMeasyState& tirerf_st = m_sim_states[vehicle_id]._tirerf_state;
            TMeasyState& tirelr_st = m_sim_states[vehicle_id]._tirelr_state;
            TMeasyState& tirerr_st = m_sim_states[vehicle_id]._tirerr_state;

            // First the tire states
            // LF
            tirelf_st._xe += tirelf_st._xedot * m_step;
            tirelf_st._ye += tirelf_st._yedot * m_step;
            tirelf_st._omega += tirelf_st._dOmega * m_step;
            // RF
            tirerf_st._xe += tirerf_st._xedot * m_step;
            tirerf_st._ye += tirerf_st._yedot * m_step;
            tirerf_st._omega += tirerf_st._dOmega * m_step;
            // LR
            tirelr_st._xe += tirelr_st._xedot * m_step;
            tirelr_st._ye += tirelr_st._yedot * m_step;
            tirelr_st._omega += tirelr_st._dOmega * m_step;
            // RR
            tirerr_st._xe += tirerr_st._xedot * m_step;
            tirerr_st._ye += tirerr_st._yedot * m_step;
            tirerr_st._omega += tirerr_st._dOmega * m_step;

            // Now the vehicle states
            if (veh_param._tcbool) {
                v_states._crankOmega += v_states._dOmega_crank * m_step;
            }

            // Integrate velocity level first
            v_states._u += v_states._udot * m_step;
            v_states._v += v_states._vdot * m_step;
            v_states._wx += v_states._wxdot * m_step;
            v_states._wz += v_states._wzdot * m_step;

            // Integrate position level next
            v_states._x += (v_states._u * cos(v_states._psi) - v_states._v * sin(v_states._psi)) * m_step;
            v_states._y += (v_states._u * sin(v_states._psi) + v_states._v * cos(v_states._psi)) * m_step;
            v_states._psi += v_states._wz * m_step;
            v_states._phi += v_states._wx * m_step;

            // Update time
            t += m_step;
            kernel_time += m_step;

            // Write to response if required -> regardless of no_outs or store_all we write all the vehicles to the
            // response
            if (m_output) {
                if (abs(kernel_time - timeStep_stored * m_dtout) < 1e-7) {
                    unsigned int time_offset = timeStep_stored * m_total_num_vehicles * m_collection_states;

                    m_device_response[time_offset + (m_total_num_vehicles * 0) + vehicle_id] = t;
                    m_device_response[time_offset + (m_total_num_vehicles * 1) + vehicle_id] = v_states._x;
                    m_device_response[time_offset + (m_total_num_vehicles * 2) + vehicle_id] = v_states._y;
                    m_device_response[time_offset + (m_total_num_vehicles * 3) + vehicle_id] = v_states._u;
                    m_device_response[time_offset + (m_total_num_vehicles * 4) + vehicle_id] = v_states._v;
                    m_device_response[time_offset + (m_total_num_vehicles * 5) + vehicle_id] = v_states._phi;
                    m_device_response[time_offset + (m_total_num_vehicles * 6) + vehicle_id] = v_states._psi;
                    m_device_response[time_offset + (m_total_num_vehicles * 7) + vehicle_id] = v_states._wx;
                    m_device_response[time_offset + (m_total_num_vehicles * 8) + vehicle_id] = v_states._wz;
                    m_device_response[time_offset + (m_total_num_vehicles * 9) + vehicle_id] = tirelf_st._omega;
                    m_device_response[time_offset + (m_total_num_vehicles * 10) + vehicle_id] = tirerf_st._omega;
                    m_device_response[time_offset + (m_total_num_vehicles * 11) + vehicle_id] = tirelr_st._omega;
                    m_device_response[time_offset + (m_total_num_vehicles * 12) + vehicle_id] = tirerr_st._omega;
                }
            }

        } else {
            // Extract the states of the vehicle and the tires
            VehicleState& v_states = m_sim_states_nr[vehicle_id]._veh_state;
            VehicleParam& veh_param = m_sim_data_nr[vehicle_id]._veh_param;
            TMeasyNrState& tirelf_st = m_sim_states_nr[vehicle_id]._tirelf_state;
            TMeasyNrState& tirerf_st = m_sim_states_nr[vehicle_id]._tirerf_state;
            TMeasyNrState& tirelr_st = m_sim_states_nr[vehicle_id]._tirelr_state;
            TMeasyNrState& tirerr_st = m_sim_states_nr[vehicle_id]._tirerr_state;

            // First the tire states
            // LF
            tirelf_st._omega += tirelf_st._dOmega * m_step;
            // RF
            tirerf_st._omega += tirerf_st._dOmega * m_step;
            // LR
            tirelr_st._omega += tirelr_st._dOmega * m_step;
            // RR
            tirerr_st._omega += tirerr_st._dOmega * m_step;

            // Now the vehicle states
            if (veh_param._tcbool) {
                v_states._crankOmega += v_states._dOmega_crank * m_step;
            }

            // Integrate velocity level first
            v_states._u += v_states._udot * m_step;
            v_states._v += v_states._vdot * m_step;
            v_states._wx += v_states._wxdot * m_step;
            v_states._wz += v_states._wzdot * m_step;
            // Integrate position level next
            v_states._x += (v_states._u * cos(v_states._psi) - v_states._v * sin(v_states._psi)) * m_step;
            v_states._y += (v_states._u * sin(v_states._psi) + v_states._v * cos(v_states._psi)) * m_step;
            v_states._psi += v_states._wz * m_step;
            v_states._phi += v_states._wx * m_step;

            // Update time
            t += m_step;
            kernel_time += m_step;

            // Write to response if required -> regardless of no_outs or store_all we write all the vehicles to the
            // response
            if (m_output) {
                if (abs(kernel_time - timeStep_stored * m_dtout) < 1e-7) {
                    unsigned int time_offset = timeStep_stored * m_total_num_vehicles * m_collection_states;

                    m_device_response[time_offset + (m_total_num_vehicles * 0) + vehicle_id] = t;
                    m_device_response[time_offset + (m_total_num_vehicles * 1) + vehicle_id] = v_states._x;
                    m_device_response[time_offset + (m_total_num_vehicles * 2) + vehicle_id] = v_states._y;
                    m_device_response[time_offset + (m_total_num_vehicles * 3) + vehicle_id] = v_states._u;
                    m_device_response[time_offset + (m_total_num_vehicles * 4) + vehicle_id] = v_states._v;
                    m_device_response[time_offset + (m_total_num_vehicles * 5) + vehicle_id] = v_states._phi;
                    m_device_response[time_offset + (m_total_num_vehicles * 6) + vehicle_id] = v_states._psi;
                    m_device_response[time_offset + (m_total_num_vehicles * 7) + vehicle_id] = v_states._wx;
                    m_device_response[time_offset + (m_total_num_vehicles * 8) + vehicle_id] = v_states._wz;
                    m_device_response[time_offset + (m_total_num_vehicles * 9) + vehicle_id] = tirelf_st._omega;
                    m_device_response[time_offset + (m_total_num_vehicles * 10) + vehicle_id] = tirerf_st._omega;
                    m_device_response[time_offset + (m_total_num_vehicles * 11) + vehicle_id] = tirelr_st._omega;
                    m_device_response[time_offset + (m_total_num_vehicles * 12) + vehicle_id] = tirerr_st._omega;
                }
            }
        }
    }
}
// ======================================================================================================================

/// @brief Function call to integrate by just a single time step. This function will always integrate to the t + m_step
/// where m_step is set using the SetTimeStep function.
/// @param t current time
/// @param throttle throttle input
/// @param steering steering input
/// @param braking braking input
/// @return t + m_step
double d18SolverHalfImplicitGPU::IntegrateStep(double t, double throttle, double steering, double braking) {
    // Store header and first time step
    if (m_output && (t < m_step)) {
        Write(t);
        m_timeStepsStored++;
    }

    DriverInput controls(t, steering, throttle, braking);
    // Call the RHS function
    rhsFun(t, controls);

    // Integrate according to half implicit method for second order states
    // Integrate according to explicit method for first order states

    if (m_tire_type == TireType::TMeasy) {  // Only TM easy has xe and ye states
        // First the tire states
        // LF
        m_tireTMlf_state._xe += m_tireTMlf_state._xedot * m_step;
        m_tireTMlf_state._ye += m_tireTMlf_state._yedot * m_step;
        m_tireTMlf_state._omega += m_tireTMlf_state._dOmega * m_step;
        // RF
        m_tireTMrf_state._xe += m_tireTMrf_state._xedot * m_step;
        m_tireTMrf_state._ye += m_tireTMrf_state._yedot * m_step;
        m_tireTMrf_state._omega += m_tireTMrf_state._dOmega * m_step;
        // LR
        m_tireTMlr_state._xe += m_tireTMlr_state._xedot * m_step;
        m_tireTMlr_state._ye += m_tireTMlr_state._yedot * m_step;
        m_tireTMlr_state._omega += m_tireTMlr_state._dOmega * m_step;
        // RR
        m_tireTMrr_state._xe += m_tireTMrr_state._xedot * m_step;
        m_tireTMrr_state._ye += m_tireTMrr_state._yedot * m_step;
        m_tireTMrr_state._omega += m_tireTMrr_state._dOmega * m_step;
    } else {  // Other tires have only omega states
        // First the tire states
        // LF
        m_tireTMNrlf_state._omega += m_tireTMNrlf_state._dOmega * m_step;
        // RF
        m_tireTMNrrf_state._omega += m_tireTMNrrf_state._dOmega * m_step;
        // LR
        m_tireTMNrlr_state._omega += m_tireTMNrlr_state._dOmega * m_step;
        // RR
        m_tireTMNrrr_state._omega += m_tireTMNrrr_state._dOmega * m_step;
    }

    // Now the vehicle states
    if (m_veh_param._tcbool) {
        m_veh_state._crankOmega += m_veh_state._dOmega_crank * m_step;
    }

    // Integrate velocity level first
    m_veh_state._u += m_veh_state._udot * m_step;
    m_veh_state._v += m_veh_state._vdot * m_step;
    m_veh_state._wx += m_veh_state._wxdot * m_step;
    m_veh_state._wz += m_veh_state._wzdot * m_step;

    // Integrate position level next
    m_veh_state._x +=
        (m_veh_state._u * std::cos(m_veh_state._psi) - m_veh_state._v * std::sin(m_veh_state._psi)) * m_step;
    m_veh_state._y +=
        (m_veh_state._u * std::sin(m_veh_state._psi) + m_veh_state._v * std::cos(m_veh_state._psi)) * m_step;
    m_veh_state._psi += m_veh_state._wz * m_step;
    m_veh_state._phi += m_veh_state._wx * m_step;

    double new_time = t + m_step;
    // Write the output
    if (m_output) {
        if (std::abs(new_time - m_timeStepsStored * m_dtout) < 1e-7) {
            Write(new_time);
            m_timeStepsStored++;
        }
    }

    return new_time;
}
// ======================================================================================================================
/// @brief Function call to integrate by just a single time step with jacobian computation. The order of the states in
// the jacobian matrix if the TMeasy tire is used are is as follows:
//  0: tirelf_st._xe;
//  1: tirelf_st._ye;
//  2: tirerf_st._xe;
//  3: tirerf_st._ye;
//  4: tirelr_st._xe;
//  5: tirelr_st._ye;
//  6: tirerr_st._xe;
//  7: tirerr_st._ye;
//  8: tirelf_st._omega;
//  9: tirerf_st._omega;
//  10: tirelr_st._omega;
//  11: tirerr_st._omega;
//  12: v_states._crankOmega; (only if torque converter is used)
//  13: v_states._x;
//  14: v_states._y;
//  15: v_states._u;
//  16: v_states._v;
//  17: v_states._psi;
//  18: v_states._wz;
//  19: v_states._phi;
//  20: v_states._wx;
// If the TMeasyNR tire is used then the order of the states in the jacobian matrix are as follows:
//  0: tirelf_st._omega;
//  1: tirerf_st._omega;
//  2: tirelr_st._omega;
//  3: tirerr_st._omega;
//  4: v_states._crankOmega; (only if torque converter is used)
//  5: v_states._x;
//  6: v_states._y;
//  7: v_states._u;
//  8: v_states._v;
//  9: v_states._psi;
//  10: v_states._wz;
//  11: v_states._phi;
//  12: v_states._wx;
/// @param t current time
/// @param throttle throttle input
/// @param steering steering input
/// @param braking braking input
/// @param on boolean to turn on jacobian computation
/// @return t + m_step
double d18SolverHalfImplicitGPU::IntegrateStepWithJacobian(double t,
                                                           double throttle,
                                                           double steering,
                                                           double braking,
                                                           bool jacOn) {
    // Store header and first time step
    if (m_output && (t < m_step)) {
        Write(t);
        m_timeStepsStored++;
    }

    DriverInput controls(t, steering, throttle, braking);

    // If the jacobian switch is on, then compute the jacobian
    if (jacOn) {
        std::vector<double> y(m_num_states, 0);
        std::vector<double> ydot(m_num_states, 0);
        // package all the current states
        if (m_tire_type == TireType::TMeasy) {
            packY(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                  m_veh_param._tcbool, y);
        } else {
            packY(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state,
                  m_veh_param._tcbool, y);
        }

        // ============================
        // Computing the state jacobian
        // ============================

        // Set a vector of del Ys - for now set this to some scale of y
        std::vector<double> delY(y.begin(), y.end());
        // In a loop pertub each state and get the corresponding perturbed ydot
        int ySize = y.size();

#pragma omp parallel for simd
        for (int i = 0; i < ySize; i++) {
            // Perterbation is 1e-3 * y (since some states are really small values wile some are huge)
            delY[i] = std::abs(delY[i] * 1e-3);
            if (delY[i] < 1e-8) {
                // This means that the particular state is 0. In this case set dels to 1e-3
                delY[i] = 1e-3;
            }

            // perturb y at i
            std::vector<double> perturbedYPlus(y.begin(), y.end());
            std::vector<double> perturbedYMinus(y.begin(), y.end());

            perturbedYPlus[i] = perturbedYPlus[i] + delY[i];
            perturbedYMinus[i] = perturbedYMinus[i] - delY[i];

            // ydots to store the output
            std::vector<double> ydotPlus(y.size(), 0.);
            std::vector<double> ydotMinus(y.size(), 0.);

            // Call the perturb function with these to get the perturbed ydot -> This does not update the state
            PerturbRhsFun(perturbedYPlus, controls, ydotPlus);
            PerturbRhsFun(perturbedYMinus, controls, ydotMinus);

// Update the jacobian matrix
#pragma omp simd
            for (int j = 0; j < ySize; j++) {
                m_jacobian_state[j][i] = (ydotPlus[j] - ydotMinus[j]) / (2 * delY[i]);
            }
        }

        // ==============================
        // Computing the control jacobian
        //===============================

        // Set a vector of del controls - for now we ingnore braking
        std::vector<double> delControls = {1e-3, 1e-3};
        // In a loop pertub each control and get the corresponding perturbed ydot
        int controlSize = delControls.size();

        for (int i = 0; i < controlSize; i++) {
            // perturb controls at i
            std::vector<double> perturbedControlsPlus = {steering, throttle};
            std::vector<double> perturbedControlsMinus = {steering, throttle};

            perturbedControlsPlus[i] = perturbedControlsPlus[i] + delControls[i];
            perturbedControlsMinus[i] = perturbedControlsMinus[i] - delControls[i];

            // ydots to store the output
            std::vector<double> ydotPlus(y.size(), 0.);
            std::vector<double> ydotMinus(y.size(), 0.);

            // Call the perturb function with these to get the perturbed ydot -> This does not update the state
            DriverInput inputPlus(t, perturbedControlsPlus[0], perturbedControlsPlus[1], 0);
            DriverInput inputMinus(t, perturbedControlsMinus[0], perturbedControlsMinus[1], 0);
            PerturbRhsFun(y, inputPlus, ydotPlus);
            PerturbRhsFun(y, inputMinus, ydotMinus);

            // Update the jacobian matrix
            for (int j = 0; j < ySize; j++) {
                m_jacobian_controls[j][i] = (ydotPlus[j] - ydotMinus[j]) / (2 * delControls[i]);
            }
        }
    }

    // Call the RHS function
    rhsFun(t, controls);

    // Integrate according to half implicit method for second order states
    // Integrate according to explicit method for first order states

    if (m_tire_type == TireType::TMeasy) {  // Only TM easy has xe and ye states
        // First the tire states
        // LF
        m_tireTMlf_state._xe += m_tireTMlf_state._xedot * m_step;
        m_tireTMlf_state._ye += m_tireTMlf_state._yedot * m_step;
        m_tireTMlf_state._omega += m_tireTMlf_state._dOmega * m_step;
        // RF
        m_tireTMrf_state._xe += m_tireTMrf_state._xedot * m_step;
        m_tireTMrf_state._ye += m_tireTMrf_state._yedot * m_step;
        m_tireTMrf_state._omega += m_tireTMrf_state._dOmega * m_step;
        // LR
        m_tireTMlr_state._xe += m_tireTMlr_state._xedot * m_step;
        m_tireTMlr_state._ye += m_tireTMlr_state._yedot * m_step;
        m_tireTMlr_state._omega += m_tireTMlr_state._dOmega * m_step;
        // RR
        m_tireTMrr_state._xe += m_tireTMrr_state._xedot * m_step;
        m_tireTMrr_state._ye += m_tireTMrr_state._yedot * m_step;
        m_tireTMrr_state._omega += m_tireTMrr_state._dOmega * m_step;
    } else {  // Other tires have only omega states
        // First the tire states
        // LF
        m_tireTMNrlf_state._omega += m_tireTMNrlf_state._dOmega * m_step;
        // RF
        m_tireTMNrrf_state._omega += m_tireTMNrrf_state._dOmega * m_step;
        // LR
        m_tireTMNrlr_state._omega += m_tireTMNrlr_state._dOmega * m_step;
        // RR
        m_tireTMNrrr_state._omega += m_tireTMNrrr_state._dOmega * m_step;
    }
    // Now the vehicle states
    if (m_veh_param._tcbool) {
        m_veh_state._crankOmega += m_veh_state._dOmega_crank * m_step;
    }

    // Integrate velocity level first
    m_veh_state._u += m_veh_state._udot * m_step;
    m_veh_state._v += m_veh_state._vdot * m_step;
    m_veh_state._wx += m_veh_state._wxdot * m_step;
    m_veh_state._wz += m_veh_state._wzdot * m_step;

    // Integrate position level next
    m_veh_state._x +=
        (m_veh_state._u * std::cos(m_veh_state._psi) - m_veh_state._v * std::sin(m_veh_state._psi)) * m_step;
    m_veh_state._y +=
        (m_veh_state._u * std::sin(m_veh_state._psi) + m_veh_state._v * std::cos(m_veh_state._psi)) * m_step;
    m_veh_state._psi += m_veh_state._wz * m_step;
    m_veh_state._phi += m_veh_state._wx * m_step;

    double new_time = t + m_step;
    // Write the output
    if (m_output) {
        if (std::abs(new_time - m_timeStepsStored * m_dtout) < 1e-7) {
            Write(new_time);
            m_timeStepsStored++;
        }
    }

    return new_time;
}

// ======================================================================================================================

/// @brief Computes the RHS of all the ODEs (tire velocities, chassis accelerations)
/// @param t Current time
__device__ void d18SolverHalfImplicitGPU::rhsFun(double t) {
    // Get the vehicle index
    unsigned int vehicle_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (vehicle_index < m_total_num_vehicles) {
        // All vehicles have one or the other tire type and thus no thread divergence
        if (m_tire_type == TireType::TMeasy) {
            VehicleParam& veh_param = m_sim_data[vehicle_index]._veh_param;
            VehicleState& veh_state = m_sim_states[vehicle_index]._veh_state;
            TMeasyParam& tireTM_param = m_sim_data[vehicle_index]._tireTM_param;
            TMeasyState& tireTMlf_state = m_sim_states[vehicle_index]._tirelf_state;
            TMeasyState& tireTMrf_state = m_sim_states[vehicle_index]._tirerf_state;
            TMeasyState& tireTMlr_state = m_sim_states[vehicle_index]._tirelr_state;
            TMeasyState& tireTMrr_state = m_sim_states[vehicle_index]._tirerr_state;

            DriverInput* driver_data = m_sim_data[vehicle_index]._driver_data;
            unsigned int len = m_sim_data[vehicle_index]._driver_data_len;
            // Get controls at the current timeStep
            auto controls = GetDriverInput(t, driver_data, len);

            double loads[4] = {0., 0., 0., 0.};
            // Compute the tire loads
            computeTireLoads(&loads[0], &veh_state, &veh_param, &tireTM_param);
            // Transform from vehicle frame to the tire frame
            vehToTireTransform(&tireTMlf_state, &tireTMrf_state, &tireTMlr_state, &tireTMrr_state, &veh_state,
                               &loads[0], &veh_param, controls.m_steering);

            // Tire velocities using TMEasy tire
            computeTireRHS(&tireTMlf_state, &tireTM_param, &veh_param, controls.m_steering);
            computeTireRHS(&tireTMrf_state, &tireTM_param, &veh_param, controls.m_steering);
            computeTireRHS(&tireTMlr_state, &tireTM_param, &veh_param, 0);  // No rear steering
            computeTireRHS(&tireTMrr_state, &tireTM_param, &veh_param, 0);  // No rear steering

            // Powertrain dynamics
            computePowertrainRHS(&veh_state, &tireTMlf_state, &tireTMrf_state, &tireTMlr_state, &tireTMrr_state,
                                 &veh_param, &tireTM_param, &controls);
            // Vehicle dynamics
            tireToVehTransform(&tireTMlf_state, &tireTMrf_state, &tireTMlr_state, &tireTMrr_state, &veh_state,
                               &veh_param, controls.m_steering);

            double fx[4] = {tireTMlf_state._fx, tireTMrf_state._fx, tireTMlr_state._fx, tireTMrr_state._fx};
            double fy[4] = {tireTMlf_state._fy, tireTMrf_state._fy, tireTMlr_state._fy, tireTMrr_state._fy};

            computeVehRHS(&veh_state, &veh_param, &fx[0], &fy[0]);

        } else {
            VehicleParam& veh_param = m_sim_data_nr[vehicle_index]._veh_param;
            VehicleState& veh_state = m_sim_states_nr[vehicle_index]._veh_state;
            TMeasyNrParam& tireTMNr_param = m_sim_data_nr[vehicle_index]._tireTMNr_param;
            TMeasyNrState& tireTMNrlf_state = m_sim_states_nr[vehicle_index]._tirelf_state;
            TMeasyNrState& tireTMNrrf_state = m_sim_states_nr[vehicle_index]._tirerf_state;
            TMeasyNrState& tireTMNrlr_state = m_sim_states_nr[vehicle_index]._tirelr_state;
            TMeasyNrState& tireTMNrrr_state = m_sim_states_nr[vehicle_index]._tirerr_state;
            DriverInput* driver_data = m_sim_data_nr[vehicle_index]._driver_data;
            unsigned int len = m_sim_data_nr[vehicle_index]._driver_data_len;
            // Get controls at the current timeStep
            auto controls = GetDriverInput(t, driver_data, len);

            double loads[4] = {0., 0., 0., 0.};

            // Compute the tire loads
            computeTireLoads(&loads[0], &veh_state, &veh_param, &tireTMNr_param);
            // Transform from vehicle frame to the tire frame
            vehToTireTransform(&tireTMNrlf_state, &tireTMNrrf_state, &tireTMNrlr_state, &tireTMNrrr_state, &veh_state,
                               &loads[0], &veh_param, controls.m_steering);
            // Tire velocities using TMEasyNr tire
            computeTireRHS(&tireTMNrlf_state, &tireTMNr_param, &veh_param, controls.m_steering);
            computeTireRHS(&tireTMNrrf_state, &tireTMNr_param, &veh_param, controls.m_steering);
            computeTireRHS(&tireTMNrlr_state, &tireTMNr_param, &veh_param, 0);  // No rear steering
            computeTireRHS(&tireTMNrrr_state, &tireTMNr_param, &veh_param, 0);  // No rear steering

            // Powertrain dynamics
            computePowertrainRHS(&veh_state, &tireTMNrlf_state, &tireTMNrrf_state, &tireTMNrlr_state, &tireTMNrrr_state,
                                 &veh_param, &tireTMNr_param, &controls);

            // Vehicle dynamics
            tireToVehTransform(&tireTMNrlf_state, &tireTMNrrf_state, &tireTMNrlr_state, &tireTMNrrr_state, &veh_state,
                               &veh_param, controls.m_steering);

            double fx[4] = {tireTMNrlf_state._fx, tireTMNrrf_state._fx, tireTMNrlr_state._fx, tireTMNrrr_state._fx};
            double fy[4] = {tireTMNrlf_state._fy, tireTMNrrf_state._fy, tireTMNrlr_state._fy, tireTMNrrr_state._fy};

            computeVehRHS(&veh_state, &veh_param, &fx[0], &fy[0]);
        }
    }
}

// ======================================================================================================================

void d18SolverHalfImplicitGPU::rhsFun(double t, DriverInput& controls) {
    // Calculate tire vertical loads
    std::vector<double> loads(4, 0);
    if (m_tire_type == TireType::TMeasy) {
        computeTireLoads(loads, m_veh_state, m_veh_param, m_tireTM_param);

        // Transform from vehicle frame to the tire frame
        vehToTireTransform(m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state, m_veh_state, loads,
                           m_veh_param, controls.m_steering);

        // Tire velocities using TMEasy tire
        computeTireRHS(m_tireTMlf_state, m_tireTM_param, m_veh_param, controls.m_steering);
        computeTireRHS(m_tireTMrf_state, m_tireTM_param, m_veh_param, controls.m_steering);
        computeTireRHS(m_tireTMlr_state, m_tireTM_param, m_veh_param, 0);  // No rear steering
        computeTireRHS(m_tireTMrr_state, m_tireTM_param, m_veh_param, 0);  // No rear steering

        // Powertrain dynamics
        computePowertrainRHS(m_veh_state, m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state,
                             m_veh_param, m_tireTM_param, controls);

        // Vehicle dynamics
        tireToVehTransform(m_tireTMlf_state, m_tireTMrf_state, m_tireTMlr_state, m_tireTMrr_state, m_veh_state,
                           m_veh_param, controls.m_steering);
        std::vector<double> fx = {m_tireTMlf_state._fx, m_tireTMrf_state._fx, m_tireTMlr_state._fx,
                                  m_tireTMrr_state._fx};
        std::vector<double> fy = {m_tireTMlf_state._fy, m_tireTMrf_state._fy, m_tireTMlr_state._fy,
                                  m_tireTMrr_state._fy};
        computeVehRHS(m_veh_state, m_veh_param, fx, fy);
    } else {  // For the other tire
        computeTireLoads(loads, m_veh_state, m_veh_param, m_tireTMNr_param);

        // Transform from vehicle frame to the tire frame
        vehToTireTransform(m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state, m_veh_state,
                           loads, m_veh_param, controls.m_steering);

        // Tire velocities using TMEasy tire
        computeTireRHS(m_tireTMNrlf_state, m_tireTMNr_param, m_veh_param, controls.m_steering);
        computeTireRHS(m_tireTMNrrf_state, m_tireTMNr_param, m_veh_param, controls.m_steering);
        computeTireRHS(m_tireTMNrlr_state, m_tireTMNr_param, m_veh_param, 0);  // No rear steering
        computeTireRHS(m_tireTMNrrr_state, m_tireTMNr_param, m_veh_param, 0);  // No rear steering

        // Powertrain dynamics
        computePowertrainRHS(m_veh_state, m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state,
                             m_tireTMNrrr_state, m_veh_param, m_tireTMNr_param, controls);

        // Vehicle dynamics
        tireToVehTransform(m_tireTMNrlf_state, m_tireTMNrrf_state, m_tireTMNrlr_state, m_tireTMNrrr_state, m_veh_state,
                           m_veh_param, controls.m_steering);
        std::vector<double> fx = {m_tireTMNrlf_state._fx, m_tireTMNrrf_state._fx, m_tireTMNrlr_state._fx,
                                  m_tireTMNrrr_state._fx};
        std::vector<double> fy = {m_tireTMNrlf_state._fy, m_tireTMNrrf_state._fy, m_tireTMNrlr_state._fy,
                                  m_tireTMNrrr_state._fy};
        computeVehRHS(m_veh_state, m_veh_param, fx, fy);
    }
}

// ======================================================================================================================

// Function takes (y +- dely) and provides a new ydot for the pertubed y (ydot is the rhs of the system of equations)

void d18SolverHalfImplicitGPU::PerturbRhsFun(std::vector<double>& y, DriverInput& controls, std::vector<double>& ydot) {
    // Extract the vehicle and tire states vector state
    VehicleState veh_st;
    if (m_tire_type == TireType::TMeasy) {
        TMeasyState tirelf_st;
        TMeasyState tirerf_st;
        TMeasyState tirelr_st;
        TMeasyState tirerr_st;
        unpackY(y, m_veh_param._tcbool, veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

        // Calculate tire vertical loads
        std::vector<double> loads(4, 0);
        computeTireLoads(loads, veh_st, m_veh_param, m_tireTM_param);

        // Transform from the vehicle frame to the tire frame
        vehToTireTransform(tirelf_st, tirerf_st, tirelr_st, tirerr_st, veh_st, loads, m_veh_param, controls.m_steering);

        // Tire dynamics
        computeTireRHS(tirelf_st, m_tireTM_param, m_veh_param, controls.m_steering);
        computeTireRHS(tirerf_st, m_tireTM_param, m_veh_param, controls.m_steering);
        computeTireRHS(tirelr_st, m_tireTM_param, m_veh_param, 0);
        computeTireRHS(tirerr_st, m_tireTM_param, m_veh_param, 0);

        // Powertrain dynamics
        computePowertrainRHS(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, m_veh_param, m_tireTM_param, controls);

        // Vehicle dynamics
        tireToVehTransform(tirelf_st, tirerf_st, tirelr_st, tirerr_st, veh_st, m_veh_param, controls.m_steering);
        std::vector<double> fx = {tirelf_st._fx, tirerf_st._fx, tirelr_st._fx, tirerr_st._fx};
        std::vector<double> fy = {tirelf_st._fy, tirerf_st._fy, tirelr_st._fy, tirerr_st._fy};
        computeVehRHS(veh_st, m_veh_param, fx, fy);

        // Pack the ydot and send it
        packYDOT(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, m_veh_param._tcbool, ydot);
    } else {
        TMeasyNrState tirelf_st;
        TMeasyNrState tirerf_st;
        TMeasyNrState tirelr_st;
        TMeasyNrState tirerr_st;
        unpackY(y, m_veh_param._tcbool, veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

        // Calculate tire vertical loads
        std::vector<double> loads(4, 0);
        computeTireLoads(loads, veh_st, m_veh_param, m_tireTMNr_param);

        // Transform from the vehicle frame to the tire frame
        vehToTireTransform(tirelf_st, tirerf_st, tirelr_st, tirerr_st, veh_st, loads, m_veh_param, controls.m_steering);

        // Tire dynamics
        computeTireRHS(tirelf_st, m_tireTMNr_param, m_veh_param, controls.m_steering);
        computeTireRHS(tirerf_st, m_tireTMNr_param, m_veh_param, controls.m_steering);
        computeTireRHS(tirelr_st, m_tireTMNr_param, m_veh_param, 0);
        computeTireRHS(tirerr_st, m_tireTMNr_param, m_veh_param, 0);

        // Powertrain dynamics
        computePowertrainRHS(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, m_veh_param, m_tireTMNr_param,
                             controls);

        // Vehicle dynamics
        tireToVehTransform(tirelf_st, tirerf_st, tirelr_st, tirerr_st, veh_st, m_veh_param, controls.m_steering);
        std::vector<double> fx = {tirelf_st._fx, tirerf_st._fx, tirelr_st._fx, tirerr_st._fx};
        std::vector<double> fy = {tirelf_st._fy, tirerf_st._fy, tirelr_st._fy, tirerr_st._fy};
        computeVehRHS(veh_st, m_veh_param, fx, fy);

        // Pack the ydot and send it
        packYDOT(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st, m_veh_param._tcbool, ydot);
    }
}

__host__ void d18SolverHalfImplicitGPU::Write(double t) {
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
            CSV_writer& csv = m_sim_data[index_by]._csv;
            // If we are in initial time step, write the header
            if (t < m_step) {
                csv << "time";
                csv << "x";
                csv << "y";
                csv << "vx";
                csv << "vy";
                csv << "roll";
                csv << "yaw";
                csv << "roll_rate";
                csv << "yaw_rate";
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
                csv << std::endl;
                return;
            }
            unsigned int steps_written = 0;
            while (steps_written < m_host_collection_timeSteps) {
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
                csv << std::endl;
                steps_written++;
            }
        } else {
            CSV_writer& csv = m_sim_data_nr[index_by]._csv;
            // If we are in initial time step, write the header
            if (t < m_step) {
                csv << "time";
                csv << "x";
                csv << "y";
                csv << "vx";
                csv << "vy";
                csv << "roll";
                csv << "yaw";
                csv << "roll_rate";
                csv << "yaw_rate";
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
                csv << std::endl;
                return;
            }
            unsigned int steps_written = 0;
            while (steps_written < m_host_collection_timeSteps) {
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
                csv << std::endl;
                steps_written++;
            }
        }
    }
}

// ======================================================================================================================

__host__ void d18SolverHalfImplicitGPU::WriteToFile() {
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
            CSV_writer& csv = m_sim_data[sim_no]._csv;
            csv.write_to_file(m_output_file + "_" + std::to_string(index_by) + ".csv");
            csv.clearData();
        } else {
            CSV_writer& csv = m_sim_data_nr[sim_no]._csv;
            csv.write_to_file(m_output_file + "_" + std::to_string(index_by) + ".csv");
            csv.clearData();
        }
    }
}

// ======================================================================================================================

// Utility functions for finite differencing

void packY(const d18::VehicleState& v_states,
           const d18::TMeasyState& tirelf_st,
           const d18::TMeasyState& tirerf_st,
           const d18::TMeasyState& tirelr_st,
           const d18::TMeasyState& tirerr_st,
           bool has_TC,
           std::vector<double>& y) {
    int index = 0;

    // Tire deflections (lf, rf, lr and rr)

    y[index++] = tirelf_st._xe;
    y[index++] = tirelf_st._ye;
    y[index++] = tirerf_st._xe;
    y[index++] = tirerf_st._ye;
    y[index++] = tirelr_st._xe;
    y[index++] = tirelr_st._ye;
    y[index++] = tirerr_st._xe;
    y[index++] = tirerr_st._ye;

    // Wheel angular velocities (lf, rf, lr and rr)
    y[index++] = tirelf_st._omega;
    y[index++] = tirerf_st._omega;
    y[index++] = tirelr_st._omega;
    y[index++] = tirerr_st._omega;

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        y[index++] = v_states._crankOmega;
    }

    // Vehicle states
    y[index++] = v_states._x;    // X position
    y[index++] = v_states._y;    // Y position
    y[index++] = v_states._u;    // longitudinal velocity
    y[index++] = v_states._v;    // lateral velocity
    y[index++] = v_states._psi;  // yaw angle
    y[index++] = v_states._wz;   // yaw rate
    y[index++] = v_states._phi;  // roll angle
    y[index++] = v_states._wx;   // roll rate
}

void packY(const d18::VehicleState& v_states,
           const d18::TMeasyNrState& tirelf_st,
           const d18::TMeasyNrState& tirerf_st,
           const d18::TMeasyNrState& tirelr_st,
           const d18::TMeasyNrState& tirerr_st,
           bool has_TC,
           std::vector<double>& y) {
    int index = 0;

    // Wheel angular velocities (lf, rf, lr and rr)
    y[index++] = tirelf_st._omega;
    y[index++] = tirerf_st._omega;
    y[index++] = tirelr_st._omega;
    y[index++] = tirerr_st._omega;

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        y[index++] = v_states._crankOmega;
    }

    // Vehicle states
    y[index++] = v_states._x;    // X position
    y[index++] = v_states._y;    // Y position
    y[index++] = v_states._u;    // longitudinal velocity
    y[index++] = v_states._v;    // lateral velocity
    y[index++] = v_states._psi;  // yaw angle
    y[index++] = v_states._wz;   // yaw rate
    y[index++] = v_states._phi;  // roll angle
    y[index++] = v_states._wx;   // roll rate
}

void packYDOT(const d18::VehicleState& v_states,
              const d18::TMeasyState& tirelf_st,
              const d18::TMeasyState& tirerf_st,
              const d18::TMeasyState& tirelr_st,
              const d18::TMeasyState& tirerr_st,
              bool has_TC,
              std::vector<double>& ydot) {
    int index = 0;

    ydot[index++] = tirelf_st._xedot;
    ydot[index++] = tirelf_st._yedot;
    ydot[index++] = tirerf_st._xedot;
    ydot[index++] = tirerf_st._yedot;
    ydot[index++] = tirelr_st._xedot;
    ydot[index++] = tirelr_st._yedot;
    ydot[index++] = tirerr_st._xedot;
    ydot[index++] = tirerr_st._yedot;

    ydot[index++] = tirelf_st._dOmega;
    ydot[index++] = tirerf_st._dOmega;
    ydot[index++] = tirelr_st._dOmega;
    ydot[index++] = tirerr_st._dOmega;

    if (has_TC) {
        ydot[index++] = v_states._dOmega_crank;
    }

    ydot[index++] = v_states._dx;
    ydot[index++] = v_states._dy;
    ydot[index++] = v_states._udot;
    ydot[index++] = v_states._vdot;
    ydot[index++] = v_states._wz;
    ydot[index++] = v_states._wzdot;
    ydot[index++] = v_states._wx;
    ydot[index++] = v_states._wxdot;
}

void packYDOT(const d18::VehicleState& v_states,
              const d18::TMeasyNrState& tirelf_st,
              const d18::TMeasyNrState& tirerf_st,
              const d18::TMeasyNrState& tirelr_st,
              const d18::TMeasyNrState& tirerr_st,
              bool has_TC,
              std::vector<double>& ydot) {
    int index = 0;

    ydot[index++] = tirelf_st._dOmega;
    ydot[index++] = tirerf_st._dOmega;
    ydot[index++] = tirelr_st._dOmega;
    ydot[index++] = tirerr_st._dOmega;

    if (has_TC) {
        ydot[index++] = v_states._dOmega_crank;
    }

    ydot[index++] = v_states._dx;
    ydot[index++] = v_states._dy;
    ydot[index++] = v_states._udot;
    ydot[index++] = v_states._vdot;
    ydot[index++] = v_states._wz;
    ydot[index++] = v_states._wzdot;
    ydot[index++] = v_states._wx;
    ydot[index++] = v_states._wxdot;
}

void unpackY(const std::vector<double>& y,
             bool has_TC,
             d18::VehicleState& v_states,
             d18::TMeasyState& tirelf_st,
             d18::TMeasyState& tirerf_st,
             d18::TMeasyState& tirelr_st,
             d18::TMeasyState& tirerr_st) {
    int index = 0;
    // Tire deflections
    tirelf_st._xe = y[index++];
    tirelf_st._ye = y[index++];
    tirerf_st._xe = y[index++];
    tirerf_st._ye = y[index++];
    tirelr_st._xe = y[index++];
    tirelr_st._ye = y[index++];
    tirerr_st._xe = y[index++];
    tirerr_st._ye = y[index++];

    // Wheel angular velocities
    tirelf_st._omega = y[index++];
    tirerf_st._omega = y[index++];
    tirelr_st._omega = y[index++];
    tirerr_st._omega = y[index++];

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        v_states._crankOmega = y[index++];
    }

    // Vehicle states
    v_states._x = y[index++];    // X position
    v_states._y = y[index++];    // Y position
    v_states._u = y[index++];    // longitudinal velocity
    v_states._v = y[index++];    // lateral velocity
    v_states._psi = y[index++];  // yaw angle
    v_states._wz = y[index++];   // yaw rate
    v_states._phi = y[index++];  // roll angle
    v_states._wx = y[index++];   // roll rate
}

void unpackY(const std::vector<double>& y,
             bool has_TC,
             d18::VehicleState& v_states,
             d18::TMeasyNrState& tirelf_st,
             d18::TMeasyNrState& tirerf_st,
             d18::TMeasyNrState& tirelr_st,
             d18::TMeasyNrState& tirerr_st) {
    int index = 0;

    // Wheel angular velocities
    tirelf_st._omega = y[index++];
    tirerf_st._omega = y[index++];
    tirelr_st._omega = y[index++];
    tirerr_st._omega = y[index++];

    // Crank angular velocity - This is a state only when a torque converter is
    // used
    if (has_TC) {
        v_states._crankOmega = y[index++];
    }

    // Vehicle states
    v_states._x = y[index++];    // X position
    v_states._y = y[index++];    // Y position
    v_states._u = y[index++];    // longitudinal velocity
    v_states._v = y[index++];    // lateral velocity
    v_states._psi = y[index++];  // yaw angle
    v_states._wz = y[index++];   // yaw rate
    v_states._phi = y[index++];  // roll angle
    v_states._wx = y[index++];   // roll rate
}