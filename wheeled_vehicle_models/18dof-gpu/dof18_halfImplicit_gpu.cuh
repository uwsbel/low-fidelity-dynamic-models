#ifndef DOF18_HALFIMPLICIT_H
#define DOF18_HALFIMPLICIT_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>
#include <memory>

#include "dof18_gpu.cuh"

// =============================================================================
// Define the solver class
// =============================================================================
class d18SolverHalfImplicitGPU {
  public:
    /// @brief Initialize the solver with the total number of vehicles to be simulated
    __host__ d18SolverHalfImplicitGPU(unsigned int total_num_vehicles);
    __host__ ~d18SolverHalfImplicitGPU();

    /// @brief Construct the solver using path to vehicle parameters, tire parameters, number of vehicles and driver
    /// inputs. Each of these vehicles will have the specified parameters and driver inputs. To add more vehicles
    /// (ensuring they are still lesser than the total number of vehicles initally specified in the class constructor)
    /// with different parameters, call the Construct function again. The tire type defaults to TMeasy
    /// @param vehicle_params_file Path to the vehicle parameter json file
    /// @param tire_params_file Path to the tire parameter json file
    /// @param num_vehicles Number of vehicles to be simulated with the specified parameters and driver inputs
    /// @param driver_inputs_file Path to the driver inputs text file
    __host__ void Construct(const std::string& veh_params_file,
                            const std::string& tire_params_file,
                            unsigned int num_vehicles,
                            const std::string& driver_file);

    /// @brief Construct the the solver using path to vehicle parameters, tire parameters, number of vehicles, driver
    /// inputs and the TireType (Either TMEasy or TMEasyNr). Each of these vehicles will have the specified parameters
    /// and driver inputs. To add more vehicles with different parameters, call the Construct function again.
    /// @param vehicle_params_file Path to the vehicle parameter json file
    /// @param tire_params_file Path to the tire parameter json file
    /// @param num_vehicles Number of vehicles to be simulated with the specified parameters and driver inputs
    /// @param driver_inputs_file Path to the driver inputs text file
    /// @param type TireType (Either TMEasy or TMEasyNr)
    __host__ void Construct(const std::string& vehicle_params_file,
                            const std::string& tire_params_file,
                            unsigned int num_vehicles,
                            const std::string& driver_inputs_file,
                            TireType type);
    /// @brief Construct the the solver using path to vehicle parameters, tire parameters, number of vehicles. TireType
    /// defualts to TMEasy tires. Each of these vehicles will have the specified parameters. This is mainly provided for
    /// cases where the driver inputs are not available at the start of the simulation but rather come from a controller
    /// during the simualtion. To add more vehicles with different parameters, call the Construct function again.
    /// @param vehicle_params_file Path to the vehicle parameter json file
    /// @param tire_params_file Path to the tire parameter json file
    /// @param num_vehicles Number of vehicles to be simulated with the specified parameters and driver inputs
    __host__ void Construct(const std::string& vehicle_params_file,
                            const std::string& tire_params_file,
                            unsigned int num_vehicles);
    /// @brief Construct the the solver using path to vehicle parameters, tire parameters, number of vehicles and the
    /// TireType (Either TMEasy or TMEasyNr). Each of these vehicles will have the specified parameters. This is mainly
    /// provided for cases where the driver inputs are not available at the start of the simulation but rather come from
    /// a controller during the simualtion. To add more vehicles with different parameters, call the Construct
    /// function again.
    /// @param vehicle_params_file Path to the vehicle parameter json file
    /// @param tire_params_file Path to the tire parameter json file
    /// @param num_vehicles Number of vehicles to be simulated with the specified parameters and driver inputs
    /// @param type TireType (Either TMEasy or TMEasyNr)
    __host__ void Construct(const std::string& vehicle_params_file,
                            const std::string& tire_params_file,
                            unsigned int num_vehicles,
                            TireType type);

    // Set the solver time step
    __host__ void SetEndTime(double tend) { m_tend = tend; }
    __host__ void SetTimeStep(double step) { m_step = step; }
    __host__ void SetKernelSimTime(double time) { m_kernel_sim_time = time; }
    __host__ void SetHostDumpTime(double time) { m_host_dump_time = time; }
    __host__ void SetThreadsPerBlock(unsigned int threads) { m_threads_per_block = threads; }
    __host__ __device__ double GetStep() { return m_step; }
    __host__ __device__ double GetEndTime() { return m_tend; }

    __host__ __device__ DriverData GetDriverData() { return m_driver_data; }

    // Jacobian getter functions
    // double** GetJacobianState() { return m_jacobian_state; }
    // double** GetJacobianControls() { return m_jacobian_controls; }

    /// @brief Switch on output to a csv file at the specified frequency.
    /// @param output_file Path to the output file
    /// @param output_freq Frequency of data output
    /// @param store_all Flag to store data from each vehicle on all the threads or no_outs vehicles
    /// @param no_outs Number of outputs to store if store_all is false
    __host__ void SetOutput(const std::string& output_file,
                            double output_freq,
                            bool store_all = false,
                            unsigned int no_outs = 50);

    /// @brief Solve the system of equations by calling the integrate function
    __host__ void Solve();

    /// @brief Solve the system of equations for a kernel step with the provided controls
    /// @param t Current time
    /// @param steering Steering input
    /// @param throttle Throttle input
    /// @param braking Braking input
    __host__ double SolveStep(double t, double steering, double throttle, double braking);

    /// @brief Initialize vehicle and tire states. This function has to be called before solve and after construct.
    /// Although it is possible to provide non-zero intial states, this is untested and it is recommended to use the
    /// default zero states. To initialize more vehicles, call the Initialize function again.
    /// @param vehicle_states Vehicle states
    /// @param tire_states_LF Left Front (LF) TMeasy tire states
    /// @param tire_states_RF Right Front (RF) TMeasy tire states
    /// @param tire_states_LR Left Rear (LR) TMeasy tire states
    /// @param tire_states_RR Right Rear (RR) TMeasy tire states
    /// @param num_vehicles Number of vehicles to be simulated with the specified states
    __host__ void Initialize(d18::VehicleState& vehicle_states,
                             d18::TMeasyState& tire_states_LF,
                             d18::TMeasyState& tire_states_RF,
                             d18::TMeasyState& tire_states_LR,
                             d18::TMeasyState& tire_states_RR,
                             unsigned int num_vehicles);
    /// @brief Initialize vehicle and tire states. This function has to be called before solve and after construct.
    /// Although it is possible to provide non-zero intial states, this is untested and it is recommended to use the
    /// default zero states. To initialize more vehicles, call the Initialize function again.
    /// @param vehicle_states Vehicle states
    /// @param tire_states_LF Left Front (LF) TMeasyNr tire states
    /// @param tire_states_RF Right Front (RF) TMeasyNr tire states
    /// @param tire_states_LR Left Rear (LR) TMeasyNr tire states
    /// @param tire_states_RR Right Rear (RR) TMeasyNr tire states
    /// @param num_vehicles Number of vehicles to be simulated with the specified states
    __host__ void Initialize(d18::VehicleState& vehicle_states,
                             d18::TMeasyNrState& tire_states_LF,
                             d18::TMeasyNrState& tire_states_RF,
                             d18::TMeasyNrState& tire_states_LR,
                             d18::TMeasyNrState& tire_states_RR,
                             unsigned int num_vehicles);

    /// @brief When using the IntegrateStep, once the integration is complete, this function can be used to Write the
    /// output to a file specified by the user in the SetOutput function.
    __host__ void WriteToFile();

    /// @brief Get a SimState object for a particular vehicle
    /// @param vehicle_index Index of the vehicle
    /// @return SimState object for the vehicle
    __host__ d18::SimState GetSimState(unsigned int vehicle_index);
    /// @brief Get Tire type
    /// @return Tire type used in the solver
    TireType GetTireType() const { return m_tire_type; }

    d18::SimData* m_sim_data;          ///< Simulation data for all the vehicles
    d18::SimDataNr* m_sim_data_nr;     ///< Simulation data but with the TMeasyNr tire for all the vehicles
    d18::SimState* m_sim_states;       ///< Simulation states for all the vehicles
    d18::SimStateNr* m_sim_states_nr;  ///< Simulation states but with the TMeasyNr tire for all the vehicles

    double m_kernel_sim_time;  ///< The maximum time a kernel launch simulates a vehicle. This is set for memory
                               // constraints as we are required to store the states of the vehicle in device array
                               // within the kernel and this can run out of memory for long simulatins. This is also the
                               // time for which the vehicles are simulated on different threads of the GPU without
                               // interaction or communication. This can be set via the SetKernelSimTime function but
                               // defaults to 2 seconds.
    double
        m_host_dump_time;  ///< Time after which the host array is dumped into a csv file. Default is set to 10 seconds
                           // but can be changed by the user
    unsigned int
        m_threads_per_block;  ///< Number of threads per block. This is set to 32 by default but can be changed by
                              // the user

    /// @brief Solve the system of equations for a kernel step by passing a functor that provides the driver inputs
    /// given a time
    /// @param t Current time
    /// @param func Functor that provides the driver inputs given a time. This functor should take 2 arguments: 1) The
    /// current time 2) A pointer to DriverInput. It should then fill in the m_steering, m_throttle and m_braking
    /// variables for the DriverInput (see demmo_hmmwv_controlsFunctor.cu for an example)
    template <typename Func>
    __host__ double SolveStep(double t, Func func) {  // Calculate the number of blocks required
        // if m_output is true, then raise assertion
        if (m_output) {
            // Handle the error: log, return an error code, etc.
            std::cerr
                << "Cannot get csv file output if SolveStep is called, please access sim_states through GetSimSate"
                << std::endl;
            exit(EXIT_FAILURE);
        }
        m_num_blocks = (m_total_num_vehicles + m_threads_per_block - 1) / m_threads_per_block;
        // If m_output is false and its the first time step then we still need to initialize device array -> We don't
        // need the host array as its purpose is just to store the final output
        if (t == 0.) {
            // Number of time steps to be collected on the device
            m_device_collection_timeSteps = ceil(m_kernel_sim_time / m_dtout);

            // Number of states to store -> For now we only allow storage of the major states which are common to both
            // tire models [time,x,y,u,v,phi,psi,wx,wz,lf_omega,rf_omega,lr_omega,rr_omega]
            m_collection_states = 13;

            // Thus device array size becomes
            m_device_size =
                sizeof(double) * m_total_num_vehicles * m_collection_states * (m_device_collection_timeSteps);

            CHECK_CUDA_ERROR(cudaMalloc((void**)&m_device_response, m_device_size));
        }
        m_current_time = t;

        // Launch the kernel
        double kernel_end_time = m_current_time + m_kernel_sim_time;

        if (m_tire_type == TireType::TMeasy) {
            Integrate<<<m_num_blocks, m_threads_per_block>>>(m_current_time, func, m_kernel_sim_time, m_step, m_output,
                                                             m_total_num_vehicles, m_collection_states, m_dtout,
                                                             m_device_response, m_sim_data, m_sim_states);
            cudaError_t err = cudaGetLastError();
            if (err != cudaSuccess) {
                fprintf(stderr, "Kernel launch failed: %s\n", cudaGetErrorString(err));
            }
        } else {
            Integrate<<<m_num_blocks, m_threads_per_block>>>(m_current_time, func, m_kernel_sim_time, m_step, m_output,
                                                             m_total_num_vehicles, m_collection_states, m_dtout,
                                                             m_device_response, m_sim_data_nr, m_sim_states_nr);
            cudaError_t err = cudaGetLastError();
            if (err != cudaSuccess) {
                fprintf(stderr, "Kernel launch failed: %s\n", cudaGetErrorString(err));
            }
        }

        m_current_time = kernel_end_time;
        m_time_since_last_dump += m_kernel_sim_time;
        return m_current_time;
    }

  private:
    __host__ void Write(double t, unsigned int time_steps_to_write = 0);

    TireType m_tire_type;  // Tire type
    double m_step;         // integration time step
    bool m_output;         // data output flag
    double m_dtout;        // time interval between data output
    double m_tend;     // end time -> This has to be set by the user through the setter function else a error is thrown
    bool m_store_all;  // Flag for whether all the vehicles being simulated need to be dumped into a csv file
    unsigned int m_num_outs;  // Number of vehicles to be dumped into a csv file if m_store_all is false. Which vehicles
                              // are dumped is picked randomly
    std::vector<unsigned int> m_which_outs;       // Vector of vehicle indices that need to be dumped into a csv file
    std::string m_output_file;                    // output file path
    DriverData m_driver_data;                     // driver inputs
    int m_num_controls;                           // Number of control states for jacobian
    int m_num_states;                             // Number of states for jacobian
    unsigned int m_total_num_vehicles;            // Number of vehicles
    unsigned int m_vehicle_count_tracker_params;  // Keeps track of number of vehicles initialized to ensure that total
                                                  // vehicles match with the vehicles initialized through construct and
                                                  // initialize. This is for construct which initializes parameters.
                                                  // There is another one for states
    unsigned int m_vehicle_count_tracker_states;  // Keeps track of number of vehicles initialized to ensure that total
    unsigned int m_device_collection_timeSteps;   // Number of timesteps out of the total time steps that need to be
                                                  // stored in the device array. Calculated in the SetOutput function
    unsigned int m_host_collection_timeSteps;  // Number of timesteps out of the total time steps that need to be stored
                                               // in the host array. Calculated in the SetOutput function based on the
                                               // host dump time and the collection time step.
    // unsigned int m_kernel_calls_per_csv_dump;  // Number of kernel calls that need to be executed before the host
    // array
    //                                            // is dumped into a csv file. Calculated in the SetOutput function
    //                                            based
    //                                            // on the m_host_dump_time and the m_kernel_sim_time.
    unsigned int m_collection_states;  // Number of states out of the total states that need to be stored in
                                       // the device/host array. Set in the SetOutput function

    unsigned int m_device_size;  // Size of the device array. Calculated in the SetOutput function

    unsigned int m_num_blocks;  // Number of blocks to be launched

    // Device and host response vectors that contain all the states from all the vehicles at all the timesteps -> This
    // is not to be used by the user, but rather is used internally to write to file
    double* m_device_response;
    double* m_host_response;

    // Need to track kernel launches and time since last kernel launches for the solveStep function
    unsigned int m_kernel_launches_since_last_dump;
    double m_time_since_last_dump;
    // Need to keep track of current time in SolveStep function
    double m_current_time;

    // Need a bunch of csv writers for each of the vehicles. We will store a pointer to this list
    std::unique_ptr<CSV_writer[]> m_csv_writers_ptr;
};

// Cannout have global function as class member function
/// @brief Integrate the system of equations using the half implicit method - Calls the RHS function at each time step
__global__ void Integrate(double current_time,
                          double kernel_sim_time,
                          double step,
                          bool output,
                          unsigned int total_num_vehicles,
                          unsigned int collection_states,
                          double dtout,
                          double* device_response,
                          d18::SimDataNr* sim_data_nr,
                          d18::SimStateNr* sim_states_nr);
// For the case where the dirver inputs are provided at each time step
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
                          d18::SimDataNr* sim_data_nr,
                          d18::SimStateNr* sim_states_nr);
__global__ void Integrate(double current_time,
                          double kernel_sim_time,
                          double step,
                          bool output,
                          unsigned int total_num_vehicles,
                          unsigned int collection_states,
                          double dtout,
                          double* device_response,
                          d18::SimData* sim_data,
                          d18::SimState* sim_states);
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
                          d18::SimData* sim_data,
                          d18::SimState* sim_states);
//===================================================================================================================
// Integrate calss rhsFun so this also cannot be a class member function
/// @brief Computes the RHS of all the ODEs (tire velocities, chassis accelerations)
/// @param t Current time
__device__ void rhsFun(double t, unsigned int total_num_vehicles, d18::SimData* sim_data, d18::SimState* sim_states);
// For the case where the dirver inputs are provided at each time step
__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d18::SimData* sim_data,
                       d18::SimState* sim_states,
                       double steering,
                       double throttle,
                       double braking);
__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d18::SimDataNr* sim_data_nr,
                       d18::SimStateNr* sim_states_nr);
__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d18::SimDataNr* sim_data_nr,
                       d18::SimStateNr* sim_states_nr,
                       double steering,
                       double throttle,
                       double braking);

//===================================================================================================================
template <typename Func>
__global__ void Integrate(double current_time,
                          Func func,
                          double kernel_sim_time,
                          double step,
                          bool output,
                          unsigned int total_num_vehicles,
                          unsigned int collection_states,
                          double dtout,
                          double* device_response,
                          d18::SimData* sim_data,
                          d18::SimState* sim_states) {
    double t = current_time;           // Set the current time
    double kernel_time = 0;            // Time since kernel was launched
    unsigned int timeStep_stored = 0;  // Number of time steps already stored in the device response
    double end_time = (t + kernel_sim_time) - step / 10.;
    unsigned int vehicle_id = blockIdx.x * blockDim.x + threadIdx.x;  // Get the vehicle id
    if (vehicle_id < total_num_vehicles) {
        while (t < end_time) {
            // Call the RHS to get accelerations for all the vehicles
            rhsFun(t, total_num_vehicles, sim_data, sim_states, func);

            // Integrate according to half implicit method for second order states
            // Integrate according to explicit method for first order states

            // Extract the states of the vehicle and the tires
            d18::VehicleState& v_states = sim_states[vehicle_id]._veh_state;
            d18::VehicleParam& veh_param = sim_data[vehicle_id]._veh_param;
            d18::TMeasyState& tirelf_st = sim_states[vehicle_id]._tirelf_state;
            d18::TMeasyState& tirerf_st = sim_states[vehicle_id]._tirerf_state;
            d18::TMeasyState& tirelr_st = sim_states[vehicle_id]._tirelr_state;
            d18::TMeasyState& tirerr_st = sim_states[vehicle_id]._tirerr_state;

            // First the tire states
            // LF
            tirelf_st._xe += tirelf_st._xedot * step;
            tirelf_st._ye += tirelf_st._yedot * step;
            tirelf_st._omega += tirelf_st._dOmega * step;
            // RF
            tirerf_st._xe += tirerf_st._xedot * step;
            tirerf_st._ye += tirerf_st._yedot * step;
            tirerf_st._omega += tirerf_st._dOmega * step;
            // LR
            tirelr_st._xe += tirelr_st._xedot * step;
            tirelr_st._ye += tirelr_st._yedot * step;
            tirelr_st._omega += tirelr_st._dOmega * step;
            // RR
            tirerr_st._xe += tirerr_st._xedot * step;
            tirerr_st._ye += tirerr_st._yedot * step;
            tirerr_st._omega += tirerr_st._dOmega * step;

            // Now the vehicle states
            if (veh_param._tcbool) {
                v_states._crankOmega += v_states._dOmega_crank * step;
            }

            // Integrate velocity level first
            v_states._u += v_states._udot * step;
            v_states._v += v_states._vdot * step;
            v_states._wx += v_states._wxdot * step;
            v_states._wz += v_states._wzdot * step;

            // Integrate position level next
            v_states._x += (v_states._u * cos(v_states._psi) - v_states._v * sin(v_states._psi)) * step;
            v_states._y += (v_states._u * sin(v_states._psi) + v_states._v * cos(v_states._psi)) * step;
            v_states._psi += v_states._wz * step;
            v_states._phi += v_states._wx * step;

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
                    device_response[time_offset + (total_num_vehicles * 7) + vehicle_id] = v_states._wx;
                    device_response[time_offset + (total_num_vehicles * 8) + vehicle_id] = v_states._wz;
                    device_response[time_offset + (total_num_vehicles * 9) + vehicle_id] = tirelf_st._omega;
                    device_response[time_offset + (total_num_vehicles * 10) + vehicle_id] = tirerf_st._omega;
                    device_response[time_offset + (total_num_vehicles * 11) + vehicle_id] = tirelr_st._omega;
                    device_response[time_offset + (total_num_vehicles * 12) + vehicle_id] = tirerr_st._omega;
                    timeStep_stored++;
                }
            }
        }
    }
}
// When controls is provided as a lambda function
template <typename Func>
__global__ void Integrate(double current_time,
                          Func func,
                          double kernel_sim_time,
                          double step,
                          bool output,
                          unsigned int total_num_vehicles,
                          unsigned int collection_states,
                          double dtout,
                          double* device_response,
                          d18::SimDataNr* sim_data_nr,
                          d18::SimStateNr* sim_states_nr) {
    double t = current_time;           // Set the current time
    double kernel_time = 0;            // Time since kernel was launched
    unsigned int timeStep_stored = 0;  // Number of time steps already stored in the device response

    unsigned int vehicle_id = blockIdx.x * blockDim.x + threadIdx.x;  // Get the vehicle id
    double end_time = (t + kernel_sim_time) - step / 10.;
    if (vehicle_id < total_num_vehicles) {
        while (t < end_time) {
            // Call the RHS to get accelerations for all the vehicles
            rhsFun(t, total_num_vehicles, sim_data_nr, sim_states_nr, func);
            // Extract the states of the vehicle and the tires
            d18::VehicleState& v_states = sim_states_nr[vehicle_id]._veh_state;
            d18::VehicleParam& veh_param = sim_data_nr[vehicle_id]._veh_param;
            d18::TMeasyNrState& tirelf_st = sim_states_nr[vehicle_id]._tirelf_state;
            d18::TMeasyNrState& tirerf_st = sim_states_nr[vehicle_id]._tirerf_state;
            d18::TMeasyNrState& tirelr_st = sim_states_nr[vehicle_id]._tirelr_state;
            d18::TMeasyNrState& tirerr_st = sim_states_nr[vehicle_id]._tirerr_state;

            // First the tire states
            // LF
            tirelf_st._omega += tirelf_st._dOmega * step;
            // RF
            tirerf_st._omega += tirerf_st._dOmega * step;
            // LR
            tirelr_st._omega += tirelr_st._dOmega * step;
            // RR
            tirerr_st._omega += tirerr_st._dOmega * step;

            // Now the vehicle states
            if (veh_param._tcbool) {
                v_states._crankOmega += v_states._dOmega_crank * step;
            }

            // Integrate velocity level first
            v_states._u += v_states._udot * step;
            v_states._v += v_states._vdot * step;
            v_states._wx += v_states._wxdot * step;
            v_states._wz += v_states._wzdot * step;
            // Integrate position level next
            v_states._x += (v_states._u * cos(v_states._psi) - v_states._v * sin(v_states._psi)) * step;
            v_states._y += (v_states._u * sin(v_states._psi) + v_states._v * cos(v_states._psi)) * step;
            v_states._psi += v_states._wz * step;
            v_states._phi += v_states._wx * step;

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
                    device_response[time_offset + (total_num_vehicles * 7) + vehicle_id] = v_states._wx;
                    device_response[time_offset + (total_num_vehicles * 8) + vehicle_id] = v_states._wz;
                    device_response[time_offset + (total_num_vehicles * 9) + vehicle_id] = tirelf_st._omega;
                    device_response[time_offset + (total_num_vehicles * 10) + vehicle_id] = tirerf_st._omega;
                    device_response[time_offset + (total_num_vehicles * 11) + vehicle_id] = tirelr_st._omega;
                    device_response[time_offset + (total_num_vehicles * 12) + vehicle_id] = tirerr_st._omega;
                    timeStep_stored++;
                }
            }
        }
    }
}
//===================================================================================================================
// For the case where the dirver inputs are provided as a lambda function
template <typename Func>
__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d18::SimData* sim_data,
                       d18::SimState* sim_states,
                       Func func) {
    // Get the vehicle index
    unsigned int vehicle_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (vehicle_index < total_num_vehicles) {
        // All vehicles have one or the other tire type and thus no thread divergence
        d18::VehicleParam& veh_param = sim_data[vehicle_index]._veh_param;
        d18::VehicleState& veh_state = sim_states[vehicle_index]._veh_state;
        d18::TMeasyParam& tireTM_param = sim_data[vehicle_index]._tireTM_param;
        d18::TMeasyState& tireTMlf_state = sim_states[vehicle_index]._tirelf_state;
        d18::TMeasyState& tireTMrf_state = sim_states[vehicle_index]._tirerf_state;
        d18::TMeasyState& tireTMlr_state = sim_states[vehicle_index]._tirelr_state;
        d18::TMeasyState& tireTMrr_state = sim_states[vehicle_index]._tirerr_state;

        // Get controls at the current timeStep
        DriverInput controls;
        func(t, &controls);  // Use the functor to get the controls for this time step

        double loads[4] = {0., 0., 0., 0.};
        // Compute the tire loads
        computeTireLoads(&loads[0], &veh_state, &veh_param, &tireTM_param);
        // Transform from vehicle frame to the tire frame
        vehToTireTransform(&tireTMlf_state, &tireTMrf_state, &tireTMlr_state, &tireTMrr_state, &veh_state, &loads[0],
                           &veh_param, controls.m_steering);

        // Tire velocities using TMEasy tire
        computeTireRHS(&tireTMlf_state, &tireTM_param, &veh_param, controls.m_steering);
        computeTireRHS(&tireTMrf_state, &tireTM_param, &veh_param, controls.m_steering);
        computeTireRHS(&tireTMlr_state, &tireTM_param, &veh_param, 0);  // No rear steering
        computeTireRHS(&tireTMrr_state, &tireTM_param, &veh_param, 0);  // No rear steering

        // Powertrain dynamics
        computePowertrainRHS(&veh_state, &tireTMlf_state, &tireTMrf_state, &tireTMlr_state, &tireTMrr_state, &veh_param,
                             &tireTM_param, &controls);
        // Vehicle dynamics
        tireToVehTransform(&tireTMlf_state, &tireTMrf_state, &tireTMlr_state, &tireTMrr_state, &veh_state, &veh_param,
                           controls.m_steering);

        double fx[4] = {tireTMlf_state._fx, tireTMrf_state._fx, tireTMlr_state._fx, tireTMrr_state._fx};
        double fy[4] = {tireTMlf_state._fy, tireTMrf_state._fy, tireTMlr_state._fy, tireTMrr_state._fy};

        computeVehRHS(&veh_state, &veh_param, &fx[0], &fy[0]);
    }
}

template <typename Func>
__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d18::SimDataNr* sim_data_nr,
                       d18::SimStateNr* sim_states_nr,
                       Func func) {
    // Get the vehicle index
    unsigned int vehicle_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (vehicle_index < total_num_vehicles) {
        d18::VehicleParam& veh_param = sim_data_nr[vehicle_index]._veh_param;
        d18::VehicleState& veh_state = sim_states_nr[vehicle_index]._veh_state;
        d18::TMeasyNrParam& tireTMNr_param = sim_data_nr[vehicle_index]._tireTMNr_param;
        d18::TMeasyNrState& tireTMNrlf_state = sim_states_nr[vehicle_index]._tirelf_state;
        d18::TMeasyNrState& tireTMNrrf_state = sim_states_nr[vehicle_index]._tirerf_state;
        d18::TMeasyNrState& tireTMNrlr_state = sim_states_nr[vehicle_index]._tirelr_state;
        d18::TMeasyNrState& tireTMNrrr_state = sim_states_nr[vehicle_index]._tirerr_state;

        // Use the functor to get the controls for this time step
        DriverInput controls;
        func(t, &controls);
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
#endif