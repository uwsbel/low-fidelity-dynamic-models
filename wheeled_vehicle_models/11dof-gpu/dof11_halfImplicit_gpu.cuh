#ifndef DOF11_HALFIMPLICIT_GPU_CUH
#define DOF11_HALFIMPLICIT_GPU_CUH

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <vector>
#include <memory>

#include "dof11_gpu.cuh"

// =============================================================================
// Solver class
// =============================================================================

class d11SolverHalfImplicitGPU {
  public:
    /// @brief Initialize the solver with the total number of vehicles to be simulated
    /// @param num_vehicles Number of vehicles to be simulated
    d11SolverHalfImplicitGPU(unsigned int total_num_vehicles);
    ~d11SolverHalfImplicitGPU();

    /// @brief Construct the solver using path to vehicle parameters, tire parameters, number of
    /// vehicles and driver inputs.

    /// Each of these vehicles will have the specified parameters and driver inputs. To add
    /// more vehicles (ensuring they are still lesser than the total number of vehicles initally specified in the class
    /// constructor) with different parameters, call the Construct function again. The tire type defaults to TMEasy and
    /// is set for all the vehilces. It is also important to note thet the TireType has to be consistent across all the
    /// vehicles currently. For examples of use, see demos.
    /// @param vehicle_params_file Path to the vehicle parameter json file
    /// @param tire_params_file Path to the tire parameter json file
    /// @param num_vehicles Number of vehicles to be simulated with the specified parameters and driver inputs
    /// @param driver_inputs_file Path to the driver inputs text file
    __host__ void Construct(const std::string& veh_params_file,
                            const std::string& tire_params_file,
                            unsigned int num_vehicles,
                            const std::string& driver_file);

    /// @brief Construct the the solver using path to vehicle parameters, tire parameters,  number
    /// of vehicles, driver inputs and the TireType (Either TMEasy or TMEasyNr).

    /// Each of these vehicles will have the
    /// specified parameters and driver inputs. To add more vehicles with different parameters, call the Construct
    /// function again. It is also important to note thet the TireType has to be consistent across all the
    /// vehicles currently. For examples of use, see demos.
    /// @param vehicle_params_file Path to the vehicle parameter json file
    /// @param tire_params_file Path to the tire parameter json file
    /// @param num_vehicles Number of vehicles to be simulated with the specified parameters and driver inputs
    /// @param driver_inputs_file Path to the driver inputs text file
    /// @param type TireType (Either TMEasy or TMEasyNr)
    __host__ void Construct(const std::string& veh_params_file,
                            const std::string& tire_params_file,
                            unsigned int num_vehicles,
                            const std::string& driver_file,
                            TireType type);

    /// @brief Construct the the solver using path to vehicle parameters, tire parameters, number
    /// of vehicles.

    /// Each of these vehicles will have the specified parameters. This function signature
    /// is mainly provided for cases where the driver inputs are not available at the start of the simulation but rather
    /// come from a controller during the simualtion. TireType defualts to TMEasy tires. To add more vehicles with
    /// different parameters, call the Construct function again. It is also important to note thet the TireType has to
    /// be consistent across all the vehicles currently. For examples of use, see demos.
    /// @param vehicle_params_file Path to the vehicle parameter json file
    /// @param tire_params_file Path to the tire parameter json file
    /// @param num_vehicles Number of vehicles to be simulated with the specified parameters and driver inputs
    __host__ void Construct(const std::string& vehicle_params_file,
                            const std::string& tire_params_file,
                            unsigned int num_vehicles);

    /// @brief Construct the the solver using path to vehicle parameters, tire parameters, number
    /// of vehicles and the TireType (Either TMEasy or TMEasyNr).

    /// Each of these vehicles will have the specified
    /// parameters. This is mainly provided for cases where the driver inputs are not available at the start of the
    /// simulation but rather come from a controller during the simualtion and the user wants to specify a TireType. To
    /// add more vehicles with different parameters, call the Construct function again. It is also important to note
    /// thet the TireType has to be consistent across all the vehicles currently. For examples of use, see demos.
    /// @param vehicle_params_file Path to the vehicle parameter json file
    /// @param tire_params_file Path to the tire parameter json file
    /// @param num_vehicles Number of vehicles to be simulated with the specified parameters and driver inputs
    /// @param type TireType (Either TMEasy or TMEasyNr)
    __host__ void Construct(const std::string& vehicle_params_file,
                            const std::string& tire_params_file,
                            unsigned int num_vehicles,
                            TireType type);

    /// @brief  Set the simulation end time.

    /// This function has to be called by the user. All vehicles will be simulated to this time.
    /// @param tend End time to set
    __host__ void SetEndTime(double tend) { m_tend = tend; }

    /// @brief Set the simulation time step used to integrate all the vehicles using the half implicit solver
    /// @param step time step to set
    __host__ void SetTimeStep(double step) { m_step = step; }

    /// @brief Set the time for which the GPU kernel simulates without a sync between the vehicles

    /// The simulation proceeds by multiple launches of the GPU kernel for m_kernel_sim_time duration. This is mainly
    /// for memory reasons and is discussed in this document
    /// https://uwmadison.box.com/s/2tsvr4adbrzklle30z0twpu2nlzvlayc. For the user, this means that in case you use
    /// SolveStep to solve, control will only be returned to the CPU (or to you) after m_kernel_sim_time. Thus, in a
    /// control setting, m_kernel_sim_time should be set to the time between two control calls. This is set to 2 seconds
    /// by default but can be changed by the user using this function.
    /// @param time Time to set
    __host__ void SetKernelSimTime(double time) { m_kernel_sim_time = time; }

    /// @brief Set the time interval between data output to a csv file

    /// This is usually not required to be set by the user and defaults to 10 seconds. However, this will impact speed
    /// of simulation as the host array is written into a csv writer every m_host_dump_time. Thus, setting a low value
    /// will slow down the simulation. However, based on the users memory constraints, this can be set to a higher value
    /// to speed up the simulation.
    /// @param time Time to be set
    __host__ void SetHostDumpTime(double time) { m_host_dump_time = time; }

    /// @brief Sets the threads per block for the GPU kernel. Defaults to 32
    /// @param threads Threads per block to be set
    __host__ void SetThreadsPerBlock(unsigned int threads) { m_threads_per_block = threads; }

    /// @brief Getter function for simulation time step
    __host__ __device__ double GetStep() { return m_step; }

    /// @brief Getter function for the sim,ulation end time
    __host__ __device__ double GetEndTime() { return m_tend; }

    /// Note: If the SolveStep is used, no output can be set currently. If the user desires to obtain vehicle state
    /// information, they can query the states using the GetSimState function. See demo demos/HMMWV/demo_hmmwv_step.cu
    /// for an example.
    /// @param output_file Path to the output file
    /// @param output_freq Frequency of data output
    /// @param store_all Flag to store data from each vehicle on all the threads or no_outs vehicles
    /// @param no_outs Number of outputs to store if store_all is false
    __host__ void SetOutput(const std::string& output_file,
                            double output_freq,
                            bool store_all = false,
                            unsigned int no_outs = 50);

    /// @brief Solve the system of equations and run the simulation

    /// This will run the simulation for all the vehicles upto the end time specified by the user. The user can then use
    /// the GetSimState function to query the states of the vehicles. If the user has set the output to a csv file, then
    /// the csv files will be written to the data/output folder.
    __host__ void Solve();

    /// User can set the kernel step with the SetKernelSimTime function.
    /// @param t Current time
    /// @param steering Steering input
    /// @param throttle Throttle input
    /// @param braking Braking input
    /// @return Time after the kernel step
    __host__ double SolveStep(double t, double steering, double throttle, double braking);

    /// @brief Initialize vehicle and tire states.

    /// This function has to be called before solve and after the
    ///  Construct function is called. Although it is possible to provide non-zero intial states, this is untested and
    ///  it is recommended to use the default zero states. To initialize more vehicles, call the Initialize function
    ///  again.  For examples of use, see demos.
    /// @param vehicle_states Vehicle states
    /// @param tire_states_F Front (F) TMeasy tire states
    /// @param tire_states_R Rear (R) TMeasy tire states
    /// @param num_vehicles Number of vehicles to be simulated with the specified states
    __host__ void Initialize(d11GPU::VehicleState& vehicle_states,
                             d11GPU::TMeasyState& tire_states_F,
                             d11GPU::TMeasyState& tire_states_R,
                             unsigned int num_vehicles);

    /// @brief Initialize vehicle and tire states. Overloaded for TMesayNr tires.

    /// This function has to be called before solve and after the
    /// construct. Although it is possible to provide non-zero intial states, this is untested and it is recommended to
    /// use the default zero states. To initialize more vehicles, call the Initialize function again.
    /// @param vehicle_states Vehicle states
    /// @param tire_states_F Front (F) TMeasyNr tire states
    /// @param tire_states_R Rear (R) TMeasyNr tire states
    /// @param num_vehicles Number of vehicles to be simulated with the specified states
    __host__ void Initialize(d11GPU::VehicleState& vehicle_states,
                             d11GPU::TMeasyNrState& tire_states_F,
                             d11GPU::TMeasyNrState& tire_states_R,
                             unsigned int num_vehicles);

    /// @brief Get a SimState object for a particular vehicle given by its index

    /// This is particularly useful when SolveStep is called where the user cannot access the states of the vehicles via
    /// csv file outputs
    /// @param vehicle_index Index of the vehicle
    /// @return SimState object for the vehicle
    __host__ d11GPU::SimState GetSimState(unsigned int vehicle_index);

    /// @brief Get Tire type
    /// @return Tire type used in the solver
    TireType GetTireType() const { return m_tire_type; }

    d11GPU::SimData* m_sim_data;          ///< Simulation data for all the vehicles. See d11GPU::SimData for more info
    d11GPU::SimDataNr* m_sim_data_nr;     ///< Simulation data but with the TMeasyNr tire for all the vehicles. See
                                          // d11GPU::SimDataNr for more info
    d11GPU::SimState* m_sim_states;       ///< Simulation states for all the vehicles. See d11GPU::SimState for more
                                          // info
    d11GPU::SimStateNr* m_sim_states_nr;  ///< Simulation states but with the TMeasyNr tire for all the vehicles. See
                                          // d11GPU::SimStateNr for more info

    double m_kernel_sim_time;  ///< The maximum time a kernel launch simulates a vehicle. Defaults to 2 seconds. See
                               ///< SetKernelSimTime for more
                               // info
    double m_host_dump_time;   ///< Time after which the host array is dumped into a csv file. Default is set to 10
                               ///< seconds. See SetHostDumpTime for more info
    unsigned int
        m_threads_per_block;  ///< Number of threads per block. This is set to 32 by default but can be changed by
                              // the user

    /// @brief Solve the system of equations for a kernel step by passing a functor that provides the driver inputs
    /// at a given time

    /// An example of the use of this function can be seen in demos/HMMWV/demo_hmmwv_controlsFunctor.cu.
    /// @param t Current time
    /// @param func Functor that provides the driver inputs given a time. This functor should take 2 arguments: 1) The
    /// current time 2) A pointer to DriverInput. It should then fill in the m_steering, m_throttle and m_braking
    /// variables for the DriverInput
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
    /// @brief When using the IntegrateStep, once the integration is complete, this function can be used to Write the
    /// output to a file specified by the user in the SetOutput function.
    __host__ void WriteToFile();

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

/// Used internally within Solve
__global__ void Integrate(double current_time,
                          double kernel_sim_time,
                          double step,
                          bool output,
                          unsigned int total_num_vehicles,
                          unsigned int collection_states,
                          double dtout,
                          double* device_response,
                          d11GPU::SimDataNr* sim_data_nr,
                          d11GPU::SimStateNr* sim_states_nr);

/// @brief Overload to provide driver inputs at each time step.

/// Used internally within SolveStep
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
                          d11GPU::SimDataNr* sim_data_nr,
                          d11GPU::SimStateNr* sim_states_nr);

/// @brief Overload for TMesay tires

/// Used internally within Solve
__global__ void Integrate(double current_time,
                          double kernel_sim_time,
                          double step,
                          bool output,
                          unsigned int total_num_vehicles,
                          unsigned int collection_states,
                          double dtout,
                          double* device_response,
                          d11GPU::SimData* sim_data,
                          d11GPU::SimState* sim_states);

/// @brief Overload for TMeasy tires and when driver inputs are provided at each time step

/// Used internally when SolveStep is called
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
                          d11GPU::SimData* sim_data,
                          d11GPU::SimState* sim_states);
//===================================================================================================================
// Integrate calss rhsFun so this also cannot be a class member function

/// @brief Computes the RHS of all the ODEs (tire velocities, chassis accelerations) using the computeRHS functions
/// within the vehicle model

/// Used internally within Integrate
__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d11GPU::SimData* sim_data,
                       d11GPU::SimState* sim_states);

/// @brief Overloaded for when driver inputs are provided at each time step

/// Used Internally within Integrate
__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d11GPU::SimData* sim_data,
                       d11GPU::SimState* sim_states,
                       double steering,
                       double throttle,
                       double braking);

/// @brief Overloaded for TMeasyNr tires

/// Used internally within Integrate
__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d11GPU::SimDataNr* sim_data_nr,
                       d11GPU::SimStateNr* sim_states_nr);

/// @brief Overloaded for TMeasyNr tires and when driver inputs are provided at each time step

/// Used internally within Integrate
__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d11GPU::SimDataNr* sim_data_nr,
                       d11GPU::SimStateNr* sim_states_nr,
                       double steering,
                       double throttle,
                       double braking);
//===================================================================================================================
/// @brief Integrate the system of equations using the half implicit method

/// Calls the RHS function at each time step and uses the controls functor for the controls

/// Used internally within SolveStep
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
                          d11GPU::SimData* sim_data,
                          d11GPU::SimState* sim_states) {
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
            d11GPU::VehicleState& v_states = sim_states[vehicle_id]._veh_state;
            d11GPU::VehicleParam& veh_param = sim_data[vehicle_id]._veh_param;
            d11GPU::TMeasyState& tiref_st = sim_states[vehicle_id]._tiref_state;
            d11GPU::TMeasyState& tirer_st = sim_states[vehicle_id]._tirer_state;

            // First the tire states
            // F
            tiref_st._xe += tiref_st._xedot * step;
            tiref_st._ye += tiref_st._yedot * step;
            tiref_st._omega += tiref_st._dOmega * step;
            // R
            tirer_st._xe += tirer_st._xedot * step;
            tirer_st._ye += tirer_st._yedot * step;
            tirer_st._omega += tirer_st._dOmega * step;

            // Now the vehicle states
            if (veh_param._tcbool) {
                v_states._crankOmega += v_states._dOmega_crank * step;
            }

            // Integrate velocity level first
            v_states._u += v_states._udot * step;
            v_states._v += v_states._vdot * step;
            v_states._wz += v_states._wzdot * step;

            // Integrate position level next
            v_states._x += (v_states._u * cos(v_states._psi) - v_states._v * sin(v_states._psi)) * step;
            v_states._y += (v_states._u * sin(v_states._psi) + v_states._v * cos(v_states._psi)) * step;
            v_states._psi += v_states._wz * step;

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
                    device_response[time_offset + (total_num_vehicles * 5) + vehicle_id] = v_states._psi;
                    device_response[time_offset + (total_num_vehicles * 6) + vehicle_id] = v_states._wz;
                    device_response[time_offset + (total_num_vehicles * 7) + vehicle_id] = tiref_st._omega;
                    device_response[time_offset + (total_num_vehicles * 8) + vehicle_id] = tirer_st._omega;
                    timeStep_stored++;
                }
            }
        }
    }
}
/// @brief Overloaded for TMeasyNr tires

/// Used internally within SolveStep
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
                          d11GPU::SimDataNr* sim_data,
                          d11GPU::SimStateNr* sim_states) {
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
            d11GPU::VehicleState& v_states = sim_states[vehicle_id]._veh_state;
            d11GPU::VehicleParam& veh_param = sim_data[vehicle_id]._veh_param;
            d11GPU::TMeasyNrState& tiref_st = sim_states[vehicle_id]._tiref_state;
            d11GPU::TMeasyNrState& tirer_st = sim_states[vehicle_id]._tirer_state;

            // First the tire states
            // F
            tiref_st._omega += tiref_st._dOmega * step;
            // R
            tirer_st._omega += tirer_st._dOmega * step;

            // Now the vehicle states
            if (veh_param._tcbool) {
                v_states._crankOmega += v_states._dOmega_crank * step;
            }

            // Integrate velocity level first
            v_states._u += v_states._udot * step;
            v_states._v += v_states._vdot * step;
            v_states._wz += v_states._wzdot * step;

            // Integrate position level next
            v_states._x += (v_states._u * cos(v_states._psi) - v_states._v * sin(v_states._psi)) * step;
            v_states._y += (v_states._u * sin(v_states._psi) + v_states._v * cos(v_states._psi)) * step;
            v_states._psi += v_states._wz * step;

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
                    device_response[time_offset + (total_num_vehicles * 5) + vehicle_id] = v_states._psi;
                    device_response[time_offset + (total_num_vehicles * 6) + vehicle_id] = v_states._wz;
                    device_response[time_offset + (total_num_vehicles * 7) + vehicle_id] = tiref_st._omega;
                    device_response[time_offset + (total_num_vehicles * 8) + vehicle_id] = tirer_st._omega;
                    timeStep_stored++;
                }
            }
        }
    }
}
//===================================================================================================================
/// @brief Computes the RHS of all the ODEs (tire velocities, chassis accelerations) using the computeRHS functions
/// within the vehicle model.

/// Overloaded for Controls Functor. Used internally within Integrate
template <typename Func>
__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d11GPU::SimData* sim_data,
                       d11GPU::SimState* sim_states,
                       Func func) {
    // Get the vehicle index
    unsigned int vehicle_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (vehicle_index < total_num_vehicles) {
        // All vehicles have one or the other tire type and thus no thread divergence
        d11GPU::VehicleParam& veh_param = sim_data[vehicle_index]._veh_param;
        d11GPU::VehicleState& veh_state = sim_states[vehicle_index]._veh_state;
        d11GPU::TMeasyParam& tireTM_param = sim_data[vehicle_index]._tireTM_param;
        d11GPU::TMeasyState& tireTMf_state = sim_states[vehicle_index]._tiref_state;
        d11GPU::TMeasyState& tireTMr_state = sim_states[vehicle_index]._tirer_state;

        // Get controls at the current timeStep
        DriverInput controls;
        func(t, &controls);  // Use the functor to get the controls for this time step

        double loads[2] = {0., 0.};
        // Compute the tire loads
        computeTireLoads(&loads[0], &veh_state, &veh_param, &tireTM_param);
        // Transform from vehicle frame to the tire frame
        vehToTireTransform(&tireTMf_state, &tireTMr_state, &veh_state, &loads[0], &veh_param, controls.m_steering);

        // Tire velocities using TMEasy tire
        computeTireRHS(&tireTMf_state, &tireTM_param, &veh_param, controls.m_steering);
        computeTireRHS(&tireTMr_state, &tireTM_param, &veh_param, 0.);

        // Powertrain dynamics
        computePowertrainRHS(&veh_state, &tireTMf_state, &tireTMr_state, &veh_param, &tireTM_param, &controls);
        // Vehicle dynamics
        tireToVehTransform(&tireTMf_state, &tireTMr_state, &veh_state, &veh_param, controls.m_steering);

        double fx[2] = {tireTMf_state._fx, tireTMr_state._fx};
        double fy[2] = {tireTMf_state._fy, tireTMr_state._fy};

        computeVehRHS(&veh_state, &veh_param, &fx[0], &fy[0]);
    }
}
/// @brief Overloaded for TMeasyNr tires and for the Controls Functor

/// Used internally within Integrate
template <typename Func>
__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d11GPU::SimDataNr* sim_data_nr,
                       d11GPU::SimStateNr* sim_states_nr,
                       Func func) {  // Get the vehicle index
    unsigned int vehicle_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (vehicle_index < total_num_vehicles) {
        d11GPU::VehicleParam& veh_param = sim_data_nr[vehicle_index]._veh_param;
        d11GPU::VehicleState& veh_state = sim_states_nr[vehicle_index]._veh_state;
        d11GPU::TMeasyNrParam& tireTMNr_param = sim_data_nr[vehicle_index]._tireTMNr_param;
        d11GPU::TMeasyNrState& tireTMNrf_state = sim_states_nr[vehicle_index]._tiref_state;
        d11GPU::TMeasyNrState& tireTMNrr_state = sim_states_nr[vehicle_index]._tirer_state;

        // Get controls at the current timeStep
        DriverInput controls;
        func(t, &controls);  // Use the functor to get the controls for this time step

        double loads[2] = {0., 0.};

        // Compute the tire loads
        computeTireLoads(&loads[0], &veh_state, &veh_param, &tireTMNr_param);
        // Transform from vehicle frame to the tire frame
        vehToTireTransform(&tireTMNrf_state, &tireTMNrr_state, &veh_state, &loads[0], &veh_param, controls.m_steering);
        // Tire velocities using TMEasyNr tire
        computeTireRHS(&tireTMNrf_state, &tireTMNr_param, &veh_param, controls.m_steering);
        computeTireRHS(&tireTMNrr_state, &tireTMNr_param, &veh_param, 0.);

        // Powertrain dynamics
        computePowertrainRHS(&veh_state, &tireTMNrf_state, &tireTMNrr_state, &veh_param, &tireTMNr_param, &controls);

        // Vehicle dynamics
        tireToVehTransform(&tireTMNrf_state, &tireTMNrr_state, &veh_state, &veh_param, controls.m_steering);

        double fx[2] = {tireTMNrf_state._fx, tireTMNrr_state._fx};
        double fy[2] = {tireTMNrf_state._fy, tireTMNrr_state._fy};

        computeVehRHS(&veh_state, &veh_param, &fx[0], &fy[0]);
    }
}
#endif