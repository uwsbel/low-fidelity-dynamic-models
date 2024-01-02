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

  private:
    __host__ void Write(double t);

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
// Integrate calss rhsFun so this also cannot be a class member function
/// @brief Computes the RHS of all the ODEs (tire velocities, chassis accelerations)
/// @param t Current time
__device__ void rhsFun(double t, unsigned int total_num_vehicles, d18::SimData* sim_data, d18::SimState* sim_states);
__device__ void rhsFun(double t,
                       unsigned int total_num_vehicles,
                       d18::SimDataNr* sim_data_nr,
                       d18::SimStateNr* sim_states_nr);

#endif