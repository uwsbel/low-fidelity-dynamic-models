# This demo is intended to provide an example of how to use the Low fidelity models with the ART vehicle for doing something like a PID control where you might have to interate multiple times in a loop from the same state

import numpy as np
import pydof18 as dof18
import sys


def dof18_model(solverOld, t, required_time, throttle, steering):

    # Create a new solver object
    solverNew = dof18.d18SolverHalfImplicit()

    # copy over solver parameters that were there at the start of the iterations (ideally also the same as the initial parameters)
    solverNew.m_veh_param = dof18.VehicleParam(solverOld.m_veh_param)
    solverNew.m_tire_param = dof18.TMeasyParam(solverOld.m_tire_param)

    # Set the simulation time step
    solverNew.SetTimeStep(1e-3)

    # Copy over the solver states that were there at the start of the iterations
    solverNew.m_veh_st = dof18.VehicleState(solverOld.m_veh_state)
    solverNew.m_tirelf_st = dof18.TMeasyState(solverOld.m_tirelf_state)
    solverNew.m_tirerf_st = dof18.TMeasyState(solverOld.m_tirerf_state)
    solverNew.m_tirelr_st = dof18.TMeasyState(solverOld.m_tirelr_state)
    solverNew.m_tirerr_st = dof18.TMeasyState(solverOld.m_tirerr_state)

    # Integrate till the required time
    while t < required_time:
        # Integrate till the next time step
        new_time = solverNew.IntegrateStep(t, throttle, steering, 0)
        t = new_time  # new_time is where the solver is right now

    # Once we have intgrated till the required time, we can return the solver object

    return solverNew


def pid_control(error, error_sum, error_diff, error_prev):
    # Set PID parameters
    Kp = 1
    Ki = 1
    Kd = 1

    # do some PID

    error_sum += error
    error_diff = error - error_prev
    error_prev = error

    throttle = Kp*error + Ki*error_sum + Kd*error_diff

    throttle = np.clip(throttle, -1, 1)

    return throttle


if __name__ == "__main__":
    # Initialize the vehicle here

    # Parameters path
    vehParamsJsonPath = "../../data/json/ART/vehicle.json"
    tireParamsJsonPath = "../../data/json/ART/tmeasy.json"

    # Construct the solver
    solver = dof18.d18SolverHalfImplicit()
    solver.Construct(vehParamsJsonPath, tireParamsJsonPath)

    # Set the simulation time step
    solver.SetTimeStep(1e-3)

    # Set the initial conditions
    veh_st = dof18.VehicleState()
    tirelf_st = dof18.TMeasyState()
    tirerf_st = dof18.TMeasyState()
    tirelr_st = dof18.TMeasyState()
    tirerr_st = dof18.TMeasyState()
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st)

    max_iteration = 10
    iteration = 0
    error_sum = 0
    error_diff = 0
    error_prev = 0

    throttle = 0
    steering = 0
    while iteration < max_iteration:

        # Call the moddle with the current throttle and steering
        solverUpdated = dof18_model(solver, 0, 1, throttle, steering)

        # Extract the velocity from the solver
        dof18_vel = solverUpdated.m_veh_st._u

        # Calculate the error (for now just set refernce to 0.2)
        error = 0.2 - dof18_vel

        # Call the PID controller
        throttle = pid_control(error, error_sum, error_diff, error_prev)

        iteration += 1

    # Finally update the global solver states
    solver.m_veh_st = dof18.VehicleState(solverUpdated.m_veh_state)
    solver.m_tirelf_st = dof18.TMeasyState(solverUpdated.m_tirelf_state)
    solver.m_tirerf_st = dof18.TMeasyState(solverUpdated.m_tirerf_state)
    solver.m_tirelr_st = dof18.TMeasyState(solverUpdated.m_tirelr_state)
    solver.m_tirerr_st = dof18.TMeasyState(solverUpdated.m_tirerr_state)

    # Return the throttle to the actual vehicle if you need to
    print(throttle)
