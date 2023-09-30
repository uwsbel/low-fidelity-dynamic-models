# This demo is intended to provide an example of how to use the Low fidelity models with the ART vehicle for doing something like a PID control where you might have to interate multiple times in a loop from the same state

import time
import numpy as np
import pydof18 as dof18
import sys

# Parameters path
vehParamsJsonPath = "../../data/json/HMMWV/vehicle.json"
tireParamsJsonPath = "../../data/json/HMMWV/tmeasy.json"

# Input file
driver_file = "../../data/input/" + str(sys.argv[1]) + ".txt"

# Construct the solver
solver = dof18.d18SolverHalfImplicit()
solver.Construct(vehParamsJsonPath, tireParamsJsonPath, driver_file)

# Set the simulation time step
solver.SetTimeStep(1e-3)

# Set the initial conditions
veh_st = dof18.VehicleState()
tirelf_st = dof18.TMeasyState()
tirerf_st = dof18.TMeasyState()
tirelr_st = dof18.TMeasyState()
tirerr_st = dof18.TMeasyState()
solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st)


timeStep = solver.GetStep()
endTime = solver.GetEndT()
t = 0
new_time = 0

driver_data = solver.GetDriverData()

# time this loop
start = time.time()
while (t < (endTime - timeStep / 10.)):
    controls = dof18.GetDriverInput(t, driver_data)
    new_time = solver.IntegrateStepWithJacobian(
        t, controls.m_throttle, controls.m_steering, 0, True)
    # state_jac = np.array(solver.GetJacobianState())
    # print(state_jac)

    t = new_time

end = time.time()
# Print time in ms
print("Time taken: ", (end - start) * 1000)
