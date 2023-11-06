
import numpy as np
import pydof18 as dof18
import matplotlib.pyplot as plt
import sys


# Input file
driver_file = "../../data/input/" + str(sys.argv[1]) + ".txt"

# Vehicle specification
vehParamsJson = "../../data/json/Sedan/vehicle.json"
tireParamsJson = "../../data/json/Sedan/tmeasyNr.json"


# Construct the solver
solver = dof18.d18SolverHalfImplicit()
tireType = dof18.TireType_TMeasyNr
solver.Construct(vehParamsJson, tireParamsJson, driver_file, tireType)


# Set the simulation time step
solver.SetTimeStep(1e-3)

# Set the initial conditions
veh_st = dof18.VehicleState()
tirelf_st = dof18.TMeasyNrState()
tirerf_st = dof18.TMeasyNrState()
tirelr_st = dof18.TMeasyNrState()
tirerr_st = dof18.TMeasyNrState()
solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st)

timeStep = solver.GetStep()
endTime = solver.GetEndT()
t = 0
new_time = 0

driver_data = solver.GetDriverData()

result = []
timeStepNo = 0
while (t < (endTime - timeStep / 10.)):
    controls = dof18.GetDriverInput(t, driver_data)
    new_time = solver.IntegrateStep(
        t, controls.m_throttle, controls.m_steering, 0)

    # append result every 10 time steps
    if (timeStepNo % 10 == 0):
        result.append([t, solver.m_veh_state._u, solver.m_veh_state._psi])

    t = new_time
    timeStepNo += 1

result = np.array(result)
plt.plot(result[:, 0], result[:, 1], 'r')
plt.xlabel('Time (s)')
plt.ylabel('Longitudinal velocity (m/s)')
plt.show()
