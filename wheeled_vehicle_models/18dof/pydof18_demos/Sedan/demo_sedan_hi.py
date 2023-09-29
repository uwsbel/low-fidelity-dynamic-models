# Demo for hmmwv simulation using the half implicit integrator in python

import numpy as np
import pydof18 as dof18
import sys

# Input file
driver_file = "../../data/input/" + str(sys.argv[1]) + ".txt"

# Vehicle specification
vehParamsJson = "../../data/json/Sedan/vehicle.json"
tireParamsJson = "../../data/json/Sedan/tmeasy.json"


# Construct the half Implicit solver
solver = dof18.d18SolverHalfImplicit()
solver.Construct(vehParamsJson, tireParamsJson, driver_file)

# trial_ks = [1e4]
# trial_bs = [1.1e4, 1.2e4, 1.3e4, 1.4e4, 1.5e4]
trial_ks = [1e4, 2e4, 3e4, 4e4]
# trial_bs = [1e3, 1e4, 1e5, 1e6]
trial_bs = [1e3, 1e4]

for k in trial_ks:
    for b in trial_bs:
        # Set the roll stiffness and damping
        solver.m_veh_param._krof = k
        solver.m_veh_param._kror = k
        solver.m_veh_param._brof = b
        solver.m_veh_param._bror = b

        # Set the simulation time step
        solver.SetTimeStep(1e-3)

        # Iniitial conditions
        veh_st = dof18.VehicleState()
        tirelf_st = dof18.TMeasyState()
        tirerf_st = dof18.TMeasyState()
        tirelr_st = dof18.TMeasyState()
        tirerr_st = dof18.TMeasyState()
        solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st)

        # Enable simulation output at 100 Hz
        # solver.SetOutput("../../data/output/" +
        #                  str(sys.argv[1]) + "_sedan18HiPy_" + "{:.2e}".format(k).split('e')[1].replace('+', '') + "_" + "{:.2e}".format(b).split('e')[1].replace('+', '') + ".csv", 100)
        # solver.SetOutput("../../data/output/" +
        #                  str(sys.argv[1]) + "_sedan18HiPy_" + "{:.2e}".format(k).split('e')[1].replace('+', '') + "_" + "{:.2e}".format(b).split('e')[0].replace('.', '') + ".csv", 100)
        solver.SetOutput("../../data/output/" +
                         str(sys.argv[1]) + "_sedan18HiPy_" + "{:.2e}".format(k).split('e')[0].replace('.', '') + "_" + "{:.2e}".format(b).split('e')[1].replace('+', '') + ".csv", 100)

        # Solve
        solver.Solve()
