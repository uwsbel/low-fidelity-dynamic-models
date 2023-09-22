# Demo for hmmwv simulation using the half implicit integrator in python

import numpy as np
import pydof18 as dof18
import sys

# Input file
driver_file = "../../data/input/" + str(sys.argv[1]) + ".txt"

# Vehicle specification
vehParamsJson = "../../data/json/HMMWV/vehicle.json"
tireParamsJson = "../../data/json/HMMWV/tmeasy.json"


# Construct the sundials solver
solver = dof18.d18SolverSundials()
solver.Construct(vehParamsJson, tireParamsJson, driver_file)


# Set optional inputs
# Solver tolerance
solver.SetTolerances(1e-5, 1e-3)
# Set the maximum time step solver can take
solver.SetMaxStep(1e-2)

# Iniitial conditions
veh_st = dof18.VehicleState()
tirelf_st = dof18.TMeasyState()
tirerf_st = dof18.TMeasyState()
tirelr_st = dof18.TMeasyState()
tirerr_st = dof18.TMeasyState()
solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st)

# Enable simulation output - sundials solver for now only outputs at 100 Hz
solver.SetOutput("../../data/output/" +
                 str(sys.argv[1]) + "_hmmwv18SundialsPy.csv")
solver.SetVerbose(False)
# Solve
solver.Solve(False)  # No FSA
