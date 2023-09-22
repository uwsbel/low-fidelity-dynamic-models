# Demo for hmmwv simulation using the half implicit integrator in python

import numpy as np
import pydof18 as dof18
import sys

# Input file
driver_file = "../../data/input/" + str(sys.argv[1]) + ".txt"

# Vehicle specification
vehParamsJson = "../../data/json/HMMWV/vehicle.json"
tireParamsJson = "../../data/json/HMMWV/tmeasy.json"


# Construct the half Implicit solver
solver = dof18.d18SolverHalfImplicit()
solver.Construct(vehParamsJson, tireParamsJson, driver_file)


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
solver.SetOutput("../../data/output/" +
                 str(sys.argv[1]) + "_hmmwv18HiPy.csv", 100)

# Solve
solver.Solve()
