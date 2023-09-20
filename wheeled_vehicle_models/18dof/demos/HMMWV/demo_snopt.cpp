#include <numeric>

#include "dof18_sundials.h"
#include "snoptProblem.hpp"

using namespace d18;

int param_flags = ParamFlag::ENGINE_MAP;
d18SolverSundials solver;

int num_evals = 0;
int num_evals_func = 0;
int num_evals_grad = 0;

// =============================================================================

void costFun(int* Status,
             int* n,
             double x[],
             int* needF,
             int* nF,
             double F[],
             int* needG,
             int* neG,
             double G[],
             char cu[],
             int* lencu,
             int iu[],
             int* leniu,
             double ru[],
             int* lenru) {
    // Set model parameters
    solver.SetSensitivityParameters(param_flags, x);

    //// DEBUG -----
    ////for (int i = 0; i < *n; i++) {
    ////    std::cout << x[i] << " ";
    ////}
    ////std::cout << std::endl;
    //// -----------

    // Evaluate cost function and gradient (if requested)
    Objective cf = solver.Evaluate_FSA(*needG > 0);
    if (*needF > 0) {
        F[0] = cf.value;
    }
    if (*needG > 0) {
        for (int i = 0; i < *neG; i++)
            G[i] = cf.gradient[i];
    }

    // Increment number of calls
    num_evals++;
    num_evals_func += (*needF > 0);
    num_evals_grad += (*needG > 0);

    // Return flag
    if (cf.success)
        *Status = 0;
    else
        *Status = -2;
}

// =============================================================================

int main(int argc, char** argv) {
    // Driver inputs and reference trajectory
    std::string driver_file = "../data/input/acc.txt";
    std::string path_file = "../data/reference/chrono_acc.txt";

    // Vehicle specification
    std::string vehParamsJSON = (char*)"../data/json/HMMWV/vehicle.json";
    std::string tireParamsJSON = (char*)"../data/json/HMMWV/tmeasy.json";

    // Parameters (engine torque map)
    std::vector<double> params = {300, 382, 490, 579, 650, 706, 746, 774, 789, 793,
                                  788, 774, 754, 728, 697, 664, 628, 593, 558, -400};

    ////std::vector<double> params = {258.0,  328.52, 421.4,  497.94, 559.0,  607.16, 641.56, 665.64, 678.54, 681.98,
    ////                              677.68, 665.64, 648.44, 626.08, 599.42, 571.04, 540.08, 509.98, 479.88, -344.0};

    // Construct the solver
    solver.Construct(vehParamsJSON, tireParamsJSON, driver_file);

    // Set optional inputs
    solver.SetTolerances(1e-5, 1e-3);
    solver.SetReferencePath(path_file);
    solver.SetSensitivityParameters(ParamFlag::ENGINE_MAP, params);

    // Initialize solver (set initial conditions)
    VehicleState veh_st;
    TMeasyState tirelf_st;
    TMeasyState tirerf_st;
    TMeasyState tirelr_st;
    TMeasyState tirerr_st;
    solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st);

    int num_params = solver.GetNumParameters();

    // Initialize SNOPT
    snoptProblemA optimizer;
    optimizer.initialize("d18dof_snopt.out", 1);  // print file, summary on
    optimizer.setProbName("d18dof");
    optimizer.setIntParameter("Derivative option", 1);

    std::vector<int> iGfun(num_params, 0);
    std::vector<int> jGvar(num_params);
    // Fill in the sparsity pattern - we have no sparsity
    for (int i = 0; i < num_params; i++) {
        jGvar[i] = i;
    }

    std::vector<double> xlow(num_params, -1e3);
    std::vector<double> xupp(num_params, +1e3);

    double Flow = -1e20;
    double Fupp = +1e20;

    std::vector<int> xstate(num_params, 0);
    std::vector<double> xmul(num_params);

    double F = 0;
    int Fstate = 0;
    double Fmul = 0;

    int nS;
    int nInf;
    double sInf;

    // Run SNOPT
    int info = optimizer.solve(0,             // cold start
                               1,             // number of cost functions
                               num_params,    // number optimization parameters
                               0.0,           // ObjAdd
                               0,             // ObjRow
                               costFun,       // evaluation function
                               nullptr,       // matrix A does not exist
                               nullptr,       // matrix A does not exist
                               nullptr,       // matrix A does not exist
                               0,             // matrix A does not exist
                               iGfun.data(),  // Specifies the rows of the Jacobian matrix that are filled
                               jGvar.data(),  // Specifies the columns of the Jacobian matrix that are filled
                               num_params,    // Number of elements in the Jacobian matrix
                               xlow.data(), xupp.data(),  // lower and upper parameter bounds
                               &Flow, &Fupp,              // lower and upper cost function bounds
                               params.data(),             // initial parameter guess
                               xstate.data(),             // initial parameter states
                               xmul.data(),               // dual variables for bound constraints
                               &F,                        // initial / final cost function value
                               &Fstate,                   // initial cost function state
                               &Fmul,                     // initial estimate / final values for Lagrange multipliers
                               nS, nInf, sInf);

    // Report statistics
    std::cout << "Number of CF evaluations (total):         " << num_evals << std::endl;
    std::cout << "Number of CF evaluations (function only): " << num_evals_func << std::endl;
    std::cout << "Number of CF evaluations (with gradient): " << num_evals_grad << std::endl;
}
