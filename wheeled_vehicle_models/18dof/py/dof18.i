%module pydof18

%include "std_string.i"
%include "std_vector.i"

%{
    #include "utils.h"
    #include "dof18.h"
    using namespace d18;
    #include "dof18_halfImplicit.h"
    #include "dof18_sundials.h"
%}

// %include "../../utils/utils.h"
// %include "../../dof18.h"
// %include "../dof18_halfImplicit.h"
// %include "../dof18_sundials.h"

%include "../../utils/utils.h"
%include "../dof18.h"
%include "../dof18_halfImplicit.h"
%include "../dof18_sundials.h"


typedef std::vector<DriverInput> DriverData;
typedef std::vector<PathPoint> PathData;

// Ignore classes that the user does not need to see
// %ignore UserData;

// Ignore functions that the user does not need to see
// %ignore packY
// %ignore packYDOT
// %ignore unpackY
// %ignore calcF
// %ignore calcFtL
// %ignore calcJtL
// %ignore rhsFun
// %ignore rhsQuad
// %ignore rhsQuadSens
// %ignore printStatsCvode
// %ignore check_retval
// %ignore Objective

// Initiate some templates
%template(vector_double) std::vector <double>;
%template(DriverData) std::vector<DriverInput>;
%template(PathData) std::vector<PathPoint>;




