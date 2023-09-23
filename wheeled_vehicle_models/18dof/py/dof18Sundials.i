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

%include "../../utils/utils.h"
%include "../dof18.h"
%include "../dof18_halfImplicit.h"
%include "../dof18_sundials.h"


typedef std::vector<DriverInput> DriverData;
typedef std::vector<PathPoint> PathData;


// Initiate some templates
%template(vector_double) std::vector <double>;
%template(DriverData) std::vector<DriverInput>;
%template(PathData) std::vector<PathPoint>;




