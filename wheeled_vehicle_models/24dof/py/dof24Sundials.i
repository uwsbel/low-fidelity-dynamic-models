%module pydof24

%include "std_string.i"
%include "std_vector.i"

%{
    #include "utils.h"
    #include "dof24.h"
    using namespace d24;
    #include "dof24_halfImplicit.h"
    #include "dof24_sundials.h"
%}

// Initiate some templates
%template(vector_double) std::vector <double>;
%template(DriverData) std::vector<DriverInput>;
%template(matrix_double) std::vector <std::vector<double>>;
%template(vector_mapEntry) std::vector <MapEntry>;
%template(PathData) std::vector<PathPoint>;

// matrix_double to a list of lists -> This can then easily be converted into a numpy array
%typemap(out) std::vector<std::vector<double>> (PyObject* _inner,PyObject* _outer) %{
    // Allocate a PyList object of the requested size.
    _outer = PyList_New($1.size());
    // Populate the PyList.  PyFloat_FromDouble converts a C++ "double" to a
    // Python PyFloat object.
    for(int x = 0; x < $1.size(); x++) {
        _inner = PyList_New($1[x].size());
        for(int y = 0; y < $1[x].size(); y++)
            PyList_SetItem(_inner,y,PyFloat_FromDouble($1[x][y]));
        PyList_SetItem(_outer,x,_inner);
    }
   $result = SWIG_Python_AppendOutput($result,_outer);
%}



%include "../../utils/utils.h"
%include "../dof24.h"
%include "../dof24_halfImplicit.h"
%include "../dof24_sundials.h"


typedef std::vector<DriverInput> DriverData;
typedef std::vector<PathPoint> PathData;




