#ifndef UTILS_H
#define UTILS_H

#include <fstream>
#include <cuda.h>
#include <vector>
#include <iostream>
#include <sstream>

static const double C_PI = 3.141592653589793238462643383279;
static const double C_2PI = 6.283185307179586476925286766559;
static const double G = 9.81;  // gravity constant
static const double rpm2rad = C_PI / 30;

// --------------------------------------------------------------------------------------------------------------------
// error checking macros

#define CHECK_CUDA_ERROR(val) check((val), #val, __FILE__, __LINE__)
template <typename T>
void check(T err, const char* const func, const char* const file, const int line) {
    if (err != cudaSuccess) {
        std::cerr << "CUDA Runtime Error at: " << file << ":" << line << std::endl;
        std::cerr << cudaGetErrorString(err) << " " << func << std::endl;
    }
}
// --------------------------------------------------------------------------------------------------------------------

/// @brief Struct through which driver inputs can be provided to the library of vehicle models - GPU version
struct DriverInput {
    __device__ __host__ DriverInput() : m_time(0), m_steering(0), m_throttle(0), m_braking(0) {}
    __device__ __host__ DriverInput(double time, double steering, double throttle, double braking)
        : m_time(time), m_steering(steering), m_throttle(throttle), m_braking(braking) {}

    // static bool compare(const DriverInput& a, const DriverInput& b) { return a.m_time < b.m_time; };

    double m_time;
    double m_steering;
    double m_throttle;
    double m_braking;
};

typedef std::vector<DriverInput> DriverData;

/// @brief Function used to load the driver data from a text file.
/// @param m_data DriverData object to be populated with the data from the file.
/// @param filename Path to file
__host__ void LoadDriverData(DriverData& m_data, const std::string& filename);

/// @brief Linearly interpolates through the driver data to find the driver input at a given time.
/// @param time Current time
/// @param driver_data DriverData object containing the driver data
/// @return DriverInput object containing the interpolated driver input
__device__ DriverInput GetDriverInput(double time, const DriverInput* driver_data, const unsigned int data_length);

// --------------------------------------------------------------------------------------------------------------------

/// @brief Struct to points on any sort of map.

/// Used for the engine maps, torque converter maps and steering maps.
struct MapEntry {
    __device__ __host__ MapEntry() : _x(0), _y(0) {}
    __device__ __host__ MapEntry(double x, double y) : _x(x), _y(y) {}

    // static bool compare(const MapEntry& a, const MapEntry& b) { return a._x < b._x; };

    double _x;  // x coordinate of the map
    double _y;  // y coordinate of the map
};

/// @brief Function that takes a map which is a vector of MapEntry structs and returns the y value at a given x value by
/// linear interpolation.
/// @param map Pointer to the map
/// @param x Query x value
/// @param map_size Size of the map
/// @return Y value at the given x value (linearly interpolated in case given x value is not in the map)
__device__ double getMapY(const MapEntry* map, const double x, const unsigned int map_size);

/// @brief custom lower bound function for interpolating DriverInputs - Used internally by GetDriverInput
__device__ unsigned int lower_bound(const DriverInput* a, unsigned int n, DriverInput x);
/// @brief custom lower bound function for interpolating MapEntries - Used internally by getMapY
__device__ unsigned int lower_bound_map(const MapEntry* a, unsigned int n, MapEntry x);

// --------------------------------------------------------------------------------------------------------------------

// linear interpolation function
__device__ __host__ inline double InterpL(double fz, double w1, double w2, double pn) {
    return w1 + (w2 - w1) * (fz / pn - 1.);
}

// quadratic interpolation function
__device__ __host__ inline double InterpQ(double fz, double w1, double w2, double pn) {
    return (fz / pn) * (2. * w1 - 0.5 * w2 - (w1 - 0.5 * w2) * (fz / pn));
}

// template safe signum function
template <typename T>
__device__ int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

__device__ double sineStep(double x, double x1, double y1, double x2, double y2);

// clamp function from chrono
template <typename T>
__device__ T clamp(T value, T limitMin, T limitMax) {
    if (value < limitMin)
        return limitMin;
    if (value > limitMax)
        return limitMax;

    return value;
};

// --------------------------------------------------------------------------------------------------------------------
/// @brief Class to write data to a csv file.
class CSV_writer {
  public:
    explicit CSV_writer(const std::string& delim = ",") : m_delim(delim) {}

    CSV_writer(const CSV_writer& source) : m_delim(source.m_delim) {
        // Note that we do not copy the stream buffer (as then it would be shared!)
        m_ss.copyfmt(source.m_ss);          // copy all data
        m_ss.clear(source.m_ss.rdstate());  // copy the error state
    }

    ~CSV_writer() {}

    void write_to_file(const std::string& filename, const std::string& header = "") const {
        std::ofstream ofile(filename.c_str());
        ofile << header;
        ofile << m_ss.str();
        ofile.close();
    }

    // To clear the stream buffer
    void clearData() {
        m_ss.str("");  // Set the content of the stream buffer to an empty string
        m_ss.clear();  // Clear any error flags that might have been set
    }

    const std::string& delim() const { return m_delim; }
    std::ostringstream& stream() { return m_ss; }

    template <typename T>
    CSV_writer& operator<<(const T& t) {
        m_ss << t << m_delim;
        return *this;
    }

    CSV_writer& operator<<(std::ostream& (*t)(std::ostream&)) {
        m_ss << t;
        return *this;
    }
    CSV_writer& operator<<(std::ios& (*t)(std::ios&)) {
        m_ss << t;
        return *this;
    }
    CSV_writer& operator<<(std::ios_base& (*t)(std::ios_base&)) {
        m_ss << t;
        return *this;
    }

  private:
    std::string m_delim;
    std::ostringstream m_ss;
};

template <typename T>
inline CSV_writer& operator<<(CSV_writer& out, const std::vector<T>& vec) {
    for (const auto& v : vec)
        out << v;
    return out;
}

#endif