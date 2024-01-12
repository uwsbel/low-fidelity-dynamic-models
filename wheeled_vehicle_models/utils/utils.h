#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <functional>

static const double C_PI = 3.141592653589793238462643383279;
static const double C_2PI = 6.283185307179586476925286766559;
static const double G = 9.81;  // gravity constant
static const double rpm2rad = C_PI / 30;

// --------------------------------------------------------------------------------------------------------------------

/// @brief Struct through which driver inputs can be provided to the library of vehicle models.
struct DriverInput {
    DriverInput() : m_time(0), m_steering(0), m_throttle(0), m_braking(0) {}
    DriverInput(double time, double steering, double throttle, double braking)
        : m_time(time), m_steering(steering), m_throttle(throttle), m_braking(braking) {}

    static bool compare(const DriverInput& a, const DriverInput& b) { return a.m_time < b.m_time; };

    double m_time;
    double m_steering;
    double m_throttle;
    double m_braking;
};

typedef std::vector<DriverInput> DriverData;

/// @brief Function used to load the driver data from a text file.
/// @param m_data DriverData object to be populated with the data from the file.
/// @param filename Path to file
void LoadDriverData(DriverData& m_data, const std::string& filename);

/// @brief Linearly interpolates through the driver data to find the driver input at a given time.
/// @param time Current time
/// @param driver_data DriverData object containing the driver data
/// @return DriverInput object containing the interpolated driver input
DriverInput GetDriverInput(double time, const DriverData& driver_data);

// --------------------------------------------------------------------------------------------------------------------
/// @brief Reference path for FSA using sundials solver is made up of these PathPoints
struct PathPoint {
    PathPoint() : m_t(0), m_x(0), m_y(0) {}
    PathPoint(double time, double x, double y) : m_t(time), m_x(x), m_y(y) {}

    static bool compare(const PathPoint& a, const PathPoint& b) { return a.m_t < b.m_t; };

    double m_t; //!< time
    double m_x; //!< x coordinate
    double m_y; //!< y coordinate
};

typedef std::vector<PathPoint> PathData;

/// @brief Loads the PathData from a text file
/// @param path PathData object to be populated with the data from the file
/// @param filename Path to text file
void LoadPath(PathData& path, const std::string& filename);

/// @brief Linearly interpolates through the path data to find the path point at a given time.
PathPoint GetPathPoint(double time, const PathData& path);

// --------------------------------------------------------------------------------------------------------------------

/// @brief Struct to points on any sort of map.

/// Used for the engine maps, torque converter maps and steering maps.
struct MapEntry {
    MapEntry() : _x(0), _y(0) {}
    MapEntry(double x, double y) : _x(x), _y(y) {}

    static bool compare(const MapEntry& a, const MapEntry& b) { return a._x < b._x; };

    double _x;  // x coordinate of the map
    double _y;  // y coordinate of the map
};

/// @brief Function that takes a map which is a vector of MapEntry structs and returns the y value at a given x value by
/// linear interpolation.
/// @param map Vector of MapEntry structs
/// @param x Query x value
/// @return Y value at the given x value (linearly interpolated in case given x value is not in the map)
double getMapY(const std::vector<MapEntry>& map, const double x);

// --------------------------------------------------------------------------------------------------------------------

// linear interpolation function
inline double InterpL(double fz, double w1, double w2, double pn) {
    return w1 + (w2 - w1) * (fz / pn - 1.);
}

// quadratic interpolation function
inline double InterpQ(double fz, double w1, double w2, double pn) {
    return (fz / pn) * (2. * w1 - 0.5 * w2 - (w1 - 0.5 * w2) * (fz / pn));
}

// template safe signum function
template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double sineStep(double x, double x1, double y1, double x2, double y2);

// clamp function from chrono
template <typename T>
T clamp(T value, T limitMin, T limitMax) {
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