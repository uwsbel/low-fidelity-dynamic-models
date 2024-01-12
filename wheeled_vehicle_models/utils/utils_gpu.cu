#include <fstream>
#include <sstream>
#include "utils_gpu.cuh"

// --------------------------------------------------------------------------------------------------------------------

__host__ void LoadDriverData(DriverData& data, const std::string& filename) {
    std::ifstream ifile(filename.c_str());
    std::string line;

    if (!ifile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // get each line
    while (std::getline(ifile, line)) {
        std::istringstream iss(line);

        double time, steering, throttle, braking;
        iss >> time >> steering >> throttle >> braking;

        if (iss.fail())
            break;

        // push into our structure
        data.push_back(DriverInput(time, steering, throttle, braking));
    }

    ifile.close();
}

// Function to get the vehicle controls at a given time
__device__ DriverInput GetDriverInput(double time, const DriverInput* driver_data, const unsigned int data_length) {
    DriverInput input;

    if (time <= driver_data[0].m_time) {
        input = driver_data[0];
    } else if (time >= driver_data[data_length - 1].m_time) {
        input = driver_data[data_length - 1];
    } else {
        // if time is within map range, get an iterator and do linear interpolation
        int right = lower_bound(driver_data, data_length, DriverInput(time, 0, 0, 0));
        int left = right - 1;

        double tbar = (time - driver_data[left].m_time) / (driver_data[right].m_time - driver_data[left].m_time);
        input.m_time = time;
        input.m_steering =
            driver_data[left].m_steering + tbar * (driver_data[right].m_steering - driver_data[left].m_steering);
        input.m_throttle =
            driver_data[left].m_throttle + tbar * (driver_data[right].m_throttle - driver_data[left].m_throttle);
        input.m_braking =
            driver_data[left].m_braking + tbar * (driver_data[right].m_braking - driver_data[left].m_braking);
    }
    return input;
}

// --------------------------------------------------------------------------------------------------------------------

__device__ double getMapY(const MapEntry* map, const double x, const unsigned int map_size) {
    // if x outside map range
    if (x <= map[0]._x) {
        return map[0]._y;
    } else if (x >= map[map_size - 1]._x) {
        return map[map_size - 1]._y;
    }

    // if x within map range, get an iterator and do linear interpolation

    int right = lower_bound_map(map, map_size, MapEntry(x, 0));
    int left = right - 1;
    // linear interplolation
    double mbar = (x - map[left]._x) / (map[right]._x - map[left]._x);
    return (map[left]._y + mbar * (map[right]._y - map[left]._y));
}

// sine step function for some smoothing operations
__device__ double sineStep(double x, double x1, double y1, double x2, double y2) {
    if (x <= x1)
        return y1;
    if (x >= x2)
        return y2;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double y = y1 + dy * (x - x1) / dx - (dy / C_2PI) * std::sin(C_2PI * (x - x1) / dx);
    return y;
}

// --------------------------------------------------------------------------------------------------------------------

// utility function to std::lower_bound on device
// Implementing lower bound c++ function in C with our custom container "Entry"
// Just a binary search
__device__ unsigned int lower_bound(const DriverInput* a, unsigned int n, DriverInput x) {
    int l = 0;
    int h = n;  // not sure if this should be n or n-1
    while (l < h) {
        int mid = l + (h - l) / 2;
        if (x.m_time <= a[mid].m_time) {
            h = mid;
        } else {
            l = mid + 1;
        }
    }
    return l;
}

__device__ unsigned int lower_bound_map(const MapEntry* a, unsigned int n, MapEntry x) {
    int l = 0;
    int h = n;
    while (l < h) {
        int mid = l + (h - l) / 2;
        if (x._x <= a[mid]._x) {
            h = mid;
        } else {
            l = mid + 1;
        }
    }
    return l;
}