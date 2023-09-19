#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <stdint.h>
#include "utils.h"

// --------------------------------------------------------------------------------------------------------------------

/// Driver inputs from data file.
/// A driver model based on user inputs provided as time series. If provided as a
/// text file, each line in the file must contain 4 values:
///   time steering throttle braking
/// It is assumed that the time values are unique and soted from 0 to T
void LoadDriverData(DriverData& data, const std::string& filename) {
    std::ifstream ifile(filename.c_str());
    std::string line;

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
DriverInput GetDriverInput(double time, const DriverData& driver_data) {
    DriverInput input;

    if (time <= driver_data[0].m_time) {
        input = driver_data[0];
    } else if (time >= driver_data.back().m_time) {
       input = driver_data.back();
    } else {
        // if time is within map range, get an iterator and do linear interpolation
        auto& foo = const_cast<DriverData&>(driver_data);
        DriverData::iterator right =
            std::lower_bound(foo.begin(), foo.end(), DriverInput(time, 0, 0, 0), DriverInput::compare);
        DriverData::iterator left = right - 1;

        double tbar = (time - left->m_time) / (right->m_time - left->m_time);
       input.m_time = time;
       input.m_steering = left->m_steering + tbar * (right->m_steering - left->m_steering);
       input.m_throttle = left->m_throttle + tbar * (right->m_throttle - left->m_throttle);
       input.m_braking = left->m_braking + tbar * (right->m_braking - left->m_braking);
    }

    return input;
}

// --------------------------------------------------------------------------------------------------------------------

void LoadPath(PathData& path, const std::string& filename) {
    std::ifstream ifile(filename.c_str());
    std::string line;

    // skip first line
    std::getline(ifile, line);

    // get each line
    while (std::getline(ifile, line)) {
       std::istringstream iss(line);

       double time, x, y;
       char comma;
       iss >> time >> comma >> x >> comma >> y;

       if (iss.fail())
            break;

       // push into our structure
       path.push_back(PathPoint(time, x, y));
    }

    ifile.close();
}

PathPoint GetPathPoint(double time, const PathData& path) {
    PathPoint point;

    if (time <= path[0].m_t) {
       point = path[0];
    } else if (time >= path.back().m_t) {
       point = path.back();
    } else {
       // if time is within map range, get an iterator and do linear interpolation
       auto& foo = const_cast<PathData&>(path);
       PathData::iterator right = std::lower_bound(foo.begin(), foo.end(), PathPoint(time, 0, 0), PathPoint::compare);
       PathData::iterator left = right - 1;

       double tbar = (time - left->m_t) / (right->m_t - left->m_t);
       point.m_t = time;
       point.m_x = left->m_x + tbar * (right->m_x - left->m_x);
       point.m_y = left->m_y + tbar * (right->m_y - left->m_y);
    }

    return point;
}

// --------------------------------------------------------------------------------------------------------------------

double getMapY(const std::vector<MapEntry>& map, const double x) {
    // if x outside map range
    if (x <= map[0]._x) {
       return map[0]._y;
    } else if (x >= map.back()._x) {
       return map.back()._y;
    }

    // if x within map range, get an iterator and do linear interpolation
    auto& foo = const_cast<std::vector<MapEntry>&>(map);
    std::vector<MapEntry>::iterator right = std::lower_bound(foo.begin(), foo.end(), MapEntry(x, 0), MapEntry::compare);
    std::vector<MapEntry>::iterator left = right - 1;

    // linear interplolation
    double mbar = (x - left->_x) / (right->_x - left->_x);
    return (left->_y + mbar * (right->_y - left->_y));
}

// sine step function for some smoothing operations
double sineStep(double x, double x1, double y1, double x2, double y2) {
    if (x <= x1)
        return y1;
    if (x >= x2)
        return y2;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double y = y1 + dy * (x - x1) / dx - (dy / C_2PI) * std::sin(C_2PI * (x - x1) / dx);
    return y;
}
