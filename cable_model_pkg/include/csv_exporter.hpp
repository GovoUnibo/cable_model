#ifndef CSV_EXPORTER_HPP
#define CSV_EXPORTER_HPP

#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include <iostream>

namespace cable_utils {

struct DataPoint {
    double time;
    std::vector<double> forces;
    std::vector<double> positions_x;
    std::vector<double> positions_z;
    std::vector<double> velocities_x;
    std::vector<double> velocities_z;
};

class CSVExporter {
private:
    std::vector<DataPoint> data_points;
    std::chrono::steady_clock::time_point start_time;
    int num_particles;
    std::string folder_path;

public:
    CSVExporter(int num_particles, const std::string& folder_path);
    
    void addDataPoint(const std::vector<double>& forces,
                     const std::vector<double>& positions_x,
                     const std::vector<double>& positions_z,
                     const std::vector<double>& velocities_x,
                     const std::vector<double>& velocities_z);
    
    void saveForces(const std::string& filename = "forces_data.csv");
    void savePositions(const std::string& filename = "positions_data.csv");
    void saveVelocities(const std::string& filename = "velocities_data.csv");
    
    void clearData();
    size_t getDataSize() const;
};

} // namespace cable_utils

#endif // CSV_EXPORTER_HPP
