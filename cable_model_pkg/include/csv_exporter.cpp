#include "../include/csv_exporter.hpp"
#include <cmath>

#define RESET "\033[0m"
#define GREEN "\033[32m"
#define ORANGE "\033[38;5;214m"

namespace cable_utils {

CSVExporter::CSVExporter(int num_particles, const std::string& folder_path) 
    : num_particles(num_particles), folder_path(folder_path) {
    start_time = std::chrono::steady_clock::now();
    data_points.reserve(10000); // Pre-allocate for better performance
}

void CSVExporter::addDataPoint(const std::vector<double>& forces,
                              const std::vector<double>& positions_x,
                              const std::vector<double>& positions_z,
                              const std::vector<double>& velocities_x,
                              const std::vector<double>& velocities_z) {
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
    
    DataPoint point;
    point.time = elapsed;
    point.forces = forces;
    point.positions_x = positions_x;
    point.positions_z = positions_z;
    point.velocities_x = velocities_x;
    point.velocities_z = velocities_z;
    
    data_points.push_back(point);
}

void CSVExporter::saveForces(const std::string& filename) {
    std::string file_path = folder_path + filename;
    std::ofstream file(file_path, std::ios::out);
    
    if (!file.is_open()) {
        std::cerr << "Errore: impossibile aprire il file " << filename << "!" << std::endl;
        return;
    }
    
    std::cout << GREEN << "Salvando " << filename << "..." << RESET << std::endl;
    
    // Header
    file << "time";
    for (int i = 0; i < num_particles; ++i) {
        file << ",force_magnitude_mass_" << i;
    }
    file << "\n";
    
    // Data
    for (const auto& point : data_points) {
        file << point.time;
        for (int i = 0; i < num_particles && i < point.forces.size(); ++i) {
            file << "," << point.forces[i];
        }
        file << "\n";
    }
    
    file.close();
    std::cout << ORANGE << "File " << filename << " salvato correttamente con " 
              << data_points.size() << " punti dati." << RESET << std::endl;
}

void CSVExporter::savePositions(const std::string& filename) {
    std::string file_path = folder_path + filename;
    std::ofstream file(file_path, std::ios::out);
    
    if (!file.is_open()) {
        std::cerr << "Errore: impossibile aprire il file " << filename << "!" << std::endl;
        return;
    }
    
    std::cout << GREEN << "Salvando " << filename << "..." << RESET << std::endl;
    
    // Header
    file << "time";
    for (int i = 0; i < num_particles; ++i) {
        file << ",position_x_" << i << ",position_z_" << i;
    }
    file << "\n";
    
    // Data
    for (const auto& point : data_points) {
        file << point.time;
        for (int i = 0; i < num_particles && i < point.positions_x.size(); ++i) {
            file << "," << point.positions_x[i] << "," << point.positions_z[i];
        }
        file << "\n";
    }
    
    file.close();
    std::cout << ORANGE << "File " << filename << " salvato correttamente con " 
              << data_points.size() << " punti dati." << RESET << std::endl;
}

void CSVExporter::saveVelocities(const std::string& filename) {
    std::string file_path = folder_path + filename;
    std::ofstream file(file_path, std::ios::out);
    
    if (!file.is_open()) {
        std::cerr << "Errore: impossibile aprire il file " << filename << "!" << std::endl;
        return;
    }
    
    std::cout << GREEN << "Salvando " << filename << "..." << RESET << std::endl;
    
    // Header
    file << "time";
    for (int i = 0; i < num_particles; ++i) {
        file << ",velocity_x_" << i << ",velocity_z_" << i;
    }
    file << "\n";
    
    // Data
    for (const auto& point : data_points) {
        file << point.time;
        for (int i = 0; i < num_particles && i < point.velocities_x.size(); ++i) {
            file << "," << point.velocities_x[i] << "," << point.velocities_z[i];
        }
        file << "\n";
    }
    
    file.close();
    std::cout << ORANGE << "File " << filename << " salvato correttamente con " 
              << data_points.size() << " punti dati." << RESET << std::endl;
}

void CSVExporter::clearData() {
    data_points.clear();
    start_time = std::chrono::steady_clock::now();
}

size_t CSVExporter::getDataSize() const {
    return data_points.size();
}

} // namespace cable_utils
