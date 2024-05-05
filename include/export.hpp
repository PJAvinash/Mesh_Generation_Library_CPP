#ifndef EXPORT_HPP
#define EXPORT_HPP
#include <fstream>
#include "simplices.hpp"
#include <iostream>

void saveTrianglesToCSV(const std::string& filename,const std::vector<triangle_3d<float>>& Triangles) {
    std::ofstream outfile(filename);
    if (!outfile) {
        std::cerr << "Error: Failed to open file " << filename << " for writing" << std::endl;
        return;
    }
    for (const auto& triangle : Triangles) {
        for (const auto& vertex : triangle.vertices) {
            outfile << "["<< vertex.x << "," << vertex.y << "," << vertex.z << "]";
        }
        outfile << "\n";
    }
    outfile.close();
}


template <typename T>
void write_mesh_surface_to_ply(const mesh_surface<T>& mesh, const std::string& filename) {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open file for writing." << std::endl;
        return;
    }

    // Write PLY header
    outfile << "ply" << std::endl;
    outfile << "format ascii 1.0" << std::endl;
    outfile << "element vertex " << mesh.vertices.size() << std::endl;
    outfile << "property float x" << std::endl;
    outfile << "property float y" << std::endl;
    outfile << "property float z" << std::endl;
    outfile << "element face " << mesh.triangle_vertex_indices.size() << std::endl;
    outfile << "property list uchar int vertex_indices" << std::endl;
    outfile << "end_header" << std::endl;

    // Write vertex data
    for (const auto& vertex : mesh.vertices) {
        outfile << vertex.x << " " << vertex.y << " " << vertex.z << std::endl;
    }

    // Write face data
    for (const auto& face : mesh.triangle_vertex_indices) {
        outfile << "3 " << std::get<0>(face) << " " << std::get<1>(face) << " " << std::get<2>(face) << std::endl;
    }

    outfile.close();
}

#endif