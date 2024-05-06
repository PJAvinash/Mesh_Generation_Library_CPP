#include <iostream>
#include <vector>
#include "../include/mcalgorithm.hpp"
#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
    std::vector<std::vector<std::vector<float>>> computedValues;
    std::vector<std::vector<std::vector<u_int8_t>>> mesh_indices;
    // computeGridPoints(grid_params_3d(100, 100, 100), sphere, computedValues);
    //printf("Bits\n");
    auto start_time = std::chrono::high_resolution_clock::now();
    //generateMarchingCubeVertices(grid_params_3d(1000, 1000, 1000), sphere, 0, mesh_indices);
    std::vector< triangle_3d<float> > Triangles = {};
    generateMeshFromScalarField(grid_params_3d(-80,-80,-80,80,80,80),ringtorus,1,Triangles);
    //generateMeshFromScalarField(point_3d<int>(0,0,0),grid_params_3d(30,30,30),warm,0,Triangles);
    //generateMeshFromScalarField(grid_params_3d(0,0,0,20,20,20),sphere1,0,Triangles);
   //generateMeshFromScalarFieldParallel(8,grid_params_3d(-100,-100,-100,100,100,100),warm,0,Triangles);
    //generateMeshFromScalarField(grid_params_3d(-10,-10,-10,10,10,10),sphere1,0,Triangles);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Time taken: " << duration.count() << " milliseconds" << std::endl;
    mesh_surface<float> ms;
    generate_surface_mesh_ply<float>(Triangles,ms.vertices,ms.edge_vertex_indices,ms.triangle_vertex_indices);
    saveTrianglesToCSV("../test/triangles.txt",Triangles);
    write_mesh_surface_to_ply(ms,"../test/mesh.ply");
    return 0;
}