#ifndef MCALGORITHM_HPP
#define MCALGORITHM_HPP

#include <future>
#include "constants.h"
#include "scalarfields.hpp"
#include "simplices.hpp"
#include "export.hpp"
#include "interpolation.hpp"

/*
**
*/
struct grid_params_3d
{
    int x_min;
    int y_min;
    int z_min;
    int x_max;
    int y_max;
    int z_max;
    grid_params_3d(int _x_min, int _y_min, int _z_min, int _x_max, int _y_max, int _z_max) : x_min(_x_min), y_min(_y_min), z_min(_z_min),x_max(_x_max),y_max(_y_max),z_max(_z_max){}

    int xrange(){
        return x_max - x_min;
    }
    int yrange(){
        return y_max - y_min;
    }
    int zrange(){
        return z_max - z_min;
    }
};

// requires
void computeGridPoints(grid_params_3d grid_params, ScalarFunction scalar_function, std::vector<std::vector<std::vector<float>>> &computedValues)
{
    for (int i = grid_params.x_min; i < grid_params.x_max; i++)
    {
        std::vector<std::vector<float>> v_y(grid_params.yrange());
        for (int j = grid_params.y_min; j < grid_params.y_max; j++)
        {
            std::vector<float> v_z(grid_params.zrange());
            for (int k = grid_params.z_min; k < grid_params.z_max; k++)
            {
                v_z[k] = scalar_function(i, j, k); // Call the scalar function
            }
            v_y[j] = v_z;
        }
        computedValues.push_back(v_y);
    }
}





void scalarAtCube_xyz(ScalarFunction scalar_function, int x, int y, int z, float (&arr)[8])
{
    arr[0] = scalar_function(x, y, z);
    arr[1] = scalar_function(x + 1, y, z);
    arr[2] = scalar_function(x, y + 1, z);
    arr[3] = scalar_function(x + 1, y + 1, z);
    arr[4] = scalar_function(x, y, z + 1);
    arr[5] = scalar_function(x + 1, y, z + 1);
    arr[6] = scalar_function(x, y + 1, z + 1);
    arr[7] = scalar_function(x + 1, y + 1, z + 1);
}

// arr is scalar evaluation
u_int8_t cubeIndexAtXYZ(float (&arr)[8], float threshold)
{
    u_int8_t cubeIndex = 0;
    cubeIndex |= (arr[0] < threshold);
    cubeIndex |= ((arr[1] < threshold) << 1);
    cubeIndex |= ((arr[2] < threshold) << 2);
    cubeIndex |= ((arr[3] < threshold) << 3);
    cubeIndex |= ((arr[4] < threshold) << 4);
    cubeIndex |= ((arr[5] < threshold) << 5);
    cubeIndex |= ((arr[6] < threshold) << 6);
    cubeIndex |= ((arr[7] < threshold) << 7);
    return cubeIndex;
}

void generateCubeIndexes(grid_params_3d grid_params, ScalarFunction scalar_function, float threshold, std::vector<std::vector<std::vector<u_int8_t>>> &scalar_binary_evaluation)
{
    for (int i = grid_params.z_min; i < grid_params.z_max - 1; i += 2)
    {
        std::vector<std::vector<uint8_t>> v_y((grid_params.y_max - grid_params.y_min) >> 1);
        for (int j = grid_params.y_min; j < grid_params.y_max - 1; j += 2)
        {
            std::vector<uint8_t> v_x((grid_params.x_max - grid_params.x_min) >> 1);
            for (int k = grid_params.x_min; k < grid_params.x_max - 1; k += 2)
            {
                u_int8_t cubeIndex = 0;
                cubeIndex |= (scalar_function(i, j, k) < threshold);
                cubeIndex |= ((scalar_function(i + 1, j, k) < threshold) << 1);
                cubeIndex |= ((scalar_function(i, j + 1, k) < threshold) << 2);
                cubeIndex |= ((scalar_function(i + 1, j + 1, k) < threshold) << 3);
                cubeIndex |= ((scalar_function(i, j, k + 1) < threshold) << 4);
                cubeIndex |= ((scalar_function(i + 1, j, k + 1) < threshold) << 5);
                cubeIndex |= ((scalar_function(i, j + 1, k + 1) < threshold) << 6);
                cubeIndex |= ((scalar_function(i + 1, j + 1, k + 1) < threshold) << 7);
                v_x[(k >> 1)] = cubeIndex;
            }
            v_y[(j >> 1)] = v_x;
        }
        scalar_binary_evaluation.push_back(v_y);
    }
}

void getTrainglesAtXYZ(ScalarFunction scalar_function, float threshold, int x, int y, int z, std::vector<triangle_3d<float>> &Triangles)
{
    float scalar_evaluation[8];
    scalarAtCube_xyz(scalar_function, x, y, z, scalar_evaluation);
    const uint8_t cubeIndex = cubeIndexAtXYZ(scalar_evaluation, threshold);
    if (cubeIndex != (uint8_t)255 || cubeIndex != (uint8_t)0)
    {
        float edgeoffset[12];
        std::vector<int> cubetraingles = TriangleTable[cubeIndex];
        for (size_t edgeindex = 0; edgeindex < cubetraingles.size() - 1; edgeindex += 3)
        {
            edgeoffset[cubetraingles[edgeindex]] = linearinterpolation(scalar_evaluation[edge_vertices[cubetraingles[edgeindex]][0]], scalar_evaluation[edge_vertices[cubetraingles[edgeindex]][1]], threshold);
            edgeoffset[cubetraingles[edgeindex + 1]] = linearinterpolation(scalar_evaluation[edge_vertices[cubetraingles[edgeindex + 1]][0]], scalar_evaluation[edge_vertices[cubetraingles[edgeindex + 1]][1]], threshold);
            edgeoffset[cubetraingles[edgeindex + 2]] = linearinterpolation(scalar_evaluation[edge_vertices[cubetraingles[edgeindex + 2]][0]], scalar_evaluation[edge_vertices[cubetraingles[edgeindex + 2]][1]], threshold);
        }
        
        for (size_t edgeindex = 0; edgeindex < cubetraingles.size() - 1; edgeindex += 3)
        {
            float P0[3] = {0, 0, 0};
            float P1[3] = {0, 0, 0};
            float P2[3] = {0, 0, 0};
            // i is axis direction x ->0, y-> 1, z-> 2
            for (int i = 0; i < 3; i++)
            {
                P0[i] = vertices_float[edge_vertices[cubetraingles[edgeindex]][0]][i] + (edge_vertices[cubetraingles[edgeindex]][2] == i) * edgeoffset[cubetraingles[edgeindex]];
                P1[i] = vertices_float[edge_vertices[cubetraingles[edgeindex + 1]][0]][i] + (edge_vertices[cubetraingles[edgeindex + 1]][2] == i) * edgeoffset[cubetraingles[edgeindex + 1]];
                P2[i] = vertices_float[edge_vertices[cubetraingles[edgeindex + 2]][0]][i] + (edge_vertices[cubetraingles[edgeindex + 2]][2] == i) * edgeoffset[cubetraingles[edgeindex + 2]];
            }
            point_3d<float> traingle_vertices[3] = {point_3d<float>(x+P0[0], y+P0[1], z+P0[2]), point_3d<float>(x+P1[0],y+P1[1],z+P1[2]), point_3d<float>(x+P2[0],y+ P2[1],z+P2[2])};
            triangle_3d<float> t(traingle_vertices);
            Triangles.push_back(t);
        }
    }
}

void generateMeshFromScalarField(grid_params_3d grid_params, ScalarFunction scalar_function, float threshold, std::vector< triangle_3d<float> > &Triangles)
{
    #pragma omp parallel for collapse(3)
    for (int i = grid_params.z_min; i < grid_params.z_max; i += 1)
    {
        for (int j = grid_params.y_min; j < grid_params.y_max; j += 1)
        {
            for (int k = grid_params.x_min; k < grid_params.x_max; k += 1)
            {
                getTrainglesAtXYZ(scalar_function, threshold, k,j,i,Triangles);
            }
        }
    }
}


void generateMeshFromScalarFieldParallel(int numThreads, grid_params_3d grid_params, ScalarFunction scalar_function, float threshold, std::vector< triangle_3d<float> > &Triangles)
{
    std::vector<std::future< std::vector <triangle_3d<float> > >> futures;
    int thread_stride_length = grid_params.xrange()/numThreads;
    for (int threadIndex = 0; threadIndex < numThreads; ++threadIndex) {
        int x_start = thread_stride_length*threadIndex+grid_params.x_min;
        int x_end = std::min(x_start + thread_stride_length, grid_params.x_max);
        futures.push_back(std::async(std::launch::async, [=]() {
            std::vector< triangle_3d<float> > result = {};
            generateMeshFromScalarField(grid_params_3d(x_start,grid_params.y_min,grid_params.z_min,x_end,grid_params.y_max,grid_params.z_max),scalar_function,threshold,result);
            return result;
        }
        ));
    }
    for (auto &fut : futures) {
        std::vector<triangle_3d<float>> result = fut.get();
        Triangles.insert(Triangles.end(), result.begin(), result.end());
    }
}


// Function to transform triangles into vertices, edges, and triangles

namespace std {
    template <>
    struct hash<std::tuple<int, int, int, int>> {
        size_t operator()(const std::tuple<int, int, int, int>& t) const {
            return std::get<0>(t) ^ std::get<1>(t) ^ std::get<2>(t) ^ std::get<3>(t);
        }
    };
}
template <typename T>
std::tuple<int, int, int, int> getTupleIndex(const point_3d<T> &vertex){
    int d = 0;
    float max = -1;
    int ax[3] = {(int)std::floor(vertex.x),(int)std::floor(vertex.y),(int)std::floor(vertex.z)};
    if((vertex.x - ax[0]) > max){
        max = vertex.x - ax[0];
        d = 0;
    }
    if((vertex.y - ax[1]) > max){
        max = vertex.y - ax[1];
        d = 1;
    }
    if((vertex.z- ax[2]) > max){
        max = vertex.z - ax[2];
        d = 3;
    }
    return std::make_tuple(ax[0],ax[1],ax[2],d);
}

template <typename T>
void generate_surface_mesh_ply(const std::vector<triangle_3d<T>>& triangles, std::vector<point_3d<T>> &vertices,std::set< std::tuple<int,int> > &edge_vertex_indices, std::vector< std::tuple <int,int,int> > &triangle_vertex_indices) {
    std::unordered_map<std::tuple<int, int, int,int>,point_3d<T> > verticesMap ;
    for (size_t i = 0; i < triangles.size(); i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            //verticesMap[getTupleIndex<T>(triangles[i].vertices[j])] = triangles[i].vertices[j];
            verticesMap.insert(std::make_pair(getTupleIndex<T>(triangles[i].vertices[j]), triangles[i].vertices[j]));
        }
    }
    std::unordered_map<std::tuple<int, int, int,int>,int> verticesIndex ;
    int i = vertices.size();
    for(typename std::unordered_map<std::tuple<int, int, int,int>,point_3d<T> >::iterator it = verticesMap.begin(); it != verticesMap.end(); it++) {
        vertices.push_back(it->second);
        verticesIndex[(it->first)] = i;
        i++;
    }
    for (size_t i = 0; i < triangles.size(); i++){
        auto index0 = verticesIndex[getTupleIndex<T>(triangles[i].vertices[0])];
        auto index1 = verticesIndex[getTupleIndex<T>(triangles[i].vertices[1])];
        auto index2 = verticesIndex[getTupleIndex<T>(triangles[i].vertices[2])];
        edge_vertex_indices.insert(std::make_tuple(std::min(index0,index1),std::max(index0,index1)));
        edge_vertex_indices.insert(std::make_tuple(std::min(index1,index2),std::max(index1,index2)));
        edge_vertex_indices.insert(std::make_tuple(std::min(index2,index0),std::max(index2,index0)));
        triangle_vertex_indices.push_back({index0,index1,index2});
    }
}
#endif