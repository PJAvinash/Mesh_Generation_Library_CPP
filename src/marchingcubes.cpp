#include <iostream>
#include <vector>

#include <fstream>
#include "../include/constants.h"
#include "scalarfields.hpp"
#include <thread>

template <typename T>
struct point_3d
{
    T x;
    T y;
    T z;
    point_3d(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
};

/*
** 2D simplex
*/
template <typename T>
struct triangle_3d
{
    point_3d<T> vertices[3];
    triangle_3d(point_3d<T> _vertices[3]) : vertices{_vertices[0], _vertices[1], _vertices[2]} {}
};

/*
** 3D simplex, can also be represted with 4 trangles
*/
template <typename T>
struct tetrahedron_3d
{
    point_3d<T> *vertices[4];
};

/*
** Generic mesh structure
*/
template <typename T>
struct mesh
{
    std::vector<point_3d<T>> vertices;
    std::vector<triangle_3d<T>> triangles;
    std::vector<tetrahedron_3d<T>> tetrahedrons;
};

/*
**
*/
struct grid_params_3d
{
    int x;
    int y;
    int z;
    grid_params_3d(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}
};


// requires
void computeGridPoints(grid_params_3d grid_params, ScalarFunction scalar_function, std::vector<std::vector<std::vector<float>>> &computedValues)
{
    for (int i = 0; i < grid_params.x; i++)
    {
        std::vector<std::vector<float>> v_y(grid_params.y);
        for (int j = 0; j < grid_params.y; j++)
        {
            std::vector<float> v_z(grid_params.z);
            for (int k = 0; k < grid_params.z; k++)
            {
                v_z[k] = scalar_function(i, j, k); // Call the scalar function
            }
            v_y[j] = v_z;
        }
        computedValues.push_back(v_y);
    }
}

template <typename T>
void printGridEvaluations(const std::vector<std::vector<std::vector<T>>> &computedValues)
{
    // Print the computed values (for testing)
    for (int i = 0; i < computedValues.size(); ++i)
    {
        for (int j = 0; j < computedValues[0].size(); ++j)
        {
            for (int k = 0; k < computedValues[0][0].size(); ++k)
            {
                std::cout << "(" << i << ", " << j << ", " << k << "): " << (int)computedValues[i][j][k] << std::endl;
            }
        }
    }
}

float linearinterpolation(float scalar_value1, float scalar_value2, float threshold)
{
    return (scalar_value1-threshold) / (scalar_value1 - scalar_value2);
}
float harmonicinterpolation(float scalar_value1, float scalar_value2, float threshold)
{
    return (scalar_value1 * (scalar_value2 - threshold)) / (threshold * (scalar_value2 - scalar_value1));
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
    for (int i = 0; i < grid_params.z - 1; i += 2)
    {
        std::vector<std::vector<uint8_t>> v_y(grid_params.y >> 1);
        for (int j = 0; j < grid_params.y - 1; j += 2)
        {
            std::vector<uint8_t> v_x(grid_params.x >> 1);
            for (int k = 0; k < grid_params.x - 1; k += 2)
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
        std::vector<int> cubetraingles = TriangleTable[cubeIndex];
        float edgeoffset[12];
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

void generateMeshFromScalarField(point_3d<int> referece, grid_params_3d grid_params, ScalarFunction scalar_function, float threshold, std::vector< triangle_3d<float> > &Triangles)
{
    #pragma omp parallel for collapse(3)
    for (int i = 0; i < grid_params.z; i += 1)
    {
        int z_new = referece.z + i;
        for (int j = 0; j < grid_params.y; j += 1)
        {
            int y_new = referece.y + j;
            for (int k = 0; k < grid_params.x; k += 1)
            {
                int x_new = referece.x + k;
                getTrainglesAtXYZ(scalar_function, threshold, x_new,y_new,z_new,Triangles);
            }
        }
    }
}






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


int main(int argc, char *argv[])
{
    std::vector<std::vector<std::vector<float>>> computedValues;
    std::vector<std::vector<std::vector<u_int8_t>>> mesh_indices;
    // computeGridPoints(grid_params_3d(100, 100, 100), sphere, computedValues);
    printf("Bits\n");
    //generateMarchingCubeVertices(grid_params_3d(1000, 1000, 1000), sphere, 0, mesh_indices);
    std::vector< triangle_3d<float> > Triangles = {};
    generateMeshFromScalarField(point_3d<int>(-40,-40,-40),grid_params_3d(40,40,40),randomtest,1,Triangles);
    saveTrianglesToCSV("../test/MCsphere.txt",Triangles);
    return 0;
}
