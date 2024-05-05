#ifndef SIMPLICES_HPP
#define SIMPLICES_HPP
#include <vector>
#include <set>

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
struct mesh_volume
{
    std::vector<point_3d<T>> vertices;
    std::vector<triangle_3d<T>> triangles;
    std::vector<tetrahedron_3d<T>> tetrahedrons;
};

template <typename T>
struct mesh_surface
{
    std::vector<point_3d<T>> vertices;
    std::set< std::tuple<int,int> > edge_vertex_indices;
    std::vector< std::tuple <int,int,int> > triangle_vertex_indices;
    mesh_surface(){
        vertices = {};
        edge_vertex_indices = {};
        triangle_vertex_indices = {};
    }
};

#endif