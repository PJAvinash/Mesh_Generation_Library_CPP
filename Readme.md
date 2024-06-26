# Mesh Generation Library implementation in C++ 
In this project, we plan to implement mesh generation algorithms for 2D and 3D shapes in C++.

## Algorithm
 - Marching Cubes

**compilation and execution [in src]**
 ```
   g++ -std=c++14 -o marchingcubes main.cpp
   ./marchingcubes
 ```
 **visualization[in test/]**
 ```
 python3 viz.py <.txt filepath>
 python3 vizply.py <ply filepath>
 ```

## Test results
**Ring torus(https://3dviewer.net/)**
![Ring torus](results/ringtorus.png)

**Ring torus using open 3d**
![Ring torus](results/Ringtorus_open3d.png)

**sphere in sphere**
![Sphere in sphere](results/sphere_in_sphere_mesh.png)

![Ball on top of xy+yz+zx = c with edges](results/ball_on_top_40_40_40_edges.png)

![Distorted sphere along with few other sphere](results/Figure_1.png)

![Random](results/Figure_6.png)

## Existing features
- supports multiple threads
- function to export to .ply 
- use open3d ( python lib) for visualization 

## Future plans
- OpenCL / CUDA-based functions 
- addition of better interpolation functions
- support for  IO (.obj, .mesh )




**contributors**
- PJ Avinash
- Akhila Petnikota
**contact-email**
- petnikotaakhila@gmail.com
- avinashindian2.0@gmail.com
