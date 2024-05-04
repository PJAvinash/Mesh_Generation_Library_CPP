# Mesh Generation Library implementation in C++ 
In this project we plan to implement a mesh generation algorithms for 2D and 3D shapes in c++.

## Algorithm
 - Marching Cubes

**compilation and execution [in src]**
 ```
   g++ -std=c++14 -o marchingcubes marchingcubes.cpp
   ./marchingcubes
 ```
 **visualization[in test/]**
 ```
 python3 viz.py
 ```

## Test results
![Ball on top of xy+yz+zx = c](Mesh_Generation_Library_CPP/results/BallOnTop.png)

![Distorted sphere along with few other sphere](Mesh_Generation_Library_CPP/results/Figure_1.png)

## Future plans
- add support for multi threading 
- OpenCL / CUDA based functions 
- adition of better interpolation functions
- support for  IO (.ply , .obj, .mesh )




**contributors**
- PJ Avinash
- Akhila Petnikota
**contact-email**
- petnikotaakhila@gmail.com
- avinashindian2.0@gmail.com