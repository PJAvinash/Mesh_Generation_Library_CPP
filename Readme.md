# Mesh Generation Library implementation in C++ 
In this project, we plan to implement mesh generation algorithms for 2D and 3D shapes in C++.

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
![Sphere in sphere](results/sphere_in_sphere_mesh.png)

![Ball on top of xy+yz+zx = c](results/BallOnTop.png)

![Ball on top of xy+yz+zx = c with edges](results/ball_on_top_40_40_40_edges.png)

![Distorted sphere along with few other sphere](results/Figure_1.png)

![Random](results/Figure_6.png)

## Future plans
- add support for multi-threading 
- OpenCL / CUDA-based functions 
- addition of better interpolation functions
- support for  IO (.ply, .obj, .mesh )




**contributors**
- PJ Avinash
- Akhila Petnikota
**contact-email**
- petnikotaakhila@gmail.com
- avinashindian2.0@gmail.com
