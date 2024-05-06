#ifndef SCALARFIELDS_HPP
#define SCALARFIELDS_HPP
#include <cmath>
#include <functional>
#include <algorithm>
using ScalarFunction = std::function<float(int, int, int)>;
using InterpolationFunction = std::function<float(float, float)>;

float sineWaveScalarFunction(int x, int y, int z)
{
    float frequency = 0.01f;
    float amplitude = 1.0f;
    float scalar_value = amplitude * sin(frequency * x) * sin(frequency * y) * sin(frequency * z);
    return scalar_value;
}

const float r1 = 20.0f; // Radius of the sphere
const float r2 = 7.0f;
ScalarFunction sphere1 = [](int x, int y, int z) -> float
{
    float a = (x * x + y * y + z * z - r1 * r1);
    float b = -(x * x + y * y + z * z - r2 * r2);
    return a*b;
};

ScalarFunction warm = [](int x, int y, int z) -> float{
    float a = (x-10)*(x-10)/3 + (y -10)*(y-10) + (z-10)*(z-10) - 10*10;
    float b = (x-14)*(x-14) + (y -10)*(y-10)/3 + (z-10)*(z-10) - 10*10;
    float c = (x-17)*(x-17) + (y -10)*(y-10)/3 + (z-10)*(z-10)/4 - 10*10;
    return std::min(std::min(a,b),c);
};


ScalarFunction randomtest = [](int x, int y, int z) -> float{
    float a = x*y + y*z + z*x -100;
    float b = (x+4) * (x+4) + (y+4) * (y+4) + (z+4) * (z+4) -25;
    return a*b;
};


ScalarFunction ringtorus = [](int x, int y, int z) -> float{
   float srt =  std::sqrt(x*x+y*y);
   float a = (float)x/srt;
   float b = (float)y/srt;
   constexpr float R1 = 40;
   constexpr float R2 = 15;
   return (x - R1*a)*(x - R1*a) + (y - R1*b)*(y - R1*b) + z*z - R2*R2;
};

#endif

