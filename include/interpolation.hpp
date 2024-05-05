#ifndef INTEPOLATION_HPP
#define INTEPOLATION_HPP

float linearinterpolation(float scalar_value1, float scalar_value2, float threshold)
{
    return (scalar_value1-threshold) / (scalar_value1 - scalar_value2);
}

float harmonicinterpolation(float scalar_value1, float scalar_value2, float threshold)
{
    return (scalar_value1 * (scalar_value2 - threshold)) / (threshold * (scalar_value2 - scalar_value1));
}
#endif