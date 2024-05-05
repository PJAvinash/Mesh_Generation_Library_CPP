#ifndef DEBUGHELPERS_HPP
#define DEBUGHELPERS_HPP

#include <vector>
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
#endif