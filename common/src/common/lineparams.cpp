#include "common/lineparams.h"

LineParams::LineParams(const ImplType& other)
{
    std::copy(other.begin(), other.end(), params.begin());
}

LineParams::LineParams(float a, float b, float c)
{
    params[0] = a;
    params[1] = b;
    params[2] = c;
}

LineParams::LineParams()
    : LineParams{0, 0, 0}
{
}

float& LineParams::operator[](int index)
{
    return params[index];
}

float LineParams::operator[](int index) const
{
    return params[index];
}

LineParams LineParams::operator+(const LineParams& other) const
{
    LineParams result;
    for (size_t i = 0; i < params.size(); i++)
    {
        result[i] = other.params[i] + params[i];
    }
    return result;
}

LineParams LineParams::operator-(const LineParams& other) const
{
    LineParams result;
    for (size_t i = 0; i < params.size(); i++)
    {
        result[i] = other.params[i] - params[i];
    }
    return result;
}
LineParams LineParams::operator*(const LineParams& other) const
{
    LineParams result;
    for (size_t i = 0; i < params.size(); i++)
    {
        result[i] = other.params[i] * params[i];
    }
    return result;
}

template<>
LineParams LineParams::operator/(const LineParams& other) const
{
    LineParams result;
    for (size_t i = 0; i < params.size(); i++)
    {
        result[i] = other.params[i] / params[i];
    }
    return result;
}