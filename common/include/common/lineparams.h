#pragma once
#include <array>
#include <memory>

class LineParams
{
public:
    typedef std::array<float, 3> ImplType;

    LineParams(const ImplType& params);
    LineParams(float a, float b, float c);
    LineParams();

    float& operator[](int index);
    float operator[](int index) const;

    inline float const * begin() const
    {
        return params.begin();
    }

    inline float const * begin()
    {
        return params.begin();
    }

    inline float const * end() const
    {
        return params.end();
    }

    inline float const * end()
    {
        return params.end();
    }

    LineParams operator+(const LineParams& other) const;
    LineParams operator-(const LineParams& other) const;
    LineParams operator*(const LineParams& other) const;

    template<typename _T=LineParams>
    LineParams operator/(const LineParams& other) const;

    template<typename _T>
    LineParams operator/(const _T& number) const
    {
        LineParams result;
        for (int i = 0; i < params.size(); i++)
        {
            result[i] = params[i] / number;
        }
        return result;
    }

private:
    ImplType params;  
};