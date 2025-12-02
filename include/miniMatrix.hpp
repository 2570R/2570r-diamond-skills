#pragma once
#include <array>
#include <initializer_list>
#include <stdexcept>
#include <cstddef>

// Matrix<T, ROWS, COLS> — Compatible with Eigen::Matrix<double,R,C>
template<typename T, int ROWS, int COLS>
class MiniMatrix
{
public:
    std::array<T, ROWS * COLS> data{};

    MiniMatrix() { data.fill(T{}); }

    // element access
    T& operator()(int r, int c)
    {
        return data[r * COLS + c];
    }

    T operator()(int r, int c) const
    {
        return data[r * COLS + c];
    }

    // eigen-style << initializer
    MiniMatrix& operator<<(std::initializer_list<T> values)
    {
        if (values.size() != ROWS * COLS)
            throw std::runtime_error("Wrong number of values in Matrix <<");

        int i = 0;
        for (T v : values) data[i++] = v;
        return *this;
    }
};

// -----------------------------------------------------
// Matrix multiplication: (R×A) * (A×C) = (R×C)
// -----------------------------------------------------
template<typename T, int R, int A, int C>
MiniMatrix<T, R, C> operator*(const MiniMatrix<T, R, A>& lhs,
                          const MiniMatrix<T, A, C>& rhs)
{
    MiniMatrix<T, R, C> result;

    for (int r = 0; r < R; r++)
        for (int c = 0; c < C; c++)
        {
            T sum = 0;
            for (int k = 0; k < A; k++)
                sum += lhs(r, k) * rhs(k, c);

            result(r, c) = sum;
        }

    return result;
}
